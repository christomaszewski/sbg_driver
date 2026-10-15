#!/usr/bin/env python3
# Copyright 2026 Chris Tomaszewski
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Render a generic *rig* sensor config into this driver's ROS 2 params file.

The sensor config is the single source of truth (uniform across all rig services). `driver_params`
is OPAQUE -- copied verbatim into ros__parameters -- while `transport:` is DERIVED from the generic
`connection` block (the template knows its own transport schema) and `health:` from the fleet
identity (config `name` + the VEHICLE_ID environment variable rig exports) plus the optional
generic `health:` block. Used by `sbg-up`:

  render_params.py <config.yaml>          # -> the ROS 2 params YAML on stdout
  render_params.py --env <config.yaml>    # -> SERVICE/NAME/NAMESPACE/TYPE/DEVICE env lines

The params doc is keyed by the `/**` wildcard node so it binds regardless of the namespace rig
pushes (do NOT key by node name). Needs PyYAML on the host (apt: python3-yaml).
"""
import math
import os
import re
import shlex
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.stderr.write("render_params: PyYAML required on the host (apt install python3-yaml)\n")
    sys.exit(2)


# Instance identity tokens end up in compose project names, ROS namespaces, staging file
# paths, and an eval'd env block in the launcher — keep them to a safe, unambiguous alphabet.
_IDENT_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def require_ident(what: str, value: str) -> str:
    if not _IDENT_RE.match(value):
        sys.stderr.write(
            f"render_params: {what} {value!r} must match [a-z][a-z0-9_]* "
            "(it becomes a compose project / ROS namespace / file name)\n")
        sys.exit(2)
    return value


def _pick(block: dict, out: dict, key: str, src_key: str, cast) -> None:
    """Copy block[src_key] into out[key] (cast applied) only when the user provided it —
    absent keys are OMITTED so the driver's own schema defaults apply instead of this
    script fabricating divergent ones."""
    if src_key in block:
        out[key] = cast(block[src_key])


def derive_transport(connection: dict) -> dict:
    """Map the generic `connection` block onto this driver's `transport:` param schema.

    Only the transports this driver was scaffolded with are mapped; anything else is a config
    error surfaced here (NOT silently rendered into params the driver would reject).
    """
    ttype = str(connection.get("type") or "file")
    block = connection.get(ttype) or {}
    if ttype == "serial":
        # serial.by_id is used twice: here as the in-container serial port, and by the launcher as
        # the host --device to pass through.
        sub: dict = {}
        _pick(block, sub, "port", "by_id", str)
        _pick(block, sub, "baud", "baud", int)
        return {"type": "serial", "serial": sub} if sub else {"type": "serial"}
    if ttype == "udp":
        sub = {}
        _pick(block, sub, "remote_ip", "remote_ip", str)
        _pick(block, sub, "in_port", "in_port", int)
        _pick(block, sub, "out_port", "out_port", int)
        return {"type": "udp", "udp": sub} if sub else {"type": "udp"}
    if ttype == "file":
        sub = {}
        _pick(block, sub, "path", "path", str)
        _pick(block, sub, "real_time_pace", "real_time_pace", bool)
        return {"type": "file", "file": sub} if sub else {"type": "file"}
    sys.stderr.write(f"render_params: unsupported connection.type {ttype!r}"
                     " (this driver supports: serial, udp, file)\n")
    sys.exit(2)


# Fleet identity tokens become a Zenoh key chunk on the dashboard side (fleet/<vehicle>/svc/<name>).
_SEGMENT_RE = re.compile(r"^[^/*?#$\s]+$")

# The generic health block (same shape across fleet services) -> this driver's health.* params.
_HEALTH_TIMINGS = {"interval_s": "publish_interval_s", "stale_after_s": "stale_after_s",
                   "stream_timeout_s": "stream_timeout_s"}
_HEALTH_DEFAULTS = {"publish_interval_s": 1.0, "stale_after_s": 5.0}  # the driver's own
_HEALTH_METRICS = {"temp.imu_c": "temp_imu_c"}
_HEALTH_BOUNDS = ("warn_above", "warn_below", "error_above", "error_below")
_HEALTH_GNSS = ("none", "fix", "rtk_float", "rtk_fixed")
_HEALTH_ALERTS = ("MAIN_POWER", "IMU_POWER", "GPS_POWER", "SETTINGS", "TEMPERATURE", "DATALOGGER",
                  "CPU")


def _fail(message: str):
    sys.stderr.write(f"render_params: {message}\n")
    sys.exit(2)


def _number(what: str, value) -> float:
    """A finite number as a float: ROS rejects an integer for a double parameter, and a config
    author writing `interval_s: 1` should not have to know that."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        _fail(f"{what} must be a finite number")
    return float(value)


def derive_health(cfg: dict, name: str, environ) -> dict:
    """Fleet identity + the generic `health:` block -> the driver's `health:` params.

    Checked here so a bad value fails BEFORE a container is started or replaced; the node
    validates the same things again when it is configured.
    """
    out: dict = {"instance": name}
    vehicle = str(environ.get("VEHICLE_ID") or "").strip()
    if vehicle:
        if not _SEGMENT_RE.match(vehicle):
            _fail(f"VEHICLE_ID {vehicle!r} must be a single path segment"
                  " (no '/', '*', '?', '#', '$' or whitespace)")
        out["vehicle_id"] = vehicle

    block = cfg.get("health")
    if block is None:
        return out
    if not isinstance(block, dict):
        _fail("health must be a mapping")
    if "poll_interval_s" in block:
        _fail("health.poll_interval_s does not apply to this driver: it polls nothing, every"
              " health observation is a log the device pushes. Use health.stale_after_s for how"
              " long an observation stays valid.")
    known = set(_HEALTH_TIMINGS) | {"required_gnss_fix", "ignored_alerts", "limits"}
    unknown = sorted(set(block) - known)
    if unknown:
        _fail(f"unknown health setting(s) {unknown}; supported: {sorted(known)}")

    for key, param in _HEALTH_TIMINGS.items():
        if key in block:
            out[param] = _number(f"health.{key}", block[key])
            if out[param] <= 0:
                _fail(f"health.{key} must be positive")
    interval = out.get("publish_interval_s", _HEALTH_DEFAULTS["publish_interval_s"])
    stale = out.get("stale_after_s", _HEALTH_DEFAULTS["stale_after_s"])
    if stale < 2 * interval:
        _fail(f"health.stale_after_s ({stale:g}) must be at least twice health.interval_s"
              f" ({interval:g})")

    if "required_gnss_fix" in block:
        grade = block["required_gnss_fix"]
        if grade not in _HEALTH_GNSS:
            _fail(f"health.required_gnss_fix must be one of {list(_HEALTH_GNSS)}")
        out["required_gnss_fix"] = grade
    if "ignored_alerts" in block:
        codes = block["ignored_alerts"]
        if (not isinstance(codes, list) or len(set(map(str, codes))) != len(codes)
                or any(code not in _HEALTH_ALERTS for code in codes)):
            _fail(f"health.ignored_alerts must be a list of distinct codes from"
                  f" {list(_HEALTH_ALERTS)}")
        out["ignored_alerts"] = list(codes)

    limits = block.get("limits") or {}
    if not isinstance(limits, dict):
        _fail("health.limits must be a mapping")
    for metric, bounds in limits.items():
        if metric not in _HEALTH_METRICS:
            _fail(f"health.limits.{metric}: this driver measures only {sorted(_HEALTH_METRICS)}")
        if not isinstance(bounds, dict) or set(bounds) - set(_HEALTH_BOUNDS):
            _fail(f"health.limits.{metric} must map any of {list(_HEALTH_BOUNDS)} to a number")
        rendered = {bound: _number(f"health.limits.{metric}.{bound}", value)
                    for bound, value in bounds.items()}
        if rendered:
            out.setdefault("limits", {})[_HEALTH_METRICS[metric]] = rendered
    return out


def deep_merge(base: dict, override: dict) -> dict:
    """`override` onto `base`, recursing into mappings (so a partial override keeps the rest)."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def main() -> int:
    args = sys.argv[1:]
    env_mode = bool(args and args[0] == "--env")
    if env_mode:
        args = args[1:]
    if len(args) != 1:
        sys.stderr.write("usage: render_params.py [--env] <sensor-config.yaml>\n")
        return 2

    with open(args[0]) as handle:
        cfg = yaml.safe_load(handle) or {}

    service = require_ident("service", str(cfg.get("service") or "sbg"))
    name = require_ident("name", str(cfg.get("name") or "sbg"))
    connection = cfg.get("connection") or {}
    ttype = str(connection.get("type") or "file")
    namespace = require_ident("ros.namespace", str((cfg.get("ros") or {}).get("namespace") or name))

    if ttype not in ("serial", "udp", "file"):
        _fail(f"unsupported connection.type {ttype!r} (expected serial, udp, file)")

    if env_mode:
        values = {"SERVICE": service, "NAME": name, "NAMESPACE": f"/{namespace}", "TYPE": ttype}
        if ttype == "serial":
            device = (connection.get('serial') or {}).get('by_id', '')
            if not isinstance(device, str) or not device or '\0' in device:
                _fail("connection.serial.by_id must be a nonempty path without NUL bytes")
            values["DEVICE"] = device
        print("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()))
        return 0

    # params mode: transport + health (derived) first, then driver_params verbatim (minus the
    # transport key). driver_params.health is merged INTO the derived block rather than replacing
    # it: an explicit health.* param wins, but setting one must not drop the fleet identity.
    merged: dict = {"transport": derive_transport(connection),
                    "health": derive_health(cfg, name, os.environ)}
    for key, value in (cfg.get("driver_params") or {}).items():
        if key == "health" and isinstance(value, dict):
            merged[key] = deep_merge(merged[key], value)
        elif key != "transport":
            merged[key] = value
    doc = {"/**": {"ros__parameters": merged}}
    yaml.safe_dump(doc, sys.stdout, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
