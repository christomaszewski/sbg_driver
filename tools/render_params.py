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
provides explicit ROS overrides; missing frame IDs are namespaced, while `transport:` is derived from the generic
`connection` block (the template knows its own transport schema). Used by `sbg-up`:

  render_params.py <config.yaml>          # -> the ROS 2 params YAML on stdout
  render_params.py --env <config.yaml>    # -> shell-quoted environment assignments
  render_params.py --env0 <config.yaml>   # -> literal NUL-delimited key/value pairs for sbg-up
  render_params.py --container <config.yaml> # -> params using the container replay path

The params doc is keyed by the `/**` wildcard node so it binds regardless of the namespace rig
pushes (do NOT key by node name). Needs PyYAML on the host (apt: python3-yaml).
"""
import re
import shlex
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.stderr.write("render_params: PyYAML required on the host (apt install python3-yaml)\n")
    sys.exit(2)


# Instance identity tokens end up in compose project names, ROS namespaces, staging file
# paths, and environment keys in the launcher — keep them to a safe, unambiguous alphabet.
_IDENT_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def require_ident(what: str, value: str) -> str:
    if not _IDENT_RE.fullmatch(value):
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


def main() -> int:
    args = sys.argv[1:]
    flags = set()
    while args and args[0].startswith("--"):
        flags.add(args.pop(0))
    if len(args) != 1 or flags - {"--env", "--env0", "--container"}:
        sys.stderr.write("usage: render_params.py [--env|--env0] [--container] <sensor-config.yaml>\n")
        return 2
    env_mode = bool(flags & {"--env", "--env0"})

    with open(args[0]) as handle:
        cfg = yaml.safe_load(handle) or {}

    service = require_ident("service", str(cfg.get("service") or "sbg"))
    name = require_ident("name", str(cfg.get("name") or "sbg"))
    connection = cfg.get("connection") or {}
    ttype = str(connection.get("type") or "file")
    namespace = require_ident("ros.namespace", str((cfg.get("ros") or {}).get("namespace") or name))

    # Validate transport before producing anything the launcher consumes.
    transport = derive_transport(connection)
    replay_host = None
    if ttype == "file":
        path = (connection.get("file") or {}).get("path")
        if not path:
            sys.stderr.write("render_params: file config needs connection.file.path\n")
            return 2
        replay_host = str((Path(args[0]).resolve().parent / str(path)).resolve())
        transport.setdefault("file", {})["path"] = (
            "/var/lib/sbg/replay.bin" if "--container" in flags else replay_host)

    if env_mode:
        values = {"SERVICE": service, "NAME": name, "NAMESPACE": f"/{namespace}", "TYPE": ttype}
        if ttype == "serial":
            values["DEVICE"] = str((connection.get("serial") or {}).get("by_id", ""))
        if replay_host is not None:
            values["REPLAY_HOST"] = replay_host
        if any("\0" in value for value in values.values()):
            sys.stderr.write("render_params: NUL bytes are not allowed in launcher values\n")
            return 2
        if "--env0" in flags:
            sys.stdout.buffer.write(b"".join(
                key.encode() + b"\0" + value.encode() + b"\0" for key, value in values.items()))
        else:
            print("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()))
        return 0

    # Isolate default TF frame IDs as well as ROS topic names. Explicit
    # mounting/vehicle frames remain authoritative (e.g. a shared base_link).
    merged: dict = {"transport": transport}
    for key, value in (cfg.get("driver_params") or {}).items():
        if key != "transport":
            merged[key] = value
    frames = merged.setdefault("frames", {})
    for key, suffix in {"imu": "imu_link", "gps": "gps_link", "base": "base_link", "odom": "odom"}.items():
        if f"frames.{key}" not in merged:
            frames.setdefault(key, f"{namespace}/{suffix}")
    doc = {"/**": {"ros__parameters": merged}}
    yaml.safe_dump(doc, sys.stdout, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
