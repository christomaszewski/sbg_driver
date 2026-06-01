#!/usr/bin/env python3
# Copyright 2026 Chris Tomaszewski
# Licensed under the Apache License, Version 2.0 (the "License"); see LICENSE.
"""Render a generic *rig* sensor config into this driver's ROS 2 params file.

The sensor config is the single source of truth (uniform across all rig services). `driver_params`
is OPAQUE -- copied verbatim into ros__parameters -- while `transport:` is DERIVED from the generic
`connection` block (the template knows its own transport schema). Used by `sbg-up`:

  render_params.py <config.yaml>          # -> the ROS 2 params YAML on stdout
  render_params.py --env <config.yaml>    # -> SERVICE/NAME/NAMESPACE/TYPE/DEVICE env lines

The params doc is keyed by the `/**` wildcard node so it binds regardless of the namespace rig
pushes (do NOT key by node name). Needs PyYAML on the host (apt: python3-yaml).
"""
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.stderr.write("render_params: PyYAML required on the host (apt install python3-yaml)\n")
    sys.exit(2)


def derive_transport(connection: dict) -> dict:
    """Map the generic `connection` block onto this driver's `transport:` param schema."""
    ttype = str(connection.get("type") or "file")
    block = connection.get(ttype) or {}
    if ttype == "serial":
        # serial.by_id is used twice: here as the in-container serial port, and by the launcher as
        # the host --device to pass through.
        return {"type": "serial",
                "serial": {"port": str(block.get("by_id", "")),
                           "baud": int(block.get("baud", 115200))}}
    if ttype == "tcp":
        return {"type": "tcp",
                "tcp": {"host": str(block.get("host", "")),
                        "port": int(block.get("port", 0))}}
    if ttype == "udp":
        return {"type": "udp",
                "udp": {"remote_ip": str(block.get("remote_ip", "")),
                        "in_port": int(block.get("in_port", 0)),
                        "out_port": int(block.get("out_port", 0))}}
    return {"type": "file",
            "file": {"path": str(block.get("path", "")),
                     "real_time_pace": bool(block.get("real_time_pace", False))}}


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

    service = str(cfg.get("service") or "sbg")
    name = str(cfg.get("name") or "sbg")
    connection = cfg.get("connection") or {}
    ttype = str(connection.get("type") or "file")
    namespace = str((cfg.get("ros") or {}).get("namespace") or name)

    if env_mode:
        lines = [f"SERVICE={service}", f"NAME={name}", f"NAMESPACE=/{namespace}", f"TYPE={ttype}"]
        if ttype == "serial":
            lines.append(f"DEVICE={(connection.get('serial') or {}).get('by_id', '')}")
        print("\n".join(lines))
        return 0

    # params mode: transport (derived) first, then driver_params verbatim (minus any transport key).
    merged: dict = {"transport": derive_transport(connection)}
    for key, value in (cfg.get("driver_params") or {}).items():
        if key != "transport":
            merged[key] = value
    doc = {"/**": {"ros__parameters": merged}}
    yaml.safe_dump(doc, sys.stdout, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
