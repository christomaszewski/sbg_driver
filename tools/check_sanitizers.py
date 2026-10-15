#!/usr/bin/env python3
# Copyright 2026 sbg_driver maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Fail CI if ASan/UBSan only reached the linker, or missed source/test files."""
import argparse
import json
import pathlib
import shlex


def check(build_dir: pathlib.Path, source_dir: pathlib.Path) -> list[str]:
    errors = []
    for package in ("sbg_driver_core", "sbg_driver"):
        database = build_dir / package / "compile_commands.json"
        if not database.is_file():
            errors.append(f"missing {database}")
            continue
        entries = json.loads(database.read_text())
        seen = set()
        for entry in entries:
            path = (pathlib.Path(entry["directory"]) / entry["file"]).resolve()
            seen.add(path)
            args = entry.get("arguments") or shlex.split(entry["command"])
            enabled = set()
            for arg in args:
                if arg.startswith("-fsanitize="):
                    enabled.update(arg.split("=", 1)[1].split(","))
                elif arg.startswith("-fno-sanitize="):
                    enabled.difference_update(arg.split("=", 1)[1].split(","))
            if not {"address", "undefined"} <= enabled or "-fno-omit-frame-pointer" not in args:
                errors.append(f"not ASan/UBSan instrumented: {path}")
        if not entries:
            errors.append(f"empty compilation database: {database}")
        for folder in ("src", "test", "tools"):
            for source in (source_dir / package / folder).glob("*.cpp"):
                if source.resolve() not in seen:
                    errors.append(f"source/test was not compiled: {source}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=pathlib.Path)
    parser.add_argument("--source-dir", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parent.parent / "src")
    args = parser.parse_args()
    errors = check(args.build_dir, args.source_dir)
    if errors:
        print("\n".join(errors))
        return 1
    print("ASan/UBSan verified for all compiled core/SDK/ROS driver sources and C++ tests")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
