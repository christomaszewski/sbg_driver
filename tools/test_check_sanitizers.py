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
import json
from pathlib import Path
import tempfile
import unittest

from check_sanitizers import check


class SanitizerCoverage(unittest.TestCase):
    def fixture(self, root, driver_flags, include_test=True):
        for package in ("sbg_driver", "sbg_driver_core"):
            entries = []
            flags = driver_flags if package == "sbg_driver" else [
                "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
            for folder in ("src", "test"):
                source = root / "src" / package / folder / "example.cpp"
                source.parent.mkdir(parents=True)
                source.touch()
                if folder != "test" or include_test:
                    entries.append({"file": str(source), "directory": str(root),
                                    "arguments": ["c++", *flags, "-c", str(source)]})
            build = root / "build" / package
            build.mkdir(parents=True)
            (build / "compile_commands.json").write_text(json.dumps(entries))

    def test_accepts_instrumented_driver_and_tests(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"])
            self.assertEqual(check(root / "build", root / "src"), [])

    def test_rejects_the_previous_core_only_instrumentation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, [])
            errors = check(root / "build", root / "src")
            self.assertEqual(len(errors), 2)
            self.assertTrue(all("not ASan/UBSan instrumented" in e for e in errors))

    def test_missing_tests_cannot_pass_the_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"], False)
            self.assertTrue(any("was not compiled" in e for e in check(root / "build", root / "src")))

    def test_rejects_disabled_sanitizer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, ["-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                                "-fno-sanitize=undefined"])
            self.assertTrue(check(root / "build", root / "src"))


if __name__ == "__main__":
    unittest.main()
