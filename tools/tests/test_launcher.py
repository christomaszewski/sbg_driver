#!/usr/bin/env python3
# Copyright 2026 sbg_driver maintainers
# SPDX-License-Identifier: Apache-2.0
"""Run the real launcher against a recording Docker stub; no daemon or devices needed."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.repo = self.root / "driver"
        self.repo.mkdir()
        for item in ("sbg-up", "tools/render_params.py", "docker/compose"):
            source = ROOT / item
            target = self.repo / item
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_dir():
                shutil.copytree(source, target)
            else:
                shutil.copy2(source, target)
        binaries = self.root / "bin"
        binaries.mkdir()
        docker = binaries / "docker"
        docker.write_text(f"#!{sys.executable}\n" + """
import json, os, sys
if sys.argv[1:] == ['compose', 'version']:
    sys.exit(0)
with open(os.environ['DOCKER_CAPTURE'], 'w') as output:
    json.dump({'args': sys.argv[1:], 'env': dict(os.environ)}, output)
""")
        docker.chmod(0o755)
        self.capture = self.root / "docker.json"
        self.env = dict(os.environ, PATH=f"{binaries}:{os.environ['PATH']}",
                        SBG_PYTHON=sys.executable, DOCKER_CAPTURE=str(self.capture))
        self.env.pop("COMPOSE_PROJECT_NAME", None)

    def config(self, connection, name="front", overrides=None):
        path = self.root / f"{name}.yaml"
        path.write_text(yaml.safe_dump({"name": name, "connection": connection,
                                      "driver_params": overrides or {}}))
        return path

    def launch(self, config):
        return subprocess.run(["bash", str(self.repo / "sbg-up"), str(config), "config"],
                              env=self.env, cwd=self.root, capture_output=True, text=True)

    def params(self, name="front"):
        return yaml.safe_load((self.repo / "var/run" / f"{name}.params.yaml").read_text())["/**"]["ros__parameters"]

    def test_shell_syntax_and_newlines_remain_literal(self):
        marker = self.root / "executed"
        device = f"/dev/test $(touch {marker}) `touch {marker}`;\n literal path"
        result = self.launch(self.config({"type": "serial", "serial": {"by_id": device}}))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(marker.exists())
        self.assertEqual(json.loads(self.capture.read_text())["env"]["SBG_DEVICE"], device)
        self.assertEqual(self.params()["transport"]["serial"]["port"], device)

    def test_invalid_transport_fails_before_compose_or_staging(self):
        marker = self.root / "executed"
        result = self.launch(self.config({"type": f"udp; touch {marker}"}))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(marker.exists())
        self.assertFalse(self.capture.exists())
        self.assertFalse((self.repo / "var").exists())

    def test_replay_mount_uses_host_path_and_params_use_container_path(self):
        capture = self.root / "capture with spaces.bin"
        capture.write_bytes(b"test")
        result = self.launch(self.config({"type": "file", "file": {"path": capture.name,
                                                                    "real_time_pace": False}}))
        self.assertEqual(result.returncode, 0, result.stderr)
        recorded = json.loads(self.capture.read_text())
        self.assertEqual(recorded["env"]["SBG_REPLAY_HOST"], str(capture))
        self.assertIn(str(self.repo / "docker/compose/compose.deploy.file.yaml"), recorded["args"])
        self.assertEqual(self.params()["transport"]["file"]["path"], "/var/lib/sbg/replay.bin")
        self.assertFalse(self.params()["transport"]["file"]["real_time_pace"])
        overlay = yaml.safe_load((self.repo / "docker/compose/compose.deploy.file.yaml").read_text())
        mount = overlay["services"]["driver"]["volumes"][0]
        self.assertTrue(mount["read_only"])
        self.assertFalse(mount["bind"]["create_host_path"])

    def test_namespace_tokens_cannot_contain_trailing_newlines(self):
        result = self.launch(self.config({"type": "udp"}, "front\n"))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.capture.exists())

    def test_missing_replay_path_is_rejected(self):
        result = self.launch(self.config({"type": "file"}))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.capture.exists())

    def test_instances_have_distinct_default_frames_and_topics(self):
        frames = []
        for name in ("front", "rear"):
            result = self.launch(self.config({"type": "udp"}, name))
            self.assertEqual(result.returncode, 0, result.stderr)
            recorded = json.loads(self.capture.read_text())
            self.assertEqual(recorded["env"]["SBG_NAMESPACE"], f"/{name}")
            frames.append(self.params(name)["frames"])
        for key in ("imu", "gps", "base", "odom"):
            self.assertNotEqual(frames[0][key], frames[1][key])

    def test_explicit_frames_and_inherited_compose_project_are_preserved(self):
        self.env["COMPOSE_PROJECT_NAME"] = "vehicle-a"
        result = self.launch(self.config({"type": "udp"}, overrides={
            "frames": {"base": "base_link"}, "frames.imu": "mounted_imu"}))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.params()["frames"]["base"], "base_link")
        self.assertNotIn("imu", self.params()["frames"])
        self.assertEqual(self.params()["frames.imu"], "mounted_imu")
        self.assertEqual(json.loads(self.capture.read_text())["env"]["COMPOSE_PROJECT_NAME"], "vehicle-a")

    def test_release_versions_and_message_pin_match(self):
        versions = {ET.parse(ROOT / "src" / package / "package.xml").findtext("version")
                    for package in ("sbg_driver", "sbg_driver_core", "sbg_msgs")}
        self.assertEqual(len(versions), 1)
        version = versions.pop()
        manifest = yaml.safe_load((ROOT / "rigging.yaml").read_text())
        self.assertEqual(manifest["msgs"]["source"][0]["ref"], f"v{version}")
        for package in ("sbg_driver", "sbg_driver_core", "sbg_msgs"):
            self.assertIn(f"VERSION {version}", (ROOT / "src" / package / "CMakeLists.txt").read_text())
        for header in ("sbg_driver/include/sbg_driver/version.hpp",
                       "sbg_driver_core/include/sbg/version.hpp"):
            self.assertIn(f'"{version}"', (ROOT / "src" / header).read_text())
        self.assertIn("docker/compose/compose.deploy.file.yaml", manifest["launch_surface"])


if __name__ == "__main__":
    unittest.main()
