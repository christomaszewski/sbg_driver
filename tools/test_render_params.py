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
"""Launcher-side health wiring: fleet identity and the generic `health:` block -> ROS params.

    python3 -m unittest discover -s tools -p 'test_*.py'
"""
import contextlib
import io
import os
import pathlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import render_params  # noqa: E402


def render(config: dict, env: dict | None = None, *args: str) -> subprocess.CompletedProcess:
    """Run the script the way sbg-up does, against a temporary config."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
        yaml.safe_dump(config, handle)
    try:
        environ = {k: v for k, v in os.environ.items() if k != "VEHICLE_ID"}
        environ.update(env or {})
        return subprocess.run([sys.executable, str(TOOLS / "render_params.py"), *args, handle.name],
                              capture_output=True, text=True, env=environ, check=False)
    finally:
        os.unlink(handle.name)


def params(config: dict, env: dict | None = None) -> dict:
    result = render(config, env)
    assert result.returncode == 0, result.stderr
    return yaml.safe_load(result.stdout)["/**"]["ros__parameters"]


BASE = {"service": "sbg", "name": "ins", "connection": {"type": "udp"},
        "ros": {"namespace": "front"}}


class Identity(unittest.TestCase):
    def test_instance_is_the_config_name_not_the_ros_namespace(self):
        health = params(BASE, {"VEHICLE_ID": "uav6"})["health"]
        self.assertEqual(health, {"instance": "ins", "vehicle_id": "uav6"})

    def test_vehicle_is_omitted_when_the_environment_has_none(self):
        self.assertEqual(params(BASE)["health"], {"instance": "ins"})
        self.assertEqual(params(BASE, {"VEHICLE_ID": "  "})["health"], {"instance": "ins"})

    def test_numeric_vehicle_id_stays_a_string_parameter(self):
        result = render(BASE, {"VEHICLE_ID": "42"})
        self.assertIn("vehicle_id: '42'", result.stdout)
        self.assertEqual(yaml.safe_load(result.stdout)["/**"]["ros__parameters"]["health"]
                         ["vehicle_id"], "42")

    def test_vehicle_id_must_be_one_path_segment(self):
        for bad in ("a/b", "uav*", "uav 6", "uav#6", "u?v"):
            result = render(BASE, {"VEHICLE_ID": bad})
            self.assertEqual(result.returncode, 2, bad)
            self.assertIn("VEHICLE_ID", result.stderr)

    def test_env_mode_is_unchanged(self):
        result = render(BASE, {"VEHICLE_ID": "uav6"}, "--env")
        self.assertEqual(result.stdout.split(),
                         ["SERVICE=sbg", "NAME=ins", "NAMESPACE=/front", "TYPE=udp"])


class HealthBlock(unittest.TestCase):
    def test_maps_the_generic_block_onto_driver_params(self):
        config = dict(BASE, health={
            "interval_s": 1, "stale_after_s": 10, "stream_timeout_s": 3,
            "required_gnss_fix": "rtk_fixed", "ignored_alerts": ["DATALOGGER"],
            "limits": {"temp.imu_c": {"warn_above": 70, "error_above": 85.5, "error_below": -40}}})
        health = params(config, {"VEHICLE_ID": "uav6"})["health"]
        self.assertEqual(health, {
            "instance": "ins", "vehicle_id": "uav6",
            "publish_interval_s": 1.0, "stale_after_s": 10.0, "stream_timeout_s": 3.0,
            "required_gnss_fix": "rtk_fixed", "ignored_alerts": ["DATALOGGER"],
            "limits": {"temp_imu_c": {"warn_above": 70.0, "error_above": 85.5,
                                      "error_below": -40.0}}})

    def test_integers_are_rendered_as_doubles(self):
        # ROS refuses an integer for a double parameter; `interval_s: 1` must still work.
        result = render(dict(BASE, health={"interval_s": 1, "limits": {
            "temp.imu_c": {"warn_above": 0}}}))
        self.assertIn("publish_interval_s: 1.0", result.stdout)
        self.assertIn("warn_above: 0.0", result.stdout)  # and a zero bound is a bound

    def test_empty_or_absent_block_renders_identity_only(self):
        self.assertEqual(params(dict(BASE, health={}))["health"], {"instance": "ins"})
        self.assertEqual(params(dict(BASE, health=None))["health"], {"instance": "ins"})
        self.assertEqual(params(dict(BASE, health={"limits": {}}))["health"], {"instance": "ins"})

    def assert_rejected(self, health, needle: str):
        result = render(dict(BASE, health=health))
        self.assertEqual(result.returncode, 2, f"{health!r} was accepted: {result.stdout}")
        self.assertIn(needle, result.stderr)
        self.assertEqual(result.stdout, "")  # nothing half-rendered

    def test_rejects_invalid_timing(self):
        for key in ("interval_s", "stale_after_s", "stream_timeout_s"):
            for bad in (0, -1, True, "1", float("nan"), float("inf")):
                self.assert_rejected({key: bad}, f"health.{key}")
        self.assert_rejected({"interval_s": 3}, "at least twice")           # default stale = 5
        self.assert_rejected({"interval_s": 2, "stale_after_s": 3}, "at least twice")
        self.assertEqual(render(dict(BASE, health={"interval_s": 2.5})).returncode, 0)

    def test_rejects_unknown_and_inapplicable_settings(self):
        self.assert_rejected({"interval": 1}, "unknown health setting")
        self.assert_rejected({"poll_interval_s": 5}, "polls nothing")
        self.assert_rejected([1, 2], "health must be a mapping")

    def test_rejects_invalid_limits(self):
        self.assert_rejected({"limits": {"supply.voltage_v": {"warn_below": 10}}},
                             "measures only")
        self.assert_rejected({"limits": {"temp.imu_c": {"warn": 70}}}, "must map any of")
        self.assert_rejected({"limits": {"temp.imu_c": 70}}, "must map any of")
        self.assert_rejected({"limits": {"temp.imu_c": {"warn_above": float("inf")}}},
                             "finite number")
        self.assert_rejected({"limits": {"temp.imu_c": {"warn_above": True}}}, "finite number")
        self.assert_rejected({"limits": [70]}, "health.limits must be a mapping")

    def test_rejects_invalid_gnss_requirement_and_alert_codes(self):
        self.assert_rejected({"required_gnss_fix": "rtk"}, "required_gnss_fix")
        self.assert_rejected({"ignored_alerts": ["NOPE"]}, "ignored_alerts")
        self.assert_rejected({"ignored_alerts": ["CPU", "CPU"]}, "ignored_alerts")
        self.assert_rejected({"ignored_alerts": "CPU"}, "ignored_alerts")


class DriverParamsOverride(unittest.TestCase):
    def test_explicit_health_params_win_without_dropping_identity(self):
        config = dict(BASE, health={"stale_after_s": 10, "limits": {
            "temp.imu_c": {"warn_above": 70, "error_above": 85}}},
            driver_params={"health": {"stale_after_s": 8.0, "instance": "ins_b",
                                      "limits": {"temp_imu_c": {"warn_above": 65.0}}},
                           "convention": {"use_enu": True}})
        rendered = params(config, {"VEHICLE_ID": "uav6"})
        self.assertEqual(rendered["health"], {
            "instance": "ins_b", "vehicle_id": "uav6", "stale_after_s": 8.0,
            "limits": {"temp_imu_c": {"warn_above": 65.0, "error_above": 85.0}}})
        self.assertEqual(rendered["convention"], {"use_enu": True})
        self.assertEqual(rendered["transport"], {"type": "udp"})

    def test_transport_still_cannot_be_overridden(self):
        rendered = params(dict(BASE, driver_params={"transport": {"type": "file"}}))
        self.assertEqual(rendered["transport"], {"type": "udp"})


class DeriveHealth(unittest.TestCase):
    """The function itself: failures exit before anything is printed."""

    def test_exits_with_a_message(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
            render_params.derive_health({"health": {"interval_s": -1}}, "ins", {})
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("health.interval_s", stderr.getvalue())

    def test_deep_merge_does_not_mutate_its_inputs(self):
        base = {"limits": {"temp_imu_c": {"warn_above": 70.0}}}
        merged = render_params.deep_merge(base, {"limits": {"temp_imu_c": {"error_above": 85.0}}})
        self.assertEqual(base, {"limits": {"temp_imu_c": {"warn_above": 70.0}}})
        self.assertEqual(merged["limits"]["temp_imu_c"], {"warn_above": 70.0, "error_above": 85.0})



class LauncherShellSafety(unittest.TestCase):
    def run_launcher(self, root, config):
        (root / "tools").mkdir(exist_ok=True)
        (root / "bin").mkdir(exist_ok=True)
        shutil.copy2(TOOLS.parent / "sbg-up", root / "sbg-up")
        shutil.copy2(TOOLS / "render_params.py", root / "tools" / "render_params.py")
        docker = root / "bin" / "docker"
        docker.write_text(
            f"#!{sys.executable}\nimport json, os, sys\n"
            "if sys.argv[1:] != ['compose', 'version']:\n"
            "    print(json.dumps(dict(os.environ)))\n")
        docker.chmod(0o755)
        config_file = root / "sensor.yaml"
        config_file.write_text(yaml.safe_dump(config))
        environ = dict(os.environ, PATH=str(root / "bin") + os.pathsep + os.environ["PATH"],
                       SBG_PYTHON=sys.executable)
        environ.pop("VEHICLE_ID", None)
        return subprocess.run([str(root / "sbg-up"), str(config_file), "config"],
                              env=environ, text=True, capture_output=True)

    def test_serial_path_round_trips_without_executing_shell_syntax(self):
        with tempfile.TemporaryDirectory(prefix="sbg launcher ") as directory:
            root = pathlib.Path(directory)
            marker = root / "must_not_exist"
            device = f"/dev/serial/by-id/$(touch '{marker}')`touch '{marker}'`; quote'\"\nspaces $HOME"
            result = self.run_launcher(root, dict(BASE, connection={
                "type": "serial", "serial": {"by_id": device}}))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(marker.exists(), "sensor YAML executed a shell command")
            self.assertEqual(json.loads(result.stdout)["SBG_DEVICE"], device)
            staged = yaml.safe_load((root / "var/run/ins.params.yaml").read_text())
            self.assertEqual(staged["/**"]["ros__parameters"]["transport"]["serial"]["port"], device)

    def test_bad_transport_is_rejected_before_shell_evaluation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            marker = root / "must_not_exist"
            result = self.run_launcher(root, dict(BASE, connection={
                "type": f"udp;touch {marker}"}))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("unsupported connection.type", result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertFalse(marker.exists())
            self.assertFalse((root / "var/run/ins.params.yaml").exists())

    def test_missing_serial_path_stops_launcher_before_deployment(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_launcher(pathlib.Path(directory), dict(BASE, connection={"type": "serial"}))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("by_id", result.stderr)
            self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
