"""Real Compose inventory with every daemon mutation intercepted; no hardware required."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
SERVICE = "sbg"
LAUNCHER = "sbg-up"
IMAGE = "sbg_driver"
OVERRIDE = "SBG_IMAGE"
PYTHON_ENV = "SBG_PYTHON"
COMPOSE_SERVICE = "driver"
EXAMPLE = "sensors/sbg.example.yaml"
DOCKER = shutil.which("docker")


@unittest.skipUnless(DOCKER, "Docker Compose is required for offline model rendering")
class RuntimeImages(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix=SERVICE + "_images_")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.repo = self.folder / "vendored"
        descriptor = yaml.safe_load((ROOT / "rigging.yaml").read_text())
        self.assertEqual(descriptor["verbs"]["runtime-images"], "runtime-images")
        self.assertEqual(descriptor["verbs"]["images"], "images")
        for rel in descriptor["launch_surface"]:
            target = self.repo / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / rel, target)
        self.config = self.folder / "config.yaml"
        self.config.write_text((ROOT / EXAMPLE).read_text())
        if SERVICE == "doodle":
            value = yaml.safe_load(self.config.read_text())
            value["connection"]["ubus"]["password"] = "Test-Only-Password-9"
            self.config.write_text(yaml.safe_dump(value))
        self.log = self.folder / "commands.jsonl"
        binary = self.folder / "docker"
        binary.write_text("#!" + sys.executable + "\n" + r'''
import json, os, subprocess, sys
args = sys.argv[1:]
entry = {"args": args}
if args[:1] == ["compose"] and "up" in args:
    entry["model"] = json.load(sys.stdin)
with open(os.environ["TEST_COMMANDS"], "a") as stream:
    stream.write(json.dumps(entry) + "\n")
if args[:2] == ["compose", "version"] or (args[:1] == ["compose"] and "config" in args):
    raise SystemExit(subprocess.run([os.environ["REAL_DOCKER"], *args]).returncode)
if "model" in entry or args[:2] == ["volume", "create"]:
    raise SystemExit(0)
if args[:1] == ["compose"] and "exec" in args:
    print('{"state": "active", "converged": true}')
    raise SystemExit(0)
raise SystemExit("unexpected daemon operation: " + str(args))
''')
        binary.chmod(0o755)
        self.env = {k: v for k, v in os.environ.items() if not k.startswith(
            ("RIG_", "COMPOSE_", "SBG_", "OUSTER_", "DOODLE_")) and k != "DOCKER_DEFAULT_PLATFORM"}
        self.env.update({PYTHON_ENV: sys.executable, "REAL_DOCKER": DOCKER,
                         "TEST_COMMANDS": str(self.log), "PATH": f"{self.folder}:{os.environ['PATH']}"})

    def launch(self, verb, *args, **env):
        return subprocess.run([str(self.repo / LAUNCHER), str(self.config), verb, *args],
                              env=dict(self.env, **env), capture_output=True, text=True, timeout=30)

    def commands(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def query(self, **env):
        result = self.launch("runtime-images", **env)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def pin_document(self, images):
        return json.dumps({"version": 1, "images": {ref: "sha256:" + f"{n:064x}"
                                                    for n, ref in enumerate(images, 1)}})

    def test_inventory_overrides_yaml_and_queries_are_read_only(self):
        config = self.config.read_bytes()
        selected = self.query(RIG_IMAGE_REGISTRY="localhost:5000", RIG_IMAGE_TAG="test")
        ref = "localhost:5000/" + IMAGE + ":test"
        self.assertEqual(selected, {"version": 1, "images": [ref], "roles": {IMAGE: ref}})
        custom = self.query(**{OVERRIDE: "localhost:5000/custom@sha256:" + "a" * 64})
        self.assertEqual(custom["images"], ["localhost:5000/custom@sha256:" + "a" * 64])
        result = self.launch("images", RIG_IMAGE_REGISTRY="localhost:5000", RIG_IMAGE_TAG="test")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(yaml.safe_load(result.stdout), selected["images"])
        self.assertFalse((self.repo / "var").exists())
        self.assertEqual(self.config.read_bytes(), config)
        self.assertTrue(all(call["args"][:1] == ["compose"] and
                            ("config" in call["args"] or "version" in call["args"])
                            for call in self.commands()))

    def test_image_queries_accept_unresolved_non_image_configuration(self):
        cases = [
            {"name": "front", "connection": {"type": "udp", "udp": {
                "remote_ip": "{{sbg_ip}}", "in_port": "{{sbg_udp_port}}"}}},
            {"name": "front", "connection": {"type": "serial", "serial": {
                "by_id": "/dev/serial/by-id/{{sbg_serial}}", "baud": "{{sbg_baud}}"}}},
            {"name": "{{sensor_name}}", "ros": {"namespace": "front_{{vehicle_id}}"},
             "connection": {"type": "{{transport}}"}},
            {"name": "front", "ros": {"namespace": "front_{{vehicle_id}}"},
             "connection": {"type": "file", "file": {"path": "{{capture_path}}"}}},
        ]
        ref = "localhost:5000/custom@sha256:" + "a" * 64
        for config in cases:
            self.config.write_text(yaml.safe_dump(config))
            original = self.config.read_bytes()
            for verb in ("images", "runtime-images"):
                with self.subTest(config=config, verb=verb):
                    result = self.launch(verb, **{OVERRIDE: ref}, VEHICLE_ID="{{vehicle_id}}",
                                         COMPOSE_PROJECT_NAME="{{project_name}}")
                    self.assertEqual(result.returncode, 0, result.stderr)
                    images = (json.loads(result.stdout)["images"] if verb == "runtime-images"
                              else yaml.safe_load(result.stdout))
                    self.assertEqual(images, [ref])
                    self.assertEqual(self.config.read_bytes(), original)
                    self.assertFalse((self.repo / "var").exists())
        self.assertTrue(all(call["args"][:1] == ["compose"] and
                            ("config" in call["args"] or "version" in call["args"])
                            for call in self.commands()))

    def test_startup_still_rejects_unresolved_configuration_before_writes(self):
        document = self.pin_document(self.query()["images"])
        cases = [
            {"name": "front", "connection": {"type": "udp", "udp": {
                "in_port": "{{sbg_udp_port}}"}}},
            {"name": "front", "connection": {"type": "serial", "serial": {
                "by_id": "/dev/sbg", "baud": "{{sbg_baud}}"}}},
            {"name": "{{sensor_name}}", "connection": {"type": "udp"}},
            {"name": "front", "ros": {"namespace": "front_{{vehicle_id}}"},
             "connection": {"type": "udp"}},
        ]
        for config in cases:
            self.config.write_text(yaml.safe_dump(config))
            for env in ({}, {"RIG_RUNTIME_IMAGE_PINS": document}):
                with self.subTest(config=config, pinned=bool(env)):
                    result = self.launch("up", "-d", **env)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertFalse((self.repo / "var").exists())
        self.assertTrue(all("up" not in call["args"] for call in self.commands()))

    def test_image_queries_validate_image_selectors_and_preserve_profiles(self):
        self.config.write_text(yaml.safe_dump({"name": "{{sensor_name}}"}))
        path = self.repo / "docker/compose/compose.deploy.yaml"
        model = yaml.safe_load(path.read_text())
        model["services"]["optional"] = {"image": "optional:runtime", "profiles": ["optional"]}
        path.write_text(yaml.safe_dump(model))
        for verb in ("images", "runtime-images"):
            for env in ({OVERRIDE: "{{runtime_image}}"}, {"RIG_IMAGE_TAG": "{{image_tag}}"},
                        {"RIG_IMAGE_REGISTRY": "{{registry}}"}):
                with self.subTest(verb=verb, env=env):
                    result = self.launch(verb, **env)
                    self.assertNotEqual(result.returncode, 0)
            for profile in ("", "optional"):
                with self.subTest(verb=verb, profile=profile):
                    result = self.launch(verb, **{OVERRIDE: "custom:runtime"},
                                         RIG_IMAGE_REGISTRY="{{registry}}",
                                         RIG_IMAGE_TAG="{{image_tag}}", COMPOSE_PROFILES=profile)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    images = (json.loads(result.stdout)["images"] if verb == "runtime-images"
                              else yaml.safe_load(result.stdout))
                    self.assertEqual(images, ["custom:runtime", "optional:runtime"] if profile
                                     else ["custom:runtime"])
        self.assertFalse((self.repo / "var").exists())

    def test_query_inventory_matches_every_resolved_transport(self):
        capture = self.folder / "recording.bin"
        capture.write_bytes(b"recorded packets")
        connections = [
            {"type": "udp", "udp": {"in_port": 5678}},
            {"type": "serial", "serial": {"by_id": "/dev/sbg", "baud": 921600}},
            {"type": "file", "file": {"path": str(capture)}},
        ]
        for connection in connections:
            with self.subTest(connection=connection):
                self.config.write_text(yaml.safe_dump({"name": "front", "connection": connection}))
                selected = self.query()
                result = self.launch("config", "--format", "json")
                self.assertEqual(result.returncode, 0, result.stderr)
                model = json.loads(result.stdout)
                self.assertEqual(selected["images"], sorted({value["image"] for value in
                                                           model["services"].values()}))

    def test_pins_freeze_ids_and_preserve_detached_readiness(self):
        selected = self.query(**{OVERRIDE: "custom:runtime"})
        document = self.pin_document(selected["images"])
        result = self.launch("up", "-d", **{OVERRIDE: "custom:runtime"},
                             RIG_RUNTIME_IMAGE_PINS=document, RIG_TARGET_ARCH="arm64")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(call for call in self.commands() if "model" in call)
        self.assertEqual(launch["args"], ["compose", "-f", "-", "up", "--no-build", "--pull", "never", "-d"])
        model = launch["model"]
        for value in model["services"].values():
            self.assertEqual(value["image"], json.loads(document)["images"]["custom:runtime"])
            self.assertEqual(value["pull_policy"], "never")
            self.assertEqual(value["platform"], "linux/arm64")
            self.assertNotIn("build", value)
        if SERVICE in ("ouster", "doodle"):
            self.assertTrue(any("exec" in call["args"] for call in self.commands()),
                            "pinned launch must still perform startup readiness checks")
        if SERVICE == "sbg":
            self.assertTrue(list((self.repo / "var/run").glob("*.params.yaml")))
        if SERVICE == "doodle":
            secret = next((self.repo / "var/run").glob("*.radio_password"))
            self.assertEqual(secret.stat().st_mode & 0o777, 0o600)
            self.assertNotIn("Test-Only-Password-9", json.dumps(launch))

    def test_invalid_pins_and_launch_overrides_fail_before_side_effects(self):
        selected = self.query()
        valid = self.pin_document(selected["images"])
        invalid = ["", "null", "{}", "invalid", json.dumps({"version": 1, "images": {}}),
                   json.dumps({"version": True, "images": json.loads(valid)["images"]}),
                   json.dumps({"version": 1, "images": {selected["images"][0]: "mutable:tag"}})]
        for document in invalid:
            with self.subTest(document=document):
                result = self.launch("up", "-d", RIG_RUNTIME_IMAGE_PINS=document)
                self.assertNotEqual(result.returncode, 0)
        result = self.launch("up", "-d", **{OVERRIDE: "changed:runtime"}, RIG_RUNTIME_IMAGE_PINS=valid)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("selection changed", result.stderr)
        for flags in [("--no-recreate",), ("--no-start",), (COMPOSE_SERVICE,), ("--scale", COMPOSE_SERVICE + "=0"),
                      ("--build",), ("--pull", "always")]:
            with self.subTest(flags=flags):
                result = self.launch("up", *flags, RIG_RUNTIME_IMAGE_PINS=valid)
                self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.repo / "var").exists())
        self.assertTrue(all("model" not in call and call["args"][:1] == ["compose"]
                            for call in self.commands()))

    def test_foreign_platform_fails_before_startup(self):
        result = self.launch("runtime-images", RIG_TARGET_ARCH="arm64", DOCKER_DEFAULT_PLATFORM="linux/amd64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("native linux/arm64", result.stderr)
        document = self.pin_document(self.query()["images"])
        result = self.launch("up", RIG_TARGET_ARCH="arm64", DOCKER_DEFAULT_PLATFORM="linux/amd64",
                             RIG_RUNTIME_IMAGE_PINS=document)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.repo / "var").exists())

    def test_transport_overlays_keep_the_recording_mount_read_only(self):
        capture = self.folder / "recording with spaces.bin"
        capture.write_bytes(b"recorded packets")
        self.config.write_text(yaml.safe_dump({"name": "replay_test", "connection": {
            "type": "file", "file": {"path": str(capture)}}}))
        selected = self.query()
        self.assertFalse((self.repo / "var").exists())
        result = self.launch("up", "-d", RIG_RUNTIME_IMAGE_PINS=self.pin_document(selected["images"]))
        self.assertEqual(result.returncode, 0, result.stderr)
        frozen = next(call["model"] for call in self.commands() if "model" in call)
        mount = next(value for value in frozen["services"]["driver"]["volumes"]
                     if value["target"] == "/var/lib/sbg/replay.bin")
        self.assertEqual(mount["source"], str(capture.resolve()))
        self.assertTrue(mount["read_only"])
        self.assertFalse(mount["bind"]["create_host_path"])
        self.assertEqual(capture.read_bytes(), b"recorded packets")

    def test_active_profiles_build_removal_and_literal_dollar_round_trip(self):
        path = self.repo / "docker/compose/compose.deploy.yaml"
        model = yaml.safe_load(path.read_text())
        driver = model["services"][COMPOSE_SERVICE]
        driver["build"] = {"context": "."}
        driver.setdefault("environment", {})["LITERAL_VALUE"] = "$$literal"
        driver["depends_on"] = {"optional": {"condition": "service_started", "required": False}}
        model["services"]["optional"] = {"image": "optional:runtime", "profiles": ["optional"]}
        path.write_text(yaml.safe_dump(model))
        selected = self.query()
        self.assertNotIn("optional:runtime", selected["images"])
        self.assertIn("optional:runtime", self.query(COMPOSE_PROFILES="optional")["images"])
        result = self.launch("up", RIG_RUNTIME_IMAGE_PINS=self.pin_document(selected["images"]))
        self.assertEqual(result.returncode, 0, result.stderr)
        frozen = next(call["model"] for call in self.commands() if "model" in call)
        self.assertEqual(set(frozen["services"]), {COMPOSE_SERVICE})
        self.assertNotIn("build", frozen["services"][COMPOSE_SERVICE])
        self.assertNotIn("depends_on", frozen["services"][COMPOSE_SERVICE])
        result = subprocess.run([DOCKER, "compose", "-f", "-", "config", "--format", "json"],
                                input=json.dumps(frozen), env=self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        again = json.loads(result.stdout)
        self.assertEqual(again["services"][COMPOSE_SERVICE]["environment"]["LITERAL_VALUE"], "$$literal")


if __name__ == "__main__":
    unittest.main()
