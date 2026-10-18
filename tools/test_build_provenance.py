#!/usr/bin/env python3
# Copyright 2026 sbg_driver maintainers
# SPDX-License-Identifier: Apache-2.0
"""Exercise real Git checkouts and the build/provenance scripts without a Docker daemon."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE_REPO = "https://github.com/christomaszewski/sbg_driver.git"
TAG = "v1.2.3"


class BuildProvenanceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.repo = self.root / "source with spaces"
        self.repo.mkdir()
        for item in ("tools/build_image.sh", "tools/rig-build.sh",
                     "docker/provenance-record.sh", ".gitignore"):
            target = self.repo / item
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / item, target)
        self.source = self.repo / "message.msg"
        self.source.write_text("float64 value\n")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        docker = self.bin / "docker"
        docker.write_text(f"#!{sys.executable}\n" + """
import json, os, sys
with open(os.environ['DOCKER_CAPTURE'], 'a') as output:
    output.write(json.dumps(sys.argv[1:]) + '\\n')
""")
        docker.chmod(0o755)
        self.capture = self.root / "docker.jsonl"
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith(("GIT_", "SBG_", "RIG_"))
                    and k not in ("ROS_DISTRO", "PROVENANCE_REV", "PUSH")}
        self.env.update(PATH=f"{self.bin}:{os.environ['PATH']}",
                        DOCKER_CAPTURE=str(self.capture), GIT_CONFIG_NOSYSTEM="1",
                        GIT_CONFIG_GLOBAL=os.devnull, GIT_TERMINAL_PROMPT="0")
        self.git("init", "--quiet")
        self.git("config", "user.name", "Provenance Test")
        self.git("config", "user.email", "provenance@example.test")
        self.git("add", ".")
        self.git("commit", "--quiet", "-m", "fixture")
        self.sha = self.git("rev-parse", "HEAD")
        self.git("tag", TAG)

    def git(self, *args, repo=None):
        return subprocess.run(["git", "-C", str(repo or self.repo), *args], env=self.env,
                              capture_output=True, text=True, check=True).stdout.strip()

    def build_args(self, repo=None, env=None):
        repo = repo or self.repo
        self.capture.unlink(missing_ok=True)
        result = subprocess.run(["bash", str(repo / "tools/build_image.sh"),
                                 "reg.example:5000", "deployment-tag"], cwd=self.root,
                                env=dict(self.env, **(env or {})),
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(line) for line in self.capture.read_text().splitlines()]
        build = calls[0]
        if (env or {}).get("RIG_BUILD_PLATFORMS"):
            self.assertEqual(len(calls), 1)
            self.assertEqual(build[:2], ["buildx", "build"])
            self.assertIn("--push", build)
        else:
            self.assertEqual(len(calls), 2)
            self.assertEqual(build[0], "build")
            self.assertEqual(calls[1], ["push", "reg.example:5000/sbg_driver:deployment-tag"])
        self.assertEqual(build[-1], str(repo))
        return dict(build[i + 1].split("=", 1) for i, arg in enumerate(build)
                    if arg == "--build-arg")

    def record(self, ref, checkout, revision=None):
        output = self.root / "provenance.yaml"
        output.unlink(missing_ok=True)
        env = dict(self.env)
        if revision is not None:
            env["PROVENANCE_REV"] = revision
        result = subprocess.run(["sh", str(ROOT / "docker/provenance-record.sh"),
                                 str(output), SOURCE_REPO, ref, str(checkout), "sbg_msgs"],
                                env=env, cwd=self.root, capture_output=True, text=True)
        return result, output

    def assert_record(self, ref, revision, checkout, override=None):
        result, output = self.record(ref, checkout, override)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output.read_text(),
                         f"version: 1\nsource:\n  - repo: {SOURCE_REPO}\n"
                         f"    ref: {ref}\n    rev: {revision}\n    packages: [sbg_msgs]\n")

    def assert_build_identity(self, ref, revision, repo=None, env=None):
        args = self.build_args(repo, env)
        self.assertEqual(args["SBG_SRC_REF"], ref)
        self.assertEqual(args["SBG_SRC_REV"], revision)
        # Feed the actual build arguments to the recorder in a context without .git.
        snapshot = self.root / "snapshot"
        snapshot.mkdir(exist_ok=True)
        self.assert_record(ref, revision, snapshot, args["SBG_SRC_REV"])

    def test_clean_tagged_build_carries_commit_not_deployment_tag_or_environment(self):
        self.assert_build_identity(TAG, self.sha, env={"SBG_SRC_REF": "fake-tag",
                                                     "SBG_SRC_REV": "f" * 40})

    def test_multiarch_build_preserves_clean_source_identity(self):
        self.assert_build_identity(TAG, self.sha,
                                   env={"RIG_BUILD_PLATFORMS": "linux/arm64,linux/amd64"})

    def test_multiarch_build_does_not_verify_dirty_content_as_head(self):
        self.source.write_text("float32 value\n")
        self.assert_build_identity(TAG + "-dirty", "unknown",
                                   env={"RIG_BUILD_PLATFORMS": "linux/arm64,linux/amd64"})

    def test_annotated_tag_resolves_to_commit_not_tag_object(self):
        self.git("tag", "--delete", TAG)
        self.git("tag", "-a", TAG, "-m", "release")
        self.assertNotEqual(self.git("rev-parse", TAG), self.sha)
        self.assert_build_identity(TAG, self.sha)

    def test_untagged_detached_checkout_uses_sha_for_both_fields(self):
        self.git("commit", "--allow-empty", "--quiet", "-m", "next")
        self.git("checkout", "--detach", "--quiet")
        sha = self.git("rev-parse", "HEAD")
        self.assert_build_identity(sha, sha)

    def test_unstaged_changes_are_not_verified_as_head(self):
        self.source.write_text("float32 value\n")
        self.assert_build_identity(TAG + "-dirty", "unknown")

    def test_staged_changes_are_not_verified_as_head(self):
        self.source.write_text("float32 value\n")
        self.git("add", "message.msg")
        self.assert_build_identity(TAG + "-dirty", "unknown")

    def test_deleted_files_are_dirty(self):
        self.source.unlink()
        self.assert_build_identity(TAG + "-dirty", "unknown")

    def test_untracked_files_are_dirty_even_when_git_config_hides_them(self):
        self.git("config", "status.showUntrackedFiles", "no")
        (self.repo / "new.msg").write_text("string added\n")
        self.assert_build_identity(TAG + "-dirty", "unknown")

    def test_untagged_dirty_checkout_keeps_sha_in_ref_only(self):
        self.git("tag", "--delete", TAG)
        self.source.write_text("float32 value\n")
        self.assert_build_identity(self.sha + "-dirty", "unknown")

    def test_ignored_build_output_does_not_mark_checkout_dirty(self):
        (self.repo / "build").mkdir()
        (self.repo / "build/generated.txt").write_text("ignored output\n")
        self.assert_build_identity(TAG, self.sha)

    def test_git_worktree_file_is_supported(self):
        worktree = self.root / "worktree with spaces"
        self.git("worktree", "add", "--detach", str(worktree), "HEAD")
        self.assertTrue((worktree / ".git").is_file())
        self.assert_build_identity(TAG, self.sha, repo=worktree)

    def test_non_git_export_does_not_borrow_parent_identity(self):
        shutil.rmtree(self.repo / ".git")
        self.git("init", "--quiet", repo=self.root)
        self.git("-c", "user.name=Parent", "-c", "user.email=parent@example.test",
                 "commit", "--allow-empty", "--quiet", "-m", "parent", repo=self.root)
        self.git("tag", TAG, repo=self.root)
        self.assert_build_identity("unknown", "unknown")
        self.assert_record("unknown", "unknown", self.repo)

    def test_git_repository_without_commits_is_unknown(self):
        shutil.rmtree(self.repo / ".git")
        self.git("init", "--quiet")
        self.assert_build_identity("unknown", "unknown")

    def test_build_without_git_is_unknown(self):
        git = self.bin / "git"
        git.write_text("#!/bin/sh\nexit 127\n")
        git.chmod(0o755)
        self.assert_build_identity("unknown", "unknown")

    def test_failed_status_check_cannot_claim_clean_revision(self):
        git = self.bin / "git"
        git.write_text('#!/bin/sh\nif [ "$3" = status ]; then exit 1; fi\n'
                       f'exec {shlex.quote(shutil.which("git"))} "$@"\n')
        git.chmod(0o755)
        self.assert_build_identity(TAG + "-dirty", "unknown")
        self.assert_record(TAG, "unknown", self.repo)

    def test_recorder_can_still_derive_identity_from_clean_checkout(self):
        self.assert_record(TAG, self.sha, self.repo)

    def test_recorder_fallback_does_not_claim_dirty_checkout_is_head(self):
        self.source.write_text("float32 value\n")
        self.assert_record(TAG + "-dirty", "unknown", self.repo)

    def test_explicit_unknown_is_not_replaced_with_available_head(self):
        self.assert_record(TAG, "unknown", self.repo, "unknown")

    def test_recorder_without_git_or_override_stays_unknown(self):
        snapshot = self.root / "export"
        snapshot.mkdir()
        self.assert_record("unknown", "unknown", snapshot)

    def test_invalid_revision_override_fails_without_writing_provenance(self):
        for revision in ("", self.sha[:12], self.sha + "-dirty", "x" * 40, "bad\nsource: []"):
            with self.subTest(revision=revision):
                result, output = self.record(TAG, self.repo, revision)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("full commit SHA or unknown", result.stderr)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
