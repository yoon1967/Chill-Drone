"""Integration checks with real, disposable Git repositories; no network or secrets."""

import importlib.util
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "github_mirror", Path(__file__).resolve().parents[1] / "scripts/github_mirror.py"
)
MIRROR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MIRROR)


@unittest.skipUnless(shutil.which("git"), "Git is required for mirror integration checks")
class GithubMirrorTests(unittest.TestCase):
    def command(self, repo, *args):
        return subprocess.check_output(
            ["git", "-C", str(repo), *args], stderr=subprocess.PIPE, text=True
        ).strip()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.destination = self.root / "destination.git"
        self.worker = self.root / "worker"
        self.source.mkdir()
        self.command(self.source, "init", "-b", "develop")
        self.command(self.source, "config", "user.name", "Mirror Test")
        self.command(self.source, "config", "user.email", "mirror@example.invalid")
        self.commit(self.source, "initial source")
        self.command(self.source, "branch", "master")
        self.command(self.source, "branch", "private-feature")
        self.command(self.source, "tag", "private-tag")
        self.command(self.root, "init", "--bare", str(self.destination))
        self.command(self.root, "clone", str(self.source), str(self.worker))

    def commit(self, repo, message):
        self.command(repo, "commit", "--allow-empty", "-m", message)
        return self.command(repo, "rev-parse", "HEAD")

    def refs(self):
        lines = self.command(self.destination, "for-each-ref", "--format=%(refname) %(objectname)")
        return dict(line.split() for line in lines.splitlines())

    def synchronize(self):
        return MIRROR.mirror(self.worker, str(self.destination))

    def test_copies_exact_branches_and_preserves_existing_main(self):
        other = self.root / "other"
        other.mkdir()
        self.command(other, "init", "-b", "main")
        self.command(other, "config", "user.name", "Mirror Test")
        self.command(other, "config", "user.email", "mirror@example.invalid")
        main_sha = self.commit(other, "existing GitHub README")
        self.command(other, "push", str(self.destination), "main")
        expected = self.synchronize()
        self.assertEqual(
            self.refs(),
            {"refs/heads/main": main_sha, **{f"refs/heads/{b}": sha for b, sha in expected.items()}},
        )

    def test_fetches_new_source_tips_and_repeat_is_idempotent(self):
        self.synchronize()
        develop_sha = self.commit(self.source, "later source update")
        self.command(self.source, "switch", "master")
        master_sha = self.commit(self.source, "later master update")
        self.assertEqual(self.synchronize(), {"develop": develop_sha, "master": master_sha})
        self.assertEqual(self.synchronize(), {"develop": develop_sha, "master": master_sha})

    def test_divergence_preserves_both_destination_branches_atomically(self):
        self.synchronize()
        other = self.root / "other"
        self.command(self.root, "clone", "-b", "develop", str(self.destination), str(other))
        self.command(other, "config", "user.name", "Mirror Test")
        self.command(other, "config", "user.email", "mirror@example.invalid")
        self.commit(other, "GitHub only change")
        self.command(other, "push", "origin", "develop")
        self.commit(self.source, "conflicting source develop")
        self.command(self.source, "switch", "master")
        self.commit(self.source, "source master update")
        before = self.refs()
        with self.assertRaisesRegex(RuntimeError, "git push failed"):
            self.synchronize()
        self.assertEqual(self.refs(), before)

    def test_missing_source_branch_stops_before_any_push(self):
        self.synchronize()
        before = self.refs()
        self.command(self.source, "branch", "-D", "master")
        with self.assertRaisesRegex(RuntimeError, "git fetch failed"):
            self.synchronize()
        self.assertEqual(self.refs(), before)


if __name__ == "__main__":
    unittest.main()
