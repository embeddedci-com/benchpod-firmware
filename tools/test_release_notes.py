"""Tests for release_notes.py: python3 -m unittest tools/test_release_notes.py"""

import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import release_notes  # noqa: E402


def run(*args, cwd):
    subprocess.run(args, cwd=cwd, check=True, capture_output=True)


class ReleaseNotesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        self.old = os.getcwd()
        os.chdir(self.dir)
        env = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
               "GIT_COMMITTER_EMAIL": "t@t"}
        os.environ.update(env)
        run("git", "init", "-q", "-b", "main", cwd=self.dir)
        self.write("3.0.0", 40)
        run("git", "add", "-A", cwd=self.dir)
        run("git", "commit", "-qm", "base", cwd=self.dir)
        run("git", "tag", "stm32-v3.0.0", cwd=self.dir)
        self.merge_pr(1, "feature-a", "Add feature A")
        self.squash(2, "Fix bug B")
        self.write("3.1.0", 41)
        run("git", "commit", "-qam", "Version 3.1.0", cwd=self.dir)  # not a PR: left out

    def tearDown(self):
        os.chdir(self.old)
        self.tmp.cleanup()

    def write(self, fw, gw):
        os.makedirs(os.path.join(self.dir, "stm32h563", "src"), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "ice40", "release"), exist_ok=True)
        with open(os.path.join(self.dir, "stm32h563", "src", "version.h"), "w") as f:
            f.write(f'#define FIRMWARE_VERSION "{fw}"\n')
        with open(os.path.join(self.dir, "ice40", "release", "MANIFEST"), "w") as f:
            f.write(f"gateware_version={gw}\n")

    def merge_pr(self, number, branch, title):
        run("git", "checkout", "-qb", branch, cwd=self.dir)
        with open(os.path.join(self.dir, branch), "w") as f:
            f.write(branch)
        run("git", "add", "-A", cwd=self.dir)
        run("git", "commit", "-qm", "work", cwd=self.dir)
        run("git", "checkout", "-q", "main", cwd=self.dir)
        run("git", "merge", "-q", "--no-ff", branch, "-m",
            f"Merge pull request #{number} from embeddedci-com/{branch}\n\n{title}", cwd=self.dir)

    def squash(self, number, title):
        with open(os.path.join(self.dir, f"squash{number}"), "w") as f:
            f.write("x")
        run("git", "add", "-A", cwd=self.dir)
        run("git", "commit", "-qm", f"{title} (#{number})", cwd=self.dir)

    def test_lists_merged_prs_since_the_previous_release(self):
        ref = release_notes.resolve_ref("3.1.0", "main")
        base = release_notes.previous_tag("3.1.0", ref)
        self.assertEqual(base, "stm32-v3.0.0")
        prs = release_notes.pull_requests(base, ref)
        self.assertEqual(prs, [(1, "Add feature A"), (2, "Fix bug B")])
        notes = release_notes.render("3.1.0", ref, base, prs)
        self.assertIn("- Firmware: 3.1.0", notes)
        self.assertIn("- Embedded gateware: v41", notes)
        self.assertIn("- Add feature A ([#1]", notes)
        self.assertNotIn("Warning", notes)

    def test_warns_on_a_version_mismatch(self):
        notes = release_notes.render("3.2.0", "main", "stm32-v3.0.0", [])
        self.assertIn("Warning", notes)
        self.assertIn("No pull requests", notes)

    def test_version_key_ignores_the_stm32_prefix(self):
        self.assertEqual(release_notes.version_key("stm32-v3.6.0"), (3, 6, 0))
        self.assertLess(release_notes.version_key("stm32-v3.6.0"), release_notes.version_key("3.7.0"))


if __name__ == "__main__":
    unittest.main()
