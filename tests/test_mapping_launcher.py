"""CMD's embedded Python: image freshness and failure isolation without Docker."""
import os
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest.mock import patch


LAUNCHER = Path(__file__).resolve().parents[1] / 'scripts/start_3d_mapping.cmd'


def load_launcher():
    module = types.ModuleType('mapping_launcher_test')
    with patch.dict(os.environ, {'MAPPING_LAUNCHER': str(LAUNCHER)}):
        exec(compile(LAUNCHER.read_text(encoding='utf-8').rsplit('# === PYTHON ===', 1)[1],
                     str(LAUNCHER), 'exec'), module.__dict__)
    return module


class MappingLauncherTests(unittest.TestCase):
    def setUp(self):
        self.launcher = load_launcher()
        self.expected = self.launcher.slam_source_hash(
            self.launcher.ROOT / 'modules/mapping/localization/orbslam3')

    def reply(self, value='', code=0):
        return subprocess.CompletedProcess([], code, stdout=value, stderr='')

    def test_source_change_invalidates_the_backend_but_generated_build_files_do_not(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'bridge.cpp').write_text('version one')
            before = self.launcher.slam_source_hash(root)
            (root / 'build').mkdir()
            (root / 'build/cache').write_text('generated')
            self.assertEqual(self.launcher.slam_source_hash(root), before)
            (root / 'bridge.cpp').write_bytes(b'version one\n')
            lf = self.launcher.slam_source_hash(root)
            (root / 'bridge.cpp').write_bytes(b'version one\r\n')
            self.assertEqual(self.launcher.slam_source_hash(root), lf)
            (root / 'bridge.cpp').write_text('version two')
            self.assertNotEqual(self.launcher.slam_source_hash(root), before)

    def test_matching_image_is_reused_without_build_or_container_restart(self):
        with patch.object(self.launcher.subprocess, 'run', side_effect=[self.reply('server'), self.reply(self.expected)]), \
                patch.object(self.launcher.subprocess, 'call') as build:
            self.assertTrue(self.launcher.ensure_slam_image())
        build.assert_not_called()

    def test_missing_or_old_image_builds_from_this_checkout_and_verifies_the_result(self):
        for old in [self.reply('outdated'), self.reply('', 1)]:
            with self.subTest(old=old), patch.object(self.launcher.subprocess, 'run',
                    side_effect=[self.reply('server'), old, self.reply(self.expected)]), \
                    patch.object(self.launcher.subprocess, 'call', return_value=0) as build:
                self.assertTrue(self.launcher.ensure_slam_image())
                command = build.call_args.args[0]
                self.assertEqual(command[:2], ['docker', 'build'])
                self.assertIn(self.launcher.SLAM_SOURCE_LABEL + '=' + self.expected, command)
                self.assertEqual(Path(command[-1]), self.launcher.ROOT / 'modules/mapping/localization/orbslam3')

    def test_unavailable_engine_failed_build_and_unverified_result_never_report_ready(self):
        for replies, build_code in [([self.reply('', 1)], 0),
                ([self.reply('server'), self.reply('old')], 1),
                ([self.reply('server'), self.reply('old'), self.reply('still_old')], 0)]:
            with self.subTest(replies=replies), patch.object(self.launcher.subprocess, 'run', side_effect=replies), \
                    patch.object(self.launcher.subprocess, 'call', return_value=build_code):
                self.assertFalse(self.launcher.ensure_slam_image())
        for failure in [FileNotFoundError(), subprocess.TimeoutExpired('docker', 10)]:
            with patch.object(self.launcher.subprocess, 'run', side_effect=failure), \
                    patch.object(self.launcher.subprocess, 'call') as build:
                self.assertFalse(self.launcher.ensure_slam_image())
                build.assert_not_called()


if __name__ == '__main__':
    unittest.main()
