"""CMD setup, image freshness and failure isolation without downloads or Docker."""
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

    def test_ready_environment_checks_the_model_without_pip_or_gpu_detection(self):
        with patch.object(self.launcher.importlib.util, 'find_spec', return_value=object()), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call, \
                patch.object(self.launcher.subprocess, 'run') as gpu:
            self.assertTrue(self.launcher.ensure_python_environment())
        self.assertEqual(call.call_count, 1)
        self.assertEqual(call.call_args.args[0][:3], [self.launcher.sys.executable, '-B', '-c'])
        self.assertIn('DepthEstimator()', call.call_args.args[0][-1])
        gpu.assert_not_called()

    def test_missing_torch_uses_detected_gpu_and_installs_into_the_server_python(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name in {'torch', 'transformers'} else object()), \
                patch.object(self.launcher.subprocess, 'run', return_value=self.reply('NVIDIA GPU')), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment())
        commands = [item.args[0] for item in call.call_args_list]
        self.assertEqual(len(commands), 3)
        self.assertEqual(commands[0][:5], [self.launcher.sys.executable, '-B', '-m', 'pip', 'install'])
        self.assertEqual(commands[0][-2:], ['--index-url', 'https://download.pytorch.org/whl/cu128'])
        self.assertIn('--no-cache-dir', commands[0])
        self.assertEqual(commands[1][-2:], ['-r', str(self.launcher.ROOT / 'modules/mapping/data/requirements-stream.txt')])
        self.assertIn('DepthEstimator()', commands[2][-1])

    def test_missing_or_unavailable_nvidia_gpu_uses_cpu(self):
        for result in [self.reply('', 1), FileNotFoundError(), subprocess.TimeoutExpired('nvidia-smi', 10)]:
            with self.subTest(result=result), patch.object(self.launcher.importlib.util, 'find_spec',
                    side_effect=lambda name: None if name == 'torch' else object()), \
                    patch.object(self.launcher.subprocess, 'run', **(
                        {'side_effect': result} if isinstance(result, Exception) else {'return_value': result})), \
                    patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
                self.assertTrue(self.launcher.ensure_python_environment())
                self.assertEqual(call.call_args_list[0].args[0][-1], 'https://download.pytorch.org/whl/cpu')

    def test_existing_torch_is_preserved_when_other_packages_are_missing(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name == 'transformers' else object()), \
                patch.object(self.launcher.subprocess, 'run') as gpu, \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment())
        self.assertEqual(call.call_count, 2)
        self.assertIn('-r', call.call_args_list[0].args[0])
        gpu.assert_not_called()

    def test_virtual_environment_without_pip_bootstraps_it_first(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name in {'pip', 'transformers'} else object()), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment())
        self.assertEqual(call.call_args_list[0].args[0], [self.launcher.sys.executable, '-B', '-m', 'ensurepip'])

    def test_failed_install_or_model_preparation_stops_setup(self):
        for codes in [[1], [0, 1], [0, 0, 1]]:
            with self.subTest(codes=codes), patch.object(self.launcher.importlib.util, 'find_spec',
                    side_effect=lambda name: None if name == 'torch' else object()), \
                    patch.object(self.launcher.subprocess, 'run', return_value=self.reply('GPU')), \
                    patch.object(self.launcher.subprocess, 'call', side_effect=codes) as call:
                self.assertFalse(self.launcher.ensure_python_environment())
                self.assertEqual(call.call_count, len(codes))
        with patch.object(self.launcher.importlib.util, 'find_spec', return_value=object()), \
                patch.object(self.launcher.subprocess, 'call', side_effect=OSError('cannot start Python')):
            self.assertFalse(self.launcher.ensure_python_environment())

    def test_setup_failure_never_starts_server_or_opens_browser(self):
        with patch.object(self.launcher.sys, 'argv', [str(LAUNCHER), '--no-browser']), \
                patch.object(self.launcher.Path, 'is_file', return_value=True), \
                patch.object(self.launcher, 'ensure_python_environment', return_value=False), \
                patch.object(self.launcher, 'ensure_slam_image') as docker, \
                patch.object(self.launcher.subprocess, 'Popen') as server, \
                patch.object(self.launcher.webbrowser, 'open') as browser:
            self.assertEqual(self.launcher.main(), 1)
        docker.assert_not_called()
        server.assert_not_called()
        browser.assert_not_called()

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
