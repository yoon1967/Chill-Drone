"""실행기 준비(apps/launcher/prepare.py): 빠진 패키지·모델·웹 의존성만 받기. 다운로드 없이."""
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

from apps.launcher import prepare


class PythonPackageTests(unittest.TestCase):
    """이전 start_3d_mapping.cmd 준비 순서(옮겨 온 시험) + torchvision 함께 설치."""
    def setUp(self):
        self.launcher = prepare

    def reply(self, value='', code=0):
        return subprocess.CompletedProcess([], code, stdout=value, stderr='')

    def test_ready_environment_installs_nothing(self):
        with patch.object(self.launcher.importlib.util, 'find_spec', return_value=object()), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call, \
                patch.object(self.launcher.subprocess, 'run') as gpu:
            self.assertTrue(self.launcher.ensure_python_environment())
        call.assert_not_called()
        gpu.assert_not_called()

    def test_missing_packages_are_not_installed_unless_allowed(self):
        with patch.object(self.launcher.importlib.util, 'find_spec', return_value=None), \
                patch.object(self.launcher.subprocess, 'call') as install:
            self.assertFalse(self.launcher.ensure_python_environment())
        install.assert_not_called()

    def test_missing_torch_uses_detected_gpu_and_installs_torchvision_from_the_same_index(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name in {'torch', 'transformers'} else object()), \
                patch.object(self.launcher.subprocess, 'run', return_value=self.reply('NVIDIA GPU')), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment(allow_install=True))
        commands = [item.args[0] for item in call.call_args_list]
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0][:5], [self.launcher.sys.executable, '-B', '-m', 'pip', 'install'])
        self.assertEqual(commands[0][-2:], ['--index-url', 'https://download.pytorch.org/whl/cu128'])
        self.assertIn('torchvision', commands[0])   # 따로 두면 ultralytics가 CPU용 torch로 바꾼다
        self.assertIn('--no-cache-dir', commands[0])
        self.assertEqual(commands[1][-2:], ['-r', str(self.launcher.ROOT / 'modules/mapping/data/requirements-stream.txt')])

    def test_missing_or_unavailable_nvidia_gpu_uses_cpu(self):
        for result in [self.reply('', 1), FileNotFoundError(), subprocess.TimeoutExpired('nvidia-smi', 10)]:
            with self.subTest(result=result), patch.object(self.launcher.importlib.util, 'find_spec',
                    side_effect=lambda name: None if name == 'torch' else object()), \
                    patch.object(self.launcher.subprocess, 'run', **(
                        {'side_effect': result} if isinstance(result, Exception) else {'return_value': result})), \
                    patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
                self.assertTrue(self.launcher.ensure_python_environment(allow_install=True))
                self.assertEqual(call.call_args_list[0].args[0][-1], 'https://download.pytorch.org/whl/cpu')

    def test_existing_torch_is_preserved_when_other_packages_are_missing(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name == 'transformers' else object()), \
                patch.object(self.launcher.subprocess, 'run') as gpu, \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment(allow_install=True))
        self.assertEqual(call.call_count, 1)
        self.assertIn('-r', call.call_args_list[0].args[0])
        gpu.assert_not_called()

    def test_virtual_environment_without_pip_bootstraps_it_first(self):
        with patch.object(self.launcher.importlib.util, 'find_spec',
                side_effect=lambda name: None if name in {'pip', 'transformers'} else object()), \
                patch.object(self.launcher.subprocess, 'call', return_value=0) as call:
            self.assertTrue(self.launcher.ensure_python_environment(allow_install=True))
        self.assertEqual(call.call_args_list[0].args[0], [self.launcher.sys.executable, '-B', '-m', 'ensurepip'])

    def test_failed_package_install_stops_setup(self):
        for codes in [[1], [0, 1]]:
            with self.subTest(codes=codes), patch.object(self.launcher.importlib.util, 'find_spec',
                    side_effect=lambda name: None if name == 'torch' else object()), \
                    patch.object(self.launcher.subprocess, 'run', return_value=self.reply('GPU')), \
                    patch.object(self.launcher.subprocess, 'call', side_effect=codes) as call:
                self.assertFalse(self.launcher.ensure_python_environment(allow_install=True))
                self.assertEqual(call.call_count, len(codes))
        with patch.object(self.launcher.importlib.util, 'find_spec', side_effect=lambda name: None if name == 'transformers' else object()), \
                patch.object(self.launcher.subprocess, 'call', side_effect=OSError('cannot start Python')):
            self.assertFalse(self.launcher.ensure_python_environment(allow_install=True))


class ModelAndWebTests(unittest.TestCase):
    def test_models_are_downloaded_only_when_missing(self):
        from modules.mapping.data.model_assets import ModelAssetsMissing
        with patch('modules.mapping.data.model_assets.require_local_models', return_value={}), \
                patch.object(prepare.subprocess, 'call') as call:
            self.assertTrue(prepare.ensure_models())
        call.assert_not_called()
        with patch('modules.mapping.data.model_assets.require_local_models', side_effect=ModelAssetsMissing('depth')), \
                patch.object(prepare.subprocess, 'call', return_value=0) as call:
            self.assertTrue(prepare.ensure_models())
        self.assertEqual(call.call_args.args[0][-2:], ['modules.mapping.data.prepare_models', '--download'])

    def test_console_web_uses_pnpm_when_missing(self):
        with patch.object(prepare.Path, 'is_file', return_value=False), \
                patch.object(prepare.shutil, 'which', return_value='pnpm'), \
                patch.object(prepare.subprocess, 'call', return_value=0) as call:
            self.assertTrue(prepare.ensure_console_web())
        self.assertEqual(call.call_args.args[0], ['pnpm', '--dir', 'apps/console', 'install', '--frozen-lockfile'])
        with patch.object(prepare.Path, 'is_file', return_value=False), patch.object(prepare.shutil, 'which', return_value=None):
            self.assertFalse(prepare.ensure_console_web())

    def test_ensure_all_reports_each_part_and_skips_models_without_packages(self):
        with patch.object(prepare, 'ensure_python_environment', return_value=False), \
                patch.object(prepare, 'ensure_gateway_environment', return_value=True), \
                patch.object(prepare, 'ensure_models') as models, \
                patch.object(prepare, 'ensure_console_web', return_value=True):
            self.assertEqual(prepare.ensure_all(), {'packages': False, 'gateway': True, 'models': False, 'web': True})
        models.assert_not_called()
        with patch.object(prepare, 'ensure_python_environment', return_value=True), \
                patch.object(prepare, 'ensure_gateway_environment', return_value=True), \
                patch.object(prepare, 'ensure_models', return_value=True), \
                patch.object(prepare, 'ensure_console_web') as web:
            self.assertEqual(prepare.ensure_all(console=False), {'packages': True, 'gateway': True, 'models': True, 'web': True})
        web.assert_not_called()


if __name__ == '__main__':
    unittest.main()
