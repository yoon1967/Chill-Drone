"""실행기 준비(apps/launcher/prepare.py): 빠진 패키지·모델·웹 의존성만 받기. 다운로드 없이."""
from hashlib import sha256
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from apps.launcher import docker_assets, prepare


class HostCheckTests(unittest.TestCase):
    def test_docker_checks_read_only_and_explain_what_to_prepare(self):
        endpoint = 'npipe:////./pipe/dockerDesktopLinuxEngine'
        wsl_engine = 'linux|6.6.87-microsoft-standard-WSL2'
        for installed, replies, mona, hint in (
            (False, [], False, 'Docker Desktop을 설치'),
            (True, ['ssh://other-pc'], False, '이 PC의 Linux 엔진'),
            (True, [endpoint, None], False, 'Linux 엔진이 시작'),
            (True, [endpoint, 'windows|10.0'], False, 'Linux 컨테이너 모드'),
            (True, [endpoint, 'linux|6.12-linuxkit'], True, 'WSL 2 기반 엔진'),
            (True, [endpoint, wsl_engine, None], True, 'NVIDIA GPU 드라이버'),
            (True, [endpoint, wsl_engine], False, None),
            (True, [endpoint, wsl_engine, '591.44'], True, None),
        ):
            with self.subTest(hint=hint, mona=mona), patch.dict(docker_assets.os.environ, clear=True), \
                    patch.object(docker_assets.sys, 'platform', 'win32'), \
                    patch.object(docker_assets, 'installed_program', return_value=installed), \
                    patch.object(docker_assets, 'probe', side_effect=replies) as probe:
                problem = docker_assets.docker_problem(mona=mona)
                if hint:
                    self.assertIn(hint, problem)
                else:
                    self.assertIsNone(problem)
                self.assertEqual(probe.call_count, len(replies))
                for call in probe.call_args_list:
                    if call.args[0][0] == 'docker':
                        self.assertIn(call.args[0][1], ('context', 'info'))


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
        self.assertIn('torchvision==0.26.0', commands[0])   # 따로 두면 ultralytics가 CPU용 torch로 바꾼다
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
        with patch.object(prepare, 'console_web_ready', side_effect=[False, True]), \
                patch.object(prepare.shutil, 'which', return_value='pnpm'), \
                patch.object(prepare.subprocess, 'call', return_value=0) as call:
            self.assertTrue(prepare.ensure_console_web())
        self.assertEqual(call.call_args.args[0], ['pnpm', '--dir', 'apps/console', 'install', '--frozen-lockfile'])
        with patch.object(prepare.Path, 'is_file', return_value=False), patch.object(prepare.shutil, 'which', return_value=None), \
                patch.object(prepare.docker_assets, 'installed_program', return_value=None):
            self.assertFalse(prepare.ensure_console_web())

    def test_failed_package_preparation_stops_before_the_next_stage(self):
        with patch.object(prepare, 'ensure_python_environment', return_value=False), \
                patch.object(prepare, 'ensure_gateway_environment') as gateway, \
                patch.object(prepare, 'ensure_models') as models, \
                patch.object(prepare, 'ensure_console_web') as web, \
                patch.object(docker_assets, 'ensure_image') as build:
            self.assertFalse(prepare.ensure_all())
        gateway.assert_not_called()
        models.assert_not_called()
        web.assert_not_called()
        build.assert_not_called()

    def test_only_explicit_flight_preparation_downloads_models_and_builds_images(self):
        with patch.object(prepare, 'ensure_python_environment', return_value=True), \
                patch.object(prepare, 'ensure_gateway_environment', return_value=True), \
                patch.object(prepare, 'ensure_models', return_value=True) as models, \
                patch.object(prepare, 'ensure_console_web', return_value=True), \
                patch.object(docker_assets, 'ensure_image', return_value=True) as build, \
                patch.object(docker_assets, 'ensure_mona_model', return_value=True), \
                patch.object(docker_assets, 'probe', return_value=None), \
                patch.object(docker_assets.subprocess, 'call', return_value=0) as docker:
            self.assertTrue(prepare.ensure_all())
            models.assert_not_called()
            build.assert_not_called()
            docker.assert_not_called()
            self.assertTrue(prepare.prepare_flight())
        models.assert_called_once()
        self.assertEqual([item.args[0] for item in build.call_args_list],
                         [docker_assets.ORB_IMAGE, docker_assets.MONA_IMAGE])
        self.assertEqual(docker.call_args.args[0][:5], ['docker', 'run', '--rm', '--gpus', 'all'])

    def test_mona_reuses_verified_model_and_never_publishes_invalid_export(self):
        payload = b'verified model'

        def export(command, **_):
            (Path(command[-1]) / docker_assets.DA_V2_FP16_FILE).write_bytes(payload)
            return 0

        with tempfile.TemporaryDirectory() as folder, \
                patch.object(docker_assets, 'MONA_OUTPUT', Path(folder)), \
                patch.object(docker_assets, 'DA_V2_FP16_SHA256', sha256(payload).hexdigest()), \
                patch.object(docker_assets, 'export_python', return_value='python'), \
                patch.object(docker_assets.subprocess, 'call', side_effect=export) as call:
            self.assertTrue(docker_assets.ensure_mona_model())
            self.assertTrue(docker_assets.ensure_mona_model())
            call.assert_called_once()
            model = Path(folder) / 'models' / docker_assets.DA_V2_FP16_FILE
            model.write_bytes(b'broken existing file')
            payload = b'invalid export'
            self.assertFalse(docker_assets.ensure_mona_model())
            self.assertEqual(model.read_bytes(), b'broken existing file')
            self.assertEqual(list(model.parent.iterdir()), [model])


if __name__ == '__main__':
    unittest.main()
