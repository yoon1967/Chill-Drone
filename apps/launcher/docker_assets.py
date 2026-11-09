"""Prepare local containers and the verified MoNA ONNX artifact on first run."""

import hashlib
import importlib.metadata
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from modules.autonomy.mona_bench.runtime.depth_artifacts import DA_V2_FP16_FILE, DA_V2_FP16_SHA256

from .winproc import CREATE_NO_WINDOW

ROOT = Path(__file__).resolve().parents[2]
ORB_IMAGE = 'ssafy-orbslam3:local'
MONA_IMAGE = 'ssafy/mona-bench:noetic'
MONA_SOURCE = ROOT / 'modules/autonomy/mona_bench'
MONA_OUTPUT = ROOT / 'data/outputs/mona-bench'


def installed_program(name, installed_path):
    found = shutil.which(name)
    if found:
        return found
    if installed_path.is_file():
        # Let later launcher commands find programs installed outside this session's PATH.
        os.environ['PATH'] = str(installed_path.parent) + os.pathsep + os.environ.get('PATH', '')
        return str(installed_path)
    return None


def probe(command, timeout=15):
    try:
        result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace',
                                timeout=timeout, creationflags=CREATE_NO_WINDOW if os.name == 'nt' else 0)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def docker_problem(*, mona):
    docker_path = Path(os.environ.get('ProgramFiles', 'C:/Program Files')) / 'Docker/Docker/resources/bin/docker.exe'
    if not installed_program('docker', docker_path):
        return 'Docker Desktop을 설치하고 첫 실행 설정을 마쳐 주세요. https://docs.docker.com/desktop/setup/install/windows-install/'

    host = os.environ.get('DOCKER_HOST') if not os.environ.get('DOCKER_CONTEXT') else None
    if not host:
        host = probe(['docker', 'context', 'inspect', '--format', '{{.Endpoints.docker.Host}}'])
    if not host or not host.startswith(('npipe:////./pipe/', 'unix:///')):
        return 'Docker Desktop에서 이 PC의 Linux 엔진을 선택해 주세요. 현재 Docker 연결에서 로컬 엔진을 확인할 수 없습니다.'
    engine = probe(['docker', 'info', '--format', '{{.OSType}}|{{.KernelVersion}}'])
    if not engine:
        return 'Docker Desktop을 열어 첫 실행 설정을 마치고 Linux 엔진이 시작될 때까지 기다려 주세요.'
    os_type, _, kernel = engine.partition('|')
    if os_type != 'linux':
        return 'Docker Desktop에서 Linux 컨테이너 모드로 전환해 주세요.'
    if not mona:
        return None
    if sys.platform == 'win32' and 'wsl2' not in kernel.lower():
        return 'Docker Desktop 설정에서 WSL 2 기반 엔진을 사용해 주세요. https://docs.docker.com/desktop/features/wsl/'
    if not probe(['nvidia-smi', '--query-gpu=driver_version', '--format=csv,noheader']):
        return '실기체 모드에는 NVIDIA GPU가 필요합니다. NVIDIA GPU 드라이버를 설치하거나 업데이트해 주세요.'
    return None


def ensure_image(image, source):
    if probe(['docker', 'image', 'inspect', image, '--format', '{{.Id}}']):
        return True
    print(f'{image} 이미지를 빌드합니다. 첫 빌드는 오래 걸릴 수 있습니다.', flush=True)
    return subprocess.call(['docker', 'build', '--progress', 'plain', '-t', image, str(source)], cwd=ROOT) == 0


def model_verified(path):
    try:
        with path.open('rb') as source:
            return hashlib.file_digest(source, 'sha256').hexdigest() == DA_V2_FP16_SHA256
    except OSError:
        return False


def export_python():
    """Use the verified export toolchain without replacing a different project installation."""
    try:
        compatible = (importlib.metadata.version('torch') == '2.11.0+cu128' and
                      importlib.metadata.version('transformers') == '4.57.6')
    except importlib.metadata.PackageNotFoundError:
        compatible = False
    if compatible:
        if importlib.util.find_spec('onnx') is not None or subprocess.call(
                [sys.executable, '-m', 'pip', 'install', 'onnx>=1.17,<2'], cwd=ROOT) == 0:
            return sys.executable
        return None
    env = MONA_OUTPUT / 'export-python'
    python = env / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    if not python.is_file() and subprocess.call([sys.executable, '-m', 'venv', str(env)], cwd=ROOT):
        return None
    pip = [str(python), '-m', 'pip', 'install', '--disable-pip-version-check']
    if subprocess.call(pip + ['torch==2.11.0', '--index-url', 'https://download.pytorch.org/whl/cu128'], cwd=ROOT):
        return None
    if subprocess.call(pip + ['transformers==4.57.6', 'onnx>=1.17,<2'], cwd=ROOT):
        return None
    return str(python)


def ensure_mona_model():
    target = MONA_OUTPUT / 'models' / DA_V2_FP16_FILE
    if model_verified(target):
        return True
    print('MoNA 깊이 가중치를 준비하고 CUDA용 ONNX로 변환합니다...', flush=True)
    python = export_python()
    if not python:
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.export-', dir=target.parent) as work:
        if subprocess.call([python, '-B', str(MONA_SOURCE / 'export_da2_onnx.py'), work], cwd=ROOT):
            return False
        artifact = Path(work) / target.name
        if not model_verified(artifact):
            print('[오류] 변환한 MoNA 모델의 체크섬이 검증본과 다릅니다.', flush=True)
            return False
        if not model_verified(target):
            artifact.replace(target)
    return True


def ensure_mona():
    if not (ensure_mona_model() and ensure_image(MONA_IMAGE, MONA_SOURCE)):
        return False
    if probe(['docker', 'ps', '-q', '--filter', 'name=^mona-live$']):
        return True
    print('MoNA 컨테이너에서 CUDA 깊이 추론을 확인합니다...', flush=True)
    sample = ('import sys; sys.path.insert(0, "/opt/mona_runtime"); import numpy as np; '
              'from onnx_depth import DepthModel; '
              'DepthModel(device="cuda", kind="da_v2_metric_indoor_small").infer(np.zeros((720, 960, 3), np.uint8))')
    if subprocess.call(['docker', 'run', '--rm', '--gpus', 'all', '--mount',
                        f'type=bind,source={MONA_OUTPUT / "models"},target=/models,readonly',
                        MONA_IMAGE, 'python3', '-c', sample], cwd=ROOT):
        print('[오류] MoNA CUDA 추론 확인에 실패했습니다. NVIDIA 드라이버와 Docker Desktop의 WSL 2 GPU 설정을 확인해 주세요.',
              flush=True)
        return False
    return True
