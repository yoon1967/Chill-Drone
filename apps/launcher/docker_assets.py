"""Check and prepare local containers and the verified MoNA ONNX artifact."""

import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

from modules.autonomy.mona_bench.runtime.depth_artifacts import DA_V2_FP16_FILE, DA_V2_FP16_SHA256

from .winproc import CREATE_NO_WINDOW

ROOT = Path(__file__).resolve().parents[2]
ORB_IMAGE = 'ssafy-orbslam3:local'
ORB_SOURCE = ROOT / 'modules/mapping/localization/orbslam3'
MONA_IMAGE = 'ssafy/mona-bench:noetic'
MONA_SOURCE = ROOT / 'modules/autonomy/mona_bench'
MONA_OUTPUT = ROOT / 'data/outputs/mona-bench'
# 이미지를 만든 코드(Dockerfile과 COPY·ADD한 파일)의 해시. 지금 코드와 다르면 다시 빌드한다.
SOURCE_LABEL = 'b205.launcher.source-sha256'


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


def _files(root):
    """폴더 안 파일 전부(__pycache__·*.pyc는 실행할 때 생기는 것이라 뺀다)."""
    return [path for path in Path(root).rglob('*')
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc']


def _copy_instructions(text):
    """Dockerfile의 COPY·ADD마다 (플래그, 인자)를 준다. 줄 끝 \\ 이음과 JSON 형식(["a", "b"])도 읽는다."""
    for line in re.sub(r'\\[ \t]*\r?\n', ' ', text).splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) < 2 or parts[0].upper() not in ('COPY', 'ADD'):
            continue
        flags, rest = [], parts[1].strip()
        while rest.startswith('--'):
            flag, _, rest = rest.partition(' ')
            flags.append(flag)
            rest = rest.strip()
        try:
            args = json.loads(rest) if rest.startswith('[') else rest.split()
        except ValueError:
            args = rest.split()
        yield flags, args


def image_inputs(source):
    """이미지에 들어가는 빌드 폴더 파일: Dockerfile과 COPY·ADD가 가져가는 파일(폴더면 그 안 전부).

    다른 빌드 단계(--from)와 URL은 빼고, 가져가는 경로를 못 찾으면 빌드 폴더 전부를 쓴다(덜 다시 빌드하기보다 더 다시 빌드한다).
    문서·시험·실행 스크립트처럼 이미지에 안 들어가는 파일이 바뀌어도 다시 빌드하지 않으려고 Dockerfile을 읽는다.
    """
    source = Path(source)

    def ordered(paths):   # 운영체제와 상관없이 같은 순서(해시가 순서를 따른다)
        return sorted(set(paths), key=lambda path: path.relative_to(source).as_posix())
    try:
        text = (source / 'Dockerfile').read_text(encoding='utf-8')
    except OSError:
        return ordered(_files(source))
    picked = [source / 'Dockerfile']
    for flags, args in _copy_instructions(text):
        if any(flag.startswith('--from') for flag in flags):
            continue
        for pattern in args[:-1]:   # 마지막은 이미지 안 경로
            if '://' in pattern:
                continue
            pattern = pattern.strip('/') or '.'
            found = sorted(source.glob(pattern)) if any(c in pattern for c in '*?[') else [source / pattern]
            if not found or not all(path.exists() for path in found):
                return ordered(_files(source))
            picked += found
    files = []
    for path in picked:
        files += _files(path) if path.is_dir() else [path]
    return ordered(files)


def source_hash(source):
    """image_inputs 파일의 경로와 내용으로 만든 해시."""
    source = Path(source)
    digest = hashlib.sha256()
    for path in image_inputs(source):
        digest.update(path.relative_to(source).as_posix().encode('utf-8') + b'\0')
        digest.update(path.read_bytes() + b'\0')
    return digest.hexdigest()


def image_source(image):
    """이미지가 없으면 None, 있으면 그 이미지를 만든 코드의 해시(이 라벨을 붙이기 전에 만든 이미지는 '')."""
    labels = probe(['docker', 'image', 'inspect', image, '--format', '{{json .Config.Labels}}'])
    if labels is None:
        return None
    try:
        return (json.loads(labels) or {}).get(SOURCE_LABEL, '')
    except (ValueError, AttributeError):
        return ''


def image_build_command(image, source, digest):
    return ['docker', 'build', '--progress', 'plain', '--label', f'{SOURCE_LABEL}={digest}',
            '-t', image, str(source)]


def ensure_image(image, source):
    """이미지가 없거나 지금 코드와 다르면 빌드한다. Docker는 바뀐 단계부터 다시 하므로 보통 금방 끝난다.

    이미지가 있었는데 다시 빌드하지 못하면(드론 Wi-Fi라 인터넷이 없는 등) 있던 이미지로 켜고 알린다.
    """
    want = source_hash(source)
    have = image_source(image)
    if have == want:
        return True
    if have is None:
        print(f'{image} 이미지를 빌드합니다. 첫 빌드는 오래 걸릴 수 있습니다.', flush=True)
    elif not have:
        print(f'{image} 이미지를 어떤 코드로 만들었는지 몰라(라벨 없음) 한 번 다시 빌드합니다. '
              'Docker 빌드 캐시가 남아 있으면 금방 끝나고, 없으면 첫 빌드처럼 오래 걸릴 수 있습니다.', flush=True)
    else:
        print(f'{image} 이미지가 지금 코드와 달라 다시 빌드합니다. 바뀐 단계부터 다시 하므로 보통 금방 끝나지만, '
              'Dockerfile 앞부분이 바뀌었으면 첫 빌드처럼 오래 걸릴 수 있습니다.', flush=True)
    built = subprocess.call(image_build_command(image, source, want), cwd=ROOT) == 0
    if not built and have is not None:
        print(f'[주의] {image} 이미지를 다시 빌드하지 못해 이전 코드로 만든 이미지로 켭니다. '
              '인터넷에 연결된 상태에서 다시 실행하면 다시 빌드합니다.', flush=True)
        return True
    return built


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


def mona_ready():
    return model_verified(MONA_OUTPUT / 'models' / DA_V2_FP16_FILE) and image_source(MONA_IMAGE) is not None


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
