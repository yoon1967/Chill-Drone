"""PC의 실행 환경을 확인한 뒤 빠진 프로젝트 의존성만 준비한다."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

from . import docker_assets

ROOT = Path(__file__).resolve().parents[2]
THREE = ROOT / 'apps/console/node_modules/three'
WEB_FILES = ('build/three.module.js', 'build/three.core.js', 'examples/jsm/controls/OrbitControls.js')
NODE_INSTALL_HINT = 'Node.js LTS를 설치해 주세요. https://nodejs.org/'


def console_web_ready():
    return all((THREE / file).is_file() for file in WEB_FILES)


def console_install_command():
    pnpm = shutil.which('pnpm')
    if pnpm:
        return [pnpm]
    npx = docker_assets.installed_program(
        'npx', Path(os.environ.get('ProgramFiles', 'C:/Program Files')) / 'nodejs/npx.cmd')
    return [npx, '--yes', 'pnpm@10'] if npx else None


def check_host(*, mona):
    """호스트 설정을 바꾸거나 프로젝트 파일을 만들지 않고 확인한다."""
    problems = []
    if sys.version_info[:2] != (3, 12) or sys.maxsize <= 2**32:
        problems.append('Python 3.12(64비트)를 설치해 주세요. https://www.python.org/downloads/windows/')
    if not console_web_ready() and not console_install_command():
        problems.append(NODE_INSTALL_HINT)
    if mona:
        docker = docker_assets.docker_problem(mona=True)
        if docker:
            problems.append(docker)
    if problems:
        print('[실행 전 확인] 이 PC에 필요한 설정이 있습니다.', flush=True)
        for problem in problems:
            print('  - ' + problem, flush=True)
        print('위 항목을 준비한 후 scripts/start.cmd를 다시 실행해 주세요.', flush=True)
        return False
    return True


def create_project_environment():
    env = ROOT / '.venv'
    python = env / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    print('프로젝트 Python 가상환경을 준비합니다...', flush=True)
    if subprocess.call([sys.executable, '-B', '-m', 'venv', str(env)], cwd=ROOT) == 0 and python.is_file():
        return python
    print('[오류] 가상환경을 만들지 못했습니다. 위 메시지를 확인하고 scripts/start.cmd를 다시 실행해 주세요.', flush=True)
    return None


def ensure_python_environment(*, allow_install=False):
    required = ['cv2', 'numpy', 'PIL', 'av', 'torch', 'transformers', 'ultralytics']
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    try:
        if missing:
            if not allow_install:
                print('[오류] 지도·사람 탐지 패키지가 없습니다: ' + ', '.join(missing), flush=True)
                return False
            print('빠진 지도·사람 탐지 패키지를 받습니다: ' + ', '.join(missing) + '\n'
                  '인터넷을 유지하세요. 처음 받을 때 몇 분 걸릴 수 있습니다.', flush=True)
            if importlib.util.find_spec('pip') is None:
                if subprocess.call([sys.executable, '-B', '-m', 'ensurepip'], cwd=ROOT):
                    print('[오류] pip를 준비하지 못했습니다. 위 메시지를 확인하고 다시 실행하세요.', flush=True)
                    return False
            pip = [sys.executable, '-B', '-m', 'pip', 'install',
                   '--disable-pip-version-check', '--no-cache-dir']
            if 'torch' in missing:
                index = 'https://download.pytorch.org/whl/cpu'
                try:
                    gpu = subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
                                         capture_output=True, text=True, timeout=10)
                    if gpu.returncode == 0 and gpu.stdout.strip():
                        index = 'https://download.pytorch.org/whl/cu128'
                except (OSError, subprocess.TimeoutExpired):
                    pass
                print('PyTorch 설치: ' + ('NVIDIA CUDA' if index.endswith('cu128') else 'CPU'), flush=True)
                # torchvision도 같은 저장소에서 함께 받는다. 따로 두면 ultralytics가 PyPI의 최신 torchvision을 받으면서
                # 그에 맞는 CPU용 torch로 바꿔 버린다(GPU를 못 씀).
                if subprocess.call(pip + ['torch==2.11.0', 'torchvision==0.26.0', '--index-url', index], cwd=ROOT):
                    print('[오류] PyTorch 설치에 실패했습니다. 인터넷과 디스크 여유를 확인하고 다시 실행하세요.', flush=True)
                    return False
            requirements = ROOT / 'modules/mapping/data/requirements-stream.txt'
            pinned = ['transformers==4.57.6'] if 'transformers' in missing else []
            if subprocess.call(pip + pinned + ['-r', str(requirements)], cwd=ROOT):
                print('[오류] 지도·사람 탐지 패키지 설치에 실패했습니다. 위 메시지를 확인하고 다시 실행하세요.', flush=True)
                return False
            importlib.invalidate_caches()
        return True
    except OSError as error:
        print('[오류] 패키지 준비 실패: ' + str(error), flush=True)
        return False


def ensure_gateway_environment(allow_install):
    if all(importlib.util.find_spec(name) is not None for name in ("fastapi", "uvicorn", "websockets")):
        return True
    if not allow_install:
        print('[오류] 게이트웨이 패키지가 없습니다(fastapi·uvicorn·websockets).')
        return False
    print('빠진 게이트웨이 패키지를 받습니다...', flush=True)
    requirements = ROOT / 'modules/autonomy/tello_link/requirements.txt'
    return subprocess.call([sys.executable, '-B', '-m', 'pip', 'install', '--no-cache-dir',
                            '-r', str(requirements)], cwd=ROOT) == 0


def ensure_models():
    """깊이·장면·사람 탐지 모델 파일(사용자 캐시). 없으면 받는다."""
    sys.path.insert(0, str(ROOT))
    from modules.mapping.data.model_assets import ModelAssetsMissing, require_local_models
    try:
        require_local_models()
        return True
    except ModelAssetsMissing as error:
        print(f'모델 파일을 받습니다({error}). 인터넷을 유지하세요.', flush=True)
    return subprocess.call([sys.executable, '-B', '-m', 'modules.mapping.data.prepare_models', '--download'],
                           cwd=ROOT) == 0


def ensure_console_web():
    """지도 콘솔 웹 의존성(apps/console/node_modules). 없으면 pnpm으로 설치한다."""
    if console_web_ready():
        return True
    command = console_install_command()
    if not command:
        print('[오류] ' + NODE_INSTALL_HINT, flush=True)
        return False
    print('지도 콘솔 웹 의존성을 설치합니다(pnpm)...', flush=True)
    result = subprocess.call(command + ['--dir', 'apps/console', 'install', '--frozen-lockfile'], cwd=ROOT,
                             env={**os.environ, 'CI': 'true'})
    if result or not console_web_ready():
        print('[오류] 웹 의존성 설치에 실패했습니다. 위 메시지를 확인하고 다시 실행하세요.')
        return False
    return True


def ensure_all():
    """관제 화면에 필요한 패키지만 준비한다."""
    try:
        return (ensure_python_environment(allow_install=True)
                and ensure_gateway_environment(True)
                and ensure_console_web())
    except OSError as error:
        print(f'[오류] 프로젝트 준비 중 파일 또는 프로그램에 접근하지 못했습니다. {error}', flush=True)
        return False


def prepare_flight():
    """실시간 지도, 사람 탐지, MoNA에 필요한 파일을 요청한 경우에 준비한다."""
    return (ensure_all() and ensure_models()
            and docker_assets.ensure_image(docker_assets.ORB_IMAGE, docker_assets.ORB_SOURCE)
            and docker_assets.ensure_mona())
