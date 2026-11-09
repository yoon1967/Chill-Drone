"""켜기 전에 빠진 것을 준비한다(인터넷 필요): 패키지 → 모델 파일 → 지도 콘솔 웹 의존성.

1·2번을 켤 때마다 먼저 확인하고, 빠진 것만 받는다(다 있으면 몇 초 안에 넘어간다).
패키지 설치 순서는 이전 start_3d_mapping.cmd의 준비(--prepare-models)를 그대로 옮겼다.
"""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


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
                if subprocess.call(pip + ['torch>=2.6,<3', 'torchvision', '--index-url', index], cwd=ROOT):
                    print('[오류] PyTorch 설치에 실패했습니다. 인터넷과 디스크 여유를 확인하고 다시 실행하세요.', flush=True)
                    return False
            requirements = ROOT / 'modules/mapping/data/requirements-stream.txt'
            if subprocess.call(pip + ['-r', str(requirements)], cwd=ROOT):
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
    if (ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        return True
    pnpm = shutil.which('pnpm')
    if not pnpm:
        print('[오류] 지도 콘솔 웹 의존성이 없고 pnpm도 없습니다. Node.js와 pnpm을 설치한 뒤 다시 실행하세요.')
        return False
    print('지도 콘솔 웹 의존성을 설치합니다(pnpm)...', flush=True)
    if subprocess.call([pnpm, '--dir', 'apps/console', 'install', '--frozen-lockfile'], cwd=ROOT):
        print('[오류] 웹 의존성 설치에 실패했습니다. 위 메시지를 확인하고 다시 실행하세요.')
        return False
    return True


def ensure_all(*, console=True):
    """빠진 것을 받는다. 못 받은 게 있어도 멈추지 않고 무엇이 준비됐는지 돌려준다(그것만 빼고 켠다)."""
    packages = ensure_python_environment(allow_install=True)
    gateway = ensure_gateway_environment(True)
    models = packages and ensure_models()
    web = ensure_console_web() if console else True
    return {'packages': packages, 'gateway': gateway, 'models': models, 'web': web}
