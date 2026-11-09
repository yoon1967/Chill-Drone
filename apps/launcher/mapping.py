"""3D 지도 촬영·저장 지도 열기(지도 파트). scripts/start_3d_mapping.cmd의 내용을 그대로 옮겼다.

- 기본(통합 실행): 이 창 안에서 드론 게이트웨이를 지도 통합 모드로 함께 켜고, 관제 화면의 지도 콘솔(#/map)을 연다.
- --gateway URL: 이미 켜진 게이트웨이를 쓴다. --direct: 촬영용 직접 연결. --fake: 가짜 드론.
- 입력(영상·저장 결과 폴더)을 주면 지도 콘솔에서 연다.
- --prepare-models: 인터넷이 될 때 패키지·모델을 준비한다(start.cmd prepare).
"""
import argparse
from contextlib import contextmanager, nullcontext
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import webbrowser

ROOT = Path(__file__).resolve().parents[2]
SLAM_IMAGE = 'ssafy-orbslam3:local'
SLAM_SOURCE_LABEL = 'org.ssafy.mapping.source-sha256'
PREPARE = r'scripts\start.cmd prepare'


def ensure_python_environment(*, allow_install=False):
    required = ['cv2', 'numpy', 'PIL', 'av', 'torch', 'transformers', 'ultralytics']
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    try:
        if missing:
            if not allow_install:
                print('[오류] 지도 패키지가 없습니다: ' + ', '.join(missing) +
                      f'. 인터넷에 연결하고 먼저 {PREPARE}를 실행하세요.', flush=True)
                return False
            print('빠진 지도 패키지를 준비합니다: ' + ', '.join(missing) + '\n'
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
                print('[오류] 지도 패키지 설치에 실패했습니다. 위 메시지를 확인하고 다시 실행하세요.', flush=True)
                return False
            importlib.invalidate_caches()
        print('지도 패키지 준비됨. 모델은 복원을 시작할 때 로컬 파일에서 읽고, 영상은 모델을 다 올린 뒤 받습니다.', flush=True)
        return True
    except OSError as error:
        print('[오류] 지도 준비 실패: ' + str(error) + '. 고친 뒤 다시 실행하세요.', flush=True)
        return False


def slam_source_hash(context):
    """로컬 이미지 입력의 버전. Python·웹 변경은 실행할 때마다 다시 읽는다."""
    digest = hashlib.sha256()
    for path in sorted(context.rglob('*')):
        relative = path.relative_to(context)
        if not path.is_file() or any(part in {'build', '__pycache__', '.git'} for part in relative.parts):
            continue
        digest.update(relative.as_posix().encode('utf-8') + b'\0')
        digest.update(path.read_bytes().replace(b'\r\n', b'\n') + b'\0')
    return digest.hexdigest()


def ensure_slam_image():
    context = ROOT / 'modules/mapping/localization/orbslam3'
    expected = slam_source_hash(context)
    inspect = ['docker', 'image', 'inspect', SLAM_IMAGE, '--format',
               '{{ index .Config.Labels "' + SLAM_SOURCE_LABEL + '" }}']
    try:
        engine = subprocess.run(['docker', 'info', '--format', '{{.ServerVersion}}'],
                                capture_output=True, text=True, timeout=10)
        if engine.returncode:
            print('[오류] Docker Desktop(Linux 엔진)을 켜고 다시 실행하세요.', flush=True)
            return False
        current = subprocess.run(inspect, capture_output=True, text=True, timeout=10)
        if current.returncode == 0 and current.stdout.strip() == expected:
            print('SLAM 이미지가 이 코드와 같습니다.', flush=True)
            return True
        print('바뀐 SLAM 이미지를 준비합니다. 처음 빌드는 몇 분 걸립니다.\n'
              '빌드가 끝날 때까지 인터넷을 유지하세요. 돌고 있는 컨테이너는 그대로입니다.', flush=True)
        code = subprocess.call(['docker', 'build', '--label', SLAM_SOURCE_LABEL + '=' + expected,
                                '-t', SLAM_IMAGE, str(context)], cwd=ROOT)
        if code:
            print('[오류] SLAM 빌드 실패. 옛 이미지로는 시작하지 않았습니다.', flush=True)
            return False
        current = subprocess.run(inspect, capture_output=True, text=True, timeout=10)
        if current.returncode or current.stdout.strip() != expected:
            print('[오류] 새 SLAM 이미지를 확인하지 못했습니다. Docker를 확인하고 다시 실행하세요.', flush=True)
            return False
        print('새 SLAM 이미지 준비됨.', flush=True)
        return True
    except (OSError, subprocess.TimeoutExpired) as error:
        print('[오류] Docker를 쓸 수 없습니다: ' + type(error).__name__ +
              '. Docker Desktop을 켜고 다시 실행하세요.', flush=True)
        return False


def saved_result(folder):
    for result in (folder, folder / 'reconstruction'):
        try:
            if (json.loads((result / 'status.json').read_text(encoding='utf-8'))['status']
                    in {'complete', 'partial'} and (result / 'scene.json').is_file()):
                return result
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return None


def ensure_gateway_environment(allow_install):
    if all(importlib.util.find_spec(name) is not None for name in ("fastapi", "uvicorn", "websockets")):
        return True
    if not allow_install:
        print(f'[오류] 게이트웨이 패키지가 없습니다. 인터넷에 연결하고 먼저 {PREPARE}를 실행하세요.')
        return False
    requirements = ROOT / 'modules/autonomy/tello_link/requirements.txt'
    return subprocess.call([sys.executable, '-B', '-m', 'pip', 'install', '--no-cache-dir',
                            '-r', str(requirements)], cwd=ROOT) == 0


@contextmanager
def managed_gateway(console_port, *, fake):
    import uvicorn
    from modules.autonomy.tello_link.gateway_server import DEFAULT_HOST, DEFAULT_LOG_ROOT, build_app

    host = '127.0.0.1'
    with socket.socket() as listener:
        for port in range(8010, 8030):
            if port == console_port:
                continue
            try:
                listener.bind((host, port))
                break
            except OSError:
                pass
        else:
            raise RuntimeError('비어 있는 게이트웨이 포트가 없습니다(8010–8029).')
        app = build_app(SimpleNamespace(fake=fake, no_video=False, no_record_video=True, drone_ip=DEFAULT_HOST,
            log_root=None if fake else DEFAULT_LOG_ROOT, allow_remote_autonomy=False,
            allow_remote_detections=False, mapping_console_port=console_port))
        server = uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level='warning',
                                               timeout_graceful_shutdown=3))
        thread = threading.Thread(target=server.run, kwargs={'sockets': [listener]}, name='mapping-gateway')
        thread.start()
        try:
            deadline = time.monotonic() + 10
            while not server.started:
                if not thread.is_alive() or time.monotonic() >= deadline:
                    raise RuntimeError('게이트웨이를 시작하지 못했습니다.')
                time.sleep(.05)
            yield f'http://{host}:{port}'
        finally:
            print('드론 게이트웨이를 닫습니다. 비행 중이면 착륙을 기다립니다.', flush=True)
            server.should_exit = True
            thread.join()


def main(argv=None):
    parser = argparse.ArgumentParser(prog='start.cmd map', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', nargs='?', type=Path, help='열어 볼 영상 파일이나 저장 결과 폴더(선택)')
    parser.add_argument('--port', type=int, default=8790, help='처음 시도할 지도 콘솔 포트(기본 8790)')
    source_mode = parser.add_mutually_exclusive_group()
    source_mode.add_argument('--gateway', help='이미 켜진 게이트웨이를 사용, 예: http://127.0.0.1:8010')
    source_mode.add_argument('--direct', action='store_true', help='촬영용 직접 연결(카메라만)')
    source_mode.add_argument('--fake', action='store_true', help='가짜 드론으로 통합 실행')
    parser.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않고 주소만 표시')
    parser.add_argument('--prepare-models', action='store_true', help='드론에 연결하기 전에 인터넷으로 패키지·모델 준비')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('--port는 1~65535')
    if args.input and (args.gateway or args.direct or args.fake):
        parser.error('저장 입력과 드론 실행 중 하나만 고르세요')
    combined = not (args.input or args.gateway or args.direct)
    caller = Path(os.environ.get('CHILL_CALLER') or os.getcwd())
    source = (caller / args.input).resolve() if args.input else None
    if not (ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        print('[오류] 지도 콘솔 웹 의존성을 먼저 설치하세요:\n  pnpm --dir apps/console install --frozen-lockfile')
        return 1
    if source is not None and not source.exists():
        print('[오류] 입력 파일이나 폴더가 없습니다.')
        return 1
    print(f'코드 폴더: {ROOT}', flush=True)
    if not ensure_python_environment(allow_install=args.prepare_models):
        return 1
    if combined and not ensure_gateway_environment(args.prepare_models):
        return 1
    sys.path.insert(0, str(ROOT))
    if args.prepare_models:
        return subprocess.call([sys.executable, '-B', '-m', 'modules.mapping.data.prepare_models', '--download'], cwd=ROOT)
    from modules.mapping.data.model_assets import require_local_models, ModelAssetsMissing
    try:
        require_local_models()
    except ModelAssetsMissing as error:
        print('[오류] ' + str(error), flush=True)
        return 1
    print('모델 파일이 모두 로컬에 있습니다. 지도는 가중치를 내려받지 않습니다.', flush=True)
    result = None
    if not args.input:
        if not combined and not ensure_slam_image():
            return 1
        if not args.fake:
            candidates = [saved_result(folder) for folder in (ROOT / 'output/phone_mapping').glob('*') if folder.is_dir()]
            result = max((path for path in candidates if path),
                         key=lambda path: (path / 'scene.json').stat().st_mtime, default=None)

    for port in range(args.port, min(args.port + 20, 65536)):
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', port))
            break
        except OSError:
            continue
    else:
        print('[오류] 비어 있는 로컬 포트가 없습니다. --port 8890 처럼 바꿔 다시 실행하세요.')
        return 1
    if port != args.port:
        print(f'{args.port} 포트를 쓰는 중이라 {port}를 씁니다. 켜져 있는 서버는 그대로 둡니다.')
    try:
        with (managed_gateway(port, fake=args.fake) if combined else nullcontext(args.gateway)) as gateway, \
                (TemporaryDirectory(prefix='mapping-fake-') if args.fake else nullcontext()) as scratch:
            return run_viewer(args, port, result, source, gateway, combined, scratch)
    except (OSError, RuntimeError) as error:
        print(f'[오류] 실시간 실행을 시작하지 못했습니다: {error} 다른 게이트웨이가 켜져 있으면 --gateway를 쓰세요.')
        return 1


def run_viewer(args, port, result, source, gateway, combined, scratch):
    command = [sys.executable, '-B', str(ROOT / 'apps/console/serve.py'), '--enable-camera', '--port', str(port)]
    if args.input:
        command += ['--import-source', str(source)]
    else:
        command += ['--tello-gateway', gateway] if gateway else ['--enable-tello-map']
    if result:
        command += ['--result', str(result)]
    if scratch:
        command += ['--live-root', str(Path(scratch) / 'live'), '--profiles-root', str(Path(scratch) / 'profiles')]
    url = f'http://127.0.0.1:{port}/'
    page = gateway + '/#/map' if combined else url + ('#saved' if args.input else '#live')
    print(f'화면: {page}\n이 창을 열어 두세요. Ctrl+C로 서버를 끕니다.', flush=True)
    if combined:
        print('지도 콘솔에서 영상을 켠 뒤 Tello 지도를 시작하세요. 비행은 운용자 확인이 필요합니다.')
        print('이 창을 닫으면 게이트웨이도 꺼집니다. 비행 중에는 닫지 마세요.')
    if not args.input:
        print('Tello 지도는 고른 드론 입력에 새 전방 ToF가 있으면 함께 씁니다.')
    process = subprocess.Popen(command, cwd=ROOT)
    opener = build_opener(ProxyHandler({}))
    try:
        deadline = time.monotonic() + 30
        while process.poll() is None:
            try:
                with opener.open(url + 'api/live', timeout=1) as response:
                    if json.load(response).get('enabled'):
                        break
            except (OSError, URLError, ValueError):
                pass
            if time.monotonic() >= deadline:
                print('[오류] 서버 시작 시간이 지났습니다. 위 메시지를 확인하세요.')
                return 1
            time.sleep(.2)
        if process.poll() is not None:
            print('[오류] 서버를 시작하지 못했습니다. 위 메시지를 확인하세요.')
            return process.returncode or 1
        if not args.no_browser and not webbrowser.open(page):
            print('위 화면 주소를 브라우저에서 여세요.')
        try:
            return process.wait()
        except KeyboardInterrupt:
            # 서버도 같은 콘솔이라 Ctrl+C를 함께 받는다. 정리할 시간을 준다.
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
            return 0
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
