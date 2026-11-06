@echo off
setlocal DisableDelayedExpansion
set "MAPPING_CALLER=%CD%"
pushd "%~dp0.." || exit /b 1
call "%~dp0resolve_project_python.cmd"
if errorlevel 1 (
    pause
    popd
    exit /b 1
)
set "MAPPING_LAUNCHER=%~f0"
"%MAPPING_PROJECT_PYTHON%" -B -c "import os,pathlib; p=pathlib.Path(os.environ['MAPPING_LAUNCHER']); exec(compile(p.read_text(encoding='utf-8').rsplit('# === PYTHON ===',1)[1],str(p),'exec'))" %*
set "MAPPING_EXIT=%ERRORLEVEL%"
if not "%MAPPING_EXIT%"=="0" pause
popd
exit /b %MAPPING_EXIT%
# === PYTHON ===
"""Double-click for live mapping; drop saved data or input to inspect it in the console."""
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

ROOT = Path(os.environ['MAPPING_LAUNCHER']).resolve().parent.parent
SLAM_IMAGE = 'ssafy-orbslam3:local'
SLAM_SOURCE_LABEL = 'org.ssafy.mapping.source-sha256'


def ensure_python_environment(*, allow_install=False):
    required = ['cv2', 'numpy', 'PIL', 'av', 'torch', 'transformers', 'ultralytics']
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    try:
        if missing:
            if not allow_install:
                print('[ERROR] Missing mapping packages: ' + ', '.join(missing) +
                      '. Connect internet and run this CMD with --prepare-models first.', flush=True)
                return False
            print('Preparing missing mapping packages: ' + ', '.join(missing) + '\n'
                  'Keep internet available. The first download can take several minutes.', flush=True)
            if importlib.util.find_spec('pip') is None:
                if subprocess.call([sys.executable, '-B', '-m', 'ensurepip'], cwd=ROOT):
                    print('[ERROR] Could not prepare pip. Check the messages above and retry this CMD.', flush=True)
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
                print('Installing PyTorch for ' + ('NVIDIA CUDA' if index.endswith('cu128') else 'CPU') + '.', flush=True)
                if subprocess.call(pip + ['torch>=2.6,<3', '--index-url', index], cwd=ROOT):
                    print('[ERROR] PyTorch installation failed. Check internet and disk space, then retry this CMD.', flush=True)
                    return False
            requirements = ROOT / 'modules/mapping/data/requirements-stream.txt'
            if subprocess.call(pip + ['-r', str(requirements)], cwd=ROOT):
                print('[ERROR] Mapping package installation failed. Check the messages above and retry this CMD.', flush=True)
                return False
            importlib.invalidate_caches()
        print('Mapping packages are ready. Models load from local files when reconstruction starts; video waits until loading finishes.', flush=True)
        return True
    except OSError as error:
        print('[ERROR] Mapping setup failed: ' + str(error) + '. Retry this CMD after fixing the error.', flush=True)
        return False


def slam_source_hash(context):
    """Version local image inputs; Python/web changes are read on every launch."""
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
            print('[ERROR] Start Docker Desktop (Linux engine), then run this CMD again.', flush=True)
            return False
        current = subprocess.run(inspect, capture_output=True, text=True, timeout=10)
        if current.returncode == 0 and current.stdout.strip() == expected:
            print('SLAM backend matches this checkout.', flush=True)
            return True
        print('Preparing the updated SLAM backend. The first build can take several minutes.\n'
              'Keep internet available until the build finishes; existing containers stay running.', flush=True)
        code = subprocess.call(['docker', 'build', '--label', SLAM_SOURCE_LABEL + '=' + expected,
                                '-t', SLAM_IMAGE, str(context)], cwd=ROOT)
        if code:
            print('[ERROR] SLAM build failed. The site was not started with an outdated backend.', flush=True)
            return False
        current = subprocess.run(inspect, capture_output=True, text=True, timeout=10)
        if current.returncode or current.stdout.strip() != expected:
            print('[ERROR] Could not verify the updated SLAM image. Retry after checking Docker.', flush=True)
            return False
        print('Updated SLAM backend is ready.', flush=True)
        return True
    except (OSError, subprocess.TimeoutExpired) as error:
        print('[ERROR] Docker is unavailable: ' + type(error).__name__ +
              '. Start Docker Desktop, then run this CMD again.', flush=True)
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
        print('[ERROR] Gateway packages are missing. With internet, run this CMD with --prepare-models first.')
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
            raise RuntimeError('No free local gateway port (8010–8029).')
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
                    raise RuntimeError('Gateway startup failed.')
                time.sleep(.05)
            yield f'http://{host}:{port}'
        finally:
            print('Closing the local drone gateway. Waiting for flight shutdown if needed.', flush=True)
            server.should_exit = True
            thread.join()


def main():
    parser = argparse.ArgumentParser(prog='start_3d_mapping.cmd', description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, help='Optional video file or saved result folder')
    parser.add_argument('--port', type=int, default=8790, help='First local port to try (default: 8790)')
    source_mode = parser.add_mutually_exclusive_group()
    source_mode.add_argument('--gateway', help='Use a running Tello gateway, e.g. http://127.0.0.1:8010')
    source_mode.add_argument('--direct', action='store_true', help='Open the camera-only direct Tello input')
    source_mode.add_argument('--fake', action='store_true', help='Start the combined viewer with a simulated Tello')
    parser.add_argument('--no-browser', action='store_true', help='Print the page URL without opening a browser')
    parser.add_argument('--prepare-models', action='store_true', help='Prepare packages and model weights with internet before connecting to the drone')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if args.input and (args.gateway or args.direct or args.fake):
        parser.error('Choose either a saved input or a live drone mode')
    combined = not (args.input or args.gateway or args.direct)
    source = (Path(os.environ['MAPPING_CALLER']) / args.input).resolve() if args.input else None
    if not (ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        print('[ERROR] Install the local web dependencies first:\n  pnpm --dir apps/console install --frozen-lockfile')
        return 1
    if source is not None and not source.exists():
        print('[ERROR] The input file or folder does not exist.')
        return 1
    print(f'Code folder: {ROOT}', flush=True)
    if not ensure_python_environment(allow_install=args.prepare_models):
        return 1
    if combined and not ensure_gateway_environment(args.prepare_models):
        return 1
    sys.path.insert(0, str(ROOT))
    if args.prepare_models:
        return subprocess.call([sys.executable, '-B', '-m', 'modules.mapping.data.prepare_models', '--download'], cwd=ROOT)
    from modules.mapping.data.model_assets import require_local_models, ModelAssetsMissing
    try:
        assets = require_local_models()
    except ModelAssetsMissing as error:
        print('[ERROR] ' + str(error), flush=True)
        return 1
    print('All model files are available locally. Mapping does not download weights.', flush=True)
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
        print('[ERROR] No free local port. Retry with --port 8890.')
        return 1
    if port != args.port:
        print(f'Port {args.port} is busy. Using {port}; existing servers stay running.')
    try:
        with (managed_gateway(port, fake=args.fake) if combined else nullcontext(args.gateway)) as gateway, \
                (TemporaryDirectory(prefix='mapping-fake-') if args.fake else nullcontext()) as scratch:
            return run_viewer(args, port, result, source, gateway, combined, scratch)
    except (OSError, RuntimeError) as error:
        print(f'[ERROR] Live setup could not start: {error}. If another gateway is running, use --gateway.')
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
    print(f'Page: {page}\nKeep this window open. Press Ctrl+C to stop this server.', flush=True)
    if combined:
        print('Turn video on in the map console, then start Tello mapping. Flight requires operator confirmation.')
    if not args.input:
        print('Tello mapping uses fresh front ToF from the selected drone input when available.')
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
                print('[ERROR] Server startup timed out. Check the messages above.')
                return 1
            time.sleep(.2)
        if process.poll() is not None:
            print('[ERROR] Server could not start. Check the messages above.')
            return process.returncode or 1
        if not args.no_browser and not webbrowser.open(page):
            print('Open the page URL above in your browser.')
        try:
            return process.wait()
        except KeyboardInterrupt:
            # The server shares this console and receives Ctrl+C too; let it clean up.
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


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nMapping launcher stopped.')
        sys.exit(0)
