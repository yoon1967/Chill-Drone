@echo off
setlocal DisableDelayedExpansion
set "MAPPING_CALLER=%CD%"
pushd "%~dp0.." || exit /b 1
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Project Python is missing: .venv\Scripts\python.exe
    echo Create the Python 3.12 environment described in apps\console\README.md.
    pause
    popd
    exit /b 1
)
set "MAPPING_LAUNCHER=%~f0"
".venv\Scripts\python.exe" -B -c "import os,pathlib; p=pathlib.Path(os.environ['MAPPING_LAUNCHER']); exec(compile(p.read_text(encoding='utf-8').rsplit('# === PYTHON ===',1)[1],str(p),'exec'))" %*
set "MAPPING_EXIT=%ERRORLEVEL%"
if not "%MAPPING_EXIT%"=="0" pause
popd
exit /b %MAPPING_EXIT%
# === PYTHON ===
"""Double-click for live mapping; drop a video to reconstruct and view its result."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import uuid
import webbrowser

ROOT = Path(os.environ['MAPPING_LAUNCHER']).resolve().parent.parent


def saved_result(folder):
    for result in (folder, folder / 'reconstruction'):
        try:
            if (json.loads((result / 'status.json').read_text(encoding='utf-8'))['status']
                    in {'complete', 'partial'} and (result / 'scene.json').is_file()):
                return result
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return None


def main():
    parser = argparse.ArgumentParser(prog='start_3d_mapping.cmd', description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, help='Optional video file or saved result folder')
    parser.add_argument('--port', type=int, default=8790, help='First local port to try (default: 8790)')
    parser.add_argument('--gateway', help='Optional running Tello control gateway, e.g. http://127.0.0.1:8010')
    parser.add_argument('--no-browser', action='store_true', help='Print the page URL without opening a browser')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    source = (Path(os.environ['MAPPING_CALLER']) / args.input).resolve() if args.input else None
    if not (ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        print('[ERROR] Install the local web dependencies first:\n  pnpm --dir apps/console install --frozen-lockfile')
        return 1
    required = ['cv2', 'numpy', 'PIL', 'av']
    if source and source.is_file():
        required += ['pycolmap', 'scipy', 'matplotlib']
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    if missing:
        print('[ERROR] Missing Python dependencies: ' + ', '.join(missing))
        print('  .venv\\Scripts\\python.exe -m pip install -r modules/mapping/data/requirements-pipeline.txt')
        print('  .venv\\Scripts\\python.exe -m pip install -r modules/mapping/localization/requirements-online.txt av Pillow')
        return 1

    result = None
    if args.input:
        if source.is_dir():
            result = saved_result(source)
            if result is None:
                print('[ERROR] Select a complete or partial saved reconstruction folder.')
                return 1
        elif source.is_file():
            run_id = 'video_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:8]
            output = ROOT / 'output/phone_mapping' / run_id
            print('Video -> frames -> SIFT/matching -> 3D reconstruction -> validation/PNG', flush=True)
            code = subprocess.call([sys.executable, '-B', str(ROOT / 'modules/mapping/data/run_video_pipeline.py'),
                                    '--video', str(source), '--output', str(output), '--run-id', run_id], cwd=ROOT)
            result = saved_result(output)
            if code not in (0, 3) or result is None:
                print(f'[ERROR] Pipeline failed. Preserved logs: {output / "logs"}')
                return code if code else 1
            if code == 3:
                print('[PARTIAL] Only some frames were reconstructed. See coverage in the viewer.')
        else:
            print('[ERROR] The video or result folder does not exist.')
            return 1
    else:
        # Reuse an existing saved map alongside live mapping, without changing it.
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
    command = [sys.executable, '-B', str(ROOT / 'apps/console/serve.py'), '--enable-camera', '--port', str(port)]
    command += ['--tello-gateway', args.gateway] if args.gateway else ['--enable-tello-map']
    if result:
        command += ['--result', str(result)]
    url = f'http://127.0.0.1:{port}/'
    page = url + ('#saved' if args.input else '#live')
    print(f'Page: {page}\nKeep this window open. Press Ctrl+C to stop this server.', flush=True)
    print('Live 3D mapping requires running Docker Desktop and the ssafy-orbslam3:local image.')
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


try:
    sys.exit(main())
except KeyboardInterrupt:
    print('\nMapping launcher stopped.')
    sys.exit(0)
