@echo off
setlocal DisableDelayedExpansion
pushd "%~dp0.." || exit /b 1
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Project Python is missing: .venv\Scripts\python.exe
    echo Create it with: py -3 -m venv .venv
    pause
    popd
    exit /b 1
)
set "AUTONOMY_LAUNCHER=%~f0"
".venv\Scripts\python.exe" -B -c "import os,pathlib; p=pathlib.Path(os.environ['AUTONOMY_LAUNCHER']); exec(compile(p.read_text(encoding='utf-8').rsplit('# === PYTHON ===',1)[1],str(p),'exec'))" %*
set "AUTONOMY_EXIT=%ERRORLEVEL%"
if not "%AUTONOMY_EXIT%"=="0" pause
popd
exit /b %AUTONOMY_EXIT%
# === PYTHON ===
"""Open the autonomous exploration controller (sample data)."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import sys
import webbrowser


class ControllerServer(ThreadingHTTPServer):
    allow_reuse_address = False


def main():
    parser = argparse.ArgumentParser(prog='start_autonomy.cmd', description=__doc__)
    parser.add_argument('--port', type=int, default=8791, help='Local port (default: 8791)')
    parser.add_argument('--no-browser', action='store_true', help='Print URL without opening a browser')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    root = Path(os.environ['AUTONOMY_LAUNCHER']).resolve().parent.parent
    directory = root / 'apps/controller'
    if not (directory / 'index.html').is_file():
        print('[ERROR] Controller page is missing: apps/controller/index.html')
        return 1
    handler = partial(SimpleHTTPRequestHandler, directory=str(directory))
    try:
        server = ControllerServer(('127.0.0.1', args.port), handler)
    except OSError as error:
        print(f'[ERROR] Cannot open port {args.port}: {error}')
        print('Try another port: scripts\\start_autonomy.cmd --port 8891')
        return 1
    with server:
        url = f'http://127.0.0.1:{server.server_port}/#/live'
        print(f'Page: {url}\nSample data only; drone commands are not connected.', flush=True)
        print('Keep this window open. Press Ctrl+C to stop.', flush=True)
        if not args.no_browser:
            try:
                if not webbrowser.open(url):
                    print('Open the page URL above in your browser.')
            except webbrowser.Error:
                print('Open the page URL above in your browser.')
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print('\nController server stopped.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
