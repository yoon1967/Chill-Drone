"""Run the teammate's ROS 2 autonomy web node using Docker or WSL."""
import argparse
import json
import socket
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from urllib.request import ProxyHandler, build_opener
import uuid
import webbrowser

ROOT = Path(__file__).resolve().parents[2]
SOURCE = 'modules/autonomy/autonomy_web/'
IMAGE = 'ssafy-autonomy-web:jazzy'


def run(command, **kwargs):
    return subprocess.run(command, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['docker', 'wsl'])
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--domain-id', type=int, default=0)
    parser.add_argument('--distro', default='Ubuntu-24.04')
    parser.add_argument('--ref', default='origin/feature/tello-ros-driver')
    parser.add_argument('--host-network', action='store_true', help='Docker Desktop host networking must be enabled')
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    if args.host_network and args.mode != 'docker':
        parser.error('--host-network is a Docker option')
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', args.port))
        except OSError:
            raise RuntimeError(f'Port {args.port} is busy. Use --port with a free port.')
    process = None
    container = 'ssafy-autonomy-web-' + uuid.uuid4().hex[:12]
    with tempfile.TemporaryDirectory(prefix='ssafy-autonomy-web-') as temporary:
        directory = Path(temporary)
        for relative in ('autonomy_web.py', 'static/index.html'):
            content = run(['git', 'show', f'{args.ref}:{SOURCE}{relative}'], cwd=ROOT, capture_output=True).stdout
            target = directory / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        ros_args = ['--ros-args', '-p', f'port:={args.port}', '-p',
                    'bind:=' + ('127.0.0.1' if args.mode == 'wsl' or args.host_network else '0.0.0.0')]
        if args.mode == 'docker':
            run(['docker', 'info', '--format', '{{.ServerVersion}}'], timeout=15)
            run(['docker', 'build', '-t', IMAGE, str(ROOT / 'infra/autonomy-web')])
            network = ['--network', 'host'] if args.host_network else ['-p', f'127.0.0.1:{args.port}:{args.port}']
            command = ['docker', 'run', '--rm', '--init', '--name', container, *network,
                       '-e', f'ROS_DOMAIN_ID={args.domain_id}',
                       '--mount', f'type=bind,source={directory},target=/app,readonly',
                       IMAGE, 'python3', '-B', 'autonomy_web.py', *ros_args]
        else:
            prefix = ['wsl', '-d', args.distro, '--']
            try:
                linux_path = run([*prefix, 'wslpath', '-a', '-u', str(directory)],
                                 capture_output=True, timeout=15).stdout.decode('utf-8').strip()
            except subprocess.CalledProcessError:
                raise RuntimeError(f'Cannot access WSL distro {args.distro}. Check wsl --list --verbose; see docs/autonomy/WEB_LAUNCHERS.md.') from None
            # Paths and options are positional parameters, never interpolated into shell code.
            script = ('set -e; test -f /opt/ros/jazzy/setup.bash || '
                      '{ echo "ROS 2 Jazzy missing; see docs/autonomy/WEB_LAUNCHERS.md" >&2; exit 1; }; '
                      'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID="$1"; '
                      'shift; printf "%s" "$$" > "$1"; shift; exec python3 -B "$@"')
            command = [*prefix, 'bash', '-c', script, 'autonomy-web', str(args.domain_id),
                       linux_path + '/node.pid', linux_path + '/autonomy_web.py', *ros_args]
        print(f'Starting {args.mode} ROS 2 web node from {args.ref}.', flush=True)
        print('The flight driver, mission, map and camera must be started separately.', flush=True)
        print('Keep this window open; Ctrl+C stops this web server.', flush=True)
        try:
            process = subprocess.Popen(command)
            url = f'http://localhost:{args.port}'
            opener = build_opener(ProxyHandler({}))
            deadline = time.monotonic() + 60
            while process.poll() is None:
                try:
                    with opener.open(url + '/api/status', timeout=1) as response:
                        status = json.load(response)
                    if isinstance(status, dict) and 'mission' in status and 'camera' in status:
                        break
                except (OSError, ValueError):
                    pass
                if time.monotonic() >= deadline:
                    raise RuntimeError('Web server startup timed out. Check port, ROS dependencies and logs above.')
                time.sleep(.2)
            if process.poll() is not None:
                return process.returncode or 1
            print('Page: ' + url, flush=True)
            if not args.no_browser:
                try:
                    webbrowser.open(url)
                except webbrowser.Error:
                    print('Open the page URL above in your browser.')
            return process.wait()
        except KeyboardInterrupt:
            return 0
        finally:
            if args.mode == 'wsl' and process is not None:
                pid_file = directory / 'node.pid'
                if pid_file.is_file():
                    pid = pid_file.read_text().strip()
                    if pid.isdecimal() and process.poll() is None:
                        subprocess.run([*prefix, 'kill', '-INT', pid], capture_output=True, timeout=10)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            subprocess.run([*prefix, 'kill', '-TERM', pid], capture_output=True, timeout=10)
            if args.mode == 'docker' and process is not None:
                # Stop only the unique container created by this invocation.
                subprocess.run(['docker', 'stop', '-t', '5', container], capture_output=True, timeout=15)
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print('[ERROR] ' + str(error), file=sys.stderr)
        print('See docs/autonomy/WEB_LAUNCHERS.md for setup. Fetch the teammate branch if source is missing.', file=sys.stderr)
        sys.exit(1)
