"""Chill-Drone 실행기 메뉴와 명령줄. scripts/start.cmd가 `python -m apps.launcher`로 부른다.

메뉴는 두 개다: 1 체험(가짜 드론), 2 실기체 관제.
"""
import argparse
import subprocess
import sys

from . import drone, prepare

MENU = [
    ('1', '체험', '가짜 드론으로 관제 화면·사람 탐지·지도(드론 없이)'),
    ('2', '실기체 관제', '게이트웨이·사람 탐지·3D 지도'),
]


def build_parser():
    parser = argparse.ArgumentParser(prog='start.cmd', description='Chill-Drone 실행기. 인자 없이 실행하면 메뉴가 뜬다.')
    sub = parser.add_subparsers(dest='command', metavar='명령')
    fake = sub.add_parser('fake', help='1 체험: 가짜 드론으로 관제 화면(모두 켬)')
    fake.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
    real = sub.add_parser('real', help='2 실기체 관제')
    real.add_argument('--phone', action='store_true', help='폰에서도 열리게(같은 망 안에서만, 방화벽 규칙 필요)')
    real.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
    sub.add_parser('prepare-flight', help='실시간 지도·사람 탐지 모델과 이미지를 준비')
    return parser


def menu():
    print('Chill-Drone 실행기', flush=True)
    drone.show_code()
    print(f'Python: {sys.executable}\n')
    for key, name, note in MENU:
        print(f'  {key}) {name:<10} {note}')
    choice = input('\n번호를 고르세요: ').strip()
    print()
    if choice == '1':
        return ['fake']
    if choice == '2':
        return ['real']
    print('1이나 2를 고르세요.')
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    try:
        if not argv:
            argv = menu()
            if not argv:
                return 0
        args = build_parser().parse_args(argv)
        if not prepare.check_host(docker=args.command == 'prepare-flight'):
            return 1
        if sys.prefix == sys.base_prefix:
            try:
                python = prepare.create_project_environment()
                if python is None:
                    return 1
                with subprocess.Popen([str(python), '-B', '-m', 'apps.launcher', *argv], cwd=prepare.ROOT) as launcher:
                    while True:
                        try:
                            return launcher.wait()
                        except KeyboardInterrupt:
                            # The launcher shares this console and handles Ctrl+C itself.
                            continue
            except OSError as error:
                print(f'[오류] 가상환경을 준비하지 못했습니다. {error}\n'
                      '위 메시지를 확인한 후 scripts/start.cmd를 다시 실행해 주세요.', flush=True)
                return 1
        if args.command == 'fake':
            return drone.run_fake(no_browser=args.no_browser)
        if args.command == 'prepare-flight':
            return 0 if prepare.prepare_flight() else 1
        return drone.run_real(phone=args.phone, no_browser=args.no_browser)
    except KeyboardInterrupt:
        print('\n실행을 중단했습니다. 이미 시작한 서비스를 종료하려면 실행기를 다시 열고 Enter를 누르세요.')
        return 0
