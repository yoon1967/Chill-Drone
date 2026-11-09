"""Chill-Drone 실행기 메뉴와 명령줄. scripts/start.cmd가 `python -m apps.launcher`로 부른다.

메뉴는 두 개다: 1 체험(가짜 드론), 2 실기체 관제. 고른 쪽의 기능을 묻지 않고 모두 켠다.
"""
import argparse
import sys

from . import drone

MENU = [
    ('1', '체험', '가짜 드론으로 관제 화면·사람 탐지·지도(드론 없이)'),
    ('2', '실기체 관제', '게이트웨이·사람 탐지·3D 지도·MoNA 자율을 모두 켬'),
]


def build_parser():
    parser = argparse.ArgumentParser(prog='start.cmd', description='Chill-Drone 실행기. 인자 없이 실행하면 메뉴가 뜬다.')
    sub = parser.add_subparsers(dest='command', metavar='명령')
    fake = sub.add_parser('fake', help='1 체험: 가짜 드론으로 관제 화면(모두 켬)')
    fake.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
    real = sub.add_parser('real', help='2 실기체 관제(모두 켬)')
    real.add_argument('--phone', action='store_true', help='폰에서도 열리게(같은 망 안에서만, 방화벽 규칙 필요)')
    real.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
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
        return drone.run_fake()
    if choice == '2':
        return drone.run_real()
    print('1이나 2를 고르세요.')
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    try:
        if not argv:   # 더블클릭: 메뉴
            return menu()
        args = build_parser().parse_args(argv)
        if args.command == 'fake':
            return drone.run_fake(no_browser=args.no_browser)
        if args.command == 'real':
            return drone.run_real(phone=args.phone, no_browser=args.no_browser)
        build_parser().print_help()
        return 0
    except KeyboardInterrupt:
        print('\n실행기를 멈췄어요. 켠 프로그램은 각자 창에서 계속 돌아요.')
        return 0
