"""Chill-Drone 실행기 명령줄과 메뉴. scripts/start.cmd가 `python -m apps.launcher`로 부른다."""
import argparse
import shutil
import subprocess
import sys

from . import drone, mapping

TEAM_AUTONOMY = drone.ROOT / 'infra/autonomy-web/launcher.py'
MENU = [
    ('1', '체험', '가짜 드론으로 관제 화면·사람 탐지·지도 보기(드론 없이)'),
    ('2', '실기체 관제', '드론 Wi-Fi 연결 뒤. 사람 탐지·3D 지도·MoNA 자율은 골라서'),
    ('3', '3D 지도 촬영', '지도 파트 통합 실행(지도 콘솔). 가짜 드론·직접 연결도'),
    ('4', '저장 지도·영상 열기', '지도 콘솔에서 보기'),
    ('5', '자율비행 웹(ROS 2)', '허병우 팀원 웹 서버. Docker 또는 WSL'),
    ('6', '준비', '패키지·모델 받기(인터넷 필요, 처음 한 번)'),
    ('0', '모두 끄기', '실기체가 공중이면 끄지 않음'),
]
EPILOG = """명령:
  fake            체험(가짜 드론): 게이트웨이 8030 + 사람 탐지 + 가짜 지도
  real            실기체 관제: 게이트웨이 8010 + 고른 보조 프로그램(--detect --map --mona)
  map [...]       3D 지도 촬영·저장 지도 열기(start.cmd map --help)
  team-autonomy docker|wsl [...]   자율비행 웹(ROS 2)
  prepare         패키지·모델 준비(인터넷 필요)
  stop            모두 끄기(실기체가 공중이면 끄지 않음)
인자 없이 실행하면 메뉴가 뜬다."""


def team_autonomy(rest):
    return subprocess.call([sys.executable, '-B', str(TEAM_AUTONOMY), *rest], cwd=drone.ROOT)


def ensure_console_web():
    """지도 콘솔 웹 의존성(apps/console/node_modules). 없으면 pnpm으로 설치한다."""
    if (drone.ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        return True
    pnpm = shutil.which('pnpm')
    if not pnpm:
        print('[오류] 지도 콘솔 웹 의존성이 없고 pnpm도 없습니다. Node.js와 pnpm을 설치한 뒤 다시 실행하세요.')
        return False
    print('지도 콘솔 웹 의존성을 설치합니다(pnpm)...', flush=True)
    if subprocess.call([pnpm, '--dir', 'apps/console', 'install', '--frozen-lockfile'], cwd=drone.ROOT):
        print('[오류] 웹 의존성 설치에 실패했습니다. 위 메시지를 확인하고 다시 실행하세요.')
        return False
    return True


def prepare():
    """처음 한 번(인터넷 필요): 지도 콘솔 웹 의존성 → 지도·사람 탐지·게이트웨이 패키지 → 모델 파일."""
    if not ensure_console_web():
        return 1
    return mapping.main(['--prepare-models'])


def build_parser():
    parser = argparse.ArgumentParser(prog='start.cmd', description='Chill-Drone 실행기', epilog=EPILOG,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', metavar='명령')
    fake = sub.add_parser('fake', help='체험: 가짜 드론으로 관제 화면')
    fake.add_argument('--no-detect', action='store_true', help='사람 탐지를 켜지 않음')
    fake.add_argument('--no-map', action='store_true', help='가짜 지도를 켜지 않음')
    fake.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
    real = sub.add_parser('real', help='실기체 관제(드론 Wi-Fi 연결 뒤)')
    real.add_argument('--phone', action='store_true', help='폰에서도 열리게(같은 망 안에서만, 방화벽 규칙 필요)')
    real.add_argument('--detect', action=argparse.BooleanOptionalAction, default=None, help='사람 탐지')
    real.add_argument('--map', action=argparse.BooleanOptionalAction, default=None, help='3D 지도(지도 콘솔 + 연결기)')
    real.add_argument('--mona', choices=['no', 'hand', 'flight'], default=None,
                      help='MoNA 자율: hand = 손에 든 시험(-AutoStart), flight = 비행')
    real.add_argument('--no-browser', action='store_true', help='브라우저를 열지 않음')
    sub.add_parser('prepare', help='패키지·모델 준비(인터넷 필요)')
    stop = sub.add_parser('stop', help='모두 끄기')
    stop.add_argument('--yes', action='store_true', help='드론이 연결돼 있어도 묻지 않고 끔(공중이면 그래도 안 끔)')
    return parser


def run_real_args(args):
    given = any(v is not None for v in (args.detect, args.map, args.mona))
    return drone.run_real(phone=args.phone, detect=args.detect, map_=args.map,
                          mona=None if args.mona == 'no' else args.mona,
                          no_browser=args.no_browser, interactive=not given and sys.stdin.isatty())


def menu():
    print('Chill-Drone 실행기', flush=True)
    drone.show_code()
    print(f'Python: {sys.executable}\n')
    for key, name, note in MENU:
        print(f'  {key}) {name:<12} {note}')
    choice = input('\n번호를 고르세요: ').strip()
    print()
    if choice == '1':
        return drone.run_fake()
    if choice == '2':
        return drone.run_real(interactive=True)
    if choice == '3':
        kind = drone.ask('[Enter = 드론 통합 실행 / f = 가짜 드론 / d = 직접 연결(촬영용)]', '', {'f', 'd'})
        return mapping.main({'': [], 'f': ['--fake'], 'd': ['--direct']}[kind])
    if choice == '4':
        path = input('       영상 파일이나 저장 결과 폴더 경로(창에 끌어다 놓아도 됨): ').strip().strip('"')
        return mapping.main([path]) if path else 0
    if choice == '5':
        kind = drone.ask('[Enter = Docker / w = WSL]', '', {'w'})
        return team_autonomy(['wsl' if kind == 'w' else 'docker'])
    if choice == '6':
        return prepare()
    if choice == '0':
        return drone.run_stop()
    print('메뉴에 없는 번호예요.')
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    try:
        if not argv:   # 더블클릭: 메뉴. 끝나면 결과를 읽을 수 있게 기다린다(오류면 start.cmd가 멈춰 준다)
            code = menu()
            if code == 0:
                try:
                    input('\nEnter를 누르면 이 창을 닫아요.')
                except EOFError:
                    pass
            return code
        if argv[0] == 'map':          # 지도 촬영 옵션은 mapping.main이 그대로 받는다
            return mapping.main(argv[1:])
        if argv[0] == 'team-autonomy':
            return team_autonomy(argv[1:])
        args = build_parser().parse_args(argv)
        if args.command == 'fake':
            return drone.run_fake(detect=not args.no_detect, map_=not args.no_map, no_browser=args.no_browser)
        if args.command == 'real':
            return run_real_args(args)
        if args.command == 'prepare':
            return prepare()
        if args.command == 'stop':
            return drone.run_stop(yes=args.yes)
        build_parser().print_help()
        return 0
    except KeyboardInterrupt:
        print('\n실행기를 멈췄어요.')
        return 0
