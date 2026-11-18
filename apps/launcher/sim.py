"""자율 비행(ROS 2)을 관제 화면에 붙여 한 번에 켠다: 시뮬레이터 `start.cmd sim`, 실기체 `start.cmd autonomy`.

autonomy(실기체): 자율 비행 README(modules/autonomy/tello_driver)의 실기체 순서 그대로 Windows 중계(tello_relay.py, WSL 기본
NAT 네트워크용) → WSL real.launch.py(드라이버·위치·깊이·YOLO·비행 노드·웹 8080)를 켜고, 보기 노드·게이트웨이 --ros-view·
사람 탐지·3D 지도·관제 화면은 sim과 같다. 확장 키트(RMTT-…)가 아니면 전방 ToF를 끈다(front_tof:=false, 자율 비행 README).
--fake-tello면 드론 없이 Windows 가짜 Tello(127.0.0.1:18889)로 같은 길을 끝까지 시험한다. 끄기는 땅에서만(Ctrl+C면 드라이버가 착륙).

sim(시뮬레이터): 메뉴(1·2)에는 없고 명령으로만 켠다. 자율 비행 쪽 코드·launch·기준은 그대로다(자율 비행 README대로 켠다). 묻지 않고 다 켠다:
  1. WSL 시뮬레이터 사본(~/ttrepo)을 이 저장소의 지금 커밋으로 맞춘다(커밋 안 한 변경은 들어가지 않는다)
  2. 자율 비행 시뮬레이터: mission.launch.py(세계 s7_stage3, Gazebo 화면 없이, 자율 비행 웹 8080, YOLO 탐지)
     지난 시연의 찾은 사람·장애물 파일은 시각을 붙여 옮겨 둔다(시뮬레이션은 세계 폴더의 같은 파일을 다시 쓴다)
  3. 보기 노드(ros_view_node.py, WSL 127.0.0.1:8094): 관제 화면 버튼을 자율 비행 웹과 같은 명령으로 낸다
  4. 게이트웨이 --ros-view(8020) + 사람 탐지 + 지도 콘솔(8797)·지도 연결기 + 지도 자동 시작(mapkeeper)
  5. 관제 화면 ?preset=autonomy#/live
끄기: 실행기 창에서 Enter(모두 끄기). 시뮬레이터는 이 실행기가 켠 것만, mission.launch.py·gz sim·보기 노드만 끈다
(실기체용 real.launch.py는 건드리지 않는다).
처음 한 번: WSL Ubuntu-24.04와 ROS 2 Jazzy·자율 비행 환경이 있어야 한다. 없으면 설치 명령을 알려 준다(wsl_sim_setup.sh).
"""
import os
import re
import subprocess
import sys
import time

from . import drone, mapkeeper, winproc

ROOT = drone.ROOT
DISTRO = os.environ.get('CHILL_WSL_DISTRO', 'Ubuntu-24.04')
SIM_PORT = 8020            # 시뮬레이터 보기 게이트웨이(실기체 8010·체험 8030과 겹치지 않게)
SIM_CONSOLE_PORT = 8797    # 시뮬레이터 지도 콘솔(실기체 8790·체험 영상 복원 8796과 겹치지 않게)
VIEW_URL = 'http://127.0.0.1:8094'      # 보기 노드(WSL 127.0.0.1, Windows에서 localhost 넘겨주기로 붙음)
AUTONOMY_WEB = 'http://127.0.0.1:8080'  # 자율 비행 웹(같은 비행을 자율 비행 화면에서도)
WSL_REPO = '~/ttrepo'
TT = WSL_REPO + '/simulation/tt-sim'
WORLD = 's7_stage3'        # generate_world.py --seed 7 --stage 3: 사람 찾기 단계
ROS = 'source /opt/ros/jazzy/setup.bash'   # ROS setup은 set -u와 같이 쓰면 멈춘다
SETUP = ROOT / 'apps/launcher/wsl_sim_setup.sh'
SIM_MARK, VIEW_MARK = 'mission.launch.py', 'ros_view_node.py'   # 실행기가 띄운 wsl.exe 명령줄에서 찾는 글자
STOP_SCRIPT = (   # 시뮬레이터만: 실기체용 real.launch.py는 건드리지 않는다. [x] 꼴은 pgrep·pkill이 자기 명령줄을 잡지 않게
    "pkill -INT -f '[r]os_view_node.py'; pkill -INT -f '[m]ission.launch.py'; "
    "for i in $(seq 1 20); do pgrep -f '[m]ission.launch.py|[r]os_view_node.py' >/dev/null || break; sleep 1; done; "
    "pkill -TERM -f '[g]z sim'; sleep 2; pkill -KILL -f '[g]z sim'; "
    "pgrep -f '[m]ission.launch.py|[r]os_view_node.py|[g]z sim' >/dev/null && echo left || echo stopped")
# 실기체 자율 비행(autonomy)
REAL_MARK = 'real.launch.py'
RELAY = 'modules/autonomy/tello_driver/tello_relay.py'
RELAY_MARK = 'tello_relay.py'
RELAY_UDP = (8889, 8890, 11111, 9889)   # 중계가 Windows에서 여는 UDP: Tello로 명령 보냄·상태·영상·WSL 드라이버 명령
FAKE_TELLO_ARGS = ['--tello', '127.0.0.1', '--tello-cmd-port', '18889', '--state-port', '18890', '--video-port', '11112']
# 자율 비행 launch에 그대로 넘길 인자(예: search_time:=150). WSL bash 명령줄에 붙으므로 이름:=값 꼴과 안전한 글자만 받는다
LAUNCH_ARG = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*:=[A-Za-z0-9_.,/+-]*$')
REAL_STOP_SCRIPT = (   # 이 실행기가 켠 real.launch.py·보기 노드만(땅에서만 끈다). 드라이버는 Ctrl+C(INT)면 착륙하고 끝난다,
    # launch가 꺼질 때 비행 기록 페이지(viewer.html)를 만들 시간을 준다
    "pkill -INT -f '[r]os_view_node.py'; pkill -INT -f '[r]eal.launch.py'; "
    "for i in $(seq 1 60); do pgrep -f '[r]eal.launch.py|[r]os_view_node.py' >/dev/null || break; sleep 1; done; "
    "pgrep -f '[r]eal.launch.py|[r]os_view_node.py' >/dev/null && echo left || echo stopped")

say, warn, url = drone.say, drone.warn, drone.url


def wsl_path(path) -> str:
    """Windows 경로 → WSL 경로(C:\\a\\b → /mnt/c/a/b). 이미 WSL 경로면 그대로."""
    p = str(path).replace('\\', '/')
    m = re.match(r'^([A-Za-z]):/(.*)$', p)
    return f'/mnt/{m.group(1).lower()}/{m.group(2)}' if m else p


def wsl_argv(script, user=None):
    return ['wsl.exe', '-d', DISTRO] + (['-u', user] if user else []) + ['--exec', 'bash', '-c', script]


def wsl(script, *, timeout=60):
    """WSL 기본 사용자로 bash 스크립트를 돌린다. (종료 코드, 표준 출력). WSL을 못 부르면 (None, '')."""
    try:
        r = subprocess.run(wsl_argv(script), capture_output=True, timeout=timeout, creationflags=winproc.CREATE_NO_WINDOW)
    except (OSError, subprocess.TimeoutExpired):
        return None, ''
    return r.returncode, r.stdout.decode('utf-8', errors='replace')


def distros():
    """설치된 WSL 배포판 이름(wsl -l -q는 UTF-16으로 답한다)."""
    try:
        r = subprocess.run(['wsl.exe', '-l', '-q'], capture_output=True, timeout=20, creationflags=winproc.CREATE_NO_WINDOW)
    except (OSError, subprocess.TimeoutExpired):
        return []
    text = r.stdout.decode('utf-16-le', errors='replace').replace('\x00', '')
    return [line.strip() for line in text.splitlines() if line.strip()]


def setup_command(user):
    repo, ref = head_source()
    return f'wsl -d {DISTRO} -u root -- bash {wsl_path(SETUP)} {user} {wsl_path(ROOT)} {repo} {ref}'


# ---------- WSL에서 돌릴 것 ----------
CHECK_SCRIPT = (f'test -f /opt/ros/jazzy/setup.bash && echo has-ros; test -d {WSL_REPO}/.git && echo has-repo; '
                'test -x ~/venvs/tt/bin/python && echo has-venv; echo user=$(whoami)')


def git_out(*args):
    """이 코드 폴더에서 git을 돌린 출력(실패하면 '')."""
    try:
        r = subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, timeout=20,
                           creationflags=winproc.CREATE_NO_WINDOW)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return r.stdout.decode('utf-8', errors='replace').strip() if r.returncode == 0 else ''


def head_source():
    """WSL git이 가져올 저장소(WSL 경로)와 이름. worktree의 .git은 Windows 경로를 적은 파일이라 WSL git이 못 읽으므로
    공통 .git 폴더에서 지금 브랜치(분리 HEAD면 커밋)를 가져온다."""
    repo = git_out('rev-parse', '--path-format=absolute', '--git-common-dir') or str(ROOT)
    ref = git_out('symbolic-ref', '-q', 'HEAD') or git_out('rev-parse', 'HEAD') or 'HEAD'
    return wsl_path(repo), ref


def sync_script(repo, ref):
    """WSL 사본을 이 코드 폴더의 지금 커밋으로 맞춘다. 사본의 세계·기록 파일(추적 안 함)은 그대로 둔다."""
    return (f'cd {WSL_REPO} && git fetch -q --depth 1 "file://{repo}" "{ref}" && '
            'git checkout -q -B launcher-sim FETCH_HEAD && git log -1 --format="%h %s"')


def world_script(stamp):
    """세계가 없으면 만들고, 지난 시연의 찾은 사람·장애물 파일은 시각을 붙여 옮겨 둔다."""
    w = f'worlds/{WORLD}'
    return (f'{ROS} && cd {TT} && '
            f'if [ ! -f {w}/world.sdf ]; then python3 generate_world.py --seed 7 --stage 3 >/dev/null; fi; '
            f'for f in people.json discovered.json; do [ -f {w}/$f ] && mv {w}/$f {w}/$f.{stamp}; done; '
            f'test -f {w}/world.sdf && echo world-ok')


def sim_script(extra=()):
    """자율 비행 README의 시뮬레이션 명령 그대로(세계 s7_stage3, Gazebo 화면 없이, 자율 비행 웹, YOLO 탐지).
    extra: 사용자가 실행기에 준 이름:=값(뒤에 붙어 같은 이름이면 이것이 이긴다)."""
    return (f'{ROS} && cd {TT} && exec ros2 launch launch/{SIM_MARK} '
            f'world_dir:=$PWD/worlds/{WORLD} headless:=true web:=true detect:=true' + ''.join(f' {a}' for a in extra))


def view_script(world=True):
    """보기 노드. 시뮬레이션은 찾은 사람·찾은 장애물·3D 카메라 지도를 세계 폴더에 쓰고(mission.launch.py), 자율 비행 웹은
    세계 지도(known_map.json: 벽·가구·목적지)와 정답 위치(/tt/odometry_gt)를 그린다 → 보기 노드도 같은 파일·토픽으로(11/17 "똑같이").
    실기체는 보기 노드 기본값(~/ttrepo/data/outputs/flights의 가장 새 회차 = real.launch.py 기록 폴더의 같은 이름 파일)."""
    if not world:
        return f'{ROS} && exec python3 {WSL_REPO}/modules/autonomy/tello_link/{VIEW_MARK} --ros-args'
    w = f'$HOME/ttrepo/simulation/tt-sim/worlds/{WORLD}'
    args = (f' -p people:={w}/people.json -p discovered:={w}/discovered.json -p camera_map:={w}/camera_map.npz'
            f' -p map:={w}/known_map.json -p truth:=true')
    return f'{ROS} && exec python3 {WSL_REPO}/modules/autonomy/tello_link/{VIEW_MARK} --ros-args{args}'


def real_script(front_tof, extra=()):
    """자율 비행 README의 실기체 명령 그대로(기본 tello:=relay: Windows 중계를 거친다). 확장 키트가 없으면 전방 ToF를 끈다.
    extra: 사용자가 실행기에 준 이름:=값(예: search_time:=150, 뒤에 붙어 같은 이름이면 이것이 이긴다)."""
    return (f'{ROS} && cd {TT} && exec ros2 launch launch/{REAL_MARK} front_tof:={"true" if front_tof else "false"}'
            + ''.join(f' {a}' for a in extra))


def launch_args_ok(items, launch):
    """실행기 뒤에 붙인 이름:=값을 확인하고 넘긴다고 알린다. 꼴이 아니면 아무것도 켜지 않게 False.
    기본값은 자율 비행 파트가 정한 그대로고, 이것은 그 파트와 맞춘 값을 시연 때 넘기는 길이다(실행기는 값을 정하지 않는다)."""
    bad = [a for a in items if not LAUNCH_ARG.match(a)]
    if bad:
        say(f'자율 비행 launch에 넘길 인자는 이름:=값 꼴만 받아요(예: search_time:=150). 못 받은 것: {" ".join(bad)}')
        return False
    if items:
        say(f'       자율 비행 {launch}에 더 넘김: {" ".join(items)} (자율 비행 파트와 맞춘 값만 쓰세요)')
    return True


def relay_cmd(fake_tello=False):
    """Windows 중계(자율 비행 README 3번). 가짜 Tello면 127.0.0.1:18889로(fake_tello.py 기본 포트)."""
    return [sys.executable, '-B', RELAY, *(FAKE_TELLO_ARGS if fake_tello else [])]


def fake_tello_cmd():
    return [sys.executable, '-B', 'modules/autonomy/tello_link/fake_tello.py', '--cmd-port', '18889',
            '--state-port', '18890', '--video-port', '11112']


def gateway_cmd(sensor_pose=False, phone=False):
    """보기 게이트웨이. sensor_pose면 드라이버가 받은 Tello 상태로 지도용 센서 자세를 만든다(실시간으로 도는 실기체·가짜 Tello만:
    이 PC 시뮬레이터는 실제보다 느려 벽시계로 적분하면 거리가 맞지 않는다). phone이면 같은 망(폰 핫스팟 등)에도 연다."""
    return [sys.executable, '-B', 'modules/autonomy/tello_link/gateway_server.py', '--ros-view', VIEW_URL,
            '--port', str(SIM_PORT), *(['--ros-view-sensor-pose'] if sensor_pose else []),
            *(['--host', '0.0.0.0'] if phone else [])]


WHICH_SCRIPT = ("pgrep -f '[r]eal.launch.py' >/dev/null && echo real-on; "   # [x] 꼴: pgrep이 자기 명령줄을 잡지 않게
                "pgrep -f '[m]ission.launch.py' >/dev/null && echo sim-on; true")


def running_launches():
    """WSL에서 도는 자율 비행 launch: 'real'(real.launch.py)·'sim'(mission.launch.py). 이 실행기가 켜지 않은 것도 센다."""
    _, out = wsl(WHICH_SCRIPT, timeout=30)
    return {kind for kind, mark in (('real', 'real-on'), ('sim', 'sim-on')) if mark in out}


KIND_KO = {'real': '실기체 자율 비행(start.cmd autonomy)', 'fake': '가짜 Tello 자율 비행(start.cmd autonomy --fake-tello)',
           'sim': '자율 비행 시뮬레이터(start.cmd sim)', 'unknown': '중계 없는 자율 비행(real.launch.py)'}


def relay_kind(procs):
    """Windows 중계가 가짜 Tello용('fake', --tello 127.0.0.1)인지 실기체용('real')인지. 없으면 None."""
    relays = drone.match(procs, RELAY_MARK)
    if not relays:
        return None
    return 'fake' if any('--tello 127.0.0.1' in drone.norm(p.cmd) for p in relays) else 'real'


def refuse_other_kind(want, procs):
    """이미 켜진 자율 비행이 다른 종류(실기체·가짜 Tello·시뮬레이터)면 까닭과 끄는 법을 말하고 True(11/16 점검 A6).
    전에는 자율 비행 웹(8080)이 답하거나 중계가 있으면 켜진 것으로 보고 그대로 써서, 실기체를 남긴 채 --fake-tello·sim을
    켜면 "가짜"라고 하면서 실기체에 붙었다(화면 이륙이 실기체를 띄운다)."""
    relay = relay_kind(procs)
    if not relay and not winproc.get_json(AUTONOMY_WEB + '/api/status'):
        return False   # 아무것도 안 켜짐(WSL을 깨우지 않는다)
    launches = running_launches()
    if 'sim' in launches:
        running = 'sim'
    elif 'real' in launches:
        running = relay or 'unknown'
    else:
        running = relay if want != 'sim' else None   # 중계만 남음: 자율 비행 실기체·가짜만 그 중계를 같이 쓴다
    if running is None or running == want:
        return False
    say(f'       {KIND_KO[running]}이(가) 이미 켜져 있어요. 그대로 쓰면 {KIND_KO[want]}이(가) 아니라 그쪽에 붙어요'
        + ('(화면 이륙이 실기체를 띄울 수 있어요).' if running in ('real', 'unknown') else '.'))
    say('       그 실행기 창에서 Enter로 모두 끄고(실기체는 땅에서만) 다시 실행하세요.')
    return True


def stop_sim(real=False):
    """실행기가 켠 시뮬레이터(또는 실기체 자율 비행)·보기 노드를 끈다(모두 끄기에서 부른다). 다 꺼졌으면 True."""
    _, out = wsl(REAL_STOP_SCRIPT if real else STOP_SCRIPT, timeout=90 if real else 60)
    return 'stopped' in out


# ---------- 켜기 ----------
def check_wsl(step='[1/6]'):
    """WSL 자율 비행 환경을 확인한다. 됐으면 Ubuntu 사용자 이름, 모자라면 설치 안내를 보이고 None."""
    if DISTRO not in distros():
        say(f'{step} WSL {DISTRO}가 없어요. 관리자 PowerShell에서 설치하고 Ubuntu 사용자를 만든 뒤:')
        say(f'       wsl --install -d {DISTRO}')
        say(f'       자율 비행 환경 설치(한 번, 약 10 GB): {setup_command("<Ubuntu 사용자>")}')
        return None
    _, out = wsl(CHECK_SCRIPT, timeout=90)
    tokens = out.split()
    user = next((t[5:] for t in tokens if t.startswith('user=')), None)
    missing = [name for key, name in (('has-ros', 'ROS 2 Jazzy'), ('has-repo', '시뮬레이터 사본 ~/ttrepo'),
                                      ('has-venv', 'YOLO 환경 ~/venvs/tt')) if key not in tokens]
    if missing or not user:
        say(f'{step} WSL {DISTRO}에 {", ".join(missing) or "응답"}이(가) 없어요. 한 번 설치하세요(약 10 GB, 인터넷 필요):')
        say(f'       {setup_command(user or "<Ubuntu 사용자>")}')
        return None
    say(f'{step} WSL {DISTRO}(사용자 {user}): ROS 2 Jazzy·자율 비행 환경 있음')
    return user


def has_simulator():
    """이 코드 폴더에 자율 비행 시뮬레이터가 있는지(자율 비행 코드가 합쳐진 브랜치인지)."""
    return (ROOT / 'simulation/tt-sim/launch' / SIM_MARK).is_file()


def sync_copy():
    """WSL 사본을 지금 코드로 맞춘다. 이 저장소에 시뮬레이터가 없으면(자율 비행 코드가 안 합쳐진 브랜치) 사본을 그대로 쓴다."""
    if not has_simulator():
        warn('이 코드 폴더에는 자율 비행 시뮬레이터가 없어요(자율 비행 코드가 합쳐진 브랜치가 아님). WSL 사본을 그대로 써요.')
        return
    code, out = wsl(sync_script(*head_source()), timeout=180)
    if code == 0 and out.strip():
        say(f'       WSL 사본을 지금 커밋으로 맞춤: {out.strip().splitlines()[-1]}')
    else:
        warn('WSL 사본을 맞추지 못했어요(사본에서 고친 파일이 있거나 git 오류). 있던 사본으로 켜요.')
    if git_out('status', '--porcelain', '--untracked-files=no'):
        warn('커밋하지 않은 변경은 WSL 사본(시뮬레이터·보기 노드)에 들어가지 않아요. 게이트웨이·화면은 지금 파일로 켜요.')


def start_wsl(name, script, ready, seconds, what):
    """WSL 프로그램을 숨은 창에서 띄우고 준비될 때까지 기다린다. 이미 준비돼 있으면 다시 띄우지 않는다."""
    if ready():
        say(f'       {what}은(는) 이미 켜져 있어요.')
        return True
    winproc.start_service(name, ROOT, wsl_argv(script))
    if drone.wait_for(ready, seconds):
        say(f'       {what} 켬.')
        return True
    warn(f'{what}이(가) {seconds}초 안에 준비되지 않았어요. 로그: {winproc.service_log(ROOT, name)}')
    return False


def view_state():
    return winproc.get_json(VIEW_URL + '/api/state')


def start_monitor(steps, no_browser, sensor_pose=False, phone=False):
    """보기 게이트웨이(8020) → 사람 탐지·지도 콘솔(8797)·지도 연결기·지도 유지 → 관제 화면. sim·autonomy가 같이 쓴다.
    steps: 세 단계 이름(게이트웨이·사람 탐지와 3D 지도·화면). sensor_pose: 지도용 센서 자세(autonomy만, gateway_cmd).
    phone: 게이트웨이를 같은 망에도 열고 폰 주소를 알려 준다(11/17 송승우 "자율 비행 모드에서도 폰으로").
    됐으면 (게이트웨이 주소, 지도 유지), 못 켰으면 (None, None)."""
    gateway = url(SIM_PORT)
    st = drone.state(gateway)
    say(f'{steps[0]} 게이트웨이(자율 비행 보기)')
    if drone.is_gateway(st):
        say(f'       게이트웨이는 이미 켜져 있어요({gateway}).')
        if phone and not drone.match(winproc.processes(), 'gateway_server.py', '--ros-view', '0.0.0.0'):
            warn('이미 켜진 게이트웨이가 이 PC에서만 열려 있어 폰 접속으로 바꾸지 못해요. '
                 '폰으로 열려면 땅에서 모두 끄고(Enter) --phone으로 다시 켜세요.')
            phone = False
    else:
        busy = winproc.port_owners(tcp=(SIM_PORT,))
        if busy:
            say(f'       {SIM_PORT} 포트를 다른 프로그램이 쓰고 있어요. 끄고 다시 실행하세요.')
            say('\n'.join(drone.owner_lines(busy, winproc.processes())))
            return None, None
        winproc.start_service(f'gateway-{SIM_PORT}', ROOT, gateway_cmd(sensor_pose, phone))
        if not drone.wait_for(lambda: drone.is_gateway(drone.state(gateway)), 25):
            say('       25초 안에 게이트웨이가 켜지지 않았어요. 실행기 안내에 표시된 gateway 로그를 확인하세요.')
            return None, None
        say(f'       게이트웨이 켬({gateway}, 기록: {ROOT / "data/outputs/tello"}).')

    say(f'{steps[1]} 사람 탐지·3D 지도')
    procs = winproc.processes()
    drone.start_detect(procs, gateway)
    keeper = None
    if drone.start_console(gateway, port=SIM_CONSOLE_PORT):
        drone.start_bridge(procs, url(SIM_CONSOLE_PORT), gateway)
        keeper = mapkeeper.MapKeeper(gateway, url(SIM_CONSOLE_PORT), say)
        say('       지도: 자율 비행 영상으로 실행기가 지도를 시작하고 유지해요' +
            ('(위치는 드론 센서 자세, 손 조종 모드와 같음).' if sensor_pose else
             '(시뮬레이터는 실제보다 느려 센서 자세 없이 영상만).'))

    page = gateway + '/?preset=autonomy#/live'
    say(f'{steps[2]} 화면 열기')
    drone.open_page(page, no_browser)
    say()
    say(f'  실시간 감독  {page}')
    say('  시작: 조종 줄 "이륙" → 주변 확인 체크 → "이륙"을 4초 안에 두 번(자율 비행 화면과 같음). 2D 지도 클릭 → 여기로 보내기')
    say(f'  자율 비행 웹 {AUTONOMY_WEB} (같은 비행을 자율 비행 화면에서도)')
    say(f'  지도 콘솔    {gateway}/#/map · 탐색 결과 {gateway}/#/review')
    if phone:   # 폰: 같은 망 주소. 폰 화면(#/control)은 시작·착륙·비상 정지와 상태·영상, 손 조종은 없다(자율 비행이 조종)
        for line in drone.phone_lines(SIM_PORT, live='?preset=autonomy#/live'):
            say(line)
        say('  폰 화면: 시작(이륙 → 주변 확인 → 4초 안에 두 번)·착륙·비상 정지와 상태·영상. 손 조종은 없어요(자율 비행이 조종).')
    if sensor_pose:   # 실기체 자율 비행(11/16·11/17 실기체 시험에서 배운 것)
        say('  실기체: 노트북 충전기를 꽂고 날려요(배터리면 GPU가 느려져 자율 비행 인식이 약 10배 늦어요).')
        say('          드론을 켜고 약 1분 뒤 시작 칸 기수 값이 "멈춤"이 되면, 그 뒤 1~2분 안에 이륙하세요')
        say('          (켠 직후엔 기수 값이 흘러 이륙이 거절되고, 땅에 오래 두면 뜨거워져요).')
        say('          뒤집히거나 부딪힌 뒤에는 프로펠러·가드·모터를 살피고 다시 날려요.')
    return gateway, keeper


def wait_or_return(keeper, interactive):
    if interactive if interactive is not None else sys.stdin.isatty():
        if keeper:
            keeper.start()
        return drone.wait_and_stop(keeper)
    say('  (창 입력이 없어 기다리지 않아요. 지도 자동 시작과 Enter 끄기는 실행기 창에서 실행할 때만 돼요.)')
    return 0


def run_sim(*, no_browser=False, interactive=None, launch_args=(), phone=False):
    say('=== 자율 비행 시뮬레이터 시연: 시뮬레이터·관제 화면·사람 탐지·3D 지도 ===')
    if not launch_args_ok(launch_args, SIM_MARK):
        return 1
    drone.show_code()
    if not drone.get_ready():
        return 1
    if not check_wsl():
        return 1
    if refuse_other_kind('sim', winproc.processes()):
        return 1
    say('[2/6] 시뮬레이터 사본·세계')
    if winproc.get_json(AUTONOMY_WEB + '/api/status'):   # 도는 시뮬레이터의 코드·찾은 사람 파일은 건드리지 않는다
        say('       자율 비행 시뮬레이터가 이미 켜져 있어 사본·세계·찾은 사람 파일은 그대로 둬요.')
        if launch_args:
            warn('이미 켜진 시뮬레이터라 더 넘긴 인자는 적용되지 않아요. 넘기려면 모두 끄고 다시 켜세요.')
    else:
        sync_copy()
        _, out = wsl(world_script(time.strftime('%Y%m%d_%H%M%S')), timeout=300)
        if 'world-ok' not in out:
            warn(f'시뮬레이터 세계(worlds/{WORLD})를 만들지 못했어요. WSL에서 확인하세요: '
                 f'cd {TT} && python3 generate_world.py --seed 7 --stage 3')
            return 1
        say(f'       세계 {WORLD} 있음(지난 시연의 찾은 사람 파일은 시각을 붙여 옮겨 둠)')

    st = drone.state(url(SIM_PORT))
    if drone.is_gateway(st) and st.get('source') != 'ros':
        say(f"[3/6] {SIM_PORT}에 다른 게이트웨이(source={st.get('source')})가 켜져 있어요. 끄고 다시 실행하세요.")
        return 1
    say('[3/6] 자율 비행 시뮬레이터(Gazebo 화면 없이)·보기 노드')
    if not start_wsl('sim-autonomy', sim_script(launch_args), lambda: winproc.get_json(AUTONOMY_WEB + '/api/status'), 90,
                     '자율 비행 시뮬레이터(자율 비행 웹 8080)'):
        return 1
    if not start_wsl('sim-view-node', view_script(), view_state, 60, f'보기 노드({VIEW_URL})'):
        return 1
    gateway, keeper = start_monitor(('[4/6]', '[5/6]', '[6/6]'), no_browser, phone=phone)
    if gateway is None:
        return 1
    say(f'  시뮬레이터 로그 {winproc.service_log(ROOT, "sim-autonomy")}')
    say('  자율 비행은 한 번 끝나면(끝·중단) 다시 시작하지 않아요. 다시 날리려면 모두 끄고 start.cmd sim을 다시 켜세요.')
    return wait_or_return(keeper, interactive)


def run_autonomy(*, no_browser=False, fake_tello=False, interactive=None, launch_args=(), phone=False):
    """실기체 자율 비행(자율 비행 README 실기체 순서): Windows 중계 → WSL real.launch.py → 보기 노드 → 관제 화면 쪽.
    launch_args: real.launch.py에 그대로 더 넘길 이름:=값(예: search_time:=150). phone: 폰에서도 관제 화면(start_monitor)."""
    say('=== 자율 비행 실기체: 자율 비행(ROS)이 드론을 조종하고, 관제 화면은 같은 명령으로 보기·버튼·사람 탐지·3D 지도 ===')
    if not launch_args_ok(launch_args, REAL_MARK):
        return 1
    drone.show_code()
    if not drone.get_ready():
        return 1
    if not check_wsl('[1/7]'):
        return 1
    ssid = next((s for s in winproc.wifi_ssids() if re.match(r'(TELLO|RMTT)-', s, re.I)), None)
    if fake_tello:
        kit = False
        say('[2/7] 드론 없이 가짜 Tello(Windows 127.0.0.1:18889)로 시험해요. 가짜 Tello는 날지 않아요.')
    elif ssid:
        kit = ssid.upper().startswith('RMTT-')
        say(f'[2/7] 드론 Wi-Fi: {ssid} · ' + ('확장 키트: 전방 ToF 씀' if kit else '기본 Tello: 전방 ToF 끔(front_tof:=false)'))
    else:
        say('[2/7] 드론 Wi-Fi(TELLO-… / RMTT-…)에 연결돼 있지 않아요. 드론을 켜고 PC를 드론 Wi-Fi에 연결한 뒤 다시 실행하세요.')
        say('       (확장 키트 여부를 Wi-Fi 이름으로 정해서 먼저 연결해야 해요. 드론 없이 해 보려면 start.cmd autonomy --fake-tello)')
        return 1
    if refuse_other_kind('fake' if fake_tello else 'real', winproc.processes()):
        return 1

    say('[3/7] 자율 비행 사본')
    if winproc.get_json(AUTONOMY_WEB + '/api/status'):
        say('       자율 비행이 이미 켜져 있어 사본은 그대로 둬요.')
        if launch_args:
            warn('이미 켜진 자율 비행이라 더 넘긴 인자는 적용되지 않아요. 넘기려면 땅에서 모두 끄고 다시 켜세요.')
    else:
        sync_copy()

    say('[4/7] Windows 중계(tello_relay.py, WSL 기본 NAT 네트워크용)')
    procs = winproc.processes()
    if drone.match(procs, RELAY_MARK):
        say('       중계는 이미 켜져 있어요.')
    else:
        owners = winproc.port_owners(udp=RELAY_UDP)
        if owners:
            say('       드론 포트(UDP 8889·8890·11111)나 중계 포트(9889)를 다른 프로그램이 쓰고 있어요(실기체 게이트웨이 start.cmd real, '
                '점검 화면 check_server.py 등). Tello는 처음 붙은 쪽에만 답해서 같이 켤 수 없어요. 끄고 다시 실행하세요.')
            say('\n'.join(drone.owner_lines(owners, procs)))
            return 1
        if not fake_tello:
            drone.ensure_firewall()   # 드론 상태·영상(UDP 8890·11111)을 중계 Python이 받아야 한다(없으면 관리자 확인 창)
        elif not drone.match(procs, 'fake_tello.py'):
            winproc.start_service('fake-tello', ROOT, fake_tello_cmd())
        winproc.start_service('tello-relay', ROOT, relay_cmd(fake_tello))
        if not drone.wait_for(lambda: winproc.port_owners(udp=(9889,)), 10):
            warn(f'중계가 10초 안에 9889를 열지 않았어요. 로그: {winproc.service_log(ROOT, "tello-relay")}')
            return 1
        say('       중계 켬(Tello ↔ Windows ↔ WSL 드라이버, 5초마다 명령·응답·상태 수를 로그에 남김).')

    st = drone.state(url(SIM_PORT))
    if drone.is_gateway(st) and st.get('source') != 'ros':
        say(f"[5/7] {SIM_PORT}에 다른 게이트웨이(source={st.get('source')})가 켜져 있어요. 끄고 다시 실행하세요.")
        return 1
    say('[5/7] 자율 비행(드라이버·위치·깊이·YOLO·비행 노드·웹 8080)·보기 노드')
    if not start_wsl('autonomy-real', real_script(kit, launch_args), lambda: winproc.get_json(AUTONOMY_WEB + '/api/status'), 120,
                     '자율 비행(자율 비행 웹 8080)'):
        return 1
    if not start_wsl('autonomy-view-node', view_script(world=False), view_state, 60, f'보기 노드({VIEW_URL})'):
        return 1
    gateway, keeper = start_monitor(('[6/7]', '[7/7]', '      '), no_browser, sensor_pose=True, phone=phone)
    if gateway is None:
        return 1
    say(f'  자율 비행 로그 {winproc.service_log(ROOT, "autonomy-real")} · 중계 로그 {winproc.service_log(ROOT, "tello-relay")}')
    say('  비행 기록은 WSL ~/ttrepo/data/outputs/flights/<시각>/(자율 비행), 관제 기록은 data/outputs/tello/ros_…(게이트웨이).')
    say('  자율 비행은 한 번 끝나면(끝·중단) 다시 시작하지 않아요. 다시 날리려면 땅에서 모두 끄고 start.cmd autonomy를 다시 켜세요.')
    say('  위급하면 화면 "비상 정지"(0.6초 길게·Esc)나 자율 비행 웹의 비상 정지. 끄기(Enter)는 땅에서만 돼요.')
    return wait_or_return(keeper, interactive)
