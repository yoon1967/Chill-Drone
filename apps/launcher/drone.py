"""체험(가짜 드론)·실기체 관제: 묻지 않고 전부 켜고, 실행기 창에서 Enter로 모두 끈다.

- 켜기 전에 빠진 패키지·모델·지도 콘솔 웹 의존성을 받는다(prepare.py).
- 보조 프로그램은 숨겨 실행하고 실행기 창에서 상태를 안내한다.
- 실기체는 게이트웨이 + 사람 탐지 + 지도 콘솔·지도 연결기 + 지도 자동 시작(mapkeeper.py) + MoNA 자율(드론이 붙은 뒤).
- 실행기 창은 열어 둔다. Enter를 누르면 모두 끈다: 실기체가 공중이면 아무것도 끄지 않고, 땅이면 지도를 저장하고
  보조 프로그램을 정상 종료시킨다(게이트웨이가 회차 metrics.json·영상 조각을 마무리하게).
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import webbrowser

from . import mapkeeper, prepare, winproc

ROOT = Path(__file__).resolve().parents[2]
REAL_PORT = 8010        # 실기체 게이트웨이
FAKE_PORT = 8030        # 체험(가짜 드론) 게이트웨이. 실기체와 같이 켜도 겹치지 않게
CONSOLE_PORT = 8790     # 지도 콘솔(apps/console)
FAKE_MAP_PORT = 8795    # 가짜 지도 서버
FAKE_CONSOLE_PORT = 8796  # 체험 관제의 영상 복원용 콘솔
DRONE_UDP = (8889, 8890, 11111)   # Tello 명령·상태·영상. Tello는 처음 command를 보낸 쪽에만 답한다
GROUND = {'grounded', 'disconnected', 'emergency'}
MODE_KO = {'disconnected': '연결 끊김', 'grounded': '지상', 'manual': '수동 비행', 'auto': '자율 탐색',
           'paused': '일시정지', 'landing': '착륙 중', 'emergency': '비상 정지'}
MONA_RUN = ROOT / 'modules/autonomy/mona_bench/run.ps1'


def say(text=''):
    print(text, flush=True)


def warn(text):
    print('  ! ' + text, flush=True)


def url(port):
    return f'http://127.0.0.1:{port}'


def state(base):
    return winproc.get_json(base + '/api/drone/state')


def is_gateway(st):
    return bool(st) and st.get('type') == 'drone_state'


def airborne(st):
    """실기체가 이륙·비행·착륙 중인지. 가짜 드론·기록 재생은 끄는 걸 막지 않는다."""
    if not is_gateway(st) or st.get('source') != 'tello':
        return False
    return bool(st.get('flight_busy')) or st.get('mode') not in GROUND


def norm(cmd):
    """명령줄 비교용: 따옴표를 빼고 경로 구분자를 / 로."""
    return cmd.replace('"', '').replace('\\', '/')


def match(procs, *tokens, exclude=()):
    return [p for p in procs if all(t in norm(p.cmd) for t in tokens) and not any(x in norm(p.cmd) for x in exclude)]


def gateway_port(cmd):
    found = re.search(r'--port\s+(\d+)', norm(cmd))
    return int(found.group(1)) if found else REAL_PORT


def wait_for(check, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = check()
        if value:
            return value
        time.sleep(.7)
    return None


# ---------- 띄울 명령(파일 경로는 저장소 기준, 새 창은 저장소 폴더에서 연다) ----------
def gateway_cmd(port, *, fake=False, phone=False):
    cmd = [sys.executable, '-B', 'modules/autonomy/tello_link/gateway_server.py', '--port', str(port)]
    if fake:
        cmd.append('--fake')
    if phone:
        cmd += ['--host', '0.0.0.0']
    return cmd


def detect_cmd(gateway, yolo):
    return [sys.executable, '-B', '-m', 'modules.perception.inference.gateway_ai_client',
            '--gateway', gateway, '--yolo-model', str(yolo)]


def fake_map_cmd(port):
    return [sys.executable, '-B', 'modules/autonomy/tello_link/fake_map_server.py', '--port', str(port), '--start']


def console_cmd(gateway, port):
    return [sys.executable, '-B', 'apps/console/serve.py', '--enable-camera', '--port', str(port),
            '--tello-gateway', gateway]


def bridge_cmd(map_url, gateway):
    return [sys.executable, '-B', 'modules/autonomy/tello_link/gateway_map_bridge.py', '--map', map_url,
            '--gateway', gateway]


def mona_cmd():
    """MoNA 비행 모드: 경로는 운용자가 이륙 뒤 화면에서 "자율"을 눌러야 시작한다(그 전 속도 명령은 게이트웨이가 무시)."""
    return ['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-File', str(MONA_RUN),
            '-Action', 'Live', '-Seconds', '3600', '-GoalForwardM', '5']


def show_code():
    try:
        head = subprocess.run(['git', '-C', str(ROOT), 'log', '-1', '--format=%h %s'], capture_output=True,
                              timeout=10, creationflags=winproc.CREATE_NO_WINDOW).stdout.decode('utf-8', 'replace').strip()
    except (OSError, subprocess.TimeoutExpired):
        head = ''
    say(f'코드 폴더: {ROOT}' + (f' ({head})' if head else ''))


def show_drone(st):
    temp = f"{st['temp_high_c']}°C" if st.get('temp_high_c') is not None else '없음'
    battery = f"{st['battery_pct']}%" if st.get('battery_pct') is not None else '없음'
    say(f"       드론 연결됨 · {MODE_KO.get(st.get('mode'), st.get('mode'))} · 배터리 {battery} · 온도 {temp}")


def owner_lines(owners, procs):
    by_pid = {p.pid: p.cmd for p in procs}
    return [f'       [{proto} {port}] PID {pid}: {by_pid.get(pid, "?")}' for proto, port, pid in owners]


def get_ready(*, mona=False):
    say('[준비] 패키지·모델·웹 의존성·Docker 이미지 확인. 처음에는 인터넷 연결을 유지하세요.')
    if not prepare.ensure_all(mona=mona):
        warn('프로젝트 준비를 마치지 못했습니다. 위 메시지의 문제를 해결한 후 scripts/start.cmd를 다시 실행해 주세요.')
        return False
    say('       준비됨')
    say(f'       서비스 로그: {winproc.service_log(ROOT, "launcher").parent}')
    return True


# ---------- 보조 프로그램 ----------
def start_detect(procs, gateway):
    if match(procs, 'gateway_ai_client', '--gateway ' + gateway):
        return say('       사람 탐지는 이미 켜져 있어요.')
    sys.path.insert(0, str(ROOT))
    from modules.mapping.data.model_assets import person_checkpoint
    winproc.start_service(f'people-{gateway.rsplit(":", 1)[-1]}', ROOT, detect_cmd(gateway, person_checkpoint()))
    say('       사람 탐지 켬(모델 올리는 데 10~30초). 영상을 켜야 결과가 나와요.')


def start_bridge(procs, map_url, gateway):
    if match(procs, 'gateway_map_bridge', '--gateway ' + gateway):
        return say('       지도 연결기는 이미 켜져 있어요.')
    winproc.start_service(f'map-bridge-{gateway.rsplit(":", 1)[-1]}', ROOT, bridge_cmd(map_url, gateway))
    say('       지도 연결기 켬.')


def start_console(gateway, *, port=CONSOLE_PORT):
    """게이트웨이에 지도 콘솔을 붙인다(실시간 지도·촬영 영상 복원·기록 열기·3D 지도 편집). 켜졌으면 True."""
    map_url = url(port)
    if winproc.get_json(map_url + '/api/live'):
        say('       지도 콘솔은 이미 켜져 있어요.')
        return True
    service = f'console-{port}'
    winproc.start_service(service, ROOT, console_cmd(gateway, port))
    if wait_for(lambda: winproc.get_json(map_url + '/api/live'), 40):
        say(f'       지도 콘솔 켬({map_url}).')
        return True
    warn(f'지도 콘솔이 40초 안에 켜지지 않았어요. 로그: {winproc.service_log(ROOT, service)}')
    return False


def mona_running():
    try:
        return bool(subprocess.run(['docker', 'ps', '-q', '--filter', 'name=^mona-live$'], capture_output=True,
                                   timeout=15, creationflags=winproc.CREATE_NO_WINDOW).stdout.strip())
    except (OSError, subprocess.TimeoutExpired):
        return False


def start_mona_when_connected(gateway, stop, interval=2.0):
    """드론이 붙으면 MoNA를 한 번 켠다(MoNA는 시작할 때 게이트웨이의 드론 상태를 확인한다)."""
    def run():
        while not stop.is_set():
            st = state(gateway)
            if is_gateway(st) and st.get('connected'):
                if mona_running():
                    return say('  MoNA는 이미 돌고 있어요(mona-live).')
                if stop.is_set():
                    return
                process = winproc.start_service('mona-live', ROOT, mona_cmd())
                say('  MoNA 시작 중. 비행 모드라 이륙 뒤 화면에서 "자율"을 눌러야 기체를 움직여요.')
                connected = False
                while not stop.wait(1):
                    code = process.poll()
                    if code is not None:
                        if code:
                            warn(f'MoNA 종료({code}). 로그: {winproc.service_log(ROOT, "mona-live")}')
                        return
                    if not connected and ((state(gateway) or {}).get('autonomy') or {}).get('connected'):
                        connected = True
                        say('  MoNA가 게이트웨이에 연결됐어요.')
                return
            stop.wait(interval)
    thread = threading.Thread(target=run, name='mona-starter', daemon=True)
    thread.start()
    return thread


def open_page(page, no_browser):
    if not no_browser and not webbrowser.open(page):
        say('       위 주소를 브라우저에서 여세요.')


def wait_and_stop(keeper=None, stop_event=None, mona_starter=None):
    """실행기 창을 열어 두고, Enter를 누르면 모두 끈다(실기체가 공중이면 끄지 않고 다시 기다린다)."""
    say()
    say('이 창을 열어 두세요(지도 유지). Enter를 누르면 모두 끕니다. 실기체가 공중이면 끄지 않아요.')
    while True:
        try:
            input()
        except EOFError:
            return 0
        except KeyboardInterrupt:
            pass
        if run_stop(keeper=keeper, stop_event=stop_event, mona_starter=mona_starter) == 0:
            time.sleep(4)   # 끈 결과를 읽을 시간
            return 0
        say('착륙한 뒤 다시 Enter를 누르세요.')


# ---------- 체험(가짜 드론) ----------
def run_fake(*, no_browser=False, interactive=None):
    say('=== 1 체험: 가짜 드론으로 관제 화면(사람 탐지·지도까지 모두) ===')
    show_code()
    if not get_ready():
        return 1
    gateway = url(FAKE_PORT)
    if is_gateway(state(gateway)):
        say(f'[1/4] 가짜 드론 게이트웨이는 이미 켜져 있어요({gateway}).')
    else:
        busy = winproc.port_owners(tcp=(FAKE_PORT,))
        if busy:
            say(f'[1/4] {FAKE_PORT} 포트를 다른 프로그램이 쓰고 있어요. 끄고 다시 실행하세요.')
            say('\n'.join(owner_lines(busy, winproc.processes())))
            return 1
        winproc.start_service(f'gateway-{FAKE_PORT}', ROOT, gateway_cmd(FAKE_PORT, fake=True))
        if not wait_for(lambda: is_gateway(state(gateway)), 25):
            say('[1/4] 25초 안에 게이트웨이가 켜지지 않았어요. 실행기 안내에 표시된 gateway 로그를 확인하세요.')
            return 1
        say(f'[1/4] 가짜 드론 게이트웨이 켬({gateway}).')
    procs = winproc.processes()
    say('[2/4] 사람 탐지')
    start_detect(procs, gateway)
    say('[3/4] 가짜 지도(3D 점군)·지도 콘솔(영상 복원용 8796)')
    map_url = url(FAKE_MAP_PORT)
    if winproc.get_json(map_url + '/api/live/state'):
        say('       가짜 지도 서버는 이미 켜져 있어요.')
    else:
        winproc.start_service(f'fake-map-{FAKE_MAP_PORT}', ROOT, fake_map_cmd(FAKE_MAP_PORT))
        say('       가짜 지도 서버 켬.')
    start_bridge(procs, map_url, gateway)
    start_console(gateway, port=FAKE_CONSOLE_PORT)   # 관제 화면 #/map의 영상 복원·3D 지도 편집용
    page = gateway + '/#/live'
    say('[4/4] 화면 열기')
    open_page(page, no_browser)
    say()
    say(f'  실시간 감독  {page}')
    say(f'  지도 콘솔    {gateway}/#/map (촬영 영상 복원·3D 지도 편집)')
    say(f'  탐색 결과    {gateway}/#/review')
    say(f'  폰 조작      {gateway}/#/control')
    say('  사람 탐지: 화면에서 "영상 켜기" 뒤 영상 칸 "AI 합성"(가짜 영상은 색 막대라 사람 0명). 지도: 지도 칸 "3D 점군".')
    if interactive if interactive is not None else sys.stdin.isatty():
        return wait_and_stop()
    return 0


# ---------- 실기체 관제 ----------
def firewall_hint():
    """실기체 게이트웨이를 돌릴 Python이 방화벽에서 UDP 수신을 허용받았는지 확인하고, 없으면 추가 명령을 알려 준다."""
    exe = getattr(sys, '_base_executable', sys.executable)
    script = ("$p = '%s'; $r = Get-NetFirewallApplicationFilter -ErrorAction SilentlyContinue | "
              "Where-Object { $_.Program -eq $p } | Get-NetFirewallRule | "
              "Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' }; "
              "if ($r) { 'ok' } else { 'none' }" % exe.replace("'", "''"))
    if winproc.powershell(script, timeout=60).strip() == 'none':
        warn('방화벽에 이 Python의 수신 허용 규칙이 없어요. 드론 상태·영상(UDP 8890·11111)이 안 들어오면 '
             '관리자 PowerShell에서 추가하세요(보안 설정이라 직접):')
        say(f'       New-NetFirewallRule -DisplayName "Tello UDP (Chill-Drone)" -Direction Inbound '
            f'-Program "{exe}" -Protocol UDP -LocalPort 8890,11111 -Action Allow')


def phone_lines(port):
    out = winproc.powershell("Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | "
                             "Where-Object { $_.InterfaceAlias -notlike 'vEthernet*' } | "
                             "ForEach-Object { $_.IPAddress }")
    ips = [ip for ip in out.split() if not ip.startswith(('127.', '169.254.', '192.168.10.'))]
    lines = [f'  폰 조작      http://{ip}:{port}/#/control' for ip in ips]
    rule = winproc.powershell("Get-NetFirewallPortFilter -ErrorAction SilentlyContinue | "
                              f"Where-Object {{ $_.Protocol -eq 'TCP' -and $_.LocalPort -eq '{port}' }} | Get-NetFirewallRule | "
                              "Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' } | "
                              "ForEach-Object { $_.DisplayName }", timeout=60).strip()
    if rule:
        lines.append(f'  방화벽 허용 규칙 있음: {rule.splitlines()[0]} (시험이 끝나면 관리자 PowerShell에서 지우세요)')
    else:
        lines.append('  폰에서 열리려면 관리자 PowerShell에서 방화벽 규칙을 추가하세요(시험 뒤 지우기):')
        lines.append(f'  New-NetFirewallRule -DisplayName "Drone gateway {port} (test)" -Direction Inbound -Protocol TCP '
                     f'-LocalPort {port} -RemoteAddress LocalSubnet -Action Allow')
    return lines


def run_real(*, phone=False, no_browser=False, interactive=None):
    say('=== 2 실기체 관제: 게이트웨이·사람 탐지·3D 지도·MoNA 자율을 모두 켭니다 ===')
    show_code()
    if not get_ready(mona=True):
        return 1
    gateway = url(REAL_PORT)
    drone_wifi = next((s for s in winproc.wifi_ssids() if re.match(r'(TELLO|RMTT)-', s, re.I)), None)
    if drone_wifi:
        say(f'[1/5] 드론 Wi-Fi: {drone_wifi}')
    else:
        say('[1/5] 드론 Wi-Fi(TELLO-… / RMTT-…)에 연결돼 있지 않아요. 드론을 켜고 연결하면 게이트웨이가 알아서 붙어요.')

    st = state(gateway)
    reuse = is_gateway(st)
    if reuse and st.get('source') != 'tello':
        say(f"[2/5] {REAL_PORT}에 실기체가 아닌 게이트웨이(source={st.get('source')})가 켜져 있어요. 끄고 다시 실행하세요.")
        return 1
    if reuse:
        say(f"[2/5] 실기체 게이트웨이는 이미 켜져 있어요({MODE_KO.get(st.get('mode'), st.get('mode'))}).")
        if phone:
            warn('이미 켜진 게이트웨이라 폰 접속(0.0.0.0)으로 바꾸지 못해요. 폰으로 열려면 끄고 다시 켜세요.')
    else:
        owners = winproc.port_owners(udp=DRONE_UDP, tcp=(REAL_PORT,))
        if owners:
            say('[2/5] 게이트웨이 포트나 드론 포트를 다른 프로그램이 쓰고 있어요(점검 화면 check_server.py, 다른 게이트웨이, '
                '직접 연결 지도 등). Tello는 처음 붙은 쪽에만 답해서 같이 켤 수 없어요. 끄고 다시 실행하세요.')
            say('\n'.join(owner_lines(owners, winproc.processes())))
            return 1
        say('[2/5] 겹치는 프로그램 없음. 방화벽 확인 중...')
        firewall_hint()
        say(f'       실기체 게이트웨이 켜는 중. 기록: {ROOT / "data/outputs/tello"}')
        winproc.start_service(f'gateway-{REAL_PORT}', ROOT, gateway_cmd(REAL_PORT, phone=phone))
        if not wait_for(lambda: is_gateway(state(gateway)), 25):
            say('       25초 안에 게이트웨이가 켜지지 않았어요. 실행기 안내에 표시된 gateway 로그를 확인하세요.')
            return 1
    say('[3/5] 드론 연결 기다리는 중(최대 30초)...')
    connected = wait_for(lambda: (lambda s: s if is_gateway(s) and s.get('connected') else None)(state(gateway)), 30)
    if connected:
        show_drone(connected)
    else:
        warn('아직 드론이 안 붙었어요. 전원과 Wi-Fi를 확인하세요. 붙으면 나머지(MoNA 등)는 알아서 이어서 켜요.')

    say('[4/5] 사람 탐지·3D 지도·MoNA')
    procs = winproc.processes()
    start_detect(procs, gateway)
    keeper = None
    if start_console(gateway):
        start_bridge(procs, url(CONSOLE_PORT), gateway)
        keeper = mapkeeper.MapKeeper(gateway, url(CONSOLE_PORT), say)
        say('       지도: 영상이 켜지면(이륙하면 자동) 실행기가 지도를 시작하고 유지해요.')
        say('       지도 콘솔의 "1. 드론 연결" 버튼은 누르지 마세요(드론은 게이트웨이가 쥐고 있어요. 이륙 전 "드론 해당 없음"은 정상).')
    stop_event = threading.Event()
    mona_starter = start_mona_when_connected(gateway, stop_event)
    say('       MoNA: 드론이 붙으면 켜요(비행 모드: 이륙 뒤 화면에서 "자율"을 눌러야 움직임).')

    page = gateway + '/#/live'
    say('[5/5] 화면 열기')
    open_page(page, no_browser)
    say()
    say(f'  실시간 감독  {page}')
    say(f'  지도 콘솔    {gateway}/#/map')
    say(f'  탐색 결과    {gateway}/#/review')
    if phone and not reuse:
        for line in phone_lines(REAL_PORT):
            say(line)
    else:
        say(f'  폰 조작      {gateway}/#/control (이 PC에서만. 폰으로 열려면 start.cmd real --phone)')
    if interactive if interactive is not None else sys.stdin.isatty():
        if keeper:
            keeper.start()
        return wait_and_stop(keeper, stop_event, mona_starter)
    say('  (창 입력이 없어 기다리지 않아요. 지도 자동 시작과 Enter 끄기는 실행기 창에서 실행할 때만 돼요.)')
    return 0


# ---------- 모두 끄기(실행기 창에서 Enter) ----------
STOP_ORDER = [   # (이름, 찾을 글자들, 기다릴 초)
    ('MoNA 실행', ('mona_bench/run.ps1', '-Action Live'), 15),
    ('사람 탐지', ('gateway_ai_client',), 10),
    ('지도 연결기', ('gateway_map_bridge',), 10),
    ('가짜 지도', ('fake_map_server.py',), 10),
    ('지도 콘솔', ('apps/console/serve.py',), 15),
    ('게이트웨이', ('gateway_server.py',), 15),
]


def stop_group(name, procs, wait_s):
    if not procs:
        return False
    pids = [p.pid for p in procs]
    for pid in pids:   # 같은 창(콘솔)에 있는 프로그램은 Ctrl+C 한 번에 같이 받는다
        if winproc.alive(pid):
            winproc.send_ctrl_c(pid)
            if wait_for(lambda: not any(winproc.alive(p) for p in pids), wait_s):
                break
    left = [pid for pid in pids if winproc.alive(pid)]
    for pid in left:
        winproc.kill_tree(pid)
    say(f'       {name}: ' + ('정상 종료가 안 돼 강제로 껐어요' if left else '껐어요'))
    return True


def run_stop(*, keeper=None, stop_event=None, mona_starter=None):
    say('=== 모두 끄기 ===')
    me = {os.getpid(), os.getppid()}
    procs = [p for p in winproc.processes() if p.pid not in me]
    gateways = sorted({url(gateway_port(p.cmd)) for p in match(procs, 'gateway_server.py')} |
                      {m.group(1) for p in match(procs, 'apps/console/serve.py', '--tello-gateway')
                       for m in [re.search(r'--tello-gateway\s+(\S+)', norm(p.cmd))] if m})
    states = {g: state(g) for g in gateways}
    flying = [g for g, st in states.items() if airborne(st)]
    if flying:
        for g in flying:
            st = states[g]
            busy = f", {st['flight_busy']} 중" if st.get('flight_busy') else ''
            say(f"  드론이 공중에 있어요({g}: {MODE_KO.get(st.get('mode'), st.get('mode'))}{busy}).")
        say('  아무것도 끄지 않았어요.')
        return 1
    if stop_event:
        stop_event.set()
    if mona_starter:
        mona_starter.join(timeout=20)
        procs = [p for p in winproc.processes() if p.pid not in me]
    if keeper:
        keeper.stop()
    stopped = False
    if mona_running():
        try:
            result = subprocess.run(['docker', 'stop', 'mona-live'], capture_output=True, timeout=60,
                                    creationflags=winproc.CREATE_NO_WINDOW)
            stopped = result.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            pass
        if stopped:
            say('       MoNA 자율: 껐어요')
        else:
            warn('MoNA 컨테이너 종료를 확인하지 못했어요. Docker Desktop의 mona-live 상태를 확인하세요.')
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)  # 이전 버전 서비스도 종료할 수 있다.
    try:
        for name, tokens, wait_s in STOP_ORDER:
            stopped |= stop_group(name, match(procs, *tokens), wait_s)
    finally:
        signal.signal(signal.SIGINT, previous)
    if not stopped:
        say('  켜져 있는 프로그램이 없어요.')
    runs = sorted((d for d in (ROOT / 'data/outputs/tello').glob('*') if d.is_dir()), key=lambda d: d.stat().st_mtime)
    if runs:
        done = (runs[-1] / 'metrics.json').is_file()
        say(f'  마지막 회차 기록: {runs[-1]} ' + ('(정리 완료)' if done else '(metrics.json 없음: 아직 도는 중이거나 강제 종료)'))
    return 0
