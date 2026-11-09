"""체험(가짜 드론)·실기체 관제 켜기와 모두 끄기.

프로그램마다 새 창으로 띄운다. 그 창에서 Ctrl+C로 하나씩 끌 수 있고, 모두 끄기는 같은 Ctrl+C를 보내 정상 종료시킨다
(게이트웨이가 회차 기록의 metrics.json·영상 조각을 마무리하게). 실기체가 공중이면 아무것도 끄지 않는다.
"""
import importlib.util
from pathlib import Path
import os
import re
import signal
import subprocess
import sys
import time
import webbrowser

from . import winproc

ROOT = Path(__file__).resolve().parents[2]
REAL_PORT = 8010        # 실기체 게이트웨이
FAKE_PORT = 8030        # 체험(가짜 드론) 게이트웨이. 실기체와 같이 켜도 겹치지 않게
CONSOLE_PORT = 8790     # 지도 콘솔(apps/console)
FAKE_MAP_PORT = 8795    # 가짜 지도 서버
DRONE_UDP = (8889, 8890, 11111)   # Tello 명령·상태·영상. Tello는 처음 command를 보낸 쪽에만 답한다
GROUND = {'grounded', 'disconnected', 'emergency'}
MODE_KO = {'disconnected': '연결 끊김', 'grounded': '지상', 'manual': '수동 비행', 'auto': '자율 탐색',
           'paused': '일시정지', 'landing': '착륙 중', 'emergency': '비상 정지'}
MONA_RUN = ROOT / 'modules/autonomy/mona_bench/run.ps1'
MONA_IMAGE = 'ssafy/mona-bench:noetic'
MONA_MODEL = ROOT / 'data/outputs/mona-bench/models/da_v2_metric_indoor_small_518x686_fp16.onnx'
PREPARE = r'scripts\start.cmd prepare'


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


def ask(question, default, allowed):
    try:
        answer = input(f'       {question} ').strip().lower()
    except EOFError:
        answer = ''
    return answer if answer in allowed else default


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


def mona_cmd(hand):
    cmd = ['powershell.exe', '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass', '-File', str(MONA_RUN),
           '-Action', 'Live', '-Seconds', '900', '-GoalForwardM', '5']
    return cmd + (['-AutoStart'] if hand else [])


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


# ---------- 보조 프로그램 ----------
def detect_ready():
    """사람 탐지를 켤 수 있으면 (YOLO 가중치 경로, None), 아니면 (None, 까닭)."""
    missing = [m for m in ('torch', 'ultralytics', 'transformers', 'cv2') if importlib.util.find_spec(m) is None]
    if missing:
        return None, f'사람 탐지 패키지가 없습니다({", ".join(missing)}). {PREPARE} 뒤 다시 켜세요.'
    sys.path.insert(0, str(ROOT))
    from modules.mapping.data.model_assets import person_checkpoint
    yolo = person_checkpoint()
    if not yolo.is_file():
        return None, f'사람 탐지 모델({yolo.name})이 없습니다. {PREPARE} 뒤 다시 켜세요.'
    return yolo, None


def start_detect(procs, gateway, label):
    if match(procs, 'gateway_ai_client', '--gateway ' + gateway):
        return say('       사람 탐지는 이미 켜져 있어요.')
    yolo, why = detect_ready()
    if why:
        return warn(why)
    winproc.start_window(f'Chill-Drone 사람 탐지 ({label})', ROOT, detect_cmd(gateway, yolo))
    say('       사람 탐지 켬(모델 올리는 데 10~30초). 영상을 켜야 결과가 나와요.')


def start_bridge(procs, map_url, gateway, label):
    if match(procs, 'gateway_map_bridge', '--gateway ' + gateway):
        return say('       지도 연결기는 이미 켜져 있어요.')
    winproc.start_window(f'Chill-Drone 지도 연결기 ({label})', ROOT, bridge_cmd(map_url, gateway), minimized=True)
    say('       지도 연결기 켬(작은 창).')


def start_console(gateway):
    """실기체 게이트웨이에 지도 콘솔을 붙인다. 켜졌으면 True."""
    map_url = url(CONSOLE_PORT)
    if winproc.get_json(map_url + '/api/live'):
        say('       지도 콘솔은 이미 켜져 있어요.')
        return True
    if not (ROOT / 'apps/console/node_modules/three/build/three.module.js').is_file():
        warn('지도 콘솔 웹 의존성이 없습니다: pnpm --dir apps/console install --frozen-lockfile')
        return False
    sys.path.insert(0, str(ROOT))
    try:
        from modules.mapping.data.model_assets import ModelAssetsMissing, require_local_models
        require_local_models()
    except ImportError as error:
        warn(f'지도 패키지가 없습니다({error.name}). {PREPARE} 뒤 다시 켜세요.')
        return False
    except ModelAssetsMissing as error:
        warn(f'지도 모델이 없습니다({error}). {PREPARE} 뒤 다시 켜세요.')
        return False
    winproc.start_window(f'Chill-Drone 지도 콘솔 ({CONSOLE_PORT})', ROOT, console_cmd(gateway, CONSOLE_PORT), minimized=True)
    if wait_for(lambda: winproc.get_json(map_url + '/api/live'), 30):
        say(f'       지도 콘솔 켬({map_url}).')
        return True
    warn('지도 콘솔이 30초 안에 켜지지 않았어요. "Chill-Drone 지도 콘솔" 창을 확인하세요.')
    return False


def start_mona(hand):
    if not MONA_RUN.is_file():
        return warn(f'MoNA 실행기가 없습니다: {MONA_RUN}')
    if not MONA_MODEL.is_file():
        return warn(f'MoNA 깊이 모델이 없습니다: {MONA_MODEL} (modules/autonomy/mona_bench/README.md)')
    try:
        image = subprocess.run(['docker', 'image', 'inspect', MONA_IMAGE, '--format', '{{.Id}}'],
                               capture_output=True, timeout=15, creationflags=winproc.CREATE_NO_WINDOW)
        running = subprocess.run(['docker', 'ps', '-q', '--filter', 'name=^mona-live$'], capture_output=True,
                                 timeout=15, creationflags=winproc.CREATE_NO_WINDOW).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return warn('Docker를 쓸 수 없습니다. Docker Desktop을 켜고 다시 실행하세요.')
    if image.returncode:
        return warn(f'Docker가 꺼져 있거나 MoNA 이미지({MONA_IMAGE})가 없습니다. '
                    r'modules\autonomy\mona_bench\run.ps1 -Action Build')
    if running:
        return warn('MoNA가 이미 돌고 있어요(mona-live).')
    winproc.start_window('MoNA 자율 (Enter로 멈춤)', ROOT, mona_cmd(hand))
    say('       MoNA 켬(새 창). 시작 점검 OK → "Running up to …"이 뜬 뒤 화면에서 영상을 켜세요(지상 60초에 꺼짐).')
    if hand:
        say('       손에 든 시험: 1.2 m쯤 들고 2~3초 멈춘 뒤 천천히 움직이세요. 땅(손)이라 속도 명령은 게이트웨이가 무시해요.')
    else:
        say('       비행: 이륙은 직접 하고, 날고 나서 화면의 "자율"을 눌러야 MoNA 명령이 기체로 가요.')


def open_page(page, no_browser):
    if not no_browser and not webbrowser.open(page):
        say('       위 주소를 브라우저에서 여세요.')


# ---------- 체험(가짜 드론) ----------
def run_fake(*, detect=True, map_=True, no_browser=False):
    say('=== 체험: 가짜 드론으로 관제 화면 ===')
    show_code()
    gateway = url(FAKE_PORT)
    if is_gateway(state(gateway)):
        say(f'[1/4] 가짜 드론 게이트웨이는 이미 켜져 있어요({gateway}).')
    else:
        busy = winproc.port_owners(tcp=(FAKE_PORT,))
        if busy:
            say(f'[1/4] {FAKE_PORT} 포트를 다른 프로그램이 쓰고 있어요. 끄고 다시 실행하세요.')
            say('\n'.join(owner_lines(busy, winproc.processes())))
            return 1
        winproc.start_window(f'Chill-Drone 가짜 드론 게이트웨이 ({FAKE_PORT})', ROOT, gateway_cmd(FAKE_PORT, fake=True))
        if not wait_for(lambda: is_gateway(state(gateway)), 25):
            say('[1/4] 25초 안에 게이트웨이가 켜지지 않았어요. 게이트웨이 창의 메시지를 확인하세요.')
            return 1
        say(f'[1/4] 가짜 드론 게이트웨이 켬({gateway}).')
    procs = winproc.processes()
    say('[2/4] 사람 탐지')
    if detect:
        start_detect(procs, gateway, '체험')
    else:
        say('       켜지 않음')
    say('[3/4] 가짜 지도(3D 점군)')
    if map_:
        map_url = url(FAKE_MAP_PORT)
        if winproc.get_json(map_url + '/api/live/state'):
            say('       가짜 지도 서버는 이미 켜져 있어요.')
        else:
            winproc.start_window(f'Chill-Drone 가짜 지도 ({FAKE_MAP_PORT})', ROOT, fake_map_cmd(FAKE_MAP_PORT), minimized=True)
            say('       가짜 지도 서버 켬(작은 창).')
        start_bridge(procs, map_url, gateway, '체험')
    else:
        say('       켜지 않음')
    page = gateway + '/#/live'
    say('[4/4] 화면 열기')
    open_page(page, no_browser)
    say()
    say(f'  실시간 감독  {page}')
    say(f'  탐색 결과    {gateway}/#/review')
    say(f'  폰 조작      {gateway}/#/control')
    say()
    say('사람 탐지: 화면에서 "영상 켜기" 뒤 영상 칸 "AI 합성"(가짜 영상은 색 막대라 사람 0명). 지도: 지도 칸 "3D 점군".')
    say(r'끄려면 scripts\start.cmd stop (메뉴 0).')
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


def run_real(*, phone=False, detect=None, map_=None, mona=None, no_browser=False, interactive=True):
    say('=== 실기체 관제 ===')
    show_code()
    gateway = url(REAL_PORT)
    drone_wifi = next((s for s in winproc.wifi_ssids() if re.match(r'(TELLO|RMTT)-', s, re.I)), None)
    if drone_wifi:
        say(f'[1/6] 드론 Wi-Fi: {drone_wifi}')
    else:
        say('[1/6] 드론 Wi-Fi(TELLO-… / RMTT-…)에 연결돼 있지 않아요. 드론을 켜고 연결하면 게이트웨이가 알아서 붙어요.')

    st = state(gateway)
    reuse = is_gateway(st)
    if reuse and st.get('source') != 'tello':
        say(f"[2/6] {REAL_PORT}에 실기체가 아닌 게이트웨이(source={st.get('source')})가 켜져 있어요. 끄고 다시 실행하세요.")
        return 1
    if reuse:
        say(f"[2/6] 실기체 게이트웨이는 이미 켜져 있어요({MODE_KO.get(st.get('mode'), st.get('mode'))}).")
        if phone:
            warn('이미 켜진 게이트웨이라 폰 접속(0.0.0.0)으로 바꾸지 못해요. 폰으로 열려면 끄고 다시 켜세요.')
    else:
        owners = winproc.port_owners(udp=DRONE_UDP, tcp=(REAL_PORT,))
        if owners:
            say('[2/6] 게이트웨이 포트나 드론 포트를 다른 프로그램이 쓰고 있어요(점검 화면 check_server.py, 다른 게이트웨이, '
                '직접 연결 지도 등). Tello는 처음 붙은 쪽에만 답해서 같이 켤 수 없어요. 끄고 다시 실행하세요.')
            say('\n'.join(owner_lines(owners, winproc.processes())))
            return 1
        say('[2/6] 겹치는 프로그램 없음. 방화벽 확인 중...')
        firewall_hint()

    if reuse:
        say('[3/6] 게이트웨이 그대로 사용')
    else:
        say(f'[3/6] 실기체 게이트웨이 켜는 중(새 창). 기록: {ROOT / "data/outputs/tello"}')
        winproc.start_window(f'Chill-Drone 실기체 게이트웨이 ({REAL_PORT})', ROOT, gateway_cmd(REAL_PORT, phone=phone))
        if not wait_for(lambda: is_gateway(state(gateway)), 25):
            say('       25초 안에 게이트웨이가 켜지지 않았어요. 게이트웨이 창의 메시지를 확인하세요.')
            return 1
    say('[4/6] 드론 연결 기다리는 중(최대 30초)...')
    connected = wait_for(lambda: (lambda s: s if is_gateway(s) and s.get('connected') else None)(state(gateway)), 30)
    if connected:
        show_drone(connected)
    else:
        warn('아직 드론이 안 붙었어요. 전원과 Wi-Fi를 확인하세요. 답이 없으면 드론을 껐다 켜면 됩니다(게이트웨이는 그대로 두면 다시 붙어요).')

    say('[5/6] 같이 켤 것' + (' (Enter = 대괄호 안 대문자)' if interactive else ''))
    if detect is None:
        detect = ask('사람 탐지(AI 합성·발견 목록)도 켤까요? [Y/n]', 'y', {'y', 'n'}) == 'y' if interactive else True
    if map_ is None:
        map_ = ask('3D 지도(지도 콘솔 + 지도 연결기)도 켤까요? [y/N]', 'n', {'y', 'n'}) == 'y' if interactive else False
    if mona is None:
        mona = {'h': 'hand', 'f': 'flight'}.get(
            ask('MoNA 자율도 켤까요? [N = 안 켬 / h = 손에 든 시험 / f = 비행]', 'n', {'n', 'h', 'f'})) if interactive else None
    procs = winproc.processes()
    say('       사람 탐지: ' + ('켬' if detect else '안 켬'))
    if detect:
        start_detect(procs, gateway, '실기체')
    say('       3D 지도: ' + ('켬' if map_ else '안 켬'))
    if map_ and start_console(gateway):
        start_bridge(procs, url(CONSOLE_PORT), gateway, '실기체')
        warn('지도 시작은 아직 지도 콘솔 화면만으로는 안 될 수 있어요(게이트웨이 모드). 막히면 지도 파트에 문의하세요.')
    say('       MoNA 자율: ' + {'hand': '손에 든 시험', 'flight': '비행'}.get(mona, '안 켬'))
    if mona in ('hand', 'flight'):
        start_mona(mona == 'hand')

    preset = 'autonomy' if mona in ('hand', 'flight') else 'mapping' if map_ else ''
    page = gateway + '/' + (f'?preset={preset}' if preset else '') + '#/live'
    say('[6/6] 화면 열기')
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
    say()
    say(r'이 창은 닫아도 돼요(켠 프로그램은 각자 창에서 돌아요). 끄려면 scripts\start.cmd stop (메뉴 0).')
    return 0


# ---------- 모두 끄기 ----------
STOP_ORDER = [   # (이름, 찾을 글자들, 뺄 글자들, 기다릴 초)
    ('사람 탐지', ('gateway_ai_client',), (), 10),
    ('지도 연결기', ('gateway_map_bridge',), (), 10),
    ('가짜 지도', ('fake_map_server.py',), (), 10),
    ('지도 콘솔', ('apps/console/serve.py',), (), 15),
    ('자율비행 웹(ROS 2)', ('autonomy-web/launcher.py',), (), 15),
    ('3D 지도 촬영 실행기', ('apps.launcher', ' map'), (), 20),
    ('3D 지도 촬영 실행기(옛 CMD)', ('MAPPING_LAUNCHER',), (), 20),
    ('게이트웨이', ('gateway_server.py',), (), 15),
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


def run_stop(*, yes=False):
    say('=== 모두 끄기 ===')
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # 같은 콘솔에 보낸 Ctrl+C에 이 프로그램이 멈추지 않게
    me = {os.getpid(), os.getppid()}
    procs = [p for p in winproc.processes() if p.pid not in me and p.ppid not in me]
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
        say('  먼저 착륙한 뒤 다시 실행하세요. 아무것도 끄지 않았어요.')
        return 1
    real = [st for st in states.values() if is_gateway(st) and st.get('source') == 'tello' and st.get('connected')]
    for st in real:
        show_drone(st)
    if real and not yes and ask('게이트웨이와 보조 프로그램을 모두 끌까요? [Y/n]', 'y', {'y', 'n'}) != 'y':
        say('  끄지 않았어요.')
        return 0
    stopped = False
    try:
        mona = subprocess.run(['docker', 'ps', '-q', '--filter', 'name=^mona-live$'], capture_output=True,
                              timeout=15, creationflags=winproc.CREATE_NO_WINDOW).stdout.strip()
        if mona:
            subprocess.run(['docker', 'stop', 'mona-live'], capture_output=True, timeout=60,
                           creationflags=winproc.CREATE_NO_WINDOW)
            say('       MoNA 자율: 껐어요')
            stopped = True
    except (OSError, subprocess.TimeoutExpired):
        pass
    targets = []
    for name, tokens, exclude, wait_s in STOP_ORDER:
        group = match(procs, *tokens, exclude=exclude)
        targets += group
        stopped |= stop_group(name, group, wait_s)
    # 3D 지도 촬영·자율비행 웹은 start.cmd 창 안에서 돈다. Ctrl+C를 받은 그 창의 cmd가
    # "일괄 작업을 끝내시겠습니까 (Y/N)?"로 남으니, 안의 프로그램이 끝났으면 창도 닫는다.
    by_pid = {p.pid: p for p in procs}
    windows = {p.ppid for p in targets} - {p.pid for p in targets}
    closed = 0
    for pid in windows:
        parent = by_pid.get(pid)
        if parent and 'start.cmd' in norm(parent.cmd).lower() and winproc.alive(pid):
            winproc.kill_tree(pid)
            closed += 1
    if closed:
        say(f'       실행기 창 {closed}개를 닫았어요.')
    if not stopped:
        say('  켜져 있는 실행기 프로그램이 없어요.')
    runs = sorted((d for d in (ROOT / 'data/outputs/tello').glob('*') if d.is_dir()), key=lambda d: d.stat().st_mtime)
    if runs:
        done = (runs[-1] / 'metrics.json').is_file()
        say(f'  마지막 회차 기록: {runs[-1]} ' + ('(정리 완료)' if done else '(metrics.json 없음: 아직 도는 중이거나 강제 종료)'))
    return 0
