"""체험(가짜 드론)·실기체 관제: 묻지 않고 전부 켜고, 실행기 창에서 Enter로 모두 끈다.

- 켜기 전에 빠진 패키지·모델·지도 콘솔 웹 의존성을 받는다(prepare.py).
- 보조 프로그램은 숨겨 실행하고 실행기 창에서 상태를 안내한다.
- 실기체는 게이트웨이 + 사람 탐지 + 지도 콘솔·지도 연결기 + 지도 자동 시작(mapkeeper.py).
- 실행기 창은 열어 둔다. Enter를 누르면 모두 끈다: 실기체가 공중이면 아무것도 끄지 않고, 땅이면 지도를 저장하고
  보조 프로그램을 정상 종료시킨다(게이트웨이가 회차 metrics.json·영상 조각을 마무리하게).
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
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
STOP_CONFIRM_S = 10     # 실기체 자율 비행이 땅인지 모를 때: 사람이 보고 이 안에 Enter를 한 번 더 누르면 끈다
MODE_KO = {'disconnected': '연결 끊김', 'grounded': '지상', 'manual': '수동 비행', 'auto': '자율 탐색',
           'paused': '일시정지', 'landing': '착륙 중', 'emergency': '비상 정지'}


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


def get_ready():
    say('[준비] 관제 화면의 Python·웹 패키지를 확인합니다.')
    if not prepare.ensure_all():
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
    checkpoint = person_checkpoint()
    if not checkpoint.is_file():
        return warn('사람 탐지 모델이 필요합니다. 비행 전에 scripts/start.cmd prepare-flight를 실행해 주세요.')
    winproc.start_service(f'people-{gateway.rsplit(":", 1)[-1]}', ROOT, detect_cmd(gateway, checkpoint))
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


def open_page(page, no_browser):
    if not no_browser and not webbrowser.open(page):
        say('       위 주소를 브라우저에서 여세요.')


def wait_and_stop(keeper=None):
    """실행기 창을 열어 두고, Enter를 누르면 모두 끈다(실기체가 공중이면 끄지 않고 다시 기다린다)."""
    say()
    say('이 창을 열어 두세요(지도 유지). Enter를 누르면 모두 끕니다. 실기체가 공중이면 끄지 않아요.')
    confirm_until = 0.0
    while True:
        try:
            input()
        except EOFError:
            return 0
        except KeyboardInterrupt:
            pass
        code = run_stop(keeper=keeper, force=time.monotonic() < confirm_until)
        if code == 0:
            time.sleep(4)   # 끈 결과를 읽을 시간
            return 0
        if code == 2:   # 땅인지 모름: 사람이 보고 10초 안에 한 번 더 누르면 끈다
            confirm_until = time.monotonic() + STOP_CONFIRM_S
            continue
        confirm_until = 0.0
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
FIREWALL_RULE = 'Tello UDP (Chill-Drone)'
DRONE_IN_UDP = (8890, 11111)   # Tello가 PC로 보내는 상태·영상. 이 Python(게이트웨이·자율 비행 중계)이 받아야 한다


def firewall_rules(exe):
    """이 실행 파일에 걸린 켜진 수신 규칙 [{action, profile, protocol, ports}]. 읽지 못하면 None."""
    script = ("$p = '%s'; $r = @(Get-NetFirewallApplicationFilter -ErrorAction SilentlyContinue | "
              "Where-Object { $_.Program -eq $p } | Get-NetFirewallRule | "
              "Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' }); "
              "ConvertTo-Json -Compress -Depth 3 -InputObject @($r | ForEach-Object { $f = $_ | Get-NetFirewallPortFilter; "
              "@{ action = [string]$_.Action; profile = [string]$_.Profile; protocol = [string]$f.Protocol; "
              "ports = @($f.LocalPort | ForEach-Object { [string]$_ }) } })" % exe.replace("'", "''"))
    out = winproc.powershell(script, timeout=90).strip()
    try:
        data = json.loads(out)
    except ValueError:
        return None
    return [data] if isinstance(data, dict) else data if isinstance(data, list) else None


def drone_network_category():
    """연결된 드론 Wi-Fi(TELLO-·RMTT-)의 네트워크 종류(Public·Private·DomainAuthenticated). 연결 전이면 None."""
    out = winproc.powershell("Get-NetConnectionProfile -ErrorAction SilentlyContinue | "
                             "ForEach-Object { $_.Name + '|' + $_.NetworkCategory }", timeout=20)
    for line in out.splitlines():
        name, _, category = line.strip().partition('|')
        if re.match(r'(TELLO|RMTT)-', name, re.I):
            return category
    return None


def _port_hit(ports, port):
    for p in ports or ():
        lo, _, hi = str(p).partition('-')
        if p == 'Any' or (p.isdigit() and int(p) == port) or (hi and lo.isdigit() and hi.isdigit() and int(lo) <= port <= int(hi)):
            return True
    return False


def firewall_state(rules, category):
    """(허용됨, 막힘): 드론 상태·영상(UDP 8890·11111)이 드론 Wi-Fi 네트워크 종류에서 허용·차단되는지.
    종류를 모르면(드론 Wi-Fi 연결 전) 새 Wi-Fi가 될 수 있는 공용·개인 둘 다 허용돼야 허용으로 본다. 차단 규칙이 허용보다 먼저다."""
    profiles = ['Public', 'Private'] if category is None else [{'DomainAuthenticated': 'Domain'}.get(category, category)]

    def applies(rule, profile):
        return (rule.get('profile') == 'Any' or profile in (rule.get('profile') or '')) and rule.get('protocol') in ('UDP', 'Any')
    allowed = all(any(r.get('action') == 'Allow' and applies(r, prof) and _port_hit(r.get('ports'), port) for r in rules)
                  for prof in profiles for port in DRONE_IN_UDP)
    blocked = any(r.get('action') == 'Block' and applies(r, prof) and any(_port_hit(r.get('ports'), port) for port in DRONE_IN_UDP)
                  for r in rules for prof in profiles)
    return allowed, blocked


def firewall_command(exe):
    return (f'New-NetFirewallRule -DisplayName "{FIREWALL_RULE}" -Direction Inbound -Program "{exe}" '
            f'-Protocol UDP -LocalPort 8890,11111 -Action Allow')


def firewall_fix_script(exe):
    """관리자 권한으로 돌릴 것: 이 실행 파일의 UDP 수신 차단 규칙은 끄고(지우지 않음, Windows 허용 창에서 취소하면 생긴다),
    이 실행 파일의 드론 상태·영상 포트(UDP 8890·11111)만 모든 네트워크 종류에서 여는 규칙을 하나 넣는다(전에 넣은 것은 바꿈)."""
    p = exe.replace("'", "''")
    return ("$ErrorActionPreference = 'Stop'; $p = '%s'; try { "
            "Get-NetFirewallApplicationFilter | Where-Object { $_.Program -eq $p } | Get-NetFirewallRule | "
            "Where-Object { $_.Direction -eq 'Inbound' -and $_.Action -eq 'Block' -and $_.Enabled -eq 'True' -and "
            "(($_ | Get-NetFirewallPortFilter).Protocol -in @('UDP', 'Any')) } | Disable-NetFirewallRule; "
            "Get-NetFirewallRule -DisplayName '%s' -ErrorAction SilentlyContinue | "
            "Where-Object { ($_ | Get-NetFirewallApplicationFilter).Program -eq $p } | Remove-NetFirewallRule; "
            "New-NetFirewallRule -DisplayName '%s' -Direction Inbound -Program $p -Protocol UDP -LocalPort 8890,11111 "
            "-Action Allow -Profile Any | Out-Null; exit 0 } catch { exit 1 }" % (p, FIREWALL_RULE, FIREWALL_RULE))


def ensure_firewall():
    """실기체에서 이 Python(게이트웨이·자율 비행 중계)이 드론 상태·영상(UDP 8890·11111)을 받게 방화벽을 맞춘다.
    허용 규칙이 없거나 차단 규칙이 있으면 Windows 관리자 확인 창을 띄운다(사람이 '예'를 눌러야 바뀐다). 됐으면 True."""
    exe = getattr(sys, '_base_executable', sys.executable)
    rules = firewall_rules(exe)
    if rules is None:
        warn('방화벽 규칙을 읽지 못했어요. 드론 상태·영상이 안 들어오면 관리자 PowerShell에서 직접 넣으세요:')
        say(f'       {firewall_command(exe)}')
        return False
    category = drone_network_category()
    allowed, blocked = firewall_state(rules, category)
    if allowed and not blocked:
        say('       방화벽: 이 Python이 드론 상태·영상(UDP 8890·11111)을 받을 수 있어요.')
        return True
    where = {'Public': '공용', 'Private': '개인'}.get(category, category) if category else '공용·개인'
    say(f'       방화벽: 드론 Wi-Fi({where} 네트워크)에서 드론 상태·영상(UDP 8890·11111)이 '
        + ('차단 규칙에 막혀 있어요.' if blocked else '허용돼 있지 않아요.'))
    say('       Windows 관리자 확인 창이 뜨면 "예"를 누르세요. 이 Python의 그 두 포트만 열고, 이 Python의 UDP 차단 규칙은 꺼요.')
    result = winproc.powershell_elevated(firewall_fix_script(exe))
    if result == 'ok':
        allowed, blocked = firewall_state(firewall_rules(exe) or [], category)
        if allowed and not blocked:
            say(f'       방화벽 허용 규칙을 넣었어요("{FIREWALL_RULE}"). 지우려면 관리자 PowerShell에서 '
                f'Remove-NetFirewallRule -DisplayName "{FIREWALL_RULE}"')
            return True
    warn('방화벽을 맞추지 못했어요(' + ('확인 창에서 "아니요"' if result == 'canceled' else '실패') + '). '
         '드론 상태·영상이 안 들어오면 관리자 PowerShell에서 직접 넣으세요:')
    say(f'       {firewall_command(exe)}')
    return False


def phone_networks():
    """폰이 이 PC에 붙을 수 있는 망 [(주소, 네트워크 이름, 종류 Public·Private…)]. 드론 Wi-Fi(192.168.10.*)·WSL 등 가상 어댑터는 뺀다."""
    out = winproc.powershell("Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | "
                             "Where-Object { $_.InterfaceAlias -notlike 'vEthernet*' } | ForEach-Object { "
                             "$p = Get-NetConnectionProfile -InterfaceIndex $_.InterfaceIndex -ErrorAction SilentlyContinue | "
                             "Select-Object -First 1; $_.IPAddress + '|' + [string]$p.Name + '|' + [string]$p.NetworkCategory }")
    nets = []
    for line in out.splitlines():
        ip, name, category = (line.strip().split('|') + ['', ''])[:3]
        if ip and not ip.startswith(('127.', '169.254.', '192.168.10.')):
            nets.append((ip, name.strip(), category.strip()))
    return nets


def tcp_state(rules, category, port):
    """(허용됨, 막힘): 이 Python 규칙(firewall_rules 모양)으로 그 네트워크 종류에서 TCP port를 받을 수 있는지. 폰 접속용.
    종류를 모르면 프로필이 Any인 규칙만 센다. 차단 규칙이 허용보다 먼저다."""
    profile = {'DomainAuthenticated': 'Domain'}.get(category, category) or ''

    def applies(rule):
        return ((rule.get('profile') == 'Any' or (profile and profile in (rule.get('profile') or '')))
                and rule.get('protocol') in ('TCP', 'Any') and _port_hit(rule.get('ports'), port))
    return (any(r.get('action') == 'Allow' and applies(r) for r in rules),
            any(r.get('action') == 'Block' and applies(r) for r in rules))


def phone_lines(port, live=None):
    """폰에서 열 주소(같은 망마다 네트워크 이름·방화벽)와 안내. live: 폰으로 열 전체 감독 화면 경로(예: ?preset=autonomy#/live).
    방화벽은 이 Python의 수신 허용(그 망 종류)이나 그 포트의 TCP 허용 규칙이 있으면 열린 것으로 본다. 규칙은 만들지 않는다."""
    nets = phone_networks()
    if not nets:
        return ['  폰과 같은 망(폰 핫스팟·USB 테더링)에 연결된 주소가 없어요. PC를 폰 핫스팟에 연결하고 다시 켜세요.']
    exe = getattr(sys, '_base_executable', sys.executable)
    rules = firewall_rules(exe) or []
    rule = winproc.powershell("Get-NetFirewallPortFilter -ErrorAction SilentlyContinue | "
                              f"Where-Object {{ $_.Protocol -eq 'TCP' -and $_.LocalPort -eq '{port}' }} | Get-NetFirewallRule | "
                              "Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' } | "
                              "ForEach-Object { $_.DisplayName }", timeout=60).strip()
    lines, closed = [], False
    for ip, name, category in nets:
        allowed, blocked = tcp_state(rules, category, port)
        ok = not blocked and (allowed or bool(rule))
        closed = closed or not ok
        where = ' · '.join(x for x in (name, {'Public': '공용', 'Private': '개인'}.get(category, category)) if x)
        note = ' · '.join(x for x in (where, '' if ok else '방화벽에 막힐 수 있음') if x)
        lines.append(f'  폰 조작      http://{ip}:{port}/#/control' + (f'  ({note})' if note else ''))
    if live:
        lines.append(f'  폰 감독      위 주소 끝을 /{live} 로 바꾸면 전체 감독 화면')
    if rule:
        lines.append(f'  방화벽 허용 규칙 있음: {rule.splitlines()[0]} (시험이 끝나면 관리자 PowerShell에서 지우세요)')
    if closed:
        lines.append('  폰에서 안 열리면 관리자 PowerShell에서 방화벽 규칙을 추가하세요(시험 뒤 지우기):')
        lines.append(f'  New-NetFirewallRule -DisplayName "Drone gateway {port} (test)" -Direction Inbound -Protocol TCP '
                     f'-LocalPort {port} -RemoteAddress LocalSubnet -Action Allow')
    lines.append('  인증이 없어 그 망에 붙은 누구나 열 수 있어요. 폰 핫스팟처럼 우리 기기만 있는 망에서만 쓰세요.')
    return lines


def run_real(*, phone=False, no_browser=False, interactive=None):
    say('=== 2 실기체 관제: 게이트웨이·사람 탐지·3D 지도 ===')
    show_code()
    if not get_ready():
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
        ensure_firewall()
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
        warn('아직 드론이 안 붙었어요. 전원과 Wi-Fi를 확인하세요. 붙으면 게이트웨이가 알아서 이어 가요.')

    say('[4/5] 사람 탐지·3D 지도')
    procs = winproc.processes()
    start_detect(procs, gateway)
    keeper = None
    if start_console(gateway):
        start_bridge(procs, url(CONSOLE_PORT), gateway)
        keeper = mapkeeper.MapKeeper(gateway, url(CONSOLE_PORT), say)
        say('       지도: 영상이 켜지면(이륙하면 자동) 실행기가 지도를 시작하고 유지해요.')

    page = gateway + '/#/live'
    say('[5/5] 화면 열기')
    open_page(page, no_browser)
    say()
    say(f'  실시간 감독  {page}')
    say(f'  지도 콘솔    {gateway}/#/map (촬영 영상 복원·3D 지도 편집)')
    say(f'  탐색 결과    {gateway}/#/review')
    if phone and not reuse:
        for line in phone_lines(REAL_PORT):
            say(line)
    else:
        say(f'  폰 조작      {gateway}/#/control (이 PC에서만. 폰으로 열려면 start.cmd real --phone)')
    if interactive if interactive is not None else sys.stdin.isatty():
        if keeper:
            keeper.start()
        return wait_and_stop(keeper)
    say('  (창 입력이 없어 기다리지 않아요. 지도 자동 시작과 Enter 끄기는 실행기 창에서 실행할 때만 돼요.)')
    return 0


# ---------- 모두 끄기(실행기 창에서 Enter) ----------
STOP_ORDER = [   # (이름, 찾을 글자들, 기다릴 초)
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


def wsl_wrappers(procs, *marks):
    """start.cmd sim·autonomy가 띄운 wsl.exe(명령줄에 그 launch·보기 노드 이름이 있는 것)."""
    return [p for p in procs if 'wsl' in norm(p.cmd).lower() and any(m in p.cmd for m in marks)]


def stop_simulation(procs):
    """start.cmd sim·autonomy가 띄운 WSL 자율 비행·보기 노드(wsl.exe 명령줄로 알아본다)를 끈다.
    시뮬레이터는 mission.launch.py·gz sim, 실기체는 이 실행기가 켠 real.launch.py만(땅일 때만 여기까지 온다).
    (끈 것이 있었나, 다 꺼졌나)."""
    from . import sim   # sim이 drone을 쓰므로 여기서 불러온다
    wrappers = wsl_wrappers(procs, sim.SIM_MARK, sim.VIEW_MARK, sim.REAL_MARK)
    if not wrappers:
        return False, True
    real = any(sim.REAL_MARK in p.cmd for p in wrappers)
    ok = sim.stop_sim(real=real)
    for p in wrappers:   # WSL 안이 꺼지면 감싼 wsl.exe도 끝난다. 남았으면 정리
        if winproc.alive(p.pid):
            winproc.kill_tree(p.pid)
    what = '자율 비행(실기체)·보기 노드(WSL)' if real else '자율 비행 시뮬레이터·보기 노드(WSL)'
    say(f'       {what}: ' + ('껐어요' if ok else '일부가 남았을 수 있어요(WSL에서 확인)'))
    return True, ok


def ros_airborne(st):
    """실기체 자율 비행(보기 게이트웨이)이 이륙·비행·착륙 중인지. 시뮬레이터는 끄는 걸 막지 않는다(실기체일 때만 부른다)."""
    return is_gateway(st) and st.get('source') == 'ros' and (bool(st.get('flight_busy')) or st.get('mode') not in GROUND)


def ros_on_ground(st, fs):
    """실기체 자율 비행이 땅이라고 확인되는가(11/16 점검 A5): 보기 게이트웨이가 지상(끊김 아님)이거나,
    보기 노드가 받은 드라이버 상태가 2초 안의 것이고 드론과 붙어 있으며 땅. 둘 다 아니면 모른다."""
    if is_gateway(st) and st.get('source') == 'ros' and st.get('mode') == 'grounded' and not st.get('flight_busy'):
        return True
    return (isinstance(fs, dict) and fs.get('flying') is False and not fs.get('busy') and fs.get('connected') is not False
            and isinstance(fs.get('age_s'), (int, float)) and fs['age_s'] < 2.0)


def run_stop(*, keeper=None, force=False):
    """모두 끈다. 0 껐음, 1 실기체가 공중이라 안 끔, 2 실기체 자율 비행이 땅인지 몰라 안 끔(force면 끈다)."""
    say('=== 모두 끄기 ===')
    me = {os.getpid(), os.getppid()}
    procs = [p for p in winproc.processes() if p.pid not in me]
    gateways = sorted({url(gateway_port(p.cmd)) for p in match(procs, 'gateway_server.py')} |
                      {m.group(1) for p in match(procs, 'apps/console/serve.py', '--tello-gateway')
                       for m in [re.search(r'--tello-gateway\s+(\S+)', norm(p.cmd))] if m})
    states = {g: state(g) for g in gateways}
    from . import sim
    real_ros = bool(wsl_wrappers(procs, sim.REAL_MARK))   # 실기체 자율 비행: 공중이면 아무것도 끄지 않는다
    if not real_ros and match(procs, sim.RELAY_MARK):   # 실기체 자율 비행은 Windows 중계가 있어야 드론에 닿는다
        real_ros = 'real' in sim.running_launches()   # 이 실행기가 켜지 않은(손으로 켠) real.launch도 실기체로 본다
    flying = [g for g, st in states.items() if airborne(st) or (real_ros and ros_airborne(st))]
    if flying:
        for g in flying:
            st = states[g]
            busy = f", {st['flight_busy']} 중" if st.get('flight_busy') else ''
            say(f"  드론이 공중에 있어요({g}: {MODE_KO.get(st.get('mode'), st.get('mode'))}{busy}).")
        say('  아무것도 끄지 않았어요.')
        return 1
    if real_ros:   # 게이트웨이가 답하지 않거나 끊김이면 보기 노드의 드라이버 상태로도 본다
        view = sim.view_state()
        fs = view.get('flight_status') if isinstance(view, dict) else None
        if isinstance(fs, dict) and (fs.get('flying') or fs.get('busy')):
            say(f"  드론이 공중에 있어요(보기 노드의 드라이버 상태: {'비행 중' if fs.get('flying') else str(fs.get('busy')) + ' 중'}).")
            say('  아무것도 끄지 않았어요.')
            return 1
        if not force and not ros_on_ground(states.get(url(sim.SIM_PORT)), fs):
            say('  실기체 자율 비행이 땅에 있는지 확인하지 못했어요(게이트웨이·보기 노드가 드라이버 상태를 못 받음).')
            say(f'  드론이 땅에 있는 것을 눈으로 확인했으면 {STOP_CONFIRM_S}초 안에 Enter를 한 번 더 누르세요. 그때 끕니다.')
            return 2
    if keeper:
        keeper.stop()
    stopped = False
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)  # 이전 버전 서비스도 종료할 수 있다.
    try:
        for name, tokens, wait_s in STOP_ORDER:
            stopped |= stop_group(name, match(procs, *tokens), wait_s)
    finally:
        signal.signal(signal.SIGINT, previous)
    sim_stopped, sim_ok = stop_simulation(procs)
    stopped |= sim_stopped
    # Windows 중계(실기체 자율 비행)는 WSL 드라이버가 다 끝난 뒤에만 끈다(드라이버가 끝까지 Tello와 이야기하게).
    # WSL이 다 안 꺼졌거나 이 실행기가 켜지 않은 real.launch가 돌면 중계는 그대로 둔다
    if real_ros and not (sim_stopped and sim_ok) and match(procs, sim.RELAY_MARK):
        warn('WSL 자율 비행(real.launch)이 아직 돌아 Windows 중계는 그대로 뒀어요(드라이버가 드론과 이야기하게). '
             '그 자율 비행을 땅에서 끈 뒤 다시 Enter를 누르세요.')
    else:
        stopped |= stop_group('자율 비행 중계(Windows)', match(procs, 'tello_relay.py'), 10)
    stopped |= stop_group('가짜 Tello', match(procs, 'fake_tello.py', exclude=('gateway_server.py',)), 5)
    if not stopped:
        say('  켜져 있는 프로그램이 없어요.')
    runs = sorted((d for d in (ROOT / 'data/outputs/tello').glob('*') if d.is_dir()), key=lambda d: d.stat().st_mtime)
    if runs:
        done = (runs[-1] / 'metrics.json').is_file()
        say(f'  마지막 회차 기록: {runs[-1]} ' + ('(정리 완료)' if done else '(metrics.json 없음: 아직 도는 중이거나 강제 종료)'))
    return 0
