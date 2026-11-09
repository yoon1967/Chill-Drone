"""Windows 프로세스·창·네트워크 도우미(표준 라이브러리만). 실행기(켜기·끄기)가 쓴다."""
import ctypes
from dataclasses import dataclass
import json
import locale
import re
import subprocess
import sys
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

CREATE_NO_WINDOW = 0x08000000
STILL_ACTIVE = 259
_opener = build_opener(ProxyHandler({}))   # 같은 PC 주소는 프록시를 거치지 않는다


@dataclass(frozen=True)
class Proc:
    pid: int
    ppid: int
    cmd: str


def get_json(url, timeout=2.0):
    """GET 해서 JSON을 돌려준다. 응답이 없거나 JSON이 아니면 None."""
    try:
        with _opener.open(url, timeout=timeout) as response:
            return json.load(response)
    except (OSError, ValueError):
        return None


def post_json(url, body, headers=None, timeout=5.0):
    """JSON을 POST 해서 응답 JSON을 돌려준다. 오류 응답이면 그 JSON(없으면 {'error': 'http_코드'}), 연결 실패면 None."""
    request = Request(url, data=json.dumps(body).encode('utf-8'), method='POST',
                      headers={'Content-Type': 'application/json', **(headers or {})})
    try:
        with _opener.open(request, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as error:
        try:
            return json.load(error)
        except ValueError:
            return {'error': f'http_{error.code}'}
    except (OSError, ValueError):
        return None


def powershell(script, timeout=30):
    """PowerShell 명령의 표준 출력(UTF-8). 실패하면 빈 문자열."""
    try:
        result = subprocess.run(
            ['powershell', '-NoProfile', '-NonInteractive', '-Command',
             '[Console]::OutputEncoding=[Text.Encoding]::UTF8; ' + script],
            capture_output=True, timeout=timeout, creationflags=CREATE_NO_WINDOW)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return result.stdout.decode('utf-8', errors='replace')


def processes():
    """명령줄이 보이는 프로세스 목록."""
    out = powershell('Get-CimInstance Win32_Process | Where-Object CommandLine | '
                     'Select-Object ProcessId,ParentProcessId,CommandLine | ConvertTo-Json -Compress')
    try:
        data = json.loads(out) if out.strip() else []
    except ValueError:
        return []
    if isinstance(data, dict):
        data = [data]
    return [Proc(int(d['ProcessId']), int(d.get('ParentProcessId') or 0), d['CommandLine']) for d in data]


def port_owners(udp=(), tcp=()):
    """[(프로토콜, 포트, PID)] — UDP는 열린 것, TCP는 듣고 있는 것."""
    parts = []
    if udp:
        parts.append("Get-NetUDPEndpoint -LocalPort %s -ErrorAction SilentlyContinue | "
                     "ForEach-Object { 'udp ' + $_.LocalPort + ' ' + $_.OwningProcess }" % ','.join(map(str, udp)))
    if tcp:
        parts.append("Get-NetTCPConnection -State Listen -LocalPort %s -ErrorAction SilentlyContinue | "
                     "ForEach-Object { 'tcp ' + $_.LocalPort + ' ' + $_.OwningProcess }" % ','.join(map(str, tcp)))
    owners = set()
    for line in powershell('; '.join(parts)).splitlines() if parts else []:
        p = line.split()
        if len(p) == 3 and p[1].isdigit() and p[2].isdigit():
            owners.add((p[0], int(p[1]), int(p[2])))
    return sorted(owners)


def wifi_ssids(text=None):
    """연결된 Wi-Fi 이름들(netsh). BSSID 줄은 빼고 SSID 줄만."""
    if text is None:
        try:
            raw = subprocess.run(['netsh', 'wlan', 'show', 'interfaces'], capture_output=True,
                                 timeout=10, creationflags=CREATE_NO_WINDOW).stdout
        except (OSError, subprocess.TimeoutExpired):
            return []
        text = raw.decode(locale.getpreferredencoding(False), errors='replace')
    return [m.group(1) for m in re.finditer(r'^\s*SSID\s*:\s*(.+?)\s*$', text, re.M)]


def _quote(arg):
    return '"' + str(arg).replace('"', '\\"') + '"'


def start_window(title, cwd, argv, minimized=False):
    """새 콘솔 창에서 띄운다. 창 제목을 달고, 그 창에서 Ctrl+C로 끌 수 있다.
    인자는 모두 따옴표로 감싼다(cmd가 & | < > 같은 문자를 해석하지 않게)."""
    line = 'start ' + _quote(title) + (' /MIN' if minimized else '') + ' /D ' + _quote(cwd) + ' ' + \
        ' '.join(_quote(a) for a in argv)
    subprocess.Popen('cmd /d /c ' + line, creationflags=CREATE_NO_WINDOW)


_CTRL_C_HELPER = r'''
import ctypes, sys, time
k = ctypes.windll.kernel32
k.FreeConsole()
if not k.AttachConsole(int(sys.argv[1])):
    sys.exit(1)
k.SetConsoleCtrlHandler(None, True)
k.GenerateConsoleCtrlEvent(0, 0)
time.sleep(0.5)
'''


def send_ctrl_c(pid):
    """다른 콘솔 창의 프로그램에 Ctrl+C를 보낸다(그 창에서 Ctrl+C를 누른 것과 같다).
    숨은 Python이 그 콘솔에 붙어서 보내고, 자기는 Ctrl+C를 무시한다."""
    try:
        return subprocess.run([sys.executable, '-c', _CTRL_C_HELPER, str(pid)],
                              creationflags=CREATE_NO_WINDOW, timeout=15).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def alive(pid):
    """프로세스가 아직 살아 있는지(os.kill(pid, 0)은 Windows에서 프로세스를 끝내 버리므로 쓰지 않는다)."""
    kernel = ctypes.windll.kernel32
    handle = kernel.OpenProcess(0x1000, False, int(pid))   # PROCESS_QUERY_LIMITED_INFORMATION
    if not handle:
        return False
    code = ctypes.c_ulong()
    ok = kernel.GetExitCodeProcess(handle, ctypes.byref(code))
    kernel.CloseHandle(handle)
    return bool(ok) and code.value == STILL_ACTIVE


def kill_tree(pid):
    subprocess.run(['taskkill', '/F', '/T', '/PID', str(pid)], capture_output=True,
                   creationflags=CREATE_NO_WINDOW)
