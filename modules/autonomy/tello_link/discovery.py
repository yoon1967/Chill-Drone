"""Local Wi-Fi candidates and read-only Tello resource inspection.

SSID discovery is not proof of a reachable SDK. Only an explicit connect enters
SDK mode. No flight or network configuration commands are sent here. Video
streaming starts only on an explicit preview request and has a renewable lease.
"""
import copy
import locale
import re
import subprocess
import sys
import threading
import time

from .tello_link import TelloLink
from .wifi import scan_wifi, current_wifi


def run_os(args):
    result = subprocess.run(args, capture_output=True, timeout=12,
                            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0)
    text = result.stdout.decode(locale.getencoding(), errors='replace')
    if args[:4] == ['netsh', 'wlan', 'show', 'networks'] and (
            'ms-settings:privacy-location' in text or 'location permission' in text.lower()):
        raise RuntimeError('wifi_location_permission_required')
    if result.returncode:
        raise RuntimeError('wifi_os_request_failed_check_permissions_and_saved_profile')
    return text


def wifi_candidates(runner=run_os, platform=None):
    return scan_wifi(runner, platform)


QUERIES = [
    ('battery?', '배터리', '%'), ('height?', '고도', 'cm'),
    ('temp?', '온도', '°C'), ('attitude?', '자세', 'SDK 원시값'),
    ('baro?', '기압 고도', 'SDK 원시값 · 단위 미검증'),
    ('acceleration?', '가속도', 'SDK 원시값 · 단위 미검증'),
    ('tof?', '하향 거리', 'SDK 원시값'), ('wifi?', 'Wi-Fi 품질', 'SDK 원시값'),
    ('sdk?', 'SDK 버전', ''), ('ext version?', 'TT 확장 버전', ''),
    ('ext tof?', 'TT 전방 거리', 'SDK 원시값 · 범위 밖 값은 유효 거리 아님'),
]


class DroneDiscovery:
    def __init__(self, link_factory=TelloLink, runner=run_os, borrowed=lambda: None, busy=lambda: False,
                 video_factory=None, borrowed_video=lambda: None):
        self.link_factory, self.runner = link_factory, runner
        self.borrowed, self.busy = borrowed, busy
        self.lock = threading.RLock()
        self.link = None
        self.selected = None
        self.candidates = {}
        self.resources = []
        self.checked_at = None
        self.video = None
        self.video_error = None
        self.video_factory = video_factory
        self.borrowed_video = borrowed_video
        self.video_until = 0
        self.stream_requested = False
        self.preview_timer = None
        self.manual_disconnected = False
        self.disconnected_session = None
        self.mapping_session = None
        self.wifi_lock = threading.Lock()
        self.wifi_sample = {'status': 'pending', 'connected': None, 'ssid': None, 'signal_pct': None}
        self.wifi_checked = -float('inf')
        self.wifi_worker = None

    def refresh_wifi(self, force=False):
        # OS commands may take seconds: never delay camera/state polling or bind
        # the drone command ports just to inspect Wi-Fi signal.
        with self.wifi_lock:
            if self.wifi_worker and self.wifi_worker.is_alive():
                return
            if not force and time.monotonic() - self.wifi_checked < 3:
                return
            preferred = (self.selected or {}).get('ssid')
            def update():
                try:
                    sample = {**current_wifi(self.runner, preferred=preferred), 'status': 'ok'}
                except (OSError, RuntimeError, subprocess.TimeoutExpired):
                    sample = {'status': 'unavailable', 'connected': None, 'ssid': None, 'signal_pct': None}
                with self.wifi_lock:
                    self.wifi_sample = sample
                    self.wifi_checked = time.monotonic()
            self.wifi_worker = threading.Thread(target=update, daemon=True, name='wifi-observation')
            self.wifi_worker.start()

    def wifi_state(self):
        self.refresh_wifi()
        with self.wifi_lock:
            age = time.monotonic() - self.wifi_checked
            value = dict(self.wifi_sample)
            value['age_s'] = round(age, 1) if age != float('inf') else None
            if age > 10 and value['status'] == 'ok':
                value.update(status='stale', connected=None, signal_pct=None)
            return value

    def scan(self):
        try:
            items = wifi_candidates(self.runner)
            warning = None
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            items, warning = [], str(error)
        with self.lock:
            self.candidates = {item['id']: item for item in items}
        self.refresh_wifi(force=True)
        return {'items': items, 'warning': warning}

    def release(self):
        with self.lock:
            self.stop_preview()
            if self.link:
                self.link.close()
            self.link = None
            self.resources = []
            self.selected = None
            self.checked_at = None

    def stop_preview(self):
        with self.lock:
            if self.preview_timer:
                self.preview_timer.cancel()
                self.preview_timer = None
            try:
                if self.link and self.stream_requested:
                    self.link.send('streamoff', timeout=1, source='console-resources')
            finally:
                if self.video:
                    self.video.stop()
                self.video = None
                self.stream_requested = False

    def expire_preview(self):
        with self.lock:
            if not self.video:
                return
            remaining = self.video_until - time.monotonic()
            if remaining <= 0:
                self.stop_preview()
                return
            self.preview_timer = threading.Timer(remaining, self.expire_preview)
            self.preview_timer.daemon = True
            self.preview_timer.start()

    def start_preview(self):
        with self.lock:
            if self.borrowed():
                return self.state()
            if self.busy() or not self.link:
                raise RuntimeError('connect_a_drone_first')
            self.video_error = None
            try:
                if not self.video:
                    from .tello_video import TelloVideo
                    self.video = (self.video_factory or TelloVideo)()
                    self.video.start()
                    self.stream_requested = True
                    if not self.link.send('streamon', timeout=2, source='console-resources').ok:
                        raise RuntimeError('video_stream_not_confirmed')
                self.video_until = time.monotonic() + 15
                if not self.preview_timer:
                    self.expire_preview()
            except (ImportError, OSError, RuntimeError):
                self.stop_preview()
                self.video_error = '영상 수신을 시작하지 못했습니다. av·OpenCV 설치와 UDP 11111 사용 여부를 확인하세요.'
            return self.state()

    def preview(self):
        with self.lock:
            video = self.borrowed_video() or self.video
            if video is self.video and video:
                self.video_until = time.monotonic() + 15
            value = video.jpeg_snapshot() if video else None
            return value[0] if value and time.monotonic() - value[1] < 2 else None

    def connect(self, key):
        with self.lock:
            if self.busy():
                raise RuntimeError('drone_input_is_active')
            if key != 'current-network' and key not in self.candidates:
                raise ValueError('scan_and_select_a_drone_first')
            # Explicit current-network action; never invent a Wi-Fi scan result.
            item = ({'id': key, 'ssid': '현재 네트워크의 드론', 'kind': 'current', 'verified': False}
                    if key == 'current-network' else self.candidates[key])
            self.release()
            if item['kind'] == 'wifi':
                ssid = item['ssid']
                if sys.platform == 'win32':
                    self.runner(['netsh', 'wlan', 'connect', 'name=' + ssid, 'ssid=' + ssid])
                else:
                    self.runner(['nmcli', 'device', 'wifi', 'connect', ssid])
                # OS association is asynchronous; SDK success is the only success criterion.
                time.sleep(2)
            link = self.link_factory(allow_flight=False, allow_config=False)
            try:
                link.start()
                if not link.send('command', timeout=3, source='console-discovery').ok:
                    raise RuntimeError('tello_sdk_unavailable_check_wifi')
            except OSError as error:
                link.close()
                raise RuntimeError('drone_ports_busy_or_unavailable') from error
            except Exception:
                link.close()
                raise
            self.link, self.selected = link, dict(item)
            self.manual_disconnected = False
            self.disconnected_session = None
            self.mapping_session = None
            self.checked_at = time.time()
            return self.state()

    def state(self):
        with self.lock:
            if self.video and time.monotonic() > self.video_until:
                self.stop_preview()
            borrowed = self.borrowed()
            link = borrowed or self.link
            sample = link.latest_state() if link else None
            fresh = bool(sample and sample.get('age_s', 999) < 2)
            recent = self.checked_at is not None and time.time() - self.checked_at < 5
            connected = bool(link and (fresh or (not borrowed and recent)))
            observed = [{'name': key, 'value': value, 'status': 'observed' if fresh else 'stale',
                         'unit': 'SDK 원시값'} for key, value in (sample or {}).get('raw', {}).items()]
            return {'connected': connected, 'selected': self.selected, 'wifi': self.wifi_state(),
                    'owner': 'video-input' if borrowed else 'discovery',
                    'checked_at': self.checked_at, 'telemetry_age_s': (sample or {}).get('age_s'),
                    'resources': copy.deepcopy(self.resources if not borrowed else []) + observed,
                    'video': {'status': 'input-managed' if borrowed else 'receiving' if self.video else 'sdk-declared',
                              'error': self.video_error,
                              'note': '영상은 입력 시작 후 확인합니다. SDK 제공 형식: H.264. 하향 카메라·추가 센서는 기체별 확인이 필요합니다.'}}

    def explore(self):
        with self.lock:
            if self.borrowed():
                return self.state()  # Never inject queries into an active video/control session.
            if self.busy():
                raise RuntimeError('drone_input_is_active')
            if not self.link:
                raise RuntimeError('connect_a_drone_first')
            if not self.link.send('command', timeout=2, source='console-discovery').ok:
                self.release()
                raise RuntimeError('tello_sdk_unavailable_check_wifi')
            self.checked_at = time.time()
            self.resources = []
            for command, name, unit in QUERIES:
                result = self.link.send(command, timeout=.8, source='console-resources')
                raw = result.response
                rejected = raw and re.match(r'(?i)^(error|unknown|unsupported)', raw)
                status = 'unsupported' if rejected else 'observed' if result.ok and raw else 'unconfirmed'
                self.resources.append({'name': name, 'command': command, 'unit': unit,
                                       'value': raw, 'status': status})
            self.checked_at = time.time()
            return self.state()
