"""실기체 3D 지도: 게이트웨이 영상이 켜지면 지도 콘솔에 Tello 지도 세션을 시작하고, 2초마다 heartbeat로 유지한다.

지도 콘솔 화면만으로는 게이트웨이 모드 지도를 시작할 수 없어서 실행기가 대신 시작한다(지도 파트 코드는 그대로).
시작 설정은 콘솔 화면의 기본값과 같다(인명 구조 지도 켬, 깊이 384, 32,000점, 초당 5장, 보정은 콘솔의 Tello 내장 기본값).
콘솔은 heartbeat가 45초 끊기면 세션을 정리하므로, 실행기 창이 열려 있어야 지도가 이어진다.

- 영상이 30초 안에 돌아오면 콘솔이 같은 지도를 잇는다. 더 길게 끊기면 콘솔이 세션을 끝내고(지도는 콘솔 기록에 남는다)
  영상이 다시 켜질 때 새 지도를 시작한다. 이건 실패로 세지 않는다.
- 사람이 지도 콘솔에서 지도를 마무리하면("드론 연결 해제"·"촬영 종료") 저장할 수 있게 그대로 두고,
  다음 이륙이나 영상을 껐다 켤 때 새 지도를 시작한다.
- 콘솔에서 직접 시작한 지도가 돌고 있으면 건드리지 않고 기다린다.
"""
import threading
import time

from . import winproc

HEADERS = {'X-Mapping-Client': '1'}   # 콘솔은 이 머리글이 있는 같은 PC 요청만 받는다
START_CONFIG = {
    'width': 960, 'height': 720, 'fps': 24, 'fov': 60, 'camera': None,
    'semantic_mapping': True, 'depth_resolution': 384, 'point_budget': 32000, 'sample_fps': 5,
    'person_color': '#ff3030', 'person_outline': True, 'live_policy': 'continuous',
}
ENDED = {'failed', 'complete', 'completed', 'stopped', 'stopping', 'cancelled'}
FLYING = {'manual', 'auto', 'paused', 'landing'}   # 게이트웨이 mode_arbiter.FLYING_MODES와 같다
# 콘솔 입력을 다른 쪽(콘솔에서 시작한 지도·보정·탐지)이 쓰거나 정리하는 중: 실패가 아니라 기다린다
BUSY = {'another_camera_session_is_active', 'tello_input_cleanup_pending', 'calibration_input_is_active',
        'tello_detection_input_is_active'}
VIDEO_LOST = 'tello_reconnect_timeout'   # 영상이 30초 넘게 끊겨 콘솔이 세션을 끝냄


class MapKeeper:
    def __init__(self, gateway, console, say, *, interval=2.0, max_failures=3, clock=time.monotonic):
        self.gateway, self.console, self.say = gateway, console, say
        self.interval, self.max_failures, self.clock = interval, max_failures, clock
        self.session = None
        self.failures = 0
        self.retry_at = 0.0
        self.gave_up = False
        self.held = None   # 사람이 콘솔에서 마무리한 세션. 다음 이륙이나 영상을 껐다 켤 때까지 새로 시작하지 않는다
        self._saw_video_off = self._saw_ground = False
        self._other = None   # 콘솔에서 직접 시작한 세션(알림은 한 번만)
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name='map-keeper', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                self.step()
            except Exception as error:   # 한 번 실패해도 계속 지킨다
                self.say(f'  ! 지도 유지 중 오류: {error}')
            self._stop.wait(self.interval)

    def step(self):
        now = self.clock()
        drone = winproc.get_json(self.gateway + '/api/drone/state') or {}
        video = bool((drone.get('video') or {}).get('available'))
        if self.session is None:
            self._start_if_ready(now, drone, video)
            return
        beat = winproc.post_json(self.console + '/api/live/tello/heartbeat', {'session_id': self.session}, HEADERS, timeout=3)
        if (beat or {}).get('error') == 'camera_session_mismatch':   # 콘솔에서 다른 지도를 새로 시작함
            self.say(f'  지도 콘솔에서 다른 지도를 시작해서 세션 {self.session}은 놓아요.')
            self.session, self.retry_at = None, now + 5
            return
        live = winproc.get_json(self.console + '/api/live/state?session=' + self.session, timeout=5) or {}
        status = live.get('session_status') or live.get('status')
        if status not in ENDED:
            return
        ended, self.session = self.session, None
        error = live.get('last_error')
        if status == 'failed' and error == VIDEO_LOST:
            self.retry_at = now + 5
            self.say(f'  영상이 30초 넘게 끊겨 지도 세션 {ended}이 끝났어요(지도는 콘솔 기록에 남아요). '
                     '영상이 다시 켜지면 새 지도를 시작해요.')
        elif status == 'failed':
            self._failed(now, error or '이유 모름', ended)
        elif live.get('stop_reason') == 'user_stop':   # 실행기는 끌 때만 멈추므로, 사람이 콘솔에서 마무리한 것
            self.held = ended
            self._saw_video_off, self._saw_ground = False, drone.get('mode') not in FLYING
            self.say(f'  지도 콘솔에서 지도를 마무리했어요(세션 {ended}, 콘솔에서 저장할 수 있어요). '
                     '다음 이륙이나 영상을 껐다 켜면 새 지도를 시작해요.')
        else:
            self.retry_at = now + 5
            self.say(f'  지도 세션 끝({status}). 지도는 콘솔에 남아요. 영상이 켜져 있으면 새로 시작해요.')

    def _start_if_ready(self, now, drone, video):
        if self.held and not self._released(drone, video):
            return
        if self.gave_up or now < self.retry_at or not video:
            return
        current = (winproc.get_json(self.console + '/api/live', timeout=5) or {}).get('current') or {}
        if current.get('session_id') and (current.get('session_status') or current.get('status')) not in ENDED:
            if current['session_id'] != self._other:
                self._other = current['session_id']
                self.say(f'  지도 콘솔에서 시작한 지도(세션 {self._other})가 돌고 있어 그대로 둬요. 끝나면 이어서 시작해요.')
            return
        reply = winproc.post_json(self.console + '/api/live/tello/start', START_CONFIG, HEADERS, timeout=60)
        if reply and reply.get('session_id'):
            self.session, self.failures, self._other = reply['session_id'], 0, None
            self.say(f'  지도 시작(세션 {self.session}). 관제 화면 지도 칸 "3D 점군"과 지도 콘솔 실시간 지도'
                     f'({self.console}/#live)에서 보여요.')
        elif (reply or {}).get('error') in BUSY:
            self.retry_at = now + 5
        else:
            self._failed(now, (reply or {}).get('error') or '지도 콘솔 응답 없음')

    def _released(self, drone, video):
        """마무리한 뒤 이륙했거나 영상을 껐다 켰으면 True."""
        flying = drone.get('mode') in FLYING
        if not video:
            self._saw_video_off = True
        if not flying:
            self._saw_ground = True
        took_off = self._saw_ground and flying
        if not (took_off or (self._saw_video_off and video)):
            return False
        self.held = None
        self.say('  ' + ('이륙해서' if took_off else '영상이 다시 켜져서') + ' 새 지도를 시작해요.')
        return True

    def _failed(self, now, why, session=None):
        self.failures += 1
        what = f'지도 세션 {session} 실패' if session else '지도 시작 실패'
        if self.failures >= self.max_failures:
            self.gave_up = True
            self.say(f'  ! {what}: {why}. {self.failures}번 실패해서 더 시도하지 않아요(지도 파트 확인 필요).')
        else:
            self.retry_at = now + 10 * self.failures
            self.say(f'  ! {what}: {why}. {10 * self.failures}초 뒤 다시 시도해요.')

    def stop(self):
        """돌던 세션을 끝내 지도를 저장하게 하고 멈춘다."""
        self._stop.set()
        if self.session:
            winproc.post_json(self.console + '/api/live/stop', {'session_id': self.session, 'reason': 'user_stop'},
                              HEADERS, timeout=10)
            self.say(f'  지도 세션 {self.session} 끝냄(저장).')
            self.session = None
