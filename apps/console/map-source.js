// Every asynchronous read belongs to one tab selection, including its response body.
export class MapSource {
  constructor(source = 'saved', fetcher = (url, options) => fetch(url, options)) {
    this.fetcher = fetcher;
    this.generation = 0;
    this.select(source);
  }

  select(source) {
    if (!['saved', 'live'].includes(source)) throw new Error('알 수 없는 지도입니다.');
    this.controller?.abort();
    this.source = source;
    this.generation++;
    this.controller = new AbortController();
  }

  context() { return {source: this.source, generation: this.generation, signal: this.controller.signal}; }
  isCurrent(context) { return context.generation === this.generation; }

  url(path, context = this.context()) {
    const url = `/api/${context.source}/${path}`;
    return this.asset && context.source === 'saved' ? `${url}${url.includes('?') ? '&' : '?'}asset=${encodeURIComponent(this.asset)}` : url;
  }

  async read(path, context) {
    try {
      const response = await this.fetcher(this.url(path, context), {signal: AbortSignal.any([context.signal, AbortSignal.timeout(10000)])});
      if (!this.isCurrent(context) || response.status === 409) return null;
      const value = await response.json();
      if (!this.isCurrent(context)) return null;
      return {ok: response.ok, status: response.status, value};
    } catch (error) {
      if (!this.isCurrent(context) || error.name === 'AbortError') return null;
      throw error;
    }
  }
}

export function captureError(error) {
  const telloErrors = {
    slam_image_missing: 'SLAM 실행 이미지가 없습니다. 실행 안내의 docker build 명령으로 ssafy-orbslam3:local 이미지를 준비한 뒤 다시 시작하세요.',
    slam_docker_unavailable: 'Docker에 연결하지 못했습니다. Docker Desktop의 Linux 엔진이 실행된 뒤 다시 시작하세요.',
    tello_sdk_unavailable_check_wifi: 'Tello에 연결하지 못했습니다. 노트북의 드론 Wi-Fi 연결과 다른 드론 수신 프로그램 실행 여부를 확인하세요.',
    tello_streamon_failed: '드론 카메라를 켜지 못했습니다. 드론 연결을 확인한 뒤 다시 시작하세요.',
    tello_video_timeout_check_wifi_and_udp: '드론 카메라 영상이 들어오지 않습니다. Wi-Fi 연결과 UDP 11111 수신 설정을 확인하세요.',
    tello_gateway_video_off_or_stale: '조종 사이트의 드론 영상이 꺼져 있거나 끊겼습니다. 그 사이트에서 영상을 켠 뒤 다시 시작하세요.',
    tello_gateway_stream_ended: '드론 게이트웨이 영상 연결이 종료됐습니다. 다시 연결하면 새 지도 세션을 시작합니다.',
    tello_reconnect_timeout: '30초 동안 영상을 복구하지 못했습니다. 마지막 지도와 이전 구간을 보존했습니다. Wi-Fi·게이트웨이를 확인하세요.',
    tello_server_reconnect_timeout: '45초 동안 서버에 다시 연결하지 못했습니다. 서버를 확인한 뒤 보존된 지도를 다시 여세요.',
    tracker_numerical_failure: 'SLAM 수치 오류가 발생했습니다. 마지막 지도와 오류 진단을 보존했습니다.',
    tracker_process_crashed: 'SLAM 프로세스가 중단됐습니다. 마지막 지도와 오류 진단을 보존했습니다.',
    tello_input_cleanup_pending: '이전 드론 연결을 정리하고 있습니다. 잠시 뒤 다시 시작하세요.',
    tello_detection_input_is_active: '드론 사람 탐지 입력이 실행 중입니다. 먼저 해당 입력을 종료하세요.',
    tello_gateway_session_changed: '드론 게이트웨이 실행 회차가 바뀌었습니다. 다시 연결해 새 지도 세션을 시작하세요.',
    frame_dimensions_do_not_match_calibration: '영상 크기가 Tello 지도 설정(960×720)과 다릅니다. 원본 영상 크기와 보정값을 확인하세요.',
  };
  if (telloErrors[error.message]) return {state: 'disconnected', message: telloErrors[error.message]};
  if (error.message?.startsWith('tello_input_unavailable'))
    return {state: 'disconnected', message: '드론 영상 연결에 실패했습니다. Wi-Fi·게이트웨이 실행 상태와 다른 드론 프로그램의 포트 사용 여부를 확인하세요. 직접 연결에는 PyAV와 OpenCV가 필요합니다.'};
  if (error.name === 'NotAllowedError' || error.name === 'SecurityError')
    return {state: 'denied', message: '카메라 권한이 거부되었습니다. 브라우저의 이 사이트 카메라 권한을 확인한 뒤 다시 시작하세요.'};
  if (error.name === 'NotFoundError')
    return {state: 'failed', message: '연결된 카메라를 찾지 못했습니다. 카메라 연결이나 녹화 영상 입력을 확인하세요.'};
  if (error.name === 'NotReadableError')
    return {state: 'failed', message: '카메라를 열지 못했습니다. 다른 앱에서 사용 중인지 확인한 뒤 다시 시작하세요.'};
  if (error.name === 'OverconstrainedError')
    return {state: 'failed', message: '선택한 카메라나 영상 설정을 사용할 수 없습니다. 장치 목록을 새로고침하고 카메라를 다시 선택하세요.'};
  return {state: 'failed', message: `영상 연결 실패: ${error.message}`};
}

// Read-only status: a browser/server failure never proves that the drone is offline.
export function connectionStatus({reachable = true, responseOK = true, state = {}, capture = {}, fresh = true} = {}) {
  const row = (text, tone = 'idle') => ({text, tone});
  const result = {server: row('연결됨', 'ok'), drone: row('연결 전'), video: row('대기'), map: row('대기'), note: ''};
  if (!reachable || !responseOK) {
    result.server = row(reachable ? '응답 오류' : '연결 실패', 'error');
    for (const key of ['drone', 'video', 'map']) result[key] = row('확인 불가');
    result.note = reachable ? '서버는 응답했지만 처리 상태를 읽지 못했습니다.' : '서버에 응답이 없습니다. 드론 상태는 확인할 수 없습니다.';
    return result;
  }
  if (capture.kind === 'tello') {
    result.drone = row('별도 입력'); result.video = row('탐지 화면'); result.map = row('해당 없음');
    result.note = '사람 탐지 입력입니다. 이 상자의 지도 상태는 실시간 3D 지도 입력에 적용됩니다.';
    return result;
  }
  const pending = capture.state === 'connecting' && !capture.session;
  const ownCapture = !capture.session || capture.session === state.session_id;
  const error = ownCapture && capture.error || (!pending && state.last_error) || '';
  const status = pending ? 'starting' : state.session_status || state.status;
  const active = !pending && !['idle', 'complete', 'failed', 'stopped_or_disconnected', 'stopping'].includes(status) && Boolean(state.session_id);
  const drone = (pending ? capture.kind === 'tello-map' : state.input === 'tello_camera') || capture.kind === 'tello';
  const connected = !pending && state.input_origin && !['not_connected', 'local_camera'].includes(state.input_origin);
  if (!drone) result.drone = row('해당 없음');
  else if (active && connected) result.drone = row(state.input_origin === 'gateway_reported_fake' ? '검증용 입력' :
    state.input_origin.startsWith('gateway_') ? '중계 응답' : '응답 확인', 'ok');
  else if (pending || active) result.drone = row('연결 중', 'warn');
  else if (state.session_id) result.drone = row('종료');
  if (active) {
    result.video = state.transport_received > 0 ? row(fresh ? '수신 중' : '수신 지연', fresh ? 'ok' : 'warn') : row('입력 대기', 'warn');
    result.map = !state.transport_ready ? row('준비 중', 'warn') :
      state.mapping_state === 'lost' || state.tracking_state === 3 ? row('추적 상실', 'warn') :
      state.map_available && state.tracking_state === 2 ? row('생성 중', 'ok') : row('초기화 중', 'warn');
    if (state.input_state === 'reconnecting') {
      result.video = row('재연결 중', 'warn');
      result.map = row(state.map_available ? '지도 보존 · 복구 대기' : '영상 복구 대기', 'warn');
    }
  } else if (pending) { result.video = row('대기'); result.map = row('준비 중', 'warn'); }
  else if (state.session_id) { result.video = row('종료'); result.map = row(state.map_available ? '기록 보존' : '생성 전'); }
  if (error === 'tello_sdk_unavailable_check_wifi') {
    result.drone = row('연결 실패', 'error'); result.video = row('입력 없음'); result.map = row('입력 대기');
    result.note = '서버 정상 · 드론이 연결 명령에 응답하지 않았습니다. Wi-Fi와 드론 전원을 확인하세요.';
  } else if (/^tello_(streamon|video_timeout|gateway_video|gateway_stream|gateway_session)/.test(error)) {
    result.video = row('수신 실패', 'error'); result.map = row('입력 중단', 'warn');
    result.drone = row(error === 'tello_streamon_failed' ? '응답 확인' : '확인 필요', 'warn');
    result.note = captureError(new Error(error)).message;
  } else if (/^slam_|tracker|docker/i.test(error)) {
    result.map = row('처리 실패', 'error'); result.note = captureError(new Error(error)).message;
    if (!capture.session && capture.error) { result.drone = row('확인 전'); result.video = row('대기'); }
  } else if (error) {
    result.map = row('오류 확인', 'warn'); result.note = captureError(new Error(error)).message;
  } else result.note = active ? '드론 응답과 영상 수신·지도 처리를 구분해 표시합니다.' : '시작 전에는 드론 연결을 확인하지 않습니다. 종료 후에는 기록만 표시합니다.';
  return result;
}

export function monitorConnections({element = (id) => document.getElementById(id), fetcher = fetch,
  clock = Date.now, schedule = setTimeout, cancel = clearTimeout} = {}) {
  let capture = {}, state = {}, reachable = false, responseOK = true, timer, closed = false;
  let session, count = -1, advancedAt = 0;
  function render() {
    const status = connectionStatus({reachable, responseOK, state, capture, fresh: clock() - advancedAt < 5000});
    for (const key of ['server', 'drone', 'video', 'map']) {
      const node = element('connection-' + key);
      node.textContent = status[key].text; node.dataset.state = status[key].tone;
    }
    element('connection-note').textContent = status.note;
  }
  async function poll() {
    let responded = false;
    try {
      const response = await fetcher('/api/live/state', {cache: 'no-store', signal: AbortSignal.timeout(4000)});
      responded = true;
      reachable = true; responseOK = response.ok;
      if (response.ok) {
        const next = await response.json();
        if (next.source !== 'live' || typeof next.status !== 'string') throw new Error('invalid_status');
        state = next;
        if (session !== state.session_id || count !== state.transport_received) {
          session = state.session_id; count = state.transport_received; advancedAt = clock();
        }
        element('connection-checked').textContent = '2초마다 확인';
      }
    } catch {
      reachable = responded; responseOK = false;
    } finally {
      if (!closed) { render(); timer = schedule(poll, 2000); }
    }
  }
  function visibility() { element('connection-monitor').hidden = location.hash !== '#live'; }
  window.addEventListener('hashchange', visibility); visibility(); poll();
  return {capture(value) { capture = value; render(); },
    close() { closed = true; cancel(timer); window.removeEventListener('hashchange', visibility); }};
}
