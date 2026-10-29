// Each start owns its media, callbacks and HTTP session, including canceled starts.
import { captureError } from './map-source.js';

export function captureConfiguration(width, height, fps, fov, camera = null) {
  if (![width, height, fps].every(Number.isFinite) || width < 1 || height < 1)
    throw new Error('카메라의 영상 크기와 프레임율을 확인하지 못했습니다.');
  const scale = Math.min(960 / width, 960 / height, 1);
  const config = {width: Math.round(width * scale), height: Math.round(height * scale), fps, fov, camera};
  if (config.width < 320 || config.height < 180 || fps < 1 || fps > 60)
    throw new Error('320×180 이상, 1~60fps의 카메라 입력이 필요합니다.');
  if (camera !== null) {
    const keys = ['width', 'height', 'fx', 'fy', 'cx', 'cy'];
    if (typeof camera !== 'object' || Array.isArray(camera) || !keys.every((key) => Number.isFinite(camera[key])) ||
        !['width', 'height', 'fx', 'fy'].every((key) => camera[key] > 0) ||
        camera.cx < 0 || camera.cx > camera.width || camera.cy < 0 || camera.cy > camera.height)
      throw new Error('보정 JSON의 width·height·fx·fy·cx·cy 숫자를 확인하세요.');
    if (Math.abs(camera.width / camera.height - config.width / config.height) > .01)
      throw new Error('보정값과 현재 영상의 가로세로 비율이 다릅니다. 이 카메라·해상도·촬영 방향의 보정값을 사용하세요.');
    if (camera.distortion !== undefined && (!Array.isArray(camera.distortion) || camera.distortion.length !== 4 || !camera.distortion.every(Number.isFinite)))
      throw new Error('보정 JSON의 distortion은 유한한 숫자 4개여야 합니다.');
  } else if (!Number.isFinite(fov) || fov < 25 || fov > 120) {
    throw new Error('수평 화각은 25~120°로 입력하세요.');
  }
  return config;
}

export async function connectCapture({onStart = () => {}, onStatus = () => {}, onReady = () => {}, onFrameTiming = () => {}} = {}) {
  const $ = (id) => document.getElementById(id);
  const [info, tello] = await Promise.all([
    fetch('/api/live').then((response) => response.json()),
    fetch('/api/tello').then((response) => response.json()),
  ]);
  onReady(info);
  if (!info.enabled && !tello.enabled) {
    for (const id of ['camera-start', 'camera-stop', 'camera-device', 'camera-refresh']) $(id).disabled = true;
    const message = '이 서버는 저장된 기록 조회만 지원합니다. 새 영상 입력은 카메라 입력을 켠 서버에서 시작하세요.';
    $('camera-status').textContent = message;
    onStatus({state: 'disabled', message});
    return {update() {}};
  }
  $('input-source').querySelector('[value=camera]').hidden = !info.enabled;
  $('input-source').querySelector('[value=demo]').hidden = !info.demo_available;
  $('input-source').querySelector('[value=tello]').hidden = !tello.enabled;
  if (!info.enabled && tello.enabled) $('input-source').value = 'tello';
  const video = $('input-preview'), telloPreview = $('tello-preview'), canvas = document.createElement('canvas');
  const context = canvas.getContext('2d', {alpha: false});
  let current = null, creating = false, serverState = {}, deviceRead = 0;
  const terminal = (state) => ['idle', 'complete', 'failed', 'stopped_or_disconnected'].includes(state.session_status || state.status);
  const message = (text, state, run = current) => {
    $('camera-status').textContent = text;
    onStatus({state, message: text, session: run?.session});
  };
  function controls() {
    const occupied = serverState.session_id && !terminal(serverState);
    const locked = Boolean(current) || creating || Boolean(occupied);
    $('camera-start').disabled = locked;
    $('camera-stop').disabled = !current;
    for (const id of ['input-source', 'camera-device', 'camera-refresh', 'camera-calibration', 'camera-fov']) $(id).disabled = locked;
    $('camera-devices').hidden = $('input-source').value !== 'camera';
    $('tello-flight').hidden = $('input-source').value !== 'tello' || !tello.flight_enabled;
  }
  const post = async (url, body, headers = {}) => {
    const response = await fetch(url, {method: 'POST', headers: {'X-Mapping-Client': '1', ...headers}, body, signal: AbortSignal.timeout(15000)});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || '영상 연결 실패 ' + response.status);
    return value;
  };
  const owns = (run) => current === run && !run.cancelled;
  const held = new Set();
  const axes = {a: [0, -1], d: [0, 1], s: [1, -1], w: [1, 1], arrowdown: [2, -1], arrowup: [2, 1], arrowleft: [3, -1], arrowright: [3, 1]};
  let flightState = null, rcTimer = null;
  function rcValues() {
    const values = [0, 0, 0, 0];
    for (const key of held) { const axis = axes[key]; if (axis) values[axis[0]] += axis[1]; }
    return values.map((value) => Math.max(-1, Math.min(1, value)));
  }
  function sendRc() {
    if (current?.kind !== 'tello' || !owns(current) || !flightState) return;
    post('/api/tello/flight/rc', JSON.stringify({rc: rcValues()}), {'Content-Type': 'application/json'}).catch(() => {});
  }
  function renderFlight(flight) {
    flightState = flight;
    if (!flight) return;
    $('flight-state').textContent = flight.busy === 'takeoff' ? '이륙 중…' : flight.busy === 'land' ? '착륙 중…' : flight.flying ? '비행 중' : '지상';
    $('flight-takeoff').disabled = !flight.can_takeoff;
    $('flight-land').disabled = !flight.flying && !flight.busy;
    $('flight-checks').textContent = flight.preflight.map((check) => `${check.ok ? '✓' : '✕'} ${check.title}: ${check.detail}`).join(' · ');
    const operator = flight.preflight.find((check) => check.key === 'operator');
    if (operator) $('flight-confirm').checked = operator.ok;
  }
  async function flightAction(action, body = {}) {
    const result = await post(`/api/tello/flight/${action}`, JSON.stringify(body), {'Content-Type': 'application/json'});
    renderFlight(result.preflight ? result : (await fetch('/api/tello').then((response) => response.json())).flight);
    return result;
  }
  async function refreshDevices() {
    const request = ++deviceRead;
    if (!navigator.mediaDevices?.enumerateDevices) {
      $('camera-device-note').textContent = '이 브라우저에서는 장치 목록을 읽을 수 없습니다. 기본 카메라로 시도하세요.';
      return;
    }
    try {
      const devices = (await navigator.mediaDevices.enumerateDevices()).filter((device) => device.kind === 'videoinput');
      if (request !== deviceRead) return;
      const selected = $('camera-device').value;
      const option = (value, text) => { const item = document.createElement('option'); item.value = value; item.textContent = text; return item; };
      $('camera-device').replaceChildren(option('', '브라우저 기본 카메라'), ...devices.filter((device) => device.deviceId).map((device, index) => option(device.deviceId, device.label || '카메라 ' + (index + 1) + ' · 권한 허용 후 이름 표시')));
      $('camera-device').value = devices.some((device) => device.deviceId === selected) ? selected : '';
      $('camera-device-note').textContent = devices.length ? '영상 장치 ' + devices.length + '개 · 장치 변경은 수신 종료 후 적용합니다.' : '현재 공개된 카메라 목록이 없습니다. 연결·권한을 확인하거나 기본 카메라로 시도하세요.';
    } catch {
      if (request === deviceRead) $('camera-device-note').textContent = '장치 목록을 읽지 못했습니다. 연결·권한을 확인한 뒤 새로고침하세요.';
    }
  }
  function release(run) {
    run.cancelled = true;
    run.controller.abort();
    run.cancelMetadata?.();
    if (run.callback !== undefined) {
      if (video.cancelVideoFrameCallback && video.requestVideoFrameCallback) video.cancelVideoFrameCallback(run.callback);
      else clearTimeout(run.callback);
    }
    run.media?.getTracks().forEach((track) => track.stop());
    if (run.kind === 'tello') {
      held.clear(); clearInterval(rcTimer); rcTimer = null; flightState = null;
      telloPreview.hidden = true; telloPreview.removeAttribute('src');
    }
    if (current === run) {
      video.pause(); video.hidden = true; video.srcObject = null;
      video.removeAttribute('src'); video.load(); current = null;
    }
  }
  async function stopSession(run) {
    if (!run.session || run.stopSent) return;
    run.stopSent = true;
    if (run.kind === 'tello') {
      await post('/api/tello/stop', '{}', {'Content-Type': 'application/json'});
      return;
    }
    serverState = {session_id: run.session, session_status: 'stopping'};
    controls();
    await post('/api/live/stop', JSON.stringify({session_id: run.session}));
  }
  async function stop(run = current, reason = null, state = 'stopped') {
    if (!run || run.cancelled) return;
    reason ??= run.session ? '수신을 종료했습니다. 마지막 지도와 생성 기록을 저장합니다.' : '영상 연결을 취소했습니다.';
    release(run); controls();
    message('영상 수신을 종료하는 중입니다.', 'stopping', run);
    try { await stopSession(run); }
    catch (error) { reason += ' (' + error.message + ')'; }
    if (!current) message(reason, state, run);
    controls();
  }
  function update(state = serverState) {
    serverState = state || {};
    const run = current;
    if (run?.kind !== 'tello' && run?.session && (serverState.session_id !== run.session || terminal(serverState) || serverState.session_status === 'stopping')) {
      release(run);
      message(serverState.last_error ? '서버가 영상 처리를 종료했습니다: ' + serverState.last_error : '서버 세션이 종료되거나 바뀌어 카메라를 해제했습니다.', serverState.session_status === 'failed' || serverState.status === 'failed' ? 'failed' : 'disconnected', run);
    }
    controls();
  }
  function metadata(run) {
    if (video.readyState >= 1 && video.videoWidth && video.videoHeight) return Promise.resolve();
    return new Promise((resolve, reject) => {
      const cleanup = () => { clearTimeout(timer); video.removeEventListener('loadedmetadata', ready); video.removeEventListener('error', failed); run.cancelMetadata = null; };
      const ready = () => { cleanup(); resolve(); };
      const failed = () => { cleanup(); reject(new Error('입력 영상의 크기를 읽지 못했습니다. 카메라 연결을 확인하세요.')); };
      const timer = setTimeout(failed, 15000);
      run.cancelMetadata = () => { cleanup(); reject(new DOMException('cancelled', 'AbortError')); };
      video.addEventListener('loadedmetadata', ready); video.addEventListener('error', failed);
    });
  }
  function schedule(run) {
    if (!owns(run) || !run.active) return;
    run.callback = video.requestVideoFrameCallback ? video.requestVideoFrameCallback((now, data) => frame(run, data)) : setTimeout(() => frame(run, {mediaTime: video.currentTime}), 1000 / 30);
  }
  async function frame(run, data) {
    const capturedAtMs = performance.now();
    schedule(run);
    if (!owns(run) || !run.active || video.readyState < 2 || !Number.isFinite(data.mediaTime) || data.mediaTime <= run.lastMedia) return;
    run.lastMedia = data.mediaTime;
    if (run.sourceBase === null && data.presentedFrames != null) run.sourceBase = data.presentedFrames;
    const sequence = data.presentedFrames != null ? data.presentedFrames - run.sourceBase : run.sequence;
    run.skipped += Math.max(0, sequence - run.sequence); run.sequence = sequence + 1;
    if (run.sending) { run.skipped++; return; }
    run.sending = true;
    try {
      if (run.base === null) run.base = data.mediaTime;
      const pts = data.mediaTime - run.base;
      onFrameTiming({session: run.session, pts, capturedAtMs});
      context.drawImage(video, 0, 0, canvas.width, canvas.height);
      const jpeg = await new Promise((resolve) => canvas.toBlob(resolve, 'image/jpeg', .85));
      if (!owns(run)) return;
      if (!jpeg) throw new Error('영상 프레임 인코딩 실패');
      onFrameTiming({session: run.session, pts, encodedAtMs: performance.now()});
      const state = await post('/api/live/frame?session=' + encodeURIComponent(run.session), jpeg,
        {'Content-Type': 'image/jpeg', 'X-Frame-Timestamp': String(pts), 'X-Frame-Sequence': String(sequence)});
      if (!owns(run)) return;
      onFrameTiming({session: run.session, pts, ackAtMs: performance.now()});
      run.sent++;
      message('수신 중 · 전송 ' + run.sent + '장 · 전송 대기 중 생략 ' + run.skipped + '장 · 처리 대기 교체/입력 누락 ' + state.frame_drops + '장 · 대기 ' + state.queue_depth + '/1', 'receiving', run);
      Object.assign($('camera-status').dataset, {sent: run.sent, skipped: run.skipped, dropped: state.frame_drops, session: run.session});
    } catch (error) { if (owns(run)) await stop(run, '영상 수신 중단: ' + error.message, 'disconnected'); }
    finally { run.sending = false; }
  }
  $('camera-start').addEventListener('click', async () => {
    if ($('camera-start').disabled) return;
    const kind = $('input-source').value;
    const run = {kind, controller: new AbortController(), session: null, media: null, cancelled: false, active: false, base: null, sourceBase: null, lastMedia: -1, sequence: 0, sent: 0, skipped: 0};
    current = run; controls();
    const demo = kind === 'demo';
    try {
      if (kind === 'tello') {
        message('Tello 연결 및 YOLO 모델을 준비하는 중입니다.', 'connecting', run);
        const started = await post('/api/tello/start', '{}', {'Content-Type': 'application/json'});
        if (!owns(run)) { await post('/api/tello/stop', '{}', {'Content-Type': 'application/json'}); return; }
        run.session = 'tello'; run.active = true; renderFlight(started.flight);
        if (started.flight) rcTimer = setInterval(() => { if (held.size) sendRc(); }, 50);
        telloPreview.src = '/api/tello/video.mjpg?t=' + Date.now(); telloPreview.hidden = false;
        onStart(run.session);
        while (owns(run)) {
          const state = await fetch('/api/tello', {signal: AbortSignal.any([run.controller.signal, AbortSignal.timeout(10000)])}).then((response) => response.json());
          if (!owns(run)) return;
          if (state.error) throw new Error(state.error);
          renderFlight(state.flight);
          const fps = state.video?.fps == null ? '—' : state.video.fps;
          message(`드론 탐지 중 · 사람 ${state.person_count}명 · 영상 ${fps} FPS · 추론 ${state.inference_ms ?? '—'}ms`, 'receiving', run);
          await new Promise((resolve) => setTimeout(resolve, 500));
        }
        return;
      }
      const file = $('camera-calibration').files[0];
      if (!demo && file && file.size > 65536) throw new Error('보정 JSON은 64KB 이하 파일을 사용하세요.');
      let camera = demo ? info.demo_camera : null;
      if (!demo && file) {
        try { camera = JSON.parse(await file.text()); }
        catch { throw new Error('보정 JSON을 읽지 못했습니다. 올바른 JSON 파일인지 확인하세요.'); }
      }
      if (!owns(run)) return;
      video.hidden = false;
      if (demo) { video.src = '/api/source-video'; video.load(); }
      else {
        if (!navigator.mediaDevices?.getUserMedia) throw new Error('이 브라우저에서 카메라를 사용할 수 없습니다. localhost 또는 HTTPS에서 열어주세요.');
        message('카메라 권한을 확인하고 있습니다. 브라우저의 권한 요청을 확인하세요.', 'requesting', run);
        const device = $('camera-device').value;
        const media = await navigator.mediaDevices.getUserMedia({video: {width: {ideal: 960}, height: {ideal: 540}, frameRate: {ideal: 30, max: 30}, ...(device ? {deviceId: {exact: device}} : {})}, audio: false});
        if (!owns(run)) { media.getTracks().forEach((track) => track.stop()); return; }
        run.media = media; video.srcObject = media;
        const track = media.getVideoTracks()[0];
        if (!track || track.readyState === 'ended') throw new Error('카메라 영상 트랙이 종료되었습니다. 다시 연결하세요.');
        track.addEventListener('ended', () => { if (owns(run)) return stop(run, '카메라 연결이 끊겨 수신을 종료했습니다.', 'disconnected'); }, {once: true});
        void refreshDevices();
      }
      await metadata(run);
      if (!owns(run)) return;
      const track = run.media?.getVideoTracks()[0], settings = track?.getSettings() || {};
      const config = captureConfiguration(video.videoWidth, video.videoHeight, demo ? info.demo_camera?.fps || 30 : settings.frameRate || 30, Number($('camera-fov').value), camera);
      canvas.width = config.width; canvas.height = config.height;
      $('camera-input-info').textContent = (demo ? '녹화 영상' : track.label || '선택한 카메라') + ' · 원본 ' + video.videoWidth + '×' + video.videoHeight + ' → 전송 ' + config.width + '×' + config.height + ' · ' + config.fps.toFixed(1) + 'fps 설정값 · ' + (camera ? '보정 JSON 사용' : '화각 추정 · 미보정');
      message('ORB-SLAM3 시작 중 · ' + (camera ? '카메라 보정값 사용' : '화각 추정 · 미보정') + ' · ' + canvas.width + '×' + canvas.height, 'connecting', run);
      creating = true; controls();
      try {
        const started = await post('/api/live/start', JSON.stringify(config), {'Content-Type': 'application/json'});
        run.session = started.session_id;
        if (!owns(run)) { await stopSession(run); return; }
        serverState = started; onStart(run.session);
      } finally { creating = false; controls(); }
      const deadline = Date.now() + 65000;
      while (owns(run)) {
        const response = await fetch('/api/live/state?session=' + encodeURIComponent(run.session), {signal: AbortSignal.any([run.controller.signal, AbortSignal.timeout(10000)])});
        const state = await response.json();
        if (!owns(run)) return;
        if (!response.ok) throw new Error(state.error || '영상 세션 상태를 읽지 못했습니다.');
        if (state.status === 'failed' || state.session_status === 'failed') throw new Error(state.last_error || 'SLAM 시작 실패');
        if (terminal(state) || state.session_status === 'stopping') throw new Error('영상 세션이 시작 전에 종료되었습니다.');
        if (state.transport_ready) break;
        if (Date.now() > deadline) throw new Error('SLAM 시작 시간 초과');
        await new Promise((resolve) => setTimeout(resolve, 250));
      }
      if (!owns(run)) return;
      await video.play();
      if (!owns(run)) return;
      run.active = true; schedule(run);
      message('영상 수신 중 · 카메라 위치를 천천히 옮기면 지도를 초기화합니다.', 'receiving', run);
    } catch (error) {
      if (owns(run)) { const failure = captureError(error); await stop(run, failure.message, failure.state); }
    }
    finally { controls(); }
  });
  $('camera-stop').addEventListener('click', () => stop());
  $('flight-confirm').addEventListener('change', (event) => flightAction('confirm', {confirmed: event.target.checked}).catch((error) => message(error.message, 'failed')));
  $('flight-takeoff').addEventListener('click', () => { if (confirm('이륙할까요? 주변 3m와 프로펠러 가드를 다시 확인하세요.')) flightAction('takeoff').catch((error) => message(error.message, 'failed')); });
  $('flight-land').addEventListener('click', () => flightAction('land').catch((error) => message(error.message, 'failed')));
  $('flight-emergency').addEventListener('click', () => flightAction('emergency').catch((error) => message(error.message, 'failed')));
  document.addEventListener?.('keydown', (event) => {
    if (current?.kind !== 'tello' || !owns(current) || !flightState || event.target.tagName === 'INPUT') return;
    const key = event.key.toLowerCase();
    if (key === 'escape') { event.preventDefault(); event.stopPropagation(); flightAction('emergency').catch(() => {}); return; }
    if (axes[key]) { event.preventDefault(); event.stopPropagation(); held.add(key); sendRc(); return; }
    if (!event.repeat && key === 't') { event.preventDefault(); event.stopPropagation(); $('flight-takeoff').click(); }
    if (!event.repeat && key === 'l') { event.preventDefault(); event.stopPropagation(); flightAction('land').catch(() => {}); }
  });
  document.addEventListener?.('keyup', (event) => {
    const key = event.key.toLowerCase();
    if (axes[key] && held.delete(key)) { event.preventDefault(); event.stopPropagation(); sendRc(); }
  });
  window.addEventListener('blur', () => { held.clear(); sendRc(); });
  $('camera-refresh').addEventListener('click', refreshDevices);
  navigator.mediaDevices?.addEventListener?.('devicechange', refreshDevices);
  video.addEventListener('ended', () => { if (current?.active) return stop(current, '시연 영상이 끝났습니다. 생성된 점군과 기록을 저장합니다.'); });
  const sourceChanged = () => {
    $('camera-start').textContent = $('input-source').value === 'demo' ? '스트리밍 시연 시작' : $('input-source').value === 'tello' ? '드론 연결 · 탐지 시작' : '카메라 연결 · 시작';
    controls();
  };
  $('input-source').addEventListener('change', sourceChanged);
  window.addEventListener('pagehide', () => {
    const run = current;
    if (!run) return;
    release(run);
    if (run.session && !run.stopSent) {
      run.stopSent = true;
      const url = run.kind === 'tello' ? '/api/tello/stop' : '/api/live/stop';
      const body = run.kind === 'tello' ? '{}' : JSON.stringify({session_id: run.session});
      fetch(url, {method: 'POST', headers: {'X-Mapping-Client': '1'}, body, keepalive: true}).catch(() => {});
    }
  });
  message('입력 전 · 시작 버튼을 눌러 영상을 연결하세요. 카메라 권한은 시작할 때 요청합니다.', 'idle');
  controls();
  sourceChanged();
  await refreshDevices();
  return {update};
}
