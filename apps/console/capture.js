// Browser media PTS are kept separate from map playback time and rendering cadence.
export async function connectCapture(onStart) {
  const $ = (id) => document.getElementById(id);
  const info = await (await fetch('/api/live')).json();
  if (!info.enabled) return;
  $('live-panel').hidden = false;
  $('input-source').querySelector('[value=demo]').hidden = !info.demo_available;
  const video = $('input-preview'), canvas = document.createElement('canvas');
  const context = canvas.getContext('2d', {alpha: false});
  let media, id, sending = false, active = false, starting = false, base = null, sourceBase = null,
    lastMedia = -1, sequence = 0, sent = 0, skipped = 0, callback;
  const message = (text) => { $('camera-status').textContent = text; };
  const post = async (url, body, headers = {}) => {
    const response = await fetch(url, {method: 'POST', headers: {'X-Mapping-Client': '1', ...headers}, body, signal: AbortSignal.timeout(15000)});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || `영상 연결 실패 ${response.status}`);
    return value;
  };
  async function stop(reason = '수신을 종료했습니다. 마지막 지도와 생성 기록을 저장합니다.') {
    active = false;
    if (callback && video.cancelVideoFrameCallback) video.cancelVideoFrameCallback(callback);
    video.pause();
    video.hidden = true;
    media?.getTracks().forEach((track) => track.stop());
    media = null;
    video.srcObject = null;
    $('camera-stop').disabled = true;
    if (id) {
      const current = id;
      id = null;
      try { await post('/api/live/stop', JSON.stringify({session_id: current})); }
      catch (error) { reason += ` (${error.message})`; }
    }
    $('camera-start').disabled = false;
    message(reason);
  }
  function schedule() {
    if (!active) return;
    if (video.requestVideoFrameCallback) callback = video.requestVideoFrameCallback(frame);
    else callback = setTimeout(() => frame(0, {mediaTime: video.currentTime}), 1000 / 30);
  }
  async function frame(now, metadata) {
    schedule();
    if (!active || video.readyState < 2) return;
    if (metadata.mediaTime <= lastMedia) return;
    lastMedia = metadata.mediaTime;
    if (sourceBase === null && metadata.presentedFrames != null) sourceBase = metadata.presentedFrames;
    const currentSequence = metadata.presentedFrames != null ? metadata.presentedFrames - sourceBase : sequence;
    skipped += Math.max(0, currentSequence - sequence);
    sequence = currentSequence + 1;
    if (sending) { skipped++; return; }
    sending = true;
    const session = id;
    try {
      if (base === null) base = metadata.mediaTime;
      const pts = metadata.mediaTime - base;
      context.drawImage(video, 0, 0, canvas.width, canvas.height);
      const jpeg = await new Promise((resolve) => canvas.toBlob(resolve, 'image/jpeg', .85));
      if (!active || session !== id) return;
      if (!jpeg) throw new Error('영상 프레임 인코딩 실패');
      const state = await post(`/api/live/frame?session=${encodeURIComponent(session)}`, jpeg,
        {'Content-Type': 'image/jpeg', 'X-Frame-Timestamp': String(pts), 'X-Frame-Sequence': String(currentSequence)});
      sent++;
      message(`수신 중 · 전송 ${sent}장 · 전송 대기 중 생략 ${skipped}장 · 처리 대기 교체/입력 누락 ${state.frame_drops}장 · 대기 ${state.queue_depth}/1`);
      Object.assign($('camera-status').dataset, {sent, skipped, dropped: state.frame_drops, session});
    } catch (error) {
      if (active && session === id) await stop(`영상 수신 중단: ${error.message}`);
    } finally { sending = false; }
  }
  $('camera-start').addEventListener('click', async () => {
    if (starting || active) return;
    starting = true;
    $('camera-start').disabled = true;
    const demo = $('input-source').value === 'demo';
    try {
      base = sourceBase = null; lastMedia = -1; sequence = sent = skipped = 0;
      video.hidden = false;
      if (demo) {
        video.src = '/api/source-video';
        video.load();
      } else {
        if (!navigator.mediaDevices?.getUserMedia) throw new Error('이 브라우저에서 카메라를 사용할 수 없습니다. localhost 또는 HTTPS에서 열어주세요.');
        media = await navigator.mediaDevices.getUserMedia({video: {width: {ideal: 960}, height: {ideal: 540}, frameRate: {ideal: 30, max: 30}, facingMode: {ideal: 'environment'}}, audio: false});
        video.removeAttribute('src');
        video.srcObject = media;
        media.getVideoTracks()[0].addEventListener('ended', () => { if (active) stop('카메라 연결이 끊겨 수신을 종료했습니다.'); }, {once: true});
      }
      if (video.readyState < 1) await new Promise((resolve, reject) => {
        video.onloadedmetadata = resolve;
        video.onerror = () => reject(new Error('입력 영상을 열지 못했습니다.'));
      });
      const scale = Math.min(960 / video.videoWidth, 960 / video.videoHeight, 1);
      canvas.width = Math.round(video.videoWidth * scale);
      canvas.height = Math.round(video.videoHeight * scale);
      const calibrationFile = $('camera-calibration').files[0];
      const camera = demo ? info.demo_camera : calibrationFile ? JSON.parse(await calibrationFile.text()) : null;
      const config = {width: canvas.width, height: canvas.height, fps: demo ? info.demo_camera?.fps || 30 : media.getVideoTracks()[0].getSettings().frameRate || 30,
        fov: Number($('camera-fov').value), camera};
      message(`ORB-SLAM3 시작 중 · ${camera ? '카메라 보정값 사용' : '화각 추정 · 미보정'} · ${canvas.width}×${canvas.height}`);
      const started = await post('/api/live/start', JSON.stringify(config), {'Content-Type': 'application/json'});
      id = started.session_id;
      const deadline = Date.now() + 65000;
      while (true) {
        const state = await (await fetch('/api/state')).json();
        if (state.status === 'failed') throw new Error(state.last_error || 'SLAM 시작 실패');
        if (state.transport_ready) break;
        if (Date.now() > deadline) throw new Error('SLAM 시작 시간 초과');
        await new Promise((resolve) => setTimeout(resolve, 250));
      }
      active = true;
      $('camera-stop').disabled = false;
      onStart(id);
      schedule();
      await video.play();
    } catch (error) { await stop(`연결할 수 없습니다: ${error.message}`); }
    finally { starting = false; }
  });
  $('camera-stop').addEventListener('click', () => stop());
  video.addEventListener('ended', () => { if (active) stop('시연 영상이 끝났습니다. 생성된 점군과 기록을 저장합니다.'); });
  $('input-source').addEventListener('change', () => {
    $('camera-start').textContent = $('input-source').value === 'demo' ? '스트리밍 시연 시작' : '카메라 연결 · 시작';
  });
  window.addEventListener('pagehide', () => {
    media?.getTracks().forEach((track) => track.stop());
    if (id) fetch('/api/live/stop', {method: 'POST', headers: {'X-Mapping-Client': '1'}, body: JSON.stringify({session_id: id}), keepalive: true}).catch(() => {});
  });
}
