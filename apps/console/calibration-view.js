import {feedCalibrationFiles, validateCalibrationFiles} from './calibration-files.js';

const messages = {
  show_checkerboard: '체크보드 전체와 흰 여백이 보이게 비춰 주세요.',
  checkerboard_not_found: '교차점을 찾지 못했습니다. 내부 교차점 수·조명·전체 보임을 확인하세요.',
  checkerboard_too_close_to_edge: '종이가 화면 가장자리에 닿았습니다. 조금 뒤로 옮겨 주세요.',
  checkerboard_too_small: '체크보드가 너무 작습니다. 카메라에 조금 더 가까이 보여 주세요.',
  checkerboard_blurry: '흔들리거나 흐립니다. 자세를 잠시 고정하고 조명을 밝게 해 주세요.',
  checkerboard_duplicate_pose: '이미 수집한 자세입니다. 위치·거리·기울기를 바꿔 주세요.',
  calibration_collection_full: '40장을 모았습니다. 계산하거나 마지막 장면을 제외하고 자세를 바꾸세요.',
  last_view_removed: '마지막 장면을 제외했습니다. 다른 자세를 보여 주세요.',
  calibration_views_not_diverse_enough: '장수와 위치·거리·기울기 조건을 모두 채워 주세요.',
  calibration_reprojection_error_too_high: '오차가 큽니다. 인쇄물·칸 크기·초점과 흐린 장면을 확인하고 다시 수집하세요.',
  calibration_unstable_focal_length: '초점거리 계산이 불안정합니다. 상하·좌우 기울임과 거리 변화를 더 모으세요.',
  calibration_validation_failed: '검증 장면의 자세를 구하지 못했습니다. 더 선명한 장면으로 다시 수집하세요.',
  calibration_resolution_changed_restart: '수집 중 해상도가 달라졌습니다. 카메라 설정을 고정한 뒤 다시 시작하세요.',
  calibration_input_is_active: '다른 보정 작업이 실행 중이거나 장치를 정리 중입니다. 중단 후 잠시 기다려 주세요.',
  mapping_input_is_active_stop_before_calibration: '지도·탐지 영상 수신을 먼저 종료한 뒤 보정을 시작하세요.',
  calibration_not_enabled: '이 서버에서 보정 촬영을 지원하지 않습니다. 새 CMD 실행기로 다시 여세요.',
  calibration_tello_not_enabled: '드론 영상 입력이 활성화된 실행기로 여세요.',
  tello_sdk_unavailable_check_wifi: '드론에 연결하지 못했습니다. 노트북 Wi-Fi 연결과 기체 전원을 확인하세요.',
  tello_streamon_failed: '드론 영상 시작에 실패했습니다. 다른 영상 수신 프로그램을 종료하세요.',
  tello_video_timeout_check_wifi_and_udp: '드론 영상이 끊겼습니다. Wi-Fi·전원·UDP 수신을 확인하고 다시 수집하세요.',
  calibration_session_mismatch: '보정 세션이 바뀌었습니다. 현재 카메라를 해제하고 다시 시작하세요.',
};
export const calibrationMessage = value => messages[value] || value;
export function coverageGuide(coverage) {
  const checks = coverage?.checks || {};
  return [['views', '유효한 장면 18장 이상'], ['positions', '화면 4개 영역 이상 · 좌우·상하 이동'],
    ['distances', '가까이·멀리 거리 변화'], ['tilts', '좌우·상하 기울인 장면 3장 이상']].map(([key, label]) => ({label, done: Boolean(checks[key])}));
}

export function connectCalibration({profiles, onBusy = () => {}} = {}) {
  const $ = id => document.getElementById(id);
  let active = null, result = null, resultSource = null, method = null, selection = null;
  const video = $('calibration-video'), preview = $('calibration-preview');
  const canvas = document.createElement('canvas'), context = canvas.getContext('2d');
  const owns = run => active === run && !run.cancelled;
  function step(number) {
    document.querySelectorAll?.('.tutorial-steps li').forEach((item, index) => {
      item.dataset.current = String(index + 1 === number);
      item.setAttribute('aria-current', index + 1 === number ? 'step' : 'false');
    });
  }
  function stage(name) {
    for (const id of ['method', 'media', 'setup', 'collect', 'result'])
      $('calibration-' + id).hidden = id !== name;
    step({method: 1, media: 2, setup: 3, collect: 3, result: 4}[name]);
  }
  function open() {
    $('calibration-workspace').hidden = false;
    document.querySelector('main').classList.add('calibration-open');
    stage('method'); $('calibration-close').focus();
  }
  async function close() {
    if (active) await stop();
    $('calibration-workspace').hidden = true;
    document.querySelector('main').classList.remove('calibration-open');
    $(`profile-slot-${location.hash === '#live' ? 'live' : 'saved'}`).append($('camera-profile-panel'));
    $('calibration-open').focus();
  }
  $('calibration-open').addEventListener('click', open);
  $('calibration-close').addEventListener('click', () => { void close(); });
  for (const value of ['file', 'live']) $('calibration-method-' + value).addEventListener('click', () => {
    method = value; selection = null;
    $('calibration-media-title').textContent = value === 'file' ? '촬영 자료를 선택해 주세요' : '촬영할 카메라를 선택해 주세요';
    $('calibration-media-error').hidden = true;
    $('calibration-file-controls').hidden = value !== 'file';
    $('calibration-live-controls').hidden = value !== 'live';
    $('calibration-start').textContent = value === 'file' ? '파일 분석·보정 시작' : '촬영 시작';
    $('calibration-collect-title').textContent = value === 'file' ? '파일을 분석하고 있어요' : '체크보드 장면을 모으고 있어요';
    $('calibration-confirm-text').textContent = value === 'file' || $('calibration-source').value !== 'tello' ?
      '실제 체크보드의 교차점 수·한 칸 크기와 입력값이 같습니다.' :
      '체크보드 수치가 맞고, 비행하지 않는 드론의 기체와 프로펠러를 고정했습니다.';
    stage('media');
  });
  $('calibration-media-back').addEventListener('click', () => stage('method'));
  $('calibration-media-next').addEventListener('click', () => {
    try {
      if (method === 'file') selection = validateCalibrationFiles($('calibration-files').files);
      $('calibration-media-error').hidden = true;
      $('calibration-confirm').checked = false; stage('setup');
    } catch (error) { $('calibration-media-error').textContent = error.message; $('calibration-media-error').hidden = false; }
  });
  $('calibration-setup-back').addEventListener('click', () => stage('media'));
  async function request(action, body, options = {}) {
    const response = await fetch('/api/calibration' + (action ? '/' + action : ''), {
      signal: AbortSignal.timeout(15000), cache: 'no-store', ...options,
      ...(body === undefined ? {} : {method: 'POST', headers: {'X-Mapping-Client': '1',
        ...(body instanceof Blob ? {'Content-Type': 'image/jpeg'} : {'Content-Type': 'application/json'})},
        body: body instanceof Blob ? body : JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw new Error(calibrationMessage(value.error || '보정 요청 실패'));
    return value;
  }
  const status = text => { $('calibration-status').textContent = text; };
  function busy(value) {
    onBusy(value);
    $('calibration-start').disabled = value;
    $('calibration-stop').disabled = !value;
    for (const id of ['calibration-columns', 'calibration-rows', 'calibration-square', 'calibration-source',
      'calibration-device', 'calibration-size', 'calibration-confirm', 'calibration-devices', 'calibration-files']) $(id).disabled = value;
  }
  function release(run) {
    clearTimeout(run.frameTimer); clearTimeout(run.pollTimer); clearTimeout(run.previewTimer);
    run.media?.getTracks().forEach(track => track.stop()); run.media = null;
    video.pause(); video.srcObject = null; video.hidden = true;
  }
  function refreshPreview(run) {
    if (!owns(run) || !run.id || run.source !== 'tello') return;
    preview.src = '/api/calibration/preview?id=' + run.id + '&t=' + Date.now();
  }
  function queuePreview(delay) {
    const run = active;
    if (!run || !owns(run) || !run.id || run.source !== 'tello') return;
    clearTimeout(run.previewTimer);
    run.previewTimer = setTimeout(() => refreshPreview(run), delay);
  }
  preview.addEventListener('load', () => {
    if (active && owns(active) && active.source === 'tello') preview.hidden = false;
    queuePreview(100);
  });
  preview.addEventListener('error', () => queuePreview(250));
  async function stop(run = active, text = '보정을 중단하고 카메라를 해제했습니다. 다시 시작하면 새로 수집합니다.') {
    if (!run || !owns(run)) return;
    run.cancelled = true;
    release(run); active = null; busy(false);
    $('calibration-solve').disabled = $('calibration-undo').disabled = true;
    $('calibration-retry').hidden = false;
    status(text);
    if (!$('calibration-workspace').hidden) stage('collect');
    if (run.id) {
      try { await request('stop', {id: run.id}); }
      catch { if (!active) status(text + ' 서버에 중단을 전달하지 못했습니다. 수신은 45초 이내 자동 해제됩니다.'); }
    }
  }
  function render(run, state) {
    if (!owns(run) || state.id !== run.id) return;
    $('calibration-progress').value = Math.min(state.accepted, 18);
    $('calibration-coverage').replaceChildren(...coverageGuide(state.coverage).map(item => {
      const li = document.createElement('li'); li.textContent = `${item.done ? '✓' : '○'} ${item.label}`;
      li.dataset.done = String(item.done); return li;
    }));
    $('calibration-solve').disabled = state.status !== 'collecting' || !state.coverage.ready || run.processing || run.solving;
    $('calibration-undo').disabled = state.status !== 'collecting' || !state.accepted;
    if (run.solving && state.status === 'collecting' && state.error) {
      run.solving = false;
      if (run.source === 'camera') void frame(run);
      if (run.source === 'file') $('calibration-additional').hidden = false;
    }
    const geometry = state.size ? `${state.size[0]}×${state.size[1]} 원본 · ` : '';
    if (state.status === 'calculating') stage('collect');
    const checks = state.coverage?.checks || {};
    $('calibration-guide').textContent = !checks.views ? '정면부터 시작해 위치·거리·기울기를 바꾸며 18장 이상 모으세요. 각 자세에서 잠시 멈춥니다.' :
      !checks.positions ? '체크보드를 화면의 왼쪽·오른쪽·위·아래로 옮겨 주세요. 흰 여백과 전체 무늬는 화면 안에 둡니다.' :
      !checks.distances ? '체크보드를 가까이·멀리 움직여 화면에 보이는 크기를 바꿔 주세요.' :
      !checks.tilts ? '체크보드를 좌우·상하로 기울인 서로 다른 자세를 3장 이상 보여 주세요.' :
      '수집 조건을 채웠습니다. 보정값 계산을 누르거나 최대 40장까지 더 모을 수 있습니다.';
    status(`${geometry}${state.accepted}/40장 · ` + (state.status === 'calculating' ? '보정값 계산·검증 중입니다.' :
      state.error ? calibrationMessage(state.error) : state.reason === 'accepted' ? '새 장면을 수집했습니다.' : calibrationMessage(state.reason)));
    if (state.status === 'failed' || state.status === 'stopped') {
      void stop(run, state.error ? calibrationMessage(state.error) : '서버에서 보정을 종료했습니다. 다시 시작하세요.'); return;
    }
    if (state.status === 'complete' && state.camera) {
      result = {...state.camera, camera_identity: run.identity};
      resultSource = run.source;
      release(run); active = null; busy(false);
      $('calibration-solve').disabled = $('calibration-undo').disabled = true;
      stage('result');
      const s = result.calibration_summary;
      $('calibration-result-info').textContent = `${result.width}×${result.height} · 유효 ${s.views}장 · 검증 오차 ${s.validation_rms_px.toFixed(3)}px`;
      $('calibration-result-values').textContent = `계산 ${s.training_views}장 / 별도 검증 ${s.validation_views}장 · 계산 RMS ${s.training_rms_px.toFixed(3)}px · ` +
        `fx ${result.fx.toFixed(3)}, fy ${result.fy.toFixed(3)}, cx ${result.cx.toFixed(3)}, cy ${result.cy.toFixed(3)} · ` +
        `왜곡 ${result.distortion.map(x => x.toFixed(6)).join(', ')}`;
      $('calibration-download').href = '/api/calibration/camera.json?id=' + encodeURIComponent(run.id);
      $('calibration-report-download').href = '/api/calibration/report.pdf?id=' + encodeURIComponent(run.id);
      status('계산과 검증을 통과했습니다. JSON을 내려받거나 카메라 정보를 확인해 새 프로필로 등록하세요.');
    }
  }
  async function poll(run) {
    if (!owns(run)) return;
    try {
      const state = await request('state?id=' + run.id);
      if (!owns(run)) return;
      render(run, state);
      run.failures = 0;
    } catch (error) {
      if (!owns(run)) return;
      if (++run.failures >= 3) { await stop(run, `보정 서버 연결이 끊겼습니다. ${error.message}`); return; }
      status('서버 연결을 다시 확인 중입니다.');
    }
    if (owns(run)) run.pollTimer = setTimeout(() => poll(run), 1000);
  }
  async function frame(run) {
    if (!owns(run) || run.solving || run.frameBusy) return;
    run.frameBusy = true;
    try {
      if (video.readyState >= 2 && video.videoWidth) {
        canvas.width = video.videoWidth; canvas.height = video.videoHeight;
        context.drawImage(video, 0, 0);
        const jpeg = await new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', .9));
        if (!owns(run) || run.solving) return;
        if (!jpeg) throw new Error('카메라 장면을 읽지 못했습니다.');
        const state = await request('frame?id=' + run.id, jpeg);
        if (!owns(run)) return;
        render(run, state);
      }
    } catch (error) {
      if (owns(run) && !run.solving) { await stop(run, `수집을 중단했습니다. ${error.message}`); return; }
    } finally {
      run.frameBusy = false;
    }
    if (owns(run) && !run.solving) run.frameTimer = setTimeout(() => frame(run), 800);
  }
  async function files(run, selected) {
    if (!owns(run) || run.processing) return;
    let autoSolve = false;
    run.processing = true;
    $('calibration-additional').hidden = true;
    $('calibration-add-files').disabled = true;
    $('calibration-solve').disabled = true;
    try {
      run.fileSize = await feedCalibrationFiles(selected, {canvas, context, expectedSize: run.fileSize,
        isCurrent: () => owns(run) && !run.solving,
        onFrame: jpeg => request('frame?id=' + run.id, jpeg),
        onProgress: (index, total, state) => {
          if (!owns(run)) return;
          render(run, state);
          status(`${total}개 장면 중 ${index}개 분석 · 유효 ${state.accepted}장 · ${calibrationMessage(state.reason)}`);
          preview.src = '/api/calibration/preview?id=' + run.id + '&t=' + Date.now(); preview.hidden = false;
        }});
      if (owns(run)) {
        const state = await request('state?id=' + run.id);
        if (!owns(run)) return;
        render(run, state);
        $('calibration-additional').hidden = state.coverage.ready;
        autoSolve = state.coverage.ready;
        status(autoSolve ? '파일 분석을 마쳤습니다. 보정값을 계산하고 검증합니다.' :
          `유효 ${state.accepted}장 · ${$('calibration-guide').textContent} 부족한 장면을 추가할 수 있습니다.`);
      }
    } catch (error) {
      if (owns(run)) {
        const state = await request('state?id=' + run.id).catch(() => null);
        run.fileSize = state?.size || run.fileSize;
        status(`파일 분석을 멈췄습니다. ${error.message} 다른 자료를 추가하거나 중단하세요.`);
        $('calibration-additional').hidden = false;
      }
    } finally {
      run.processing = false;
      $('calibration-add-files').disabled = false;
      if (owns(run)) {
        const state = await request('state?id=' + run.id).catch(() => null);
        if (state && owns(run)) $('calibration-solve').disabled = autoSolve || !state.coverage.ready;
      }
    }
    if (autoSolve && owns(run)) await solve(run);
  }
  function options() {
    return {columns: Number($('calibration-columns').value), rows: Number($('calibration-rows').value),
      square_mm: Number($('calibration-square').value)};
  }
  function validBoard(board) {
    return Number.isInteger(board.columns) && board.columns >= 4 && board.columns <= 12 &&
      Number.isInteger(board.rows) && board.rows >= 4 && board.rows <= 10 &&
      Number.isFinite(board.square_mm) && board.square_mm >= 5 && board.square_mm <= 60;
  }
  function boardLink() {
    const b = options();
    $('calibration-board').href = '/api/calibration/board.svg?' + new URLSearchParams(b);
    $('calibration-board-note').textContent = `${b.columns}×${b.rows}은 안쪽 교차점 수이며 칸 수는 ${b.columns+1}×${b.rows+1}개입니다. 한 칸은 실측 ${b.square_mm}mm로 맞추세요.`;
    $('calibration-confirm').checked = false;
  }
  for (const id of ['calibration-columns', 'calibration-rows', 'calibration-square']) $(id).addEventListener('change', boardLink);
  function sourceChanged() {
    const tello = $('calibration-source').value === 'tello';
    $('calibration-device-label').hidden = $('calibration-size-label').hidden = $('calibration-devices').hidden = tello;
    $('calibration-confirm').checked = false;
    $('calibration-confirm-text').textContent = tello ?
      '체크보드 수치가 맞고, 비행하지 않는 드론의 기체와 프로펠러를 고정했습니다.' :
      '실제 체크보드의 교차점 수·한 칸 크기와 입력값이 같습니다.';
    $('calibration-source-note').hidden = tello;
    $('calibration-source-note').textContent = tello ? '' : '지도에 사용할 초점·줌·방향·크롭을 유지하며 촬영하세요.';
  }
  $('calibration-source').addEventListener('change', sourceChanged);
  for (const id of ['calibration-device', 'calibration-size']) $(id).addEventListener('change', () => { $('calibration-confirm').checked = false; });
  async function devices() {
    const found = await navigator.mediaDevices?.enumerateDevices();
    const selected = $('calibration-device').value;
    $('calibration-device').replaceChildren(new Option('브라우저 기본 카메라', ''),
      ...(found || []).filter(d=>d.kind === 'videoinput').map((d,i)=>new Option(d.label || `카메라 ${i+1}`, d.deviceId)));
    $('calibration-device').value = selected;
  }
  $('calibration-devices').addEventListener('click', () => devices().catch(error=>status(error.message)));
  $('calibration-add-files').addEventListener('click', () => {
    try { void files(active, validateCalibrationFiles($('calibration-extra-files').files, {additional: true})); }
    catch (error) { status(error.message); }
  });
  $('calibration-retry').addEventListener('click', () => stage('setup'));
  $('calibration-start').addEventListener('click', async () => {
    if (active) return;
    const error = !validBoard(options()) ? '체크보드 교차점 수와 한 칸 크기를 확인하세요.' :
      !$('calibration-confirm').checked ? '실제 체크보드 수치를 확인하고 확인란을 체크하세요.' : '';
    if (error) { $('calibration-setup-error').textContent = error; $('calibration-setup-error').hidden = false; status(error); return; }
    $('calibration-setup-error').hidden = true;
    const run = {id: null, source: method === 'file' ? 'file' : $('calibration-source').value,
      cancelled: false, failures: 0, solving: false, processing: false};
    active = run; result = null; busy(true);
    stage('collect'); $('calibration-preview-area').hidden = false; $('calibration-additional').hidden = true;
    $('calibration-retry').hidden = true;
    preview.hidden = true; preview.removeAttribute('src');
    $('calibration-progress').value = 0; $('calibration-coverage').replaceChildren();
    try {
      const caps = await request('');
      if (!owns(run)) return;
      if (!caps.enabled || caps.busy) throw new Error(calibrationMessage(caps.busy ? 'calibration_input_is_active' : 'calibration_not_enabled'));
      if (run.source === 'camera') {
        if (!navigator.mediaDevices?.getUserMedia) throw new Error('localhost/HTTPS에서 카메라 지원 브라우저로 여세요.');
        status('카메라 권한을 확인합니다. 허용한 뒤 체크보드를 비춰 주세요.');
        const [width, height] = $('calibration-size').value.split('x').map(Number), deviceId = $('calibration-device').value;
        const media = await navigator.mediaDevices.getUserMedia({video: {width: {ideal: width}, height: {ideal: height},
          ...(deviceId ? {deviceId: {exact: deviceId}} : {})}, audio: false});
        if (!owns(run)) { media.getTracks().forEach(track=>track.stop()); return; }
        run.media = media; video.srcObject = media; video.hidden = false;
        run.identity = media.getVideoTracks()[0]?.label || '연결된 카메라';
        media.getVideoTracks()[0]?.addEventListener('ended', () => { if (owns(run)) void stop(run, '카메라 연결이 종료됐습니다. 다시 연결하세요.'); }, {once: true});
        await video.play();
        if (!owns(run)) return;
        void devices().catch(()=>{});
      } else run.identity = run.source === 'file' ? '업로드한 촬영 자료' : '드론 카메라 · 튜토리얼 촬영 기체';
      const state = await request('start', {...options(), source: run.source, camera_identity: run.identity, confirmed: true});
      if (!owns(run)) { await request('stop', {id: state.id}); return; }
      run.id = state.id; render(run, state);
      if (run.source === 'tello') refreshPreview(run);
      void poll(run); if (run.source === 'camera') void frame(run);
      if (run.source === 'file') void files(run, selection);
    } catch (error) {
      if (owns(run)) await stop(run, error.name === 'NotAllowedError' ? '카메라 권한이 거부됐습니다. 브라우저 권한을 허용한 뒤 다시 시작하세요.' : error.message);
    }
  });
  $('calibration-stop').addEventListener('click', () => stop());
  $('calibration-undo').addEventListener('click', async () => {
    const run = active; if (!run?.id) return;
    try { render(run, await request('undo', {id: run.id})); } catch (error) { if (owns(run)) status(error.message); }
  });
  async function solve(run) {
    if (!run?.id || !owns(run) || run.solving || run.processing) return;
    run.solving = true; clearTimeout(run.frameTimer); $('calibration-solve').disabled = true;
    try { render(run, await request('solve', {id: run.id})); }
    catch (error) { if (owns(run)) { run.solving = false; status(error.message); if (run.source === 'camera') void frame(run);
      if (run.source === 'file') $('calibration-additional').hidden = false; } }
  }
  $('calibration-solve').addEventListener('click', () => { void solve(active); });
  $('calibration-register').addEventListener('click', async () => {
    if (!result) return;
    try {
      $('profile-slot-registration').append($('camera-profile-panel'));
      await profiles.prepareCamera(result, {mode: resultSource});
      $('profile-slot-registration').scrollIntoView?.({behavior: 'smooth', block: 'start'});
      status('아래에서 프로필 이름과 촬영 설정을 확인한 뒤 저장하세요.');
    } catch (error) { status(`프로필 등록 준비 실패: ${error.message}`); }
  });
  window.addEventListener('pagehide', () => {
    const run = active; if (!run) return;
    run.cancelled = true; release(run); active = null; onBusy(false);
    if (run.id) void request('stop', {id: run.id}, {keepalive: true}).catch(()=>{});
  });
  boardLink(); sourceChanged(); step(1);
  return {stop, get active() { return Boolean(active); }};
}
