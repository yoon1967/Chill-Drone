const stages = {upload: '자료 복사 중', inspect: '자료 검사 중', confirm: '입력 확인 대기', input: '프레임 준비 중',
  features: '특징점 추출·매칭 중', reconstruction: '3D 지도 복원 중', preview: '미리보기 생성 중', finalization: '결과 검증 중', finished: '복원 완료'};
const errors = {
  record_busy: '사용 중인 복원 기록입니다. 처리가 끝난 뒤 삭제하세요.',
  depth_mapping_dependencies_missing: '빠른 복원 준비가 필요합니다. 실행 중인 CMD 창을 닫고 3D 매핑 실행기를 다시 실행하면 필요한 패키지와 모델을 자동으로 준비합니다.',
  slam_image_missing: '카메라 위치 추적용 Docker 이미지가 없습니다. 실행 안내에 따라 ssafy-orbslam3:local 이미지를 준비하세요.',
  slam_docker_unavailable: '카메라 위치 추적을 위해 Docker Desktop을 먼저 실행하세요.',
  import_busy: '진행 중인 불러오기가 있습니다. 완료하거나 취소한 뒤 다시 선택하세요.',
  reconstruction_busy: '다른 복원이 실행 중입니다. 완료 뒤 다시 시작하세요.',
  stop_live_mapping_before_reconstruction: '실시간 매핑을 종료한 뒤 새 지도 복원을 시작하세요.',
  need_3_to_10000_frames: 'JPG·PNG 프레임을 3~10,000장 선택하세요.',
  frame_dimensions_differ: '프레임 해상도가 서로 다릅니다. 같은 카메라·해상도의 이미지를 선택하세요.',
  invalid_relative_path: '폴더 안에 지원하지 않는 파일 경로가 있습니다.',
  insufficient_disk_space: '저장 공간이 부족합니다.',
  unsupported_input_use_session_zip_video_or_frames: '세션 ZIP·결과 폴더·동영상·JPG/PNG 폴더를 선택하세요.',
  external_import_requires_reselection: '외부 폴더를 CMD에 다시 드롭하거나 웹에서 폴더를 다시 선택하세요.',
  reconstruction_history_incomplete: '복원 기록의 프레임 또는 이력 파일이 빠져 있습니다. 전체 복원 기록 ZIP을 다시 내려받아 선택하세요.',
  reconstruction_result_not_ready: '최종 결과가 없는 복원 기록입니다. 복원이 끝난 뒤 전체 복원 기록 ZIP을 다시 내려받아 선택하세요.',
  calibration_dimensions_must_match_input: '보정 JSON의 해상도가 현재 영상·프레임과 다릅니다. 같은 크기·방향·크롭의 보정 파일을 선택하세요.',
  confirm_input_order_and_camera: '카메라·해상도 또는 프레임 순서·FPS 확인란을 체크하세요.',
};
const details = {
  upload: '선택한 자료를 이 PC의 작업 폴더로 전송하고 있습니다.',
  inspect: '파일 형식·해상도·프레임 순서와 저장 결과의 구성을 확인하고 있습니다.',
  confirm: '프레임 추출 간격과 카메라 설정을 확인한 뒤 복원을 시작하세요.',
  input: '영상의 방향과 시각을 확인하고 복원에 사용할 프레임을 준비하고 있습니다.',
  reconstruction: '같은 특징점으로 카메라 위치와 3D 점을 계산·최적화하고 있습니다.',
  preview: '복원 결과를 검사하고 3D 전체 보기와 투영 이미지를 만들고 있습니다.',
  finalization: '지도·이미지·메타데이터의 연결과 파일 체크섬을 검증하고 있습니다.',
};
export function importMessage(job) {
  if (job.status === 'complete') return '복원 완료';
  if (job.status === 'failed') return `불러오기 실패: ${errors[job.error] || job.error}`;
  if (job.status === 'cancelled') return '취소했습니다. 원본 자료와 이미 생성된 진단은 보존합니다.';
  if (job.status === 'ready') return '저장 자료를 확인했습니다. 지도를 열 수 있습니다.';
  if (job.status === 'cancelling') return '현재 프레임 처리를 중단하고 작업을 정리하고 있습니다.';
  if (job.options?.mode === 'depth_rgbd') {
    const progress = job.progress || {};
    const task = {input: '영상 준비 중', model: '깊이 모델 준비 중 · 첫 실행은 모델 다운로드가 필요합니다', depth: '현재 프레임 깊이 추정 중',
      tracking: '카메라 위치·방향 추적 중', pose_wait: '위치·방향 확정 대기 · 깊이 복원 보류', mapping: '확정한 자세로 3D 지도 복원 중', presentation: '현재 프레임 표시 대기 · 표시 후 다음 프레임 진행', integration: `전체 경로 지도 통합 중 · ${progress.integrated_frames || 0} / ${progress.integration_total || 0}관측`, saving: '전체 통합 지도 저장 중'}[job.stage] || '프레임 처리 중';
    return `${task} · ${progress.processed || 0}프레임 처리 · ${progress.points || 0}점`;
  }
  const progress = job.progress || {};
  const count = progress.total ? `${progress.completed} / 총 ${progress.total}` : `${progress.completed ?? 0}`;
  if (job.status === 'running' && job.stage === 'input') {
    if (progress.phase === 'decode') return `프레임 준비 중 · 원본 ${count}프레임 읽음 · 복원용 ${progress.selected}프레임 저장`;
    if (progress.phase === 'copy_frames') return `프레임 준비 중 · ${count}장 변환·복사 완료`;
    if (progress.phase === 'verify_source') return `프레임 준비 중 · ${progress.selected}프레임 저장 완료 · 원본 파일 변경 여부 검증 중`;
  }
  if (job.status === 'running' && job.stage === 'features' && job.feature_progress) {
    const {phase, completed, total, pairs_completed, pairs_total} = job.feature_progress;
    if (phase === 'export') return '특징점 결과 정리 중 · 좌표·대응 관계·품질 보고서와 매칭 미리보기 저장 중';
    if (phase === 'validation') return '특징점 결과 검증 중 · 입력 프레임·특징점·매칭 DB의 일치 여부 확인 중';
    if (phase === 'matching') return pairs_total ? `특징점 매칭 중 · ${pairs_completed} / 총 ${pairs_total}쌍 기하 검증 완료 · ${total}프레임 추출 완료` :
      `특징점 매칭·결과 정리 중 · 총 ${total}프레임 추출 완료`;
    return completed === null ? `특징점 추출 중 · 총 ${total}프레임 · 진행 수 확인 중` :
      `특징점 추출 중 · ${completed} / 총 ${total}프레임 완료 · 나머지 프레임 병렬 처리 중`;
  }
  if (job.status === 'running' && job.stage === 'reconstruction') {
    const task = {input_validation: '입력 프레임·특징점·카메라 설정 검증 중',
      mapping: '카메라 위치 등록·3D 점 생성·위치 최적화 중', export: '점 색상 추출·지도와 카메라 경로 저장 중'}[progress.stage];
    if (task) return `3D 지도 복원 중 · ${task}`;
  }
  if (job.status === 'running' && job.stage === 'preview') {
    const task = {validation: '저장 지도와 파일 무결성 검사 중', overview: '3D 전체 보기 이미지 그리는 중 · 0 / 2장 저장',
      projections: 'XY·XZ·YZ 투영 이미지 그리는 중 · 1 / 2장 저장', save: '2 / 2장 저장 완료 · 미리보기 메타데이터와 체크섬 저장 중'}[progress.phase];
    if (task) return `미리보기 생성 중 · ${task}`;
  }
  return details[job.stage] ? `${stages[job.stage]} · ${details[job.stage]}` : stages[job.stage] || job.status;
}

export function openUrl(id) { return `/?asset=${encodeURIComponent(id)}#saved`; }

const processingStages = ['inspect', 'input', 'features', 'reconstruction', 'preview', 'finalization'];
export function savedPhase(job) {
  if (job.status === 'awaiting_options') return 'settings';
  if (['running', 'cancelling', 'uploading', 'inspecting'].includes(job.status)) return 'processing';
  if (['ready', 'complete'].includes(job.status)) return 'ready';
  return 'input';
}

export function processingSteps(job) {
  if (job.options?.mode === 'depth_rgbd') {
    const entries = [['model', '추적·깊이 모델 준비'], ['tracking', '① 위치·방향 확정'], ['depth', '② 깊이 추정'], ['mapping', '③ 확정 자세로 복원'], ['integration', '④ 전체 경로 통합'], ['saving', '전체 지도 저장']];
    const index = job.status === 'complete' ? entries.length : job.stage === 'pose_wait' ? 1 : job.stage === 'presentation' ? 3 : Math.max(0, entries.findIndex(([stage]) => stage === job.stage));
    return entries.map(([stage, label], i) => ({label, state: i < index ? 'done' : i === index && job.status === 'running' ? 'active' : 'waiting'}));
  }
  const index = job.status === 'complete' ? processingStages.length : processingStages.indexOf(job.stage);
  return processingStages.map((stage, i) => ({label: job.status === 'running' && job.stage === stage ? importMessage(job) : stages[stage],
    state: i < index ? 'done' : i === index && job.status === 'running' ? 'active' : 'waiting'}));
}

async function droppedFiles(items) {
  const files = [];
  async function visit(entry, prefix = '') {
    if (entry.isFile) {
      const file = await new Promise((resolve, reject) => entry.file(resolve, reject));
      files.push({file, path: prefix + file.name});
    } else if (entry.isDirectory) {
      const reader = entry.createReader();
      while (true) {
        const batch = await new Promise((resolve, reject) => reader.readEntries(resolve, reject));
        if (!batch.length) break;
        for (const child of batch) await visit(child, prefix + entry.name + '/');
      }
    }
  }
  for (const item of items) {
    const entry = item.webkitGetAsEntry?.();
    if (entry) await visit(entry);
    else { const file = item.getAsFile(); if (file) files.push({file, path: file.name}); }
  }
  return files;
}

export function connectImport({profiles = null, onPhase = () => {}} = {}) {
  const $ = id => document.getElementById(id);
  let active = null, polling = null, uploadController = null, working = false;
  let previewUrl = null, startedAt = null, timer = null, creating = false, openingRecord = false;
  let navigatingToResult = false;
  let initialHandled = false;
  let viewingCurrent = false;
  let lastJob = null, navigationPhase = null, observedPhase = null;
  let progressive = null, progressiveLoading = null;
  function updateProgressive(job) {
    const fast = job.options?.mode === 'depth_rgbd' || job.history_available;
    $('progressive-workspace').hidden = !fast;
    document.body.classList?.toggle('progressive-processing', fast && ['running', 'cancelling', 'ready', 'complete', 'failed', 'cancelled'].includes(job.status));
    if (!fast) { progressive?.dispose(); progressive = null; return; }
    if (progressive) { progressive.update(job); return; }
    progressiveLoading ||= import('/progressive-map.js').then(({connectProgressiveMap}) => {
      if (document.body.dataset.savedPhase === 'result') return;
      if (lastJob?.options?.mode !== 'depth_rgbd' && !lastJob?.history_available) return;
      progressive = connectProgressiveMap(); progressive.update(lastJob);
    }).catch(error => { $('progressive-state').textContent = `3D 표시를 준비하지 못했습니다: ${error.message}`; }).finally(() => { progressiveLoading = null; });
  }
  const asset = new URLSearchParams(location.search).get('asset');
  function phase(value) {
    document.body.dataset.savedPhase = value;
    $('saved-title').textContent = value === 'result' ? '복원한 공간을 살펴보세요.' :
      value === 'processing' ? '촬영 영상에서 공간을 만들고 있습니다.' : '촬영한 영상으로 공간을 복원하세요.';
    const current = value === 'processing' && ['uploading', 'inspecting'].includes(lastJob?.status) ? 0 :
      {input: 0, settings: 1, processing: 2, ready: 3, result: 3}[value] ?? 0;
    onPhase(current);
    $('saved-new').hidden = value === 'input';
    $('saved-library-section').hidden = !['input', 'ready'].includes(value);
    $('saved-input').hidden = value !== 'input';
    $('saved-output-note').hidden = value === 'input' || value === 'result';
  }
  phase('input');
  function clearPreview() {
    $('saved-preview').pause(); $('saved-preview').removeAttribute('src'); $('saved-preview').load();
    if (previewUrl) URL.revokeObjectURL(previewUrl);
    previewUrl = null; $('saved-preview-wrap').hidden = true;
  }
  function reset() {
    if (working) return;
    clearTimeout(polling); clearInterval(timer); timer = null; startedAt = null;
    active = null; lastJob = null; navigationPhase = null; observedPhase = null;
    progressive?.dispose(); progressive = null; $('progressive-workspace').hidden = true; document.body.classList.remove('progressive-processing');
    creating = true; openingRecord = false; viewingCurrent = false; clearPreview();
    $('import-details').open = true;
    for (const id of ['import-summary', 'saved-inspection', 'import-progress']) $(id).hidden = true;
    $('import-status').textContent = '먼저 촬영 영상을 선택하세요.';
    $('import-status').dataset.state = '';
    $('import-video').pause(); $('import-selected').hidden = true;
    phase('input'); $('import-file').focus();
  }
  $('saved-replay').addEventListener('click', () => { if (!lastJob?.history_available) return; creating = true; render(lastJob, false); phase('processing'); applyNavigation('processing'); $('import-details').open = true; });
  $('import-open').addEventListener('click', event => {
    if (!active || active !== asset || !['ready', 'complete'].includes(lastJob?.status) ||
        event.button > 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
    event.preventDefault();
    creating = false; openingRecord = false; viewingCurrent = true; navigationPhase = 'result';
    progressive?.dispose(); progressive = null; $('progressive-workspace').hidden = true;
    document.body.classList.remove('progressive-processing');
    phase('result'); applyNavigation('result');
  });
  $('saved-new').addEventListener('click', reset);
  $('saved-change').addEventListener('click', reset);
  $('saved-current').addEventListener('click', () => {
    if (working) return;
    creating = false; viewingCurrent = true; phase('result'); $('import-details').open = false;
  });
  $('saved-existing').addEventListener('click', () => {
    $('saved-result-folder').click();
  });
  $('saved-open-zip').addEventListener('click', () => $('saved-archive').click());
  window.addEventListener('saved-map-ready', () => {
    if (!asset) $('saved-current').hidden = false;
    if (creating) return;
    if (!asset && !viewingCurrent) return;
    phase('result'); $('import-details').open = false;
    if (asset) $('saved-media-slot').append($('import-selected'));
  });
  window.addEventListener('pagehide', () => { progressive?.dispose(); clearPreview(); clearInterval(timer); clearTimeout(polling); uploadController?.abort(); });
  async function request(action, body, query = '') {
    const response = await fetch(`/api/import/${action}${query}`, body === undefined ? {cache: 'no-store'} :
      {method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok) throw new Error(errors[value.error] || value.error);
    return value;
  }
  function fail(error) {
    $('import-status').textContent = error.name === 'AbortError' ? '전송을 취소했습니다.' : error.message;
    $('import-status').dataset.state = 'error'; $('import-progress').hidden = true;
    // A validation error must not release an already running upload/reconstruction.
    if (!working) $('saved-change').disabled = false;
  }
  function applyNavigation(value) {
    $('import-details').open = value !== 'result';
    $('import-options').hidden = value !== 'settings';
    $('saved-inspection').hidden = value === 'input' || !lastJob?.summary;
    $('import-summary').hidden = value === 'input';
    $('import-start').hidden = value !== 'settings' || lastJob?.status !== 'awaiting_options';
    $('saved-processing').hidden = value !== 'processing';
  }
  function render(job, navigateToResult = true) {
    if (['running', 'cancelling'].includes(job.status)) {
      $('saved-preview').pause(); $('import-video').pause(); $('saved-preview-wrap').hidden = true;
    }
    if (active !== job.id) { $('import-confirm').checked = false; $('import-camera').value = ''; profiles?.resetFileConfirmation(); }
    const previousPhase = document.body.dataset.savedPhase;
    active = job.id;
    lastJob = job;
    updateProgressive(job);
    const keepPreview = job.options?.mode === 'depth_rgbd' && (job.status === 'complete' ||
      (['failed', 'cancelled'].includes(job.status) && job.progress?.revision > 0));
    const nextPhase = keepPreview ? 'processing' : savedPhase(job);
    if (observedPhase !== nextPhase) navigationPhase = null;
    observedPhase = nextPhase;
    phase(navigationPhase || nextPhase);
    if (job.status === 'awaiting_options' && previousPhase !== 'settings') $('profile-manager').open = true;
    $('import-status').dataset.state = job.status === 'failed' ? 'error' : '';
    $('saved-inspection').hidden = !job.summary;
    $('saved-change').disabled = !['awaiting_options', 'ready', 'complete', 'failed', 'cancelled'].includes(job.status);
    $('saved-new').disabled = $('saved-change').disabled;
    $('saved-processing').hidden = !['running', 'cancelling', 'complete', 'failed', 'cancelled'].includes(job.status);
    $('saved-processing-title').textContent = job.status === 'complete' ? '복원 결과를 준비했습니다.' :
      job.status === 'failed' ? '복원을 마치지 못했습니다. 입력과 설정을 확인해 주세요.' :
      job.status === 'cancelled' ? '복원을 취소했습니다.' : job.status === 'cancelling' ? '복원을 취소하고 있습니다.' : '공간을 복원하고 있습니다.';
    $('saved-processing-steps').replaceChildren(...processingSteps(job).map(step => {
      const li = document.createElement('li'); li.dataset.state = step.state;
      li.textContent = `${step.state === 'done' ? '✓ ' : ''}${step.label.replace(/ 중$/, '')}`;
      if (step.state === 'active') li.setAttribute('aria-current', 'step');
      return li;
    }));
    if (job.status === 'running' && !startedAt) {
      startedAt = Date.now();
      const tick = () => { $('saved-elapsed').textContent = `이 화면에서 처리 시간 ${Math.floor((Date.now() - startedAt) / 1000)}초 · 처리 수는 저장된 결과 기준이며 병렬 계산 중에는 잠시 유지될 수 있습니다.`; };
      tick(); timer = setInterval(tick, 1000);
    }
    if (['complete', 'failed', 'cancelled', 'ready'].includes(job.status)) { clearInterval(timer); timer = null; }
    $('saved-processing-settings').textContent = `${job.summary?.name || job.name || ''} · 초당 추출 ${job.options?.sample_fps ?? $('import-sample-fps').value}프레임 · ${job.options?.profile_selection ? '선택한 보정 프로필 적용' : job.options?.camera ? '직접 보정 JSON 적용' : '카메라 값 자동 추정'}`;
    $('import-status').textContent = importMessage(job);
    $('import-summary').hidden = false;
    const summary = job.summary;
    $('import-options').hidden = job.status !== 'awaiting_options';
    $('import-start').hidden = job.status !== 'awaiting_options';
    $('import-start').disabled = !$('import-confirm').checked;
    for (const id of ['import-mode', 'import-sample-fps', 'import-frame-fps', 'import-camera', 'import-confirm'])
      $(id).disabled = job.status !== 'awaiting_options';
    $('import-open').hidden = !['ready', 'complete'].includes(job.status);
    $('import-open').href = openUrl(job.id);
    $('import-cancel').hidden = ['ready', 'complete', 'failed', 'cancelled'].includes(job.status);
    $('import-progress').hidden = !['uploading', 'inspecting', 'running', 'cancelling'].includes(job.status);
    if (job.status !== 'uploading') $('import-progress').removeAttribute('value');
    if (summary) {
      $('import-info').textContent = ['session', 'result'].includes(summary.kind) ?
        `${summary.name} · ${summary.points.toLocaleString()}점 · 키프레임 ${summary.keyframes}장 · 지도 기록 ${summary.revisions}개` :
        `${summary.name} · ${summary.width}×${summary.height}${Number.isFinite(summary.duration_s) ? ` · ${summary.duration_s.toFixed(1)}초` : ''} · ${summary.frames ?? '미확인'}프레임${summary.order ? ` · 순서: ${summary.order.join(' → ')}${summary.frames > summary.order.length ? ' → … → ' + summary.last_frame : ''}` : ''}`;
      $('import-warnings').replaceChildren(...(summary.warnings || []).map(text => { const item = document.createElement('li'); item.textContent = text; return item; }));
      $('import-sampling').hidden = summary.kind !== 'video';
      $('import-frame-timing').hidden = summary.kind !== 'frames';
    }
    if (['ready', 'complete', 'awaiting_options', 'failed', 'cancelled'].includes(job.status)) working = false;
    if (navigationPhase) applyNavigation(navigationPhase);
    if (navigateToResult && !navigatingToResult && (job.status === 'ready' ||
        (job.status === 'complete' && job.options?.mode !== 'depth_rgbd') || (openingRecord && job.status === 'complete'))) {
      navigatingToResult = true;
      clearTimeout(polling);
      location.assign(openUrl(job.id));
    }
  }
  async function poll(id) {
    clearTimeout(polling);
    try {
      const job = await request('status', undefined, '?id=' + encodeURIComponent(id));
      if (active !== id) return;
      render(job);
      if (navigatingToResult) return;
      if (['uploading', 'inspecting', 'running', 'cancelling'].includes(job.status)) polling = setTimeout(() => poll(id), 750);
      else refresh();
    } catch (error) {
      if (active !== id) return;
      fail(error);
      if (working) polling = setTimeout(() => poll(id), 2000);
    }
  }
  async function upload(files) {
    if (working) { fail(new Error('먼저 진행 중인 작업을 취소하거나 마쳐 주세요.')); return; }
    if (!files.length) return;
    navigationPhase = null; observedPhase = null;
    working = true;
    creating = true; openingRecord = false; startedAt = null; clearPreview();
    active = null;
    const video = files.length === 1 && /\.(mp4|mov|avi|mkv)$/i.test(files[0].file.name) ? files[0].file : null;
    if (video) {
      previewUrl = URL.createObjectURL(video); $('saved-preview').src = previewUrl;
      $('saved-preview-wrap').hidden = false;
      $('saved-preview-note').textContent = '선택한 원본 영상입니다. 재생이 지원되지 않아도 파일 검사를 진행할 수 있습니다.';
    }
    $('import-details').open = true;
    $('import-confirm').checked = false;
    $('import-camera').value = '';
    uploadController = new AbortController();
    try {
      const job = await request('create', {name: files[0].path.split('/')[0]});
      render(job);
      const total = files.reduce((n, item) => n + item.file.size, 0);
      let done = 0;
      for (const {file, path} of files) {
        const response = await fetch(`/api/import/upload?id=${job.id}&path=${encodeURIComponent(path)}`,
          {method: 'POST', headers: {'X-Mapping-Client': '1'}, body: file, signal: uploadController.signal});
        if (!response.ok) { const data = await response.json(); throw new Error(errors[data.error] || data.error); }
        done += file.size;
        $('import-progress').value = total ? done / total * 100 : 100;
        $('import-status').textContent = `이 PC에 자료 복사 중 · ${Math.round(done / 1048576)} / ${Math.round(total / 1048576)}MB`;
      }
      await request('inspect', {}, '?id=' + job.id);
      poll(job.id);
    } catch (error) {
      if (active) await request('cancel', {}, '?id=' + active).catch(() => {});
      working = false;
      fail(error);
    }
  }
  async function refresh() {
    try {
      const data = await request('library');
      $('import-library').replaceChildren(new Option('저장 기록 선택', ''), ...data.items.map(job =>
        new Option(`${job.name} · ${job.status === 'local' ? '이 PC의 기록' : importMessage(job)}`, job.local_id || job.id)));
      const recent = data.items.slice(0, 6);
      $('saved-library-empty').hidden = recent.length > 0;
      $('saved-library-empty').textContent = '아직 복원 기록이 없습니다. 촬영 영상을 선택해 첫 지도를 만들어 보세요.';
      $('saved-library').replaceChildren(...recent.map(job => {
        const card = document.createElement('div'); card.className = 'saved-record-card';
        const button = document.createElement('button'); button.type = 'button'; button.className = 'saved-record';
        const title = document.createElement('strong'), info = document.createElement('span'), date = document.createElement('span');
        const image = document.createElement('img'); image.loading = 'lazy'; image.alt = '';
        const mapReady = ['local', 'ready', 'complete'].includes(job.status);
        image.hidden = !mapReady;
        if (mapReady) image.src = `/api/import/thumbnail?local_id=${encodeURIComponent(job.local_id || job.id)}`;
        image.addEventListener('error', () => { image.hidden = true; });
        title.textContent = job.summary?.name || job.name;
        info.textContent = mapReady ? job.summary ? `${(job.summary.points ?? 0).toLocaleString()}점 · 촬영 시점 ${job.summary.keyframes ?? 0}개 · 지도 열기 ↗` : '이 PC의 저장 기록 · 지도 열기 ↗' : `${importMessage(job)} · 작업 확인 ↗`;
        date.textContent = `${job.updated_at ? '최근 수정 ' + new Date(job.updated_at * 1000).toLocaleString('ko-KR') + ' · ' : ''}${!mapReady ? importMessage(job) : job.summary?.warnings?.some(text => text.includes('부분 복원')) ? '부분 복원' : job.status === 'local' ? '저장 기록' : '결과 준비됨'}`;
        button.append(image, title, date, info); button.addEventListener('click', () => openRecord(job.local_id || job.id));
        const remove = document.createElement('button'); remove.type = 'button'; remove.className = 'saved-record-delete';
        remove.setAttribute('aria-label', `${job.summary?.name || job.name} 복원 기록 삭제`); remove.title = '복원 기록 삭제';
        remove.innerHTML = '<svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" stroke-width="1.8" aria-hidden="true"><path d="M3 6h18M9 6V3h6v3M5 6l1 15h12l1-15M10 10v7M14 10v7"/></svg>';
        remove.disabled = !['local', 'ready', 'complete', 'failed', 'cancelled'].includes(job.status);
        remove.addEventListener('click', async event => {
          event.stopPropagation(); if (working || remove.disabled) return;
          remove.disabled = true;
          try {
            await request('delete', {local_id: job.local_id || job.id});
            if (active === job.id || active === job.local_id) reset();
            await refresh();
          } catch (error) { remove.disabled = false; fail(error); }
        });
        card.append(button, remove); return card;
      }));
      if (data.initial && !initialHandled) {
        initialHandled = true;
        if (!active && !asset) { active = data.initial; $('import-details').open = true; poll(active); }
      }
    } catch (error) { $('import-status').textContent = error.message; }
  }
  $('import-refresh').addEventListener('click', refresh);
  async function openRecord(id) {
    if (working || !id) return;
    clearPreview(); creating = true; openingRecord = true; working = true; $('import-details').open = true;
    try { const job = await request('local', {local_id: id}); render(job); if (!navigatingToResult) poll(job.id); }
    catch (error) { working = false; fail(error); }
  }
  $('import-local').addEventListener('click', () => openRecord($('import-library').value));
  for (const id of ['import-file', 'import-folder', 'saved-archive', 'saved-result-folder']) $(id).addEventListener('change', event => {
    upload([...event.target.files].map(file => ({file, path: file.webkitRelativePath || file.name}))); event.target.value = '';
  });
  $('import-drop').addEventListener('keydown', event => {
    if (event.target === $('import-drop') && ['Enter', ' '].includes(event.key)) { event.preventDefault(); $('import-file').click(); }
  });
  $('saved-preview').addEventListener('error', () => { $('saved-preview-note').textContent = '브라우저에서 이 영상을 미리 재생할 수 없습니다. 서버의 파일 검사 결과를 확인하세요.'; });
  $('import-drop').addEventListener('dragover', event => { event.preventDefault(); $('import-drop').classList.add('drag-over'); });
  $('import-drop').addEventListener('dragleave', () => $('import-drop').classList.remove('drag-over'));
  $('import-drop').addEventListener('drop', async event => {
    event.preventDefault(); $('import-drop').classList.remove('drag-over');
    try { const items = [...event.dataTransfer.items]; await upload(await droppedFiles(items)); } catch (error) { fail(error); }
  });
  $('import-cancel').addEventListener('click', async () => {
    uploadController?.abort();
    try { if (active) { render(await request('cancel', {}, '?id=' + active)); poll(active); } }
    catch (error) { fail(error); }
  });
  $('import-confirm').addEventListener('change', () => { $('import-start').disabled = !$('import-confirm').checked || working; });
  $('import-start').addEventListener('click', async () => {
    if (working) return;
    $('saved-preview').pause(); $('import-video').pause(); $('saved-preview').currentTime = 0;
    $('saved-preview-wrap').hidden = true;
    working = true; $('import-start').disabled = true;
    try {
      let camera = null;
      const file = $('import-camera').files[0];
      if (file) {
        if (file.size > 65536) throw new Error('보정 JSON은 64KB 이하여야 합니다.');
        camera = JSON.parse(await file.text());
      }
      const profile = await profiles?.selected('file');
      const job = await request('start', {confirmed: $('import-confirm').checked,
        mode: $('import-mode').value || 'depth_rgbd',
        sample_fps: Number($('import-sample-fps').value), frame_fps: Number($('import-frame-fps').value), camera,
        ...(profile ? {profile_selection: profile.profile_selection} : {})}, '?id=' + active);
      working = true; render(job); poll(job.id);
    } catch (error) { working = false; $('import-start').disabled = !$('import-confirm').checked; fail(error); }
  });
  for (const id of ['saved-preview', 'import-video']) $(id).addEventListener('play', () => {
    if (['running', 'cancelling'].includes(lastJob?.status)) $(id).pause();
  });
  function visibility() {
    $('import-panel').hidden = location.hash === '#live';
    $('import-selected').hidden = location.hash === '#live' || !asset;
    if (location.hash === '#live') { $('import-video').pause(); $('saved-preview').pause(); $('saved-show-frame').click(); }
  }
  window.addEventListener('hashchange', visibility); visibility(); refresh();
  if (asset) request('status', undefined, '?id=' + encodeURIComponent(asset)).then(job => {
    lastJob = job;
    $('saved-replay').hidden = !job.history_available;
    const summary = job.summary;
    if (!summary) return;
    $('import-selected').hidden = location.hash === '#live';
    $('import-selected-name').textContent = `${summary.name} · 불러온 저장 기록`;
    $('import-selected-warnings').textContent = (summary.warnings || []).join(' ');
    const media = [...(summary.media || [])];
    if (job.source_video) media.unshift({index: 'source', label: '복원에 사용한 원본 영상', browser_playable: true});
    $('import-media').hidden = media.length === 0;
    $('import-media-note').hidden = media.length === 0;
    $('saved-show-video').hidden = media.length === 0;
    const showMedia = video => {
      $('saved-original').hidden = !video; document.querySelector('.capture').hidden = video;
      $('saved-show-frame').setAttribute('aria-pressed', String(!video));
      $('saved-show-video').setAttribute('aria-pressed', String(video));
      if (!video) $('import-video').pause();
    };
    $('saved-show-frame').addEventListener('click', () => showMedia(false));
    $('saved-show-video').addEventListener('click', () => showMedia(true));
    $('import-media').replaceChildren(...media.map(m => new Option(m.label, m.index)));
    const show = () => {
      const part = media.find(part => String(part.index) === $('import-media').value);
      if (!part) return;
      const url = part.index === 'source' ? `/api/import/source-video?id=${encodeURIComponent(asset)}` :
        `/api/import/media?id=${encodeURIComponent(asset)}&part=${part.index}`;
      $('import-video').hidden = !part.browser_playable;
      $('import-video').src = part.browser_playable ? url : '';
      $('import-media-download').hidden = false;
      $('import-media-download').href = url;
    };
    $('import-media').addEventListener('change', show); show();
    $('import-video').addEventListener('error', () => {
      $('import-media-note').textContent = '브라우저에서 이 영상을 재생할 수 없습니다. 녹화 파일을 내려받아 확인하세요. 지도와 자동 동기화하지 않습니다.';
    });
  }).catch(fail);
  return {navigateStep(index) {
    const value = ['input', 'settings', 'processing', 'result'][index];
    if (!value) return false;
    if (index === 1 && (!lastJob || !['awaiting_options', 'running', 'cancelling', 'ready', 'complete'].includes(lastJob.status))) return false;
    if (index === 2 && !lastJob) return false;
    if (index === 3 && !['ready', 'complete'].includes(lastJob?.status) && !viewingCurrent) return false;
    if (lastJob) render(lastJob, false);
    navigationPhase = value;
    phase(value);
    applyNavigation(value);
    $('import-panel').scrollIntoView({behavior: 'smooth', block: 'start'});
    return true;
  }};
}
