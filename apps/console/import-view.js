const stages = {upload: '자료 복사 중', inspect: '자료 검사 중', confirm: '입력 확인 대기', input: '프레임 준비 중',
  features: '특징점 추출·매칭 중', reconstruction: '3D 지도 복원 중', preview: '미리보기 생성 중', finalization: '결과 검증 중', finished: '복원 완료'};
const errors = {
  import_busy: '진행 중인 불러오기가 있습니다. 완료하거나 취소한 뒤 다시 선택하세요.',
  reconstruction_busy: '다른 복원이 실행 중입니다. 완료 뒤 다시 시작하세요.',
  stop_live_mapping_before_reconstruction: '실시간 매핑을 종료한 뒤 새 지도 복원을 시작하세요.',
  need_3_to_10000_frames: 'JPG·PNG 프레임을 3~10,000장 선택하세요.',
  frame_dimensions_differ: '프레임 해상도가 서로 다릅니다. 같은 카메라·해상도의 이미지를 선택하세요.',
  invalid_relative_path: '폴더 안에 지원하지 않는 파일 경로가 있습니다.',
  insufficient_disk_space: '저장 공간이 부족합니다.',
  unsupported_input_use_session_zip_video_or_frames: '세션 ZIP·결과 폴더·동영상·JPG/PNG 폴더를 선택하세요.',
  external_import_requires_reselection: '외부 폴더를 CMD에 다시 드롭하거나 웹에서 폴더를 다시 선택하세요.',
  calibration_dimensions_must_match_input: '보정 JSON의 해상도가 현재 영상·프레임과 다릅니다. 같은 크기·방향·크롭의 보정 파일을 선택하세요.',
  confirm_input_order_and_camera: '카메라·해상도 또는 프레임 순서·FPS 확인란을 체크하세요.',
};
export function importMessage(job) {
  if (job.status === 'complete') return '복원 완료';
  if (job.status === 'failed') return `불러오기 실패: ${errors[job.error] || job.error}`;
  if (job.status === 'cancelled') return '취소했습니다. 원본 자료와 이미 생성된 진단은 보존합니다.';
  if (job.status === 'ready') return '저장 자료를 확인했습니다. 지도를 열 수 있습니다.';
  return stages[job.stage] || job.status;
}

export function openUrl(id) { return `/?asset=${encodeURIComponent(id)}#saved`; }

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

export function connectImport({profiles = null} = {}) {
  const $ = id => document.getElementById(id);
  let active = null, polling = null, uploadController = null, working = false;
  const asset = new URLSearchParams(location.search).get('asset');
  async function request(action, body, query = '') {
    const response = await fetch(`/api/import/${action}${query}`, body === undefined ? {cache: 'no-store'} :
      {method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok) throw new Error(errors[value.error] || value.error);
    return value;
  }
  function fail(error) { $('import-status').textContent = error.name === 'AbortError' ? '전송을 취소했습니다.' : error.message; $('import-progress').hidden = true; working = false; }
  function render(job) {
    if (active !== job.id) { $('import-confirm').checked = false; $('import-camera').value = ''; profiles?.resetFileConfirmation(); }
    active = job.id;
    $('import-status').textContent = importMessage(job);
    $('import-summary').hidden = false;
    const summary = job.summary;
    $('import-options').hidden = job.status !== 'awaiting_options';
    $('import-start').hidden = job.status !== 'awaiting_options';
    $('import-open').hidden = !['ready', 'complete'].includes(job.status);
    $('import-open').href = openUrl(job.id);
    $('import-cancel').hidden = ['ready', 'complete', 'failed', 'cancelled'].includes(job.status);
    $('import-progress').hidden = !['uploading', 'inspecting', 'running', 'cancelling'].includes(job.status);
    if (job.status !== 'uploading') $('import-progress').removeAttribute('value');
    if (summary) {
      $('import-info').textContent = ['session', 'result'].includes(summary.kind) ?
        `${summary.name} · ${summary.points.toLocaleString()}점 · 키프레임 ${summary.keyframes}장 · 지도 기록 ${summary.revisions}개` :
        `${summary.name} · ${summary.width}×${summary.height} · ${summary.frames ?? '미확인'}프레임${summary.order ? ` · 순서: ${summary.order.join(' → ')}${summary.frames > summary.order.length ? ' → … → ' + summary.last_frame : ''}` : ''}`;
      $('import-warnings').replaceChildren(...(summary.warnings || []).map(text => { const item = document.createElement('li'); item.textContent = text; return item; }));
      $('import-sampling').hidden = summary.kind !== 'video';
      $('import-frame-timing').hidden = summary.kind !== 'frames';
    }
    if (['ready', 'complete', 'awaiting_options', 'failed', 'cancelled'].includes(job.status)) working = false;
  }
  async function poll(id) {
    clearTimeout(polling);
    try {
      const job = await request('status', undefined, '?id=' + encodeURIComponent(id));
      if (active !== id) return;
      render(job);
      if (['uploading', 'inspecting', 'running', 'cancelling'].includes(job.status)) polling = setTimeout(() => poll(id), 750);
      else refresh();
    } catch (error) { fail(error); }
  }
  async function upload(files) {
    if (working) { fail(new Error('먼저 진행 중인 작업을 취소하거나 마쳐 주세요.')); return; }
    if (!files.length) return;
    working = true;
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
      fail(error);
    }
  }
  async function refresh() {
    try {
      const data = await request('library');
      $('import-library').replaceChildren(new Option('저장 기록 선택', ''), ...data.items.map(job =>
        new Option(`${job.name} · ${job.status === 'local' ? '이 PC의 기록' : importMessage(job)}`, job.local_id || job.id)));
      if (data.initial && !active && !asset) { active = data.initial; $('import-details').open = true; poll(active); }
    } catch (error) { $('import-status').textContent = error.message; }
  }
  $('import-refresh').addEventListener('click', refresh);
  $('import-local').addEventListener('click', async () => {
    if (working || !$('import-library').value) return;
    try { const job = await request('local', {local_id: $('import-library').value}); render(job); working = true; poll(job.id); }
    catch (error) { fail(error); }
  });
  for (const id of ['import-file', 'import-folder']) $(id).addEventListener('change', event => {
    upload([...event.target.files].map(file => ({file, path: file.webkitRelativePath || file.name}))); event.target.value = '';
  });
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
  $('import-start').addEventListener('click', async () => {
    try {
      let camera = null;
      const file = $('import-camera').files[0];
      if (file) {
        if (file.size > 65536) throw new Error('보정 JSON은 64KB 이하여야 합니다.');
        camera = JSON.parse(await file.text());
      }
      const profile = await profiles?.selected('file');
      const job = await request('start', {confirmed: $('import-confirm').checked,
        sample_fps: Number($('import-sample-fps').value), frame_fps: Number($('import-frame-fps').value), camera,
        ...(profile ? {profile_selection: profile.profile_selection} : {})}, '?id=' + active);
      working = true; render(job); poll(job.id);
    } catch (error) { fail(error); }
  });
  function visibility() {
    $('import-panel').hidden = location.hash === '#live';
    if (location.hash === '#live') $('import-video').pause();
  }
  window.addEventListener('hashchange', visibility); visibility(); refresh();
  if (asset) request('status', undefined, '?id=' + encodeURIComponent(asset)).then(job => {
    const summary = job.summary;
    if (!summary) return;
    $('import-selected').hidden = false;
    $('import-selected-name').textContent = `${summary.name} · 불러온 저장 기록`;
    $('import-selected-warnings').textContent = (summary.warnings || []).join(' ');
    const media = summary.media || [];
    $('import-media').hidden = media.length === 0;
    $('import-media-note').hidden = media.length === 0;
    $('import-media').replaceChildren(...media.map(m => new Option(m.label, m.index)));
    const show = () => {
      const part = media[Number($('import-media').value)];
      if (!part) return;
      const url = `/api/import/media?id=${encodeURIComponent(asset)}&part=${part.index}`;
      $('import-video').hidden = !part.browser_playable;
      $('import-video').src = part.browser_playable ? url : '';
      $('import-media-download').hidden = false;
      $('import-media-download').href = url;
    };
    $('import-media').addEventListener('change', show); show();
  }).catch(fail);
}
