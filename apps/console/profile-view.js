const messages = {
  confirm_camera_profile_binding: '대상 카메라·영상 방향·크롭 확인란을 체크하세요.',
  calibration_dimensions_must_match_input: '프로필 해상도가 원본 입력과 다릅니다. 같은 해상도·방향·크롭의 프로필을 선택하세요.',
  camera_profile_input_mode_mismatch: '이 프로필은 선택한 입력 종류에 사용할 수 없습니다.',
  camera_profile_changed_refresh_selection: '프로필 정보가 바뀌었습니다. 목록을 새로고침해 다시 선택하세요.',
  invalid_profile_name: '프로필 이름을 1~80자로 입력하세요.',
  invalid_profile_camera_identity: '대상 기체·카메라 정보를 입력하세요.',
  invalid_profile_image_geometry: '영상 방향과 크롭 정보를 입력하세요.',
};
export function profileError(error) { return messages[error] || error; }
export function profileDescription(row) {
  if (!row) return '프로필을 선택하지 않았습니다. 기존 JSON 직접 선택과 입력 기본값을 사용합니다.';
  const camera = row.camera, summary = camera.calibration_summary || {};
  const created = row.created_at ? new Date(row.created_at) : null;
  return `${row.name} · ${row.camera_identity} · ${row.width}×${row.height} · ${row.image_geometry} · 버전 ${row.version}` +
    (created && Number.isFinite(created.getTime()) ? ` · 등록 ${created.toLocaleDateString('ko-KR')}` : '') +
    ` · ${row.readonly ? '기존 실측 기본값' : '가져온 파일 · 값 검사 완료'}` +
    (Number.isFinite(summary.views) ? ` · 파일 기록: ${summary.views}장` : '') +
    (Number.isFinite(summary.validation_rms_px) ? ` · 검증 RMS ${summary.validation_rms_px.toFixed(3)}px` : '') +
    (row.readonly ? '' : ' · 측정 정확도는 별도로 확인해야 합니다.');
}
export function selectionFor(row, confirmed, width, height) {
  if (!row) return null;
  if (!confirmed) throw new Error(profileError('confirm_camera_profile_binding'));
  if (width !== undefined && (row.width !== width || row.height !== height))
    throw new Error(profileError('calibration_dimensions_must_match_input'));
  return {id: row.id, version: row.version, camera_sha256: row.camera_sha256, confirmed: true};
}

export function connectProfiles() {
  const $ = id => document.getElementById(id);
  let data = {profiles: [], defaults: {}}, liveMode = 'camera', pendingCamera = null, locked = false;
  const originalImportNote = $('import-calibration-note').textContent;
  let readiness;
  async function request(path = '', body) {
    const response = await fetch('/api/camera-profiles' + path, {signal: AbortSignal.timeout(5000), ...(body === undefined ? {cache: 'no-store'} :
      {method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'}, body: JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw new Error(profileError(value.error));
    return value;
  }
  const row = id => data.profiles.find(p => p.id === id);
  function describe(context) {
    const value = row($('profile-select-' + context).value);
    $('profile-info-' + context).textContent = profileDescription(value);
    $('profile-confirm-label-' + context).hidden = !value;
    $('profile-confirm-' + context).checked = false;
    if (context === 'file') $('import-calibration-note').textContent = value ?
      `${value.name} 프로필을 사용합니다. 원본 해상도가 같아야 하며 대상 카메라·방향·크롭을 확인하세요.` : originalImportNote;
  }
  function populate(context, mode, preserve = true) {
    const select = $('profile-select-' + context);
    const selected = preserve ? select.value : null;
    select.replaceChildren(new Option(context === 'live' ? '기존 입력 설정·보정 JSON 직접 선택' : '보정 JSON 직접 선택 또는 자체 추정', ''),
      ...data.profiles.filter(p => p.input_modes.includes(mode)).map(p => new Option(`${p.name} · ${p.width}×${p.height}`, p.id)));
    const desired = selected !== null ? selected : data.defaults[mode] || '';
    select.value = row(desired)?.input_modes.includes(mode) ? desired : '';
    describe(context);
  }
  function libraryInfo() {
    const value = row($('profile-library').value);
    $('profile-library-info').textContent = profileDescription(value);
    $('profile-export').hidden = !value;
    if (value) $('profile-export').href = '/api/camera-profiles/export?id=' + encodeURIComponent(value.id);
  }
  async function refresh(initial = false) {
    data = await request();
    const selected = $('profile-library').value;
    $('profile-library').replaceChildren(new Option('기본 선택 없음', ''), ...data.profiles.map(p => new Option(`${p.name} · ${p.width}×${p.height}`, p.id)));
    $('profile-library').value = row(selected)?.id || data.profiles[0]?.id || '';
    libraryInfo();
    populate('live', liveMode, !initial); populate('file', 'file', !initial);
    $('profile-status').textContent = data.warnings.length ? data.warnings.join(' ') : `${data.profiles.length}개 프로필 · 이 PC에 저장 · 기존 결과는 변경하지 않습니다.`;
    return data;
  }
  function fail(error) { $('profile-status').textContent = `보정 프로필: ${error.message}`; }
  async function readFile(file) {
    if (!file) return;
    if (!file.name.toLowerCase().endsWith('.json') || file.size > 65536) throw new Error('64KB 이하 보정 JSON 파일을 선택하세요.');
    const camera = JSON.parse(await file.text());
    if (!camera || typeof camera !== 'object' || Array.isArray(camera) || !['width', 'height', 'fx', 'fy', 'cx', 'cy'].every(k => Number.isFinite(camera[k])))
      throw new Error('카메라 보정 숫자가 있는 JSON인지 확인하세요.');
    pendingCamera = camera;
    const profile = camera.profile || {};
    $('profile-register').hidden = false;
    $('profile-file-info').textContent = `${file.name} · ${camera.width}×${camera.height} · fx ${camera.fx} · fy ${camera.fy} · 서버 검사 후 새 프로필로 저장합니다.`;
    $('profile-name').value = profile.name || file.name.replace(/\.json$/i, '');
    $('profile-identity').value = profile.camera_identity || camera.camera_identity || '';
    $('profile-geometry').value = profile.image_geometry || '';
    $('profile-register-mode').value = profile.input_modes?.includes('tello') ? 'tello' : profile.input_modes?.includes('camera') ? 'camera' : 'file';
    $('profile-register-confirm').checked = false;
    $('profile-manager').open = true;
  }
  $('profile-file').addEventListener('change', event => readFile(event.target.files[0]).catch(fail));
  $('profile-drop').addEventListener('dragover', event => { event.preventDefault(); $('profile-drop').classList.add('drag-over'); });
  $('profile-drop').addEventListener('dragleave', () => $('profile-drop').classList.remove('drag-over'));
  $('profile-drop').addEventListener('drop', event => {
    event.preventDefault(); $('profile-drop').classList.remove('drag-over');
    if (event.dataTransfer.files.length !== 1) { fail(new Error('보정 JSON 한 파일을 놓으세요.')); return; }
    readFile(event.dataTransfer.files[0]).catch(fail);
  });
  $('profile-library').addEventListener('change', libraryInfo);
  $('profile-refresh').addEventListener('click', () => refresh().catch(fail));
  $('profile-default').addEventListener('click', async () => {
    try {
      const mode = $('profile-default-mode').value;
      data = await request('/default', {mode, id: $('profile-library').value || null});
      if (mode === 'file') populate('file', 'file', false);
      else if (!locked && mode === liveMode) populate('live', liveMode, false);
      $('profile-status').textContent = '기본 선택을 저장했습니다. 다음 입력 선택·재실행부터 사용합니다. 실행 중인 지도는 바꾸지 않습니다.';
    } catch (error) { fail(error); }
  });
  $('profile-save').addEventListener('click', async () => {
    $('profile-save').disabled = true;
    try {
      if (!pendingCamera) throw new Error('보정 JSON을 먼저 선택하세요.');
      const mode = $('profile-register-mode').value;
      const camera = await request('', {camera: pendingCamera, name: $('profile-name').value,
        camera_identity: $('profile-identity').value, image_geometry: $('profile-geometry').value,
        input_modes: mode === 'file' ? ['file'] : [mode, 'file'], confirmed: $('profile-register-confirm').checked});
      await refresh();
      $('profile-library').value = camera.profile.id; libraryInfo();
      $('profile-status').textContent = '프로필을 저장했습니다. 사용할 입력에서 선택하거나 JSON을 내보내세요.';
    } catch (error) { fail(error); }
    finally { $('profile-save').disabled = false; }
  });
  for (const context of ['live', 'file']) {
    $('profile-select-' + context).addEventListener('change', () => {
      describe(context);
      const input = $(context === 'live' ? 'camera-calibration' : 'import-camera');
      input.value = '';
    });
    $(context === 'live' ? 'camera-calibration' : 'import-camera').addEventListener('change', () => {
      $('profile-select-' + context).value = ''; describe(context);
    });
  }
  $('camera-device').addEventListener('change', () => { $('profile-confirm-live').checked = false; });
  readiness = refresh(true).catch(fail);
  return {
    ready: readiness,
    syncLiveKind(kind) {
      const next = kind === 'tello-map' ? 'tello' : 'camera';
      $('profile-live-choice').hidden = !['camera', 'tello-map'].includes(kind);
      if (next !== liveMode) { liveMode = next; populate('live', liveMode, false); }
    },
    lockLive(value) { locked = value; $('profile-select-live').disabled = locked; $('profile-confirm-live').disabled = locked; },
    async selected(context, width, height) {
      await readiness;
      const value = row($('profile-select-' + context).value);
      const selection = selectionFor(value, $('profile-confirm-' + context).checked, width, height);
      return selection ? {camera: value.camera, profile_selection: selection} : null;
    },
    resetFileConfirmation() { $('profile-confirm-file').checked = false; },
  };
}
