const messages = {
  confirm_camera_profile_binding: '대상 카메라·영상 방향·크롭 확인란을 체크하세요.',
  calibration_dimensions_must_match_input: '프로필 해상도가 원본 입력과 다릅니다. 같은 해상도·방향·크롭의 프로필을 선택하세요.',
  camera_profile_input_mode_mismatch: '이 프로필은 선택한 입력 종류에 사용할 수 없습니다.',
  camera_profile_changed_refresh_selection: '프로필 정보가 바뀌었습니다. 목록을 새로고침해 다시 선택하세요.',
  invalid_profile_name: '프로필 이름을 1~80자로 입력하세요.',
  invalid_profile_camera_identity: '상세 설정에서 대상 기체·카메라 정보를 입력하세요.',
  invalid_profile_image_geometry: '상세 설정에서 영상 방향과 크롭 정보를 입력하세요.',
};
const modeLabels = {tello: '드론 영상 수신 + 파일 복원', camera: '브라우저 카메라 + 파일 복원', file: '파일 복원만'};
export function profileError(error) { return messages[error] || error; }
export function profileDescription(row) {
  if (!row) return '프로필을 선택하지 않았습니다. 기존 JSON 직접 선택과 입력 기본값을 사용합니다.';
  const summary = row.camera.calibration_summary || {}, created = row.created_at ? new Date(row.created_at) : null;
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
function cameraDetails(camera) {
  const number = value => Number.isFinite(value) ? value.toFixed(3) : '미지정';
  return `fx ${number(camera.fx)} · fy ${number(camera.fy)} · cx ${number(camera.cx)} · cy ${number(camera.cy)}` +
    ` · 왜곡 ${Array.isArray(camera.distortion) ? camera.distortion.map(v => number(v)).join(', ') : '서버 검사 시 확인'}`;
}

export function connectProfiles() {
  const $ = id => document.getElementById(id);
  let data = {profiles: [], defaults: {}}, liveMode = 'camera', currentContext = 'live';
  let pendingCamera = null, locked = false, preferenceBusy = false, preferenceVersion = 0, autoIdentity = false;
  const originalImportNote = $('import-calibration-note').textContent;
  const row = id => data.profiles.find(p => p.id === id);
  const modeFor = context => context === 'file' ? 'file' : liveMode;
  const chosen = context => row($('profile-select-' + context).value);
  function notify(text) { $('profile-status').textContent = text; $('profile-status').hidden = !text; }
  async function request(path = '', body) {
    const response = await fetch('/api/camera-profiles' + path, {signal: AbortSignal.timeout(5000), ...(body === undefined ? {cache: 'no-store'} :
      {method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'}, body: JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw new Error(profileError(value.error));
    return value;
  }
  function defaultState() {
    for (const context of ['live', 'file']) {
      const value = chosen(context), checkbox = $('profile-remember-' + context);
      checkbox.checked = Boolean(value && data.defaults[modeFor(context)] === value.id);
      checkbox.disabled = !value || preferenceBusy || (context === 'live' && locked);
    }
  }
  function describe(context) {
    const value = chosen(context);
    $('profile-info-' + context).textContent = value ? `${value.width}×${value.height} · ${value.camera_identity}` :
      context === 'file' ? '보정 파일을 직접 지정하거나 카메라 값을 자체 추정합니다.' :
      $('camera-calibration').files?.length ? '직접 고른 보정 JSON을 이번 실행에 사용합니다.' : '기존 입력 설정을 사용합니다. 보정 JSON 직접 적용은 아래 직접 설정에서 선택할 수 있습니다.';
    $('profile-detail-' + context).hidden = !value;
    $('profile-detail-' + context).open = false;
    $('profile-values-' + context).textContent = value ? profileDescription(value) + ' · ' + cameraDetails(value.camera) : '';
    $('profile-confirm-label-' + context).hidden = !value;
    $('profile-confirm-' + context).checked = false;
    if (context === 'file') $('import-calibration-note').textContent = value ?
      `${value.name} 프로필을 사용합니다. 원본 해상도가 같아야 하며 대상 카메라·방향·크롭을 확인하세요.` : originalImportNote;
    defaultState();
  }
  function populate(context, preserve = true) {
    const select = $('profile-select-' + context), mode = modeFor(context), selected = preserve ? select.value : null;
    select.replaceChildren(new Option(context === 'live' ? '기존 입력 설정 사용' : '보정 프로필 없이 복원', ''),
      ...data.profiles.filter(p => p.input_modes.includes(mode)).map(p => new Option(`${p.name} · ${p.width}×${p.height}`, p.id)));
    const desired = selected !== null ? selected : data.defaults[mode] || '';
    select.value = row(desired)?.input_modes.includes(mode) ? desired : '';
    describe(context);
  }
  function libraryInfo() {
    const value = row($('profile-library').value), mode = modeFor(currentContext);
    $('profile-library-info').textContent = value ? profileDescription(value) + ' · ' + cameraDetails(value.camera) : '관리할 프로필을 선택하세요.';
    $('profile-export').hidden = !value;
    if (value) $('profile-export').href = '/api/camera-profiles/export?id=' + encodeURIComponent(value.id);
    $('profile-use').disabled = !value || !value.input_modes.includes(mode) || (currentContext === 'live' && locked);
    $('profile-use-note').textContent = currentContext === 'live' && locked ? '수신을 종료한 후 실행 프로필을 바꿀 수 있습니다.' :
      value && !value.input_modes.includes(mode) ? '이 프로필은 현재 입력 경로에 사용할 수 없습니다. 정보 확인·내보내기만 가능합니다.' :
      '관리 목록의 선택만으로 적용되지 않습니다. 현재 입력에 선택하면 대상 확인 후 다음 시작에 사용합니다.';
  }
  function showTools(open) {
    $('profile-tools').hidden = !open;
    for (const context of ['live', 'file']) $('profile-manage-' + context).setAttribute('aria-expanded', String(open && context === currentContext));
    if (open) {
      showImport(false);
      const value = chosen(currentContext);
      if (value) $('profile-library').value = value.id;
      libraryInfo();
    }
  }
  function showImport(open) {
    $('profile-import').hidden = !open;
    $('profile-import-open').setAttribute('aria-expanded', String(open));
    if (open) { $('profile-manager').open = true; showTools(false); }
  }
  async function refresh(initial = false) {
    const before = preferenceVersion, fresh = await request();
    if (before !== preferenceVersion || preferenceBusy) fresh.defaults = data.defaults;
    data = fresh;
    const selected = $('profile-library').value;
    $('profile-library').replaceChildren(new Option('프로필 선택', ''), ...data.profiles.map(p => new Option(`${p.name} · ${p.width}×${p.height}`, p.id)));
    $('profile-library').value = row(selected)?.id || data.profiles[0]?.id || '';
    populate('live', !initial); populate('file', !initial); libraryInfo();
    notify(data.warnings.join(' '));
    return data;
  }
  function fail(error) { notify(`보정 프로필: ${error.message}`); }
  function registrationSummary() {
    if (!pendingCamera) return;
    $('profile-registration-summary').textContent = `${$('profile-identity').value || '이름으로 카메라 식별'} · ` +
      `${$('profile-geometry').value} · 사용 범위: ${modeLabels[$('profile-register-mode').value] || '미지정'}. 상세 설정에서 변경할 수 있습니다.`;
  }
  async function readFile(file) {
    if (!file) return;
    pendingCamera = null;
    $('profile-register').hidden = true;
    if (!file.name.toLowerCase().endsWith('.json') || file.size > 65536) throw new Error('64KB 이하 보정 JSON 파일을 선택하세요.');
    const camera = JSON.parse(await file.text());
    if (!camera || typeof camera !== 'object' || Array.isArray(camera) || !['width', 'height', 'fx', 'fy', 'cx', 'cy'].every(k => Number.isFinite(camera[k])))
      throw new Error('카메라 보정 숫자가 있는 JSON인지 확인하세요.');
    pendingCamera = camera;
    const profile = camera.profile || {};
    showImport(true);
    $('profile-register').hidden = false;
    $('profile-advanced').open = $('profile-file-values').open = false;
    $('profile-file-info').textContent = `${file.name} · ${camera.width}×${camera.height}`;
    $('profile-file-details').textContent = cameraDetails(camera);
    $('profile-name').value = profile.name || file.name.replace(/\.json$/i, '');
    autoIdentity = !(profile.camera_identity || camera.camera_identity);
    $('profile-identity').value = profile.camera_identity || camera.camera_identity || $('profile-name').value;
    $('profile-geometry').value = profile.image_geometry || `${camera.width}×${camera.height} · 가져온 보정 파일의 방향·크롭 기준`;
    $('profile-register-mode').value = profile.input_modes?.includes('tello') ? 'tello' : profile.input_modes?.includes('camera') ? 'camera' : 'file';
    $('profile-register-confirm').checked = false;
    registrationSummary();
  }
  $('profile-import-open').addEventListener('click', () => showImport($('profile-import').hidden));
  $('profile-import-close').addEventListener('click', () => showImport(false));
  $('profile-tools-close').addEventListener('click', () => showTools(false));
  $('profile-file').addEventListener('change', event => readFile(event.target.files[0]).catch(fail));
  $('profile-drop').addEventListener('dragover', event => { event.preventDefault(); $('profile-drop').classList.add('drag-over'); });
  $('profile-drop').addEventListener('dragleave', () => $('profile-drop').classList.remove('drag-over'));
  $('profile-drop').addEventListener('drop', event => {
    event.preventDefault(); $('profile-drop').classList.remove('drag-over');
    if (event.dataTransfer.files.length !== 1) { fail(new Error('보정 JSON 한 파일을 놓으세요.')); return; }
    readFile(event.dataTransfer.files[0]).catch(fail);
  });
  $('profile-name').addEventListener('input', () => {
    if (autoIdentity) $('profile-identity').value = $('profile-name').value;
    registrationSummary(); $('profile-register-confirm').checked = false;
  });
  for (const id of ['profile-identity', 'profile-geometry', 'profile-register-mode']) $(id).addEventListener('change', () => {
    if (id === 'profile-identity') autoIdentity = false;
    registrationSummary(); $('profile-register-confirm').checked = false;
  });
  $('profile-library').addEventListener('change', libraryInfo);
  $('profile-refresh').addEventListener('click', () => refresh().catch(fail));
  $('profile-use').addEventListener('click', () => {
    const value = row($('profile-library').value), context = currentContext;
    if (!value || !value.input_modes.includes(modeFor(context)) || (context === 'live' && locked)) return;
    $('profile-select-' + context).value = value.id;
    $(context === 'live' ? 'camera-calibration' : 'import-camera').value = '';
    describe(context); showTools(false); $('profile-select-' + context).focus();
  });
  for (const context of ['live', 'file']) {
    $('profile-manage-' + context).addEventListener('click', () => { currentContext = context; showTools($('profile-tools').hidden); });
    $('profile-remember-' + context).addEventListener('change', async () => {
      const value = chosen(context), mode = modeFor(context), checked = $('profile-remember-' + context).checked;
      if (!value || preferenceBusy || (context === 'live' && locked)) { defaultState(); return; }
      preferenceBusy = true; preferenceVersion++; defaultState();
      try {
        const response = await request('/default', {mode, id: checked ? value.id : null});
        data.defaults = response.defaults;
        notify(checked ? '현재 선택을 이 입력의 기본 프로필로 기억했습니다.' : '이 입력의 기본 프로필 기억을 해제했습니다.');
      } catch (error) { fail(error); }
      finally { preferenceBusy = false; preferenceVersion++; defaultState(); }
    });
    $('profile-select-' + context).addEventListener('change', () => {
      $(context === 'live' ? 'camera-calibration' : 'import-camera').value = '';
      describe(context); libraryInfo();
    });
    $(context === 'live' ? 'camera-calibration' : 'import-camera').addEventListener('change', () => {
      $('profile-select-' + context).value = ''; describe(context);
    });
  }
  $('profile-save').addEventListener('click', async () => {
    $('profile-save').disabled = true;
    try {
      if (!pendingCamera) throw new Error('보정 JSON을 먼저 선택하세요.');
      const submitted = pendingCamera;
      const mode = $('profile-register-mode').value;
      const camera = await request('', {camera: pendingCamera, name: $('profile-name').value,
        camera_identity: $('profile-identity').value, image_geometry: $('profile-geometry').value,
        input_modes: mode === 'file' ? ['file'] : [mode, 'file'], confirmed: $('profile-register-confirm').checked});
      await refresh();
      $('profile-library').value = camera.profile.id; libraryInfo();
      if (pendingCamera === submitted) {
        showImport(false); pendingCamera = null; $('profile-file').value = ''; $('profile-register').hidden = true;
        $('profile-select-' + currentContext).focus();
      }
      notify('프로필을 저장했습니다. 위에서 사용할 보정 프로필을 선택한 뒤 대상 카메라를 확인하세요.');
    } catch (error) {
      if (/상세 설정|invalid_profile_input_modes/.test(error.message)) $('profile-advanced').open = true;
      fail(error);
    }
    finally { $('profile-save').disabled = false; }
  });
  $('camera-device').addEventListener('change', () => { $('profile-confirm-live').checked = false; });
  const readiness = refresh(true).catch(fail);
  return {
    ready: readiness,
    setContext(source) {
      currentContext = source === 'saved' ? 'file' : 'live';
      defaultState(); libraryInfo();
      for (const context of ['live', 'file']) $('profile-manage-' + context).setAttribute('aria-expanded', String(!$('profile-tools').hidden && context === currentContext));
    },
    async prepareCamera(camera, {mode = 'camera', name = '체크보드 보정'} = {}) {
      await readFile(new File([JSON.stringify(camera)], 'calibrated-camera.json', {type: 'application/json'}));
      $('profile-name').value = name;
      if (autoIdentity) $('profile-identity').value = name;
      $('profile-geometry').value = `${camera.width}×${camera.height} 원본 · 촬영 방향·크롭 확인 필요`;
      $('profile-register-mode').value = mode;
      registrationSummary(); $('profile-name').focus();
    },
    syncLiveKind(kind) {
      const next = kind === 'tello-map' ? 'tello' : 'camera';
      $('profile-live-choice').hidden = !['camera', 'tello-map'].includes(kind);
      if (next !== liveMode) { liveMode = next; populate('live', false); }
      defaultState(); libraryInfo();
    },
    lockLive(value) {
      locked = value; $('profile-select-live').disabled = locked; $('profile-confirm-live').disabled = locked;
      defaultState(); libraryInfo();
    },
    async selected(context, width, height) {
      await readiness;
      const value = chosen(context), selection = selectionFor(value, $('profile-confirm-' + context).checked, width, height);
      return selection ? {camera: value.camera, profile_selection: selection} : null;
    },
    resetFileConfirmation() { $('profile-confirm-file').checked = false; },
  };
}
