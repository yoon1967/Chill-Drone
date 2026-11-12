import {requestImport} from './import-api.js';
import {mappingOptions} from './mapping-settings.js';
import {editorCloud} from './editor-data.js';
import {connectCaptureLibrary} from './capture-library.js';
import {showModelPicker, selectedModels, setModelPickerBusy, modelPickerMessage, refreshModelChecks,
  closeModelPicker} from './reconstruction-models.js';

const $ = id => document.getElementById(id);
const main = document.querySelector('main');
const workspaceTabs = [$('restore-tab'), $('edit-tab')];
const workspace = $('workspace-panels');
const storageKey = 'mapping-video-reconstruction';
const activeStates = new Set(['uploading', 'inspecting', 'running', 'cancelling']);
const messages = {
  insufficient_disk_space: 'PC의 저장 공간을 확보한 뒤 다시 시도하세요.',
  import_too_large: '더 짧은 영상으로 시도하세요.',
  import_busy: '진행 중인 영상 작업이 끝난 뒤 다시 시도하세요.',
  video_required: 'MP4, MOV, AVI, MKV 중 하나를 선택하세요.',
  no_video_stream: '영상 트랙이 있는 파일을 선택하세요.',
  empty_video: '재생할 수 있는 장면이 있는 영상을 선택하세요.',
  no_valid_depth_map: '카메라가 공간 안에서 이동하며 촬영한 영상을 선택하세요.',
  reconstruction_busy: '진행 중인 복원이 끝난 뒤 다시 시도하세요.',
  stop_live_mapping_before_reconstruction: '실시간 지도 생성을 종료한 뒤 다시 시도하세요.',
  slam_docker_unavailable: 'Docker Desktop을 실행한 뒤 다시 시도하세요.',
  slam_docker_timeout: 'Docker Desktop이 준비된 뒤 다시 시도하세요.',
  slam_image_missing: '카메라 위치 모델을 준비한 뒤 다시 시도하세요.',
  pose_gpu_unavailable: '이 위치 모델에는 NVIDIA GPU와 드라이버가 필요합니다.',
  pose_worker_start_failed: '카메라 위치 모델을 시작하지 못했습니다. Docker와 NVIDIA GPU 상태를 확인하세요.',
  pose_worker_stopped: '카메라 위치 추적기가 중단되었습니다. Docker와 NVIDIA GPU 상태를 확인하세요.',
  pose_worker_timeout: '카메라 위치 모델의 응답 시간이 초과되었습니다.',
  'ValueError: pose_frame_limit': '이 위치 모델이 처리할 수 있는 영상 길이를 초과했습니다.',
  reconstruction_models_not_ready: '선택한 모델의 준비 상태를 확인하세요.',
  depth_mapping_dependencies_missing: '3D 매핑 실행기를 다시 열어 복원 환경을 준비하세요.',
  drone_recording_unavailable: '녹화 영상과 센서 기록이 모두 있는지 확인하세요.',
  drone_recording_missing_video: '재생 가능한 녹화 파일이 없습니다.',
  drone_recording_missing_logs: '녹화에 대응하는 프레임 또는 센서 기록이 없습니다.',
  drone_recording_invalid_timing: '영상 프레임을 녹화 기록의 시각과 맞출 수 없습니다.',
  drone_recording_no_sensor_pose: '녹화 영상과 같은 시각의 드론 자세를 찾을 수 없습니다.',
};

let id = null, job = null, state = 'loading', timer, upload, map;
let busy = false, watchVersion = 0, canRenderPreview = true;
let libraryVersion = 0;
let slideVersion = 0, drag;
let editLibraryVersion = 0, editRecords = [], editLoad, pointEditor;
let editChoice = 0;
const captures = connectCaptureLibrary({request: requestImport,
  onSelect: bundle => { setName(bundle.name); status('capture_selected'); }, onClear: () => reset()});

function remember(next) {
  id = next;
  try {
    if (next) sessionStorage.setItem(storageKey, next);
    else sessionStorage.removeItem(storageKey);
  } catch { /* A fresh selection still works with storage disabled. */ }
}

function status(next, label = '', notice = '') {
  state = next;
  const result = next === 'complete' || next === 'partial';
  const preview = next === 'running' || (next === 'cancelling' && job?.options?.mode === 'depth_rgbd');
  main.dataset.state = next;
  if ($('status').textContent !== label) $('status').textContent = label;
  if ($('notice').textContent !== notice) $('notice').textContent = notice;
  $('notice').hidden = !notice;
  $('status-line').hidden = !label;
  $('drop').hidden = next !== 'input';
  $('drone-recordings').hidden = !['input', 'capture_selected'].includes(next);
  $('model-picker').hidden = next !== 'awaiting_options';
  $('input-libraries').hidden = !['input', 'capture_selected'].includes(next);
  captures.setEnabled(['input', 'capture_selected'].includes(next));
  if (next !== 'capture_selected') $('capture-selection').hidden = true;
  $('change').hidden = !['capture_selected', 'awaiting_options', 'complete', 'partial', 'failed', 'cancelled', 'result-error'].includes(next);
  $('cancel').hidden = !activeStates.has(next);
  $('cancel').disabled = next === 'cancelling' || (busy && !upload);
  $('change').disabled = busy;
  $('retry').hidden = !['result-error', 'connection-error'].includes(next);
  $('edit-result').hidden = !result;
  $('progress').hidden = !activeStates.has(next) && next !== 'loading';
  $('progress-value').hidden = $('progress').hidden;
  $('download').hidden = !['complete', 'partial', 'result-error'].includes(next);
  $('visual').hidden = !preview && !result;
  $('visual').classList.toggle('result', result);
  $('current-frame').hidden = !preview;
  $('map-controls').hidden = !result;
}

function setName(name) {
  $('file-name').textContent = name || '';
  $('file-name').hidden = !name;
}

function disposeMap() { map?.dispose(); map = null; }

async function openMap() {
  if (!map) {
    const {connectProgressiveMap} = await import('./progressive-map.js');
    map = connectProgressiveMap({compact: true});
  }
  return map;
}

const query = () => '?id=' + encodeURIComponent(id);

function updateProgress(next) {
  const {stage, progress = {}, summary = {}} = next;
  const fraction = (done, total) => total > 0 ? Math.min(1, Math.max(0, (done || 0) / total)) : 0;
  let value;
  if (next.status === 'complete' || next.status === 'ready') value = 100;
  else if (stage === 'saving' || stage === 'finished') value = 99;
  else if (stage === 'integration') value = 90 + 9 * fraction(progress.integrated_frames, progress.integration_total);
  else value = 90 * (summary.duration_s > 0 ? fraction(progress.completed_timestamp_s, summary.duration_s) :
    fraction(progress.completed_frame_number, summary.frames));
  const meter = $('progress');
  meter.value = Math.max(meter.value, Math.floor(value));
  $('progress-value').textContent = `${meter.value}%`;
}

async function refreshRecent() {
  const version = ++libraryVersion;
  try {
    const {items} = await requestImport('reconstruction-library');
    if (version !== libraryVersion) return;
    const records = items.filter(item => ['local', 'ready', 'complete'].includes(item.status) ||
      ((item.source_video || item.summary?.kind === 'drone_recording') &&
        ['inspecting', 'running', 'cancelling'].includes(item.status))).slice(0, 6);
    $('recent-empty').textContent = records.length ? '' : '복원 기록이 없습니다.';
    $('recent-list').replaceChildren(...records.map(record => {
      const item = document.createElement('li'), button = document.createElement('button');
      button.type = 'button'; button.className = 'recent-record';
      const title = document.createElement('strong'), detail = document.createElement('span');
      title.textContent = record.name || record.summary?.name;
      detail.textContent = activeStates.has(record.status) ? '복원 중' : record.updated_at ?
        new Date(record.updated_at * 1000).toLocaleDateString('ko-KR') : '지도 열기';
      button.append(title, detail);
      button.addEventListener('click', () => openRecord(record.local_id || record.id));
      item.append(button);
      return item;
    }));
  } catch {
    if (version !== libraryVersion) return;
    $('recent-empty').textContent = '복원 기록을 열 수 없습니다.';
  }
}

function markEditedMap() {
  for (const button of $('edit-list').querySelectorAll('button'))
    button.setAttribute('aria-pressed', String(button.dataset.mapKey === pointEditor?.key));
}

async function refreshEditableMaps() {
  const version = ++editLibraryVersion;
  const label = $('edit-empty');
  try {
    const {items} = await requestImport('editable-maps');
    if (version !== editLibraryVersion) return;
    editRecords = items;
    $('edit-list').replaceChildren(...items.map(record => {
      const item = document.createElement('li'), button = document.createElement('button');
      button.type = 'button';
      button.className = 'edit-record';
      button.dataset.mapKey = record.key;
      const name = document.createElement('strong');
      name.textContent = record.name;
      button.append(name);
      if (record.updated_at) {
        const date = document.createElement('span');
        date.textContent = new Date(record.updated_at * 1000).toLocaleDateString('ko-KR');
        button.append(date);
      }
      button.addEventListener('click', () => openEditableMap(record));
      item.append(button);
      return item;
    }));
    markEditedMap();
    label.hidden = items.length > 0;
    label.textContent = '편집할 수 있는 저장 지도가 없습니다.';
  } catch {
    if (version !== editLibraryVersion) return;
    label.hidden = false;
    label.textContent = '저장된 지도 목록을 열 수 없습니다.';
  }
}

async function resolveEditAsset(record, signal) {
  if (!record.local_id) return record.asset;
  let opened = await requestImport('local', {local_id: record.local_id}, '', signal);
  const query = '?' + new URLSearchParams({id: opened.id});
  while (opened.status === 'inspecting') {
    await new Promise(resolve => setTimeout(resolve, 250));
    opened = await requestImport('status', undefined, query, signal);
  }
  if (!['ready', 'complete'].includes(opened.status)) throw new Error('저장된 지도에서 점군을 확인할 수 없습니다.');
  return opened.id;
}

async function openEditableMap(record) {
  editChoice++;
  if (editLoad?.key === record.key) return;
  editLoad?.controller.abort();
  const label = $('edit-load-status');
  label.hidden = record.key === pointEditor?.key;
  if (label.hidden) { editLoad = null; return; }
  const controller = new AbortController();
  editLoad = {key: record.key, controller};
  const signal = AbortSignal.any([controller.signal, AbortSignal.timeout(60000)]);
  label.textContent = `${record.name} 점군을 불러오는 중입니다.`;
  try {
    const {connectPointCloudEditor} = await import('./point-cloud-editor.js');
    signal.throwIfAborted();
    pointEditor ??= connectPointCloudEditor();
    let cloud;
    if (!pointEditor.hasDraft(record.key)) {
      const asset = await resolveEditAsset(record, signal);
      const query = asset ? '?' + new URLSearchParams({asset}) : '';
      const response = await fetch('/api/saved/run' + query, {cache: 'no-store', signal});
      if (!response.ok) throw new Error('저장된 점군을 읽을 수 없습니다.');
      cloud = editorCloud(await response.json());
    }
    signal.throwIfAborted();
    pointEditor.open(record, cloud);
    label.hidden = true;
  } catch (error) {
    if (!controller.signal.aborted) label.textContent = error.name === 'TimeoutError'
      ? '지도 불러오기 시간이 초과되었습니다. 지도를 다시 선택하세요.'
      : `${record.name} · ${error.message || '지도를 불러오지 못했습니다.'}`;
  } finally {
    if (!controller.signal.aborted) markEditedMap();
    if (editLoad?.controller === controller) editLoad = null;
  }
}

async function refreshDroneRecordings() {
  const label = $('drone-recordings-status');
  const list = $('drone-recordings-list');
  label.textContent = '드론 녹화 목록 확인 중';
  list.replaceChildren();
  try {
    const {items} = await requestImport('drone-recordings');
    label.textContent = items.length ? '' : '영상과 센서값이 함께 저장된 드론 녹화가 없습니다.';
    list.replaceChildren(...items.map(record => {
      const item = document.createElement('li'), button = document.createElement('button');
      button.type = 'button'; button.className = 'recent-record';
      const title = document.createElement('strong'), detail = document.createElement('span');
      title.textContent = record.name;
      detail.textContent = record.started_at ? new Date(record.started_at).toLocaleString('ko-KR') : '센서 기반 복원';
      button.append(title, detail);
      button.addEventListener('click', () => openRecord(record.name, true));
      item.append(button);
      return item;
    }));
  } catch { label.textContent = '드론 녹화 목록을 열 수 없습니다.'; }
}

async function openRecord(key, drone = false) {
  if (busy || !['input', 'capture_selected'].includes(state)) return;
  captures.clear();
  busy = true;
  status('loading', drone ? '드론 녹화 확인 중' : '복원 기록 확인 중');
  try {
    const record = drone ? await requestImport('drone-recording', {session_id: key}) :
      await requestImport('local', {local_id: key});
    remember(record.id);
  } catch (error) {
    status('input', drone ? '드론 녹화를 열 수 없습니다.' : '복원 기록을 열 수 없습니다.', messages[error.cause] || '');
  } finally { busy = false; }
  if (id) await watch();
}

async function showResult() {
  const asset = '?asset=' + encodeURIComponent(id);
  $('download').href = job.history_available ? `/api/import/history.zip${query()}` : `/api/saved/map.zip${asset}`;
  status('loading', '결과 불러오는 중');
  try {
    const response = await fetch(`/api/saved/run${asset}`, {cache: 'no-store', signal: AbortSignal.timeout(15000)});
    if (!response.ok) throw new Error();
    const result = await response.json();
    const view = await openMap();
    const cloud = editorCloud(result);
    view.showResult({...cloud, rescue: result.rescue});
    const partial = result.quality?.result_status === 'partial';
    const counts = result.quality?.coverage;
    const segments = counts?.map_count ?? 1;
    const coverage = segments > 1 ? `${segments}개 구간으로 나뉘어 가장 많은 장면을 포함한 구간을 표시합니다.` :
      counts?.unmapped_frames > 0 ? `${counts.recorded_frames}개 중 ${counts.integrated_frames}개 장면을 지도에 반영했습니다.` :
      partial ? '영상의 일부 장면만 지도에 반영됐습니다.' : '';
    const held = counts?.held_pose_frames ? `${counts.held_pose_frames}개 장면은 마지막으로 확인한 위치에 합쳤습니다.` : '';
    const missing = counts?.unavailable_recording_parts ? `${counts.unavailable_recording_parts}개 녹화 구간은 파일 또는 시각 정보가 없어 복원하지 못했습니다.` : '';
    const scale = cloud.unit === 'arbitrary' ? '실제 거리 측정에 사용할 수 없습니다.' : '거리 정확도는 검증되지 않았습니다.';
    status(partial ? 'partial' : 'complete', partial ? '부분 복원' : '복원 완료', [coverage, held, missing, scale].filter(Boolean).join(' '));
    if (!$('edit-panel').inert) refreshEditableMaps();
  } catch {
    status('result-error', '지도를 표시할 수 없습니다.', '지도를 저장하거나 다시 시도하세요.');
  }
}

function phaseLabel({stage, progress}) {
  if (stage === 'pose_wait' && progress?.tracking === 'initializing') return '카메라의 초기 위치와 방향 찾는 중';
  if (stage === 'pose_wait' && progress?.tracking === 'scale_wait') return '다음 장면의 깊이 기준 찾는 중';
  if (stage === 'depth' && progress?.alignment === 'pose_dpvo') return 'DPVO로 이어 찾은 자세로 장면 깊이 계산 중';
  if (stage === 'mapping' && progress?.alignment === 'pose_dpvo') return 'DPVO로 이어 찾은 위치와 방향을 지도에 반영 중';
  if (progress?.pose_estimated && stage === 'depth') return '예측한 위치로 장면 깊이 계산 중';
  if (progress?.pose_estimated && stage === 'mapping') return '예측한 위치를 지도에 반영 중';
  if (stage === 'mapping' && progress?.alignment === 'pose_reused') return '이전 장면과 맞춘 위치를 지도에 반영 중';
  if (stage === 'mapping' && progress?.alignment === 'pose_held') return '마지막 위치를 기준으로 지도에 합치는 중';
  if (stage === 'mapping' && progress?.alignment === 'scale_reused') return '이전 깊이 기준으로 지도에 반영 중';
  return {
    inspect: '영상 형식과 촬영 정보 확인 중', preflight: '복원 환경과 설정 확인 중', input: '복원 시작 준비 중',
    model: '깊이 모델 준비 중', semantic_model: '사람·구조물 인식 모델 준비 중',
    pose_optimization: '최종 지도를 저장할 카메라 경로 보정 중',
    tracking_setup: '카메라 위치 추적기 준비 중', initial_pose: '카메라의 초기 위치와 방향 찾는 중',
    tracking: '카메라 위치와 방향 추적 중', recovering_pose: '카메라 위치와 방향 다시 찾는 중',
    sensor_pose: '녹화 장면과 드론 센서 위치를 맞추는 중',
    depth: '장면의 깊이 계산 중', semantics: '사람과 구조물 구분 중', depth_alignment: '깊이와 지도 크기 맞추는 중',
    visual_registration: '이전 장면을 기준으로 위치와 방향 맞추는 중',
    mapping: '장면을 3D 지도에 합치는 중', presentation: '장면과 지도 갱신 중',
    pose_wait: '카메라 위치 다시 찾는 중', integration: '촬영 구간의 지도 통합 중', saving: '최종 지도 저장 중',
  }[stage] || '복원 상태 확인 중';
}

async function display(next) {
  job = next;
  setName(next.name);
  updateProgress(next);
  switch (next.status) {
    case 'awaiting_options':
      status('awaiting_options');
      showModelPicker(next.summary?.kind === 'drone_recording');
      break;
    case 'running':
    case 'cancelling':
      status(next.status, next.status === 'cancelling' ? '취소 중' : phaseLabel(next),
        canRenderPreview ? '' : '미리보기를 표시할 수 없습니다. 복원은 진행 중입니다.');
      if (canRenderPreview && (next.status === 'running' || map)) {
        try { (await openMap()).update(next); }
        catch { canRenderPreview = false; }
      }
      break;
    case 'complete':
    case 'ready':
      await showResult();
      break;
    case 'failed':
      disposeMap();
      status('failed', '복원 실패', messages[next.error] || '영상이 재생되는지 확인하거나 다른 영상을 선택하세요.');
      break;
    case 'cancelled':
      disposeMap();
      status('cancelled', '취소됨');
      break;
    case 'inspecting':
      status('inspecting', phaseLabel(next));
      break;
    default:
      status('failed', '영상을 다시 선택하세요.');
  }
}

async function watch() {
  clearTimeout(timer);
  if (!id || busy) return;
  const version = ++watchVersion;
  try {
    const next = await requestImport('status', undefined, query());
    if (version !== watchVersion) return;
    await display(next);
    if (version === watchVersion && activeStates.has(state)) timer = setTimeout(watch, 750);
  } catch (error) {
    if (version !== watchVersion) return;
    if ([400, 404].includes(error.httpStatus)) {
      reset();
      status('input', '이전 복원 작업을 열 수 없습니다.');
      return;
    }
    if (activeStates.has(state)) {
      status(state, '연결 확인 중');
      timer = setTimeout(watch, 1500);
    } else status('connection-error', '복원 상태를 불러올 수 없습니다.');
  }
}

async function select(file) {
  if (busy || state !== 'input' || !file) return;
  if (!/\.(mp4|mov|avi|mkv)$/i.test(file.name)) {
    status('input', '지원하는 영상 형식을 선택하세요.');
    return;
  }
  busy = true;
  upload = new AbortController();
  setName(file.name);
  status('uploading', '영상 불러오는 중');
  let error;
  try {
    ({id} = await requestImport('create', {name: file.name}));
    const response = await fetch(`/api/import/upload${query()}&path=${encodeURIComponent(file.name)}`, {
      method: 'POST', headers: {'X-Mapping-Client': '1'}, body: file, signal: upload.signal,
    });
    if (!response.ok) throw new Error('upload_failed', {cause: (await response.json()).error});
    upload.signal.throwIfAborted();
    upload = null;
    status('inspecting', '영상 형식과 촬영 정보 확인 중');
    job = await requestImport('inspect', {}, query());
    remember(id);
  } catch (failure) {
    error = failure;
    if (id) await requestImport('cancel', {}, query()).catch(() => {});
    remember(null);
  } finally {
    upload = null;
    busy = false;
  }
  if (error) status(error.name === 'AbortError' ? 'cancelled' : 'failed', error.name === 'AbortError' ? '취소됨' : '영상을 불러올 수 없습니다.',
    messages[error.cause] || '');
  else await watch();
}

function reset() {
  captures.clear();
  watchVersion++;
  clearTimeout(timer);
  disposeMap();
  closeModelPicker({clear: true});
  modelPickerMessage('');
  remember(null);
  job = null;
  canRenderPreview = true;
  $('progress').value = 0;
  $('progress-value').textContent = '0%';
  setName('');
  status('input');
  refreshRecent();
  refreshDroneRecordings();
}

function selectWorkspaceTab(tab) {
  if (tab.getAttribute('aria-selected') === 'true') return;
  const outgoing = workspace.querySelector('[role="tabpanel"]:not([inert])');
  workspace.style.minHeight = `${workspace.offsetHeight}px`;
  tab.focus({preventScroll: true});
  for (const item of workspaceTabs) {
    const selected = item === tab;
    item.setAttribute('aria-selected', String(selected));
    item.tabIndex = selected ? 0 : -1;
    const panel = $(item.getAttribute('aria-controls'));
    panel.inert = !selected;
    panel.setAttribute('aria-hidden', String(!selected));
  }
  const version = ++slideVersion;
  const releaseHeight = () => {
    if (version === slideVersion) workspace.style.minHeight = '';
  };
  const slide = outgoing.getAnimations().find(animation => animation.transitionProperty === 'transform');
  if (slide) slide.finished.then(releaseHeight, releaseHeight);
  else releaseHeight();
  if (tab.id === 'edit-tab') return refreshEditableMaps();
}

workspaceTabs.forEach((tab, index) => {
  tab.addEventListener('click', () => selectWorkspaceTab(tab));
  tab.addEventListener('keydown', event => {
    const next = {
      ArrowLeft: (index + workspaceTabs.length - 1) % workspaceTabs.length,
      ArrowRight: (index + 1) % workspaceTabs.length,
      Home: 0,
      End: workspaceTabs.length - 1,
    }[event.key];
    if (next === undefined) return;
    event.preventDefault();
    selectWorkspaceTab(workspaceTabs[next]);
  });
});

$('edit-refresh').addEventListener('click', refreshEditableMaps);
$('edit-result').addEventListener('click', async () => {
  const asset = id;
  const name = job?.name || '복원 지도';
  const choice = editChoice;
  await selectWorkspaceTab($('edit-tab'));
  if (!$('edit-panel').inert && choice === editChoice)
    openEditableMap(editRecords.find(record => record.asset === asset) || {key: asset, asset, name});
});

workspace.addEventListener('pointerdown', event => {
  if (event.pointerType !== 'mouse' || event.button !== 0 ||
      event.target.closest('a, button, input, select, textarea, label, summary, .library-list, .media, [contenteditable]')) return;
  drag = {id: event.pointerId, x: event.clientX, y: event.clientY, offset: 0};
});

window.addEventListener('pointermove', event => {
  if (event.pointerId !== drag?.id) return;
  const dx = event.clientX - drag.x, dy = event.clientY - drag.y;
  const direction = $('restore-panel').inert ? 1 : -1;
  if (!workspace.classList.contains('dragging')) {
    if (Math.abs(dy) > 8 && Math.abs(dy) >= Math.abs(dx)) { drag = null; return; }
    if (dx * direction < 8) return;
    workspace.setPointerCapture(event.pointerId);
    workspace.classList.add('dragging');
    window.getSelection()?.removeAllRanges();
  }
  drag.offset = direction * Math.max(0, Math.min(dx * direction, workspace.clientWidth));
  workspace.style.setProperty('--drag-offset', `${drag.offset}px`);
  event.preventDefault();
});

function endWorkspaceDrag(event) {
  if (event.pointerId !== drag?.id) return;
  const offset = drag.offset;
  drag = null;
  if (!workspace.classList.contains('dragging')) return;
  // Commit the pointer position before enabling the settling transition.
  workspace.getBoundingClientRect();
  workspace.classList.remove('dragging');
  workspace.style.removeProperty('--drag-offset');
  if (event.type === 'pointerup' && Math.abs(offset) >= Math.min(96, workspace.clientWidth * .2))
    selectWorkspaceTab(workspaceTabs[offset < 0 ? 1 : 0]);
}

window.addEventListener('pointerup', endWorkspaceDrag);
window.addEventListener('pointercancel', endWorkspaceDrag);
workspace.addEventListener('lostpointercapture', endWorkspaceDrag);

if (window.parent !== window) {
  let parentOrigin;
  const reportHeight = () => {
    const height = main.offsetHeight;
    if (parentOrigin && height) window.parent.postMessage({type: 'mapping-console-height', height}, parentOrigin);
  };
  new ResizeObserver(reportHeight).observe(main);
  window.addEventListener('message', event => {
    const {type, height} = event.data || {};
    if (event.source !== window.parent || type !== 'mapping-console-viewport' || !Number.isSafeInteger(height) || height <= 0) return;
    parentOrigin = event.origin;
    document.documentElement.classList.add('embedded');
    main.style.setProperty('--workspace-height', `${height}px`);
    reportHeight();
  });
}

$('start-reconstruction').addEventListener('click', async () => {
  const models = selectedModels();
  if (!models || busy || !id || state !== 'awaiting_options') return;
  busy = true;
  setModelPickerBusy(true);
  modelPickerMessage('복원 시작 확인 중');
  status('awaiting_options');
  let started = false;
  let checkJob = false;
  try {
    await requestImport('start', {...mappingOptions({includeSampling: false}), ...models,
      confirmed: true, mode: 'depth_rgbd', causal_video: Boolean(models.pose_model)}, query());
    started = true;
    closeModelPicker();
    status('running', '복원 준비 중');
  } catch (error) {
    modelPickerMessage(messages[error.cause] || '복원을 시작할 수 없습니다. 다시 시도하세요.');
    refreshModelChecks();
    checkJob = !error.httpStatus;
  } finally {
    busy = false;
    setModelPickerBusy(false);
    $('change').disabled = false;
  }
  if (started || checkJob) await watch();
});

$('choose').addEventListener('click', () => $('video').click());
$('video').addEventListener('change', event => {
  const file = event.target.files[0];
  event.target.value = '';
  select(file);
});
$('drop').addEventListener('dragover', event => { event.preventDefault(); $('drop').classList.add('dragging'); });
$('drop').addEventListener('dragleave', event => {
  if (!$('drop').contains(event.relatedTarget)) $('drop').classList.remove('dragging');
});
$('drop').addEventListener('drop', event => {
  event.preventDefault();
  $('drop').classList.remove('dragging');
  const files = event.dataTransfer.files;
  if (files.length === 1) select(files[0]);
  else status('input', '영상 한 개를 선택하세요.');
});
$('cancel').addEventListener('click', async () => {
  if (upload) {
    upload.abort();
    status('cancelling', '취소 중');
    return;
  }
  if (busy || !id || !activeStates.has(state)) return;
  clearTimeout(timer);
  watchVersion++;
  busy = true;
  status('cancelling', '취소 중');
  try { await requestImport('cancel', {}, query()); }
  catch { /* Read the current state before offering cancellation again. */ }
  finally { busy = false; }
  await watch();
});
$('change').addEventListener('click', async () => {
  if (busy) return;
  if (id && state === 'awaiting_options') {
    busy = true;
    $('change').disabled = true;
    setModelPickerBusy(true);
    try { await requestImport('cancel', {}, query()); }
    catch { /* The draft can be reopened through its saved job ID. */ }
    finally { busy = false; setModelPickerBusy(false); }
  }
  reset();
  $('choose').focus();
});
$('retry').addEventListener('click', () => {
  if (state === 'result-error') showResult();
  else watch();
});
window.addEventListener('pagehide', () => { captures.setEnabled(false); watchVersion++; clearTimeout(timer); upload?.abort(); disposeMap(); closeModelPicker(); });
window.addEventListener('pageshow', event => {
  if (event.persisted && ['input', 'capture_selected'].includes(state)) captures.setEnabled(true);
  if (event.persisted && id) {
    if (state === 'awaiting_options') refreshModelChecks();
    watch();
  }
});
document.addEventListener('visibilitychange', () => { if (!document.hidden && activeStates.has(state) && !busy) watch(); });

id = new URLSearchParams(location.search).get('asset');
try { id ||= sessionStorage.getItem(storageKey); } catch { /* Storage is optional. */ }
if (id) { status('loading', '복원 상태 확인 중'); watch(); }
else reset();
