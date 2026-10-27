import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { PlanarObjects } from '/objects.js';
import { connectCapture } from '/capture.js';
import { MapSource } from '/map-source.js';
import { updatePointGrid, pointSpacingProfile, pointGridCells, displayPointCells } from '/point-grid.js';
import { applySceneDelta } from '/map-delta.js';
const $ = (id) => document.getElementById(id);
const number = (n) => Number(n).toLocaleString('ko-KR');
const timeOf = (c) => c.source_pts_s ?? c.timestamp_s;
let run, root, cloud, route, frustums, grid, selected;
let center, radius = 1, fullCenter, fullRadius = 1, direction;
let currentIndex = 0, playing = false, previousTime = 0,
  mode = new URLSearchParams(location.search).get('mode') === 'hologram' ? 'hologram' : 'objects';
let navigating = false, dragging = false, busy = null, revision = null;
let pollDelay = 2000, previewFrame = null;
let pollTimer, captureState = {state: 'idle'}, captureHandle, lastLiveState = {};
const source = new MapSource(location.hash === '#live' ? 'live' : 'saved');
const views = new Map();
const savedCopy = {
  title: document.querySelector('.notes h2').innerHTML,
  detail: $('scale-note').nextElementSibling.textContent,
  note: document.querySelector('.notes .muted').textContent,
};
let hologramInterval = 0, intervalFrame = null, originalPositions = new Float64Array();
let hologramControlsOpen = mode === 'hologram';
let confidenceThreshold = 3, gridCells = null, filteredPoints = [];
let surfaceGroup = null, surfaceCache = null, surfaceStyle = 'faces', surfaceConnection = 2.5;
let surfaceTimer = null;
let surfaceWorker = null, surfaceInFlight = null, surfaceQueued = null, surfaceDesired = 0;
let surfaceAck = 0;
const surfacePatches = new Map();
let surfaceColorThreshold = 30, surfaceNormalAngle = 55;
let surfaceWide = true;
let spacingProfile = {step: .0001, max: .05, recommended: .003, median: 0};
const decimal = (n) => Number(n).toLocaleString('ko-KR', {maximumFractionDigits: 12});
const renderTimes = [];
let lastRenderReport = 0;
const replay = {session: null, entries: [], following: true, time: 0, lastTick: 0, generation: 0,
  loading: false, applied: null, manifestRevision: null, cache: new Map()};
const keys = new Set();
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x101d22);
$('viewport').replaceChildren(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(48, 1, .001, 10000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xe8f5ff, 0x68756c, 2.2));
const light = new THREE.DirectionalLight(0xffffff, 2);
light.position.set(3, 5, 6);
scene.add(light);
const objectLayer = new PlanarObjects(renderer, camera, controls, (frame, showCamera) => {
  if (!run) return;
  const index = run.cameras.findIndex((c) => c.frame_id === frame);
  if (index >= 0) selectFrame(index);
  if (showCamera) { stopExplore(); cameraView(); }
}, stopExplore);

function clearMap(title = '지도를 불러오는 중', detail = '잠시 기다려 주세요.') {
  stopExplore();
  clearSurfacePatches(); surfaceGroup = null; surfaceCache = null;
  clearTimeout(surfaceTimer); surfaceTimer = null;
  surfaceWorker?.terminate(); surfaceWorker = null;
  surfaceInFlight = surfaceQueued = null; surfaceDesired++; surfaceAck = 0;
  dispose();
  root = cloud = route = selected = frustums = grid = run = null;
  objectLayer.apply(null, null);
  $('viewport').dataset.source = source.source;
  $('viewport').dataset.session = '';
  $('viewport').dataset.revision = '0';
  Object.assign($('viewport').dataset, {visiblePoints: 0, surfaceStyle: 'points', surfaceGroups: 0,
    surfaceLines: 0, surfaceFaces: 0, surfaceWeakFaces: 0, surfacePointOnly: 0,
    surfaceWidePlanes: 0, surfaceWideFaces: 0, surfaceWideQuads: 0});
  for (const id of ['frame', 'slam-preview']) $(id).removeAttribute('src');
  for (const id of ['frames', 'points', 'reprojection', 'elapsed', 'timestamp', 'frame-name']) $(id).textContent = '—';
  for (const id of ['registration', 'run-date', 'render-state', 'mode-detail', 'frame-proof', 'stream-state', 'incremental-state']) $(id).textContent = '';
  for (const id of ['surface-summary', 'surface-update-summary', 'surface-wide-summary', 'hologram-confidence-summary']) $(id).textContent = '';
  $('scale-badge').textContent = source.source === 'live' ? '단안 임의 단위' : '스케일 확인 전';
  $('kind').textContent = '지도 대기';
  $('download').hidden = true;
  document.querySelectorAll('footer a').forEach((a) => { a.hidden = true; });
  document.querySelector('.object-panel').hidden = true;
  document.querySelector('.capture').hidden = true;
  document.querySelector('.notes').hidden = true;
  $('slam-panel').hidden = true;
  document.querySelectorAll('.view-buttons button, .mode-tabs button, #explore, #camera-view, #frame-timeline').forEach((b) => { b.disabled = true; });
  showEmpty(title, detail);
}

function showEmpty(title, detail) {
  $('map-empty').hidden = false;
  $('empty-title').textContent = title;
  $('empty-detail').textContent = detail;
}

function resetReplay() {
  playing = false;
  replay.generation++;
  Object.assign(replay, {session: null, entries: [], following: true, time: 0, loading: false, applied: null, manifestRevision: null});
  replay.cache.clear();
  $('map-playback').hidden = true;
}

function selectSource(next, force = false) {
  if (!force && next === source.source) return;
  if (run) views.set(source.source, {session: run.input_run_id, segment: run.segment_id, mode,
    frame: run.cameras[currentIndex]?.frame_id, position: camera.position.clone(), up: camera.up.clone(),
    target: controls.target.clone(), fov: camera.fov, view: document.querySelector('[data-view].active')?.dataset.view});
  source.select(next);
  clearTimeout(pollTimer);
  resetReplay();
  revision = previewFrame = null;
  renderTimes.length = 0;
  pollDelay = next === 'live' ? 500 : 2000;
  mode = views.get(next)?.mode || (new URLSearchParams(location.search).get('mode') === 'hologram' ? 'hologram' : next === 'saved' ? 'objects' : 'points');
  hologramControlsOpen = mode === 'hologram';
  clearMap();
  setMode(mode);
  $('error').hidden = true;
  document.querySelectorAll('[data-source]').forEach((tab) => {
    const selected = tab.dataset.source === next;
    tab.setAttribute('aria-selected', String(selected));
    tab.tabIndex = selected ? 0 : -1;
  });
  $('map-content').setAttribute('aria-labelledby', `tab-${next}`);
  $('live-panel').hidden = $('live-status').hidden = next !== 'live';
  document.querySelector('.workspace').classList.toggle('slam-workspace', next === 'live');
  document.querySelector('h1').textContent = next === 'saved' ? '영상 속 공간을, 입체로.' : '영상이 들어오는 지금, 공간으로.';
  $('run-name').textContent = next === 'saved' ? '저장된 실험 결과를 불러옵니다.' : '실시간 입력과 지도 생성 상태를 확인합니다.';
  document.querySelector('footer > span').textContent = next === 'saved' ? 'COLMAP · OFFLINE RECONSTRUCTION' : 'ORB-SLAM3 · MONOCULAR TRACKING';
  poll();
}

function liveStatus(state) {
  lastLiveState = state;
  captureHandle?.update(state);
  const local = captureState.state;
  const names = {idle: '입력 전', requesting: '권한 확인 중', connecting: '연결 중', receiving: '수신 중', stopping: '종료 중',
    stopped: '수신 종료', denied: '권한 거부', disconnected: '연결 중단', failed: '입력 실패', disabled: '새 입력 사용 안 함'};
  const locallyActive = ['requesting', 'connecting', 'receiving', 'stopping'].includes(local);
  const lifecycle = state.session_status || state.status;
  const recorded = !locallyActive && state.session_id;
  const useLocal = locallyActive || !recorded || ['denied', 'disconnected', 'failed'].includes(local) || (local === 'stopped' && !['complete', 'failed'].includes(lifecycle));
  $('input-state').textContent = useLocal ? names[local] || names.idle :
    ({starting: '연결 중', receiving: '수신 중', stopping: '종료 중', complete: '영상 종료 · 기록', failed: '수신 중단'}[lifecycle] || '저장된 세션');
  $('input-detail').textContent = useLocal ?
    captureState.message || '시작 버튼을 누르면 카메라 권한을 요청합니다.' : `처리 ${number(state.frames_received || 0)}장 · 마지막 입력 ${state.last_input_sequence ?? '기록 없음'}`;
  const trackingNames = {2: '추적 중', 3: '재위치 탐색', 4: '추적 상실'};
  $('tracking-state').textContent = !state.session_id ? '대기 중' : lifecycle === 'failed' ? '추적 중단' :
    lifecycle === 'complete' ? '추적 종료' : lifecycle === 'stopping' ? '추적 종료 중' : trackingNames[state.tracking_state] || '초기화 중';
  $('tracking-detail').textContent = state.session_id ? `정상 추적 ${number(state.tracked_frames || 0)}장 · 초기화 ${number(state.initializing_frames || 0)}장 · 추적 상실 ${number(state.lost_tracking_frames || 0)}장` : '아직 받은 영상이 없습니다.';
  const available = state.map_available;
  $('map-state').textContent = available ? !replay.following ? '기록 재생' : lifecycle === 'failed' ? '마지막 지도 유지' : lifecycle === 'complete' ? '최종 지도' : lifecycle === 'stopping' ? '지도 마무리 중' : '지도 갱신 중' : lifecycle === 'failed' ? '생성 실패' : lifecycle === 'complete' ? '생성 전 종료' : state.session_id ? '초기화 대기' : '생성 전';
  $('map-detail').textContent = available ? `최신 지도 ${state.revision}회 갱신 · ${run?.source === 'live' ? `${$('points').textContent}점 표시` : '점군 불러오는 중'}${!replay.following ? ` · 기록 버전 ${replay.applied ?? 0}` : ''} · ${state.scale?.unit === 'm' ? '미터 단위' : '임의 단위'}` :
    lifecycle === 'failed' ? '유효한 3D 지도가 생성되지 않았습니다. 입력과 카메라 보정값을 확인한 뒤 다시 시작하세요.' : lifecycle === 'complete' ? '영상 입력은 종료됐지만 유효한 지도를 초기화하지 못했습니다. 다음 시험에서는 같은 공간을 보며 카메라 위치를 옮겨 주세요.' : '카메라가 움직이며 충분한 시차와 관측점이 모이면 점이 표시됩니다.';
  $('session-detail').textContent = state.session_id ? `세션 ${state.session_id}${state.last_error ? ' · 처리 오류 발생' : ''}` : '현재 연결된 실시간 세션이 없습니다.';
}

function line(points, color, opacity = 1) {
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({color, transparent: opacity < 1, opacity}));
}
function rotation(c) {
  const a = c.world_from_camera_rotation;
  return new THREE.Matrix3().set(...a[0], ...a[1], ...a[2]);
}
function geometry(positions, colors) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions.flat(), 3));
  if (colors) {
    const linear = [];
    for (const rgb of colors) {
      const c = new THREE.Color().setRGB(...rgb.map((n) => n / 255), THREE.SRGBColorSpace);
      linear.push(c.r, c.g, c.b);
    }
    g.setAttribute('color', new THREE.Float32BufferAttribute(linear, 3));
  }
  return g;
}
function dispose() {
  if (!root) return;
  root.traverse((object) => {
    object.geometry?.dispose();
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.filter(Boolean).forEach((m) => { m.map?.dispose(); m.dispose(); });
  });
  scene.remove(root);
}
function showCloud() {
  if (!run) return;
  const points = run.points.filter((p) => p[7] <= Number($('error-limit').value));
  filteredPoints = points;
  gridCells = null;
  const colors = points.map((p) => {
    const colorMode = $('color-mode').value;
    if (colorMode === 'rgb') return p.slice(4, 7);
    const h = colorMode === 'error' ? Math.min(p[7] / 3, 1) :
      (p[2] - run.quality.bounds.min[1]) / Math.max(run.quality.bounds.max[1] - run.quality.bounds.min[1], .001);
    const c = new THREE.Color().setHSL((1 - h) * .58, .7, .6);
    return [c.r * 255, c.g * 255, c.b * 255];
  });
  cloud.geometry.dispose();
  const positions = points.map((p) => p.slice(1, 4));
  originalPositions = new Float64Array(positions.flat());
  cloud.geometry = geometry(positions, colors);
  cloud.geometry.getAttribute('position').setUsage(THREE.DynamicDrawUsage);
  applyPointInterval();
}
function configureSpacing() {
  spacingProfile = pointSpacingProfile(run.points);
  // A selected fine interval remains valid when replay advances to a coarser map.
  if (hologramInterval > 0) spacingProfile.step = Math.min(spacingProfile.step,
    10 ** Math.floor(Math.log10(hologramInterval)) / 10);
  for (const id of ['hologram-interval', 'hologram-interval-number']) {
    $(id).step = spacingProfile.step;
    $(id).max = Math.max(spacingProfile.max, hologramInterval);
    $(id).value = hologramInterval;
  }
  $('hologram-interval-profile').textContent = `조절 단위 ${decimal(spacingProfile.step)} · ` +
    (spacingProfile.median ? `최근접 거리 약 ${decimal(Number(spacingProfile.median.toPrecision(2)))} (표본 ${spacingProfile.samples}점)` : '점 사이 거리 추정 불가');
  $('hologram-interval-recommend').textContent = `권장 ${decimal(spacingProfile.recommended)} 적용`;
}
function updateIntervalLabel() {
  const unit = run?.scale.unit === 'm' ? 'm' : '임의 단위';
  const label = hologramInterval === 0 ? '원본 위치' : `${decimal(hologramInterval)} ${unit}`;
  $('hologram-interval-value').textContent = label;
  $('hologram-interval').setAttribute('aria-valuetext', label);
  $('hologram-interval-reset').disabled = hologramInterval === 0;
}
function applyPointInterval() {
  if (!cloud) return;
  const interval = mode === 'hologram' ? hologramInterval : 0;
  let count;
  if (mode === 'hologram') {
    gridCells ??= pointGridCells(filteredPoints, interval, run.point_observations, run.cameras.length);
    count = displayPointCells(cloud.geometry, gridCells, confidenceThreshold);
    const max = Math.max(gridCells.maxSupport, confidenceThreshold);
    for (const id of ['hologram-confidence', 'hologram-confidence-number']) {
      $(id).max = max; $(id).value = confidenceThreshold;
    }
    $('hologram-confidence-value').textContent = `최소 ${confidenceThreshold}프레임`;
    $('hologram-confidence').setAttribute('aria-valuetext', `최소 ${confidenceThreshold}프레임`);
    $('hologram-confidence-basis').textContent = gridCells.lowerBound ?
      '이전 자료: 프레임 연결 정보 없음 · 개별 트랙 관측 수의 하한 사용, 격자 합산 불가' :
      `${gridCells.basis === 'keyframes' ? '선택된 키프레임' : '등록된 프레임'} 기준 · 같은 프레임은 한 번만 계산`;
    $('hologram-confidence-summary').textContent = `${number(count)} / ${number(gridCells.cells.length)}곳 표시 · 원본 ${number(gridCells.sourceCount)}점` +
      (count ? '' : ' · 조건을 만족하는 점이 없습니다');
  } else {
    updatePointGrid(cloud.geometry, originalPositions, 0);
    count = filteredPoints.length;
  }
  $('points').textContent = number(root?.visible === false ? 0 : count);
  Object.assign($('viewport').dataset, {pointInterval: interval, confidenceThreshold: mode === 'hologram' ? confidenceThreshold : 1,
    visiblePoints: count, confidenceBasis: gridCells?.lowerBound ? 'track_lower_bound' : gridCells?.basis || ''});
  updateSurfacePreview();
  replayControls();
}
function clearSurfacePatches() {
  for (const patch of surfacePatches.values()) patch.group.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
  surfacePatches.clear();
  surfaceGroup?.clear();
}
function applySurfacePatch({key, kind, model}) {
  let patch = surfacePatches.get(key);
  if (!patch || kind === 'topology') {
    if (patch) {
      patch.group.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
      surfaceGroup.remove(patch.group);
    }
    const position = new THREE.Float32BufferAttribute(model.positions, 3).setUsage(THREE.DynamicDrawUsage);
    patch = {group: new THREE.Group(), position, layers: new Map()};
    const make = (name, type, color, opacity, capacity) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', position);
      g.setIndex(new THREE.BufferAttribute(new Uint32Array(capacity), 1).setUsage(THREE.DynamicDrawUsage));
      g.computeBoundingSphere();
      const material = type === 'face' ? new THREE.MeshBasicMaterial({color, side: THREE.DoubleSide, transparent: true,
        opacity, depthWrite: false, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1}) :
        type === 'point' ? new THREE.PointsMaterial({color, size: 3, sizeAttenuation: false, transparent: true, opacity, depthWrite: false}) :
        new THREE.LineBasicMaterial({color, transparent: true, opacity, depthWrite: false});
      const object = type === 'face' ? new THREE.Mesh(g, material) : type === 'point' ? new THREE.Points(g, material) : new THREE.LineSegments(g, material);
      object.userData.surfaceFace = type === 'face';
      patch.layers.set(name, object); patch.group.add(object);
    };
    make('strongTriangles', 'face', 0x62dfee, .18, model.triangles.length);
    make('weakTriangles', 'face', 0x62dfee, .07, model.triangles.length);
    make('strongEdges', 'line', 0x77e9f2, model.synthetic ? .25 : .55, model.edges.length);
    make('weakEdges', 'line', 0x77e9f2, model.synthetic ? .12 : .22, model.edges.length);
    make('strongBoundary', 'line', 0xb5faff, .85, model.boundary.length);
    make('weakBoundary', 'line', 0xb5faff, .42, model.boundary.length);
    make('strongVertices', 'point', 0xb5faff, .95, model.nodeKeys.length);
    make('weakVertices', 'point', 0xb5faff, .45, model.nodeKeys.length);
    surfacePatches.set(key, patch); surfaceGroup.add(patch.group);
  } else {
    const array = patch.position.array;
    let first = Infinity, last = -1;
    for (let i = 0; i < model.positions.length; i++) if (array[i] !== Math.fround(model.positions[i])) {
      array[i] = model.positions[i]; first = Math.min(first, i); last = i;
    }
    if (last >= 0) {
      patch.position.addUpdateRange(first, last - first + 1); patch.position.needsUpdate = true;
      patch.group.traverse(o => o.geometry?.computeBoundingSphere());
    }
  }
  const strong = new Set(model.strongEdges), weak = [...new Set(model.weakEdges)].filter(i => !strong.has(i));
  const indices = {...model, strongVertices: model.synthetic ? [] : [...strong], weakVertices: model.synthetic ? [] : weak};
  for (const [name, object] of patch.layers) {
    const values = indices[name], attr = object.geometry.getIndex();
    let changed = object.geometry.drawRange.count !== values.length;
    if (!changed) for (let i = 0; i < values.length; i++) if (attr.array[i] !== values[i]) { changed = true; break; }
    if (changed) {
      attr.array.set(values); attr.addUpdateRange(0, values.length); attr.needsUpdate = true;
      object.geometry.setDrawRange(0, values.length);
    }
    object.visible = values.length > 0 && (!object.userData.surfaceFace || surfaceStyle === 'faces');
  }
}
function pumpSurfaceWorker() {
  if (surfaceInFlight || !surfaceQueued) return;
  if (!surfaceWorker) {
    surfaceWorker = new Worker('/point-surface-worker.js', {type: 'module'});
    surfaceWorker.onmessage = ({data}) => {
      const request = surfaceInFlight;
      surfaceInFlight = null;
      if (request && data.id === surfaceDesired) {
        if (data.error) {
          $('surface-update-summary').textContent = `구조 갱신 실패 · 이전 결과 유지: ${data.error}`;
        } else {
          const result = data.result;
          surfaceGroup ??= new THREE.Group();
          if (surfaceGroup.parent !== root) root.add(surfaceGroup);
          if (result.reset) clearSurfacePatches();
          for (const key of result.removed) {
            const patch = surfacePatches.get(key);
            if (patch) {
              patch.group.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
              surfaceGroup.remove(patch.group); surfacePatches.delete(key);
            }
          }
          for (const patch of result.patches) applySurfacePatch(patch);
          surfaceAck = data.version;
          surfaceCache = {...request.tag, model: result.stats, elapsed: result.stats.analysisMs};
          updateSurfacePreview();
        }
      }
      pumpSurfaceWorker();
    };
    surfaceWorker.onerror = () => {
      surfaceInFlight = null;
      $('surface-update-summary').textContent = '구조 갱신 실패 · 페이지를 새로 고쳐 다시 시도하세요.';
    };
  }
  const request = surfaceQueued;
  surfaceQueued = null; surfaceInFlight = request;
  request.payload.ack = surfaceAck;
  surfaceWorker.postMessage(request.payload);
}
function updateSurfacePreview(immediate = false) {
  if (!run || !root) return;
  const enabled = root.visible !== false && mode === 'hologram' && surfaceStyle !== 'points';
  if (surfaceGroup) surfaceGroup.visible = enabled;
  if (!enabled) {
    clearTimeout(surfaceTimer); surfaceTimer = null;
    if (!objectLayer.objects.length) cloud.material.opacity = 1;
    $('surface-summary').textContent = root.visible === false ? '초기화 · 표시할 구조가 없습니다.' : '점만 표시 · 구조 연결 숨김';
    Object.assign($('viewport').dataset, {surfaceStyle: 'points', surfaceGroups: 0, surfaceLines: 0,
      surfaceFaces: 0, surfaceWeakFaces: 0, surfacePointOnly: root.visible === false ? 0 : cloud.geometry.drawRange.count});
    return;
  }
  const tag = {grid: gridCells, threshold: confidenceThreshold, connection: surfaceConnection,
    colorThreshold: surfaceColorThreshold, normalAngle: surfaceNormalAngle, wideSurfaces: surfaceWide};
  const matches = cached => cached && Object.keys(tag).every(key => cached[key] === tag[key]);
  if (!matches(surfaceCache)) {
    $('surface-update-summary').textContent = '바뀐 주변을 갱신합니다 · 기존 선·면 유지';
    if (!matches(surfaceQueued?.tag) && !matches(surfaceInFlight?.tag)) {
      if (!immediate) {
        clearTimeout(surfaceTimer);
        surfaceTimer = setTimeout(() => { surfaceTimer = null; updateSurfacePreview(true); }, 160);
      } else {
        const id = ++surfaceDesired;
        surfaceQueued = {tag, payload: {id, points: filteredPoints, grid: gridCells, observations: run.point_observations,
          settings: {...tag, grid: undefined, spacing: spacingProfile.median},
          context: {source: run.source, session: run.input_run_id, segment: run.segment_id, poseSource: run.pose_source, revision: run.revision,
            frameKeys: run.cameras.map(c => c.frame_id), cameras: run.cameras,
            displayRotation: run.coordinate_system?.display_rotation}}};
        pumpSurfaceWorker();
      }
    }
    $('viewport').dataset.surfaceStyle = 'pending';
    return;
  }
  for (const patch of surfacePatches.values()) for (const object of patch.layers.values())
    if (object.userData.surfaceFace) object.visible = surfaceStyle === 'faces' && object.geometry.drawRange.count > 0;
  const m = surfaceCache.model;
  if (!objectLayer.objects.length) cloud.material.opacity = m.edges ? .32 : 1;
  $('surface-summary').textContent = `선 ${number(m.edges)}개 · ` +
    (surfaceStyle === 'faces' ? `삼각면 ${number(m.faces)}개 (옅은 후보 ${number(m.weakFaces)}개) · ` : '') + `점만 ${number(m.pointOnly)}곳`;
  $('surface-update-summary').textContent = `유지 ${number(m.retainedFaces)}면 · 추가 ${number(m.addedFaces)}면 · 제거 ${number(m.removedFaces)}면` +
    ` · 블록 ${number(m.remeshedBlocks)}곳 갱신 / ${number(m.reusedBlocks)}곳 재사용 · ${m.analysisMs.toFixed(1)}ms`;
  $('surface-wide-summary').textContent = surfaceWide ?
    `넓은 표면 ${number(m.widePlanes || 0)}곳 · 특징점 기준 사각 패치 ${number(m.wideQuads || 0)}개 · 재사용 ${number(m.wideReused || 0)}곳 · 영상 ${number(m.wideImageViews || 0)}시점 확인` : '넓은 표면 채우기 꺼짐';
  Object.assign($('viewport').dataset, {surfaceStyle, surfaceGroups: m.groups, surfaceLines: m.edges,
    surfaceFaces: surfaceStyle === 'faces' ? m.faces : 0, surfaceWeakFaces: surfaceStyle === 'faces' ? m.weakFaces : 0,
    surfacePointOnly: m.pointOnly, surfaceAnalysisMs: m.analysisMs.toFixed(1), surfaceAlgorithm: m.algorithm,
    surfaceColorThreshold, surfaceNormalAngle, surfaceRetainedFaces: m.retainedFaces,
    surfaceUpdatedBlocks: m.remeshedBlocks, surfaceReusedBlocks: m.reusedBlocks, surfaceUpdateReason: m.reason,
    surfaceWidePlanes: m.widePlanes || 0, surfaceWideFaces: m.wideFaces || 0, surfaceWideQuads: m.wideQuads || 0});
}
function schedulePointDisplay() {
  if (intervalFrame === null) intervalFrame = requestAnimationFrame(() => {
    intervalFrame = null;
    applyPointInterval();
  });
}
function setPointInterval(value, editingNumber = false) {
  if (!Number.isFinite(value) || value < 0 || value > Math.max(spacingProfile.max, hologramInterval) * 100) return;
  hologramInterval = value;
  gridCells = null;
  for (const id of ['hologram-interval', 'hologram-interval-number']) $(id).max = Math.max(spacingProfile.max, value);
  $('hologram-interval').value = value;
  if (!editingNumber) $('hologram-interval-number').value = value;
  updateIntervalLabel();
  // Coalesce drag events into one position-buffer update per rendered frame.
  schedulePointDisplay();
}
function setConfidence(value) {
  if (!Number.isInteger(value) || value < 1 || value > 1000000) return;
  confidenceThreshold = value;
  schedulePointDisplay();
}
function setMode(next, toggleControls = false) {
  if (next !== 'hologram') hologramControlsOpen = false;
  else if (toggleControls) hologramControlsOpen = !hologramControlsOpen;
  mode = next;
  $('hologram-controls').hidden = !hologramControlsOpen;
  document.querySelector('[data-mode="hologram"]').setAttribute('aria-expanded', String(hologramControlsOpen));
  document.querySelectorAll('[data-mode]').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.mode === next)));
  updateIntervalLabel();
  if (!run) return;
  applyPointInterval();
  cloud.material.transparent = next !== 'points';
  cloud.material.opacity = next !== 'points' && objectLayer.objects.length ? (next === 'hologram' ? .10 : .22) :
    next === 'hologram' && surfaceGroup?.visible && surfaceCache?.model.edges ? .32 : 1;
  cloud.material.vertexColors = next !== 'hologram';
  cloud.material.color.set(next === 'hologram' ? 0x92f8fa : 0xffffff);
  cloud.material.needsUpdate = true;
  objectLayer.style(next);
  $('mode-detail').textContent = {objects: '특징점이 지지하는 평면 객체 · 실제 영상 표면',
    hologram: '검출된 평면의 발광 면·격자·경계와 원본 점군', points: '원본 SfM 점군 · 점 표시 필터 적용'}[next];
  $('kind').textContent = {objects: 'PLANAR OBJECTS', hologram: 'HOLOGRAM · PLANAR OBJECTS', points: '원본 3D 점군'}[next];
  if (run.slam) {
    $('kind').textContent = next === 'hologram' ? 'HOLOGRAM · ORB MAP' : 'ORB-SLAM3 · 특징점 지도';
    $('mode-detail').textContent = '연속 프레임의 같은 지도 점을 추적하고 카메라 위치와 희소 3D 지도를 함께 갱신합니다.';
  }
  if (next === 'hologram') $('mode-detail').textContent += ' 격자로 좌표를 모은 뒤 관측 프레임 수로 신뢰할 점을 고릅니다.';
  if (next === 'hologram') $('mode-detail').textContent += ' 원본 색상이 비슷한 이웃을 선·면으로 연결하고, 영상의 색상이 이어지는 넓은 평면도 채웁니다.';
}
function selectFrame(index) {
  currentIndex = Math.max(0, Math.min(index, run.cameras.length - 1));
  const c = run.cameras[currentIndex];
  $('frame-timeline').value = currentIndex;
  const query = run.stream ? `?revision=${String(run.revision).padStart(6, '0')}&session=${encodeURIComponent(run.stream.session_id)}` : '';
  $('frame').src = `/api/${run.source}/frame/${encodeURIComponent(c.frame_id)}${query}`;
  $('frame-name').textContent = c.frame_id;
  $('timestamp').textContent = `${timeOf(c).toFixed(1)}s${c.source_pts_s == null ? ' · 수신 기준' : ''}`;
  selected.position.fromArray(c.position);
  objectLayer.drawEvidence(currentIndex);
}
function stopExplore() {
  if (navigating) controls.target.copy(camera.position).add(camera.getWorldDirection(new THREE.Vector3()).multiplyScalar(radius));
  navigating = false;
  keys.clear();
  controls.enabled = true;
  $('explore').setAttribute('aria-pressed', 'false');
  $('explore').textContent = '공간 안으로 이동 ↗';
  $('move-controls').hidden = true;
  $('viewer-hint').textContent = '드래그 회전 · 휠 확대 · 오른쪽 드래그 이동';
}
function setView(view, all = false) {
  stopExplore();
  const target = all ? fullCenter : center, r = all ? fullRadius : radius;
  const d = view === 'front' ? new THREE.Vector3(0, 0, 1) : view === 'top' ? new THREE.Vector3(.001, 1, .001) : direction.clone();
  camera.up.set(0, 1, 0);
  if (view === 'top') camera.up.set(0, 0, -1);
  camera.fov = 48;
  camera.updateProjectionMatrix();
  camera.position.copy(target).addScaledVector(d.normalize(), r * 2.6);
  controls.target.copy(target);
  controls.update();
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('active', b.dataset.view === view));
}
function cameraView() {
  const c = run.cameras[currentIndex], rot = rotation(c);
  camera.position.fromArray(c.position);
  camera.up.set(0, -1, 0).applyMatrix3(rot);
  camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(c.height / (2 * c.params[0])));
  camera.updateProjectionMatrix();
  controls.target.copy(camera.position).add(new THREE.Vector3(0, 0, 1).applyMatrix3(rot).multiplyScalar(radius));
  controls.update();
}
function applyRun(value) {
  if (!['phone-sfm-result/1.0', 'phone-sfm-result/1.1', 'phone-sfm-result/1.2'].includes(value.schema_version) || !value.points?.length || !value.cameras?.length)
    throw new Error('지원하는 지도와 카메라 경로가 없습니다.');
  const preserve = run && run.input_run_id === value.input_run_id && run.segment_id === value.segment_id;
  const view = !run && views.get(source.source);
  const restore = view && view.session === value.input_run_id && view.segment === value.segment_id;
  const frame = run?.cameras[currentIndex]?.frame_id || (restore ? view.frame : null);
  run = value;
  configureSpacing();
  $('viewport').dataset.revision = String(run.revision ?? 0);
  $('viewport').dataset.session = run.input_run_id;
  $('viewport').dataset.source = run.source;
  $('map-empty').hidden = true;
  document.querySelector('.capture').hidden = false;
  document.querySelector('.notes').hidden = false;
  document.querySelectorAll('.view-buttons button, #explore, #camera-view, #frame-timeline').forEach((b) => { b.disabled = false; });
  document.querySelector('.workspace').classList.toggle('slam-workspace', Boolean(run.slam));
  if (preserve && surfaceGroup) root.remove(surfaceGroup);
  else {
    clearSurfacePatches(); surfaceGroup = null; surfaceCache = null;
    surfaceQueued = null; surfaceDesired++; surfaceAck = -1;
  }
  dispose();
  clearTimeout(surfaceTimer); surfaceTimer = null;
  root = new THREE.Group();
  scene.add(root);
  if (surfaceGroup) root.add(surfaceGroup);
  cloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({vertexColors: true,
    size: Number($('point-size').value), sizeAttenuation: false}));
  root.add(cloud);
  showCloud();
  const rawBoundsGeometry = geometry(run.points.map((p) => p.slice(1, 4)));
  const bounds = new THREE.Box3().setFromBufferAttribute(rawBoundsGeometry.getAttribute('position'));
  rawBoundsGeometry.dispose();
  fullCenter = bounds.getCenter(new THREE.Vector3());
  fullRadius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, .1);
  const q = [1, 2, 3].map((axis) => run.points.map((p) => p[axis]).sort((a, b) => a - b));
  const focus = new THREE.Box3(new THREE.Vector3(...q.map((a) => a[Math.floor(a.length * .02)])),
    new THREE.Vector3(...q.map((a) => a[Math.floor(a.length * .98)])));
  center = focus.getCenter(new THREE.Vector3());
  radius = Math.max(focus.getSize(new THREE.Vector3()).length() / 2, .1);
  const positions = run.cameras.map((c) => new THREE.Vector3(...c.position));
  direction = positions.reduce((sum, p) => sum.add(p), new THREE.Vector3()).divideScalar(positions.length).sub(center).normalize();
  direction.y += .15;
  route = line(positions, 0xc6bb67, .8);
  root.add(route);
  frustums = new THREE.Group();
  const cameraLines = [];
  for (const c of run.cameras) {
    const rot = rotation(c), origin = new THREE.Vector3(...c.position), d = radius * .025;
    const corners = [[-1, -.5625, 1.5], [1, -.5625, 1.5], [1, .5625, 1.5], [-1, .5625, 1.5]]
      .map((p) => new THREE.Vector3(...p).multiplyScalar(d).applyMatrix3(rot).add(origin));
    for (let i = 0; i < 4; i++) cameraLines.push(corners[i], corners[(i + 1) % 4], origin, corners[i]);
  }
  frustums.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(cameraLines),
    new THREE.LineBasicMaterial({color: 0x6c8d89, transparent: true, opacity: .45})));
  root.add(frustums);
  selected = new THREE.Mesh(new THREE.SphereGeometry(radius * .012, 12, 12), new THREE.MeshBasicMaterial({color: 0xe3ce71}));
  root.add(selected);
  const gridSize = Math.max(2, Math.ceil(fullRadius * 3));
  grid = new THREE.GridHelper(gridSize, run.scale.unit === 'm' ? Math.min(gridSize * 2, 200) : 20, 0x34504e, 0x243b3f);
  grid.position.set(center.x, bounds.min.y - radius * .025, center.z);
  root.add(grid);
  root.add(objectLayer.apply(run.planar_objects, run));
  document.querySelectorAll('[data-mode]').forEach((b) => {
    b.disabled = b.dataset.mode === 'objects' && !run.planar_objects;
  });
  if (mode === 'objects' && !run.planar_objects) mode = 'points';
  setMode(mode);
  const quality = run.quality;
  $('frames').parentElement.querySelector('span').textContent = '등록 프레임';
  $('elapsed').nextElementSibling.textContent = '입력 검증 이후 SfM·색상';
  document.querySelector('.notes h2').innerHTML = savedCopy.title;
  $('scale-note').nextElementSibling.textContent = savedCopy.detail;
  document.querySelector('.notes .muted').textContent = savedCopy.note;
  document.querySelector('footer > span').textContent = 'COLMAP · OFFLINE RECONSTRUCTION';
  $('run-name').textContent = `${run.input_run_id} · ${timeOf(run.cameras[0]).toFixed(1)}–${timeOf(run.cameras.at(-1)).toFixed(1)}초 · ${run.stream ? '온라인 지도' : '저장된 지도'}`;
  $('run-date').textContent = new Date(run.created_utc).toLocaleDateString('ko-KR');
  $('frames').textContent = `${quality.registered_frames} / ${quality.input_frames}`;
  $('registration').textContent = `${Math.round(quality.registration_ratio * 100)}% 등록 · ${quality.excluded_frames.length}장 기하 제외${quality.result_status === 'partial' ? ' · 부분 복원' : ''}`;
  $('reprojection').textContent = `${quality.mean_reprojection_error_px.toFixed(2)} px`;
  $('elapsed').textContent = `${quality.processing_seconds.toFixed(1)} s`;
  document.querySelector('.object-panel').hidden = Boolean(run.slam && !run.planar_objects);
  if (run.slam) {
    $('frames').textContent = String(run.slam.keyframes);
    $('frames').parentElement.querySelector('span').textContent = '선택된 키프레임';
    $('registration').textContent = `${run.slam.keyframes}개 키프레임 · 전체 프레임의 추적 상태는 위 영상에서 확인`;
    document.querySelector('.notes h2').innerHTML = '같은 점을 이어,<br>이동과 공간을 추적합니다.';
    $('scale-note').nextElementSibling.textContent = 'ORB-SLAM3의 카메라 경로와 실제 대응점으로 만든 희소 지도입니다. 이웃 점은 색상과 국소 방향으로 선·면을 만들고, 넓은 평면은 원본 점의 지지와 촬영 영상의 색상 연결을 함께 확인해 채웁니다. 관측 근거가 적은 후보는 옅게 표시합니다. 점 사이의 면은 추정한 영역이며 실제 사물의 접합·두께를 확인한 결과는 아닙니다.';
    document.querySelector('.notes .muted').textContent = '카메라 수신과 저장 기록 재생을 지원합니다. 단안 스케일·카메라 보정·실기체 지연은 별도 확인이 필요합니다.';
    document.querySelector('footer > span').textContent = 'ORB-SLAM3 · MONOCULAR TRACKING';
    $('elapsed').nextElementSibling.textContent = '영상 재생·추적 경과 시간';
  }
  $('scale-badge').textContent = run.scale.unit === 'm' ? '44cm 기준 · m' : '임의 단위 · 크기 미보정';
  $('scale-note').textContent = run.scale.unit === 'm' ? '44cm 기준 스케일을 모든 점·객체·경로에 동일 적용했습니다. 독립 치수 정확도와 중력 방향은 미검증입니다.' :
    '단안 영상의 임의 단위입니다. 좌표와 객체 크기를 cm·m로 해석할 수 없습니다. 실제 거리 센서나 실측 기준으로 스케일을 검증해야 합니다.';
  $('frame-timeline').max = run.cameras.length - 1;
  const previousIndex = run.cameras.findIndex((c) => c.frame_id === frame);
  const objectIndex = run.cameras.findIndex((c) => c.frame_id === objectLayer.current()?.source_frame_id);
  selectFrame(previousIndex >= 0 ? previousIndex : objectIndex >= 0 ? objectIndex : Math.floor(run.cameras.length / 2));
  route.visible = selected.visible = $('show-path').checked;
  frustums.visible = $('show-cameras').checked;
  grid.visible = $('show-grid').checked;
  if (!preserve) {
    if (mode !== 'points' && objectLayer.objects.length) objectLayer.focusAll();
    else setView('perspective');
  }
  if (restore) {
    camera.position.copy(view.position);
    camera.up.copy(view.up);
    camera.fov = view.fov;
    camera.updateProjectionMatrix();
    controls.target.copy(view.target);
    controls.update();
    document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('active', b.dataset.view === view.view));
  }
  $('download').hidden = false;
  $('download').firstChild.textContent = run.stream ? '지도 JSON 내려받기 ' : '점군 PLY 내려받기 ';
  $('download').download = run.stream ? `${run.input_run_id}-${run.revision}.json` : 'points.ply';
  $('download').href = run.stream ? `/api/live/run?revision=${String(run.revision).padStart(6, '0')}&session=${encodeURIComponent(run.stream.session_id)}` : '/api/saved/points.ply';
  document.querySelectorAll('footer a').forEach((a) => {
    a.hidden = Boolean(run.stream);
    a.setAttribute('href', a.getAttribute('href').replace(/^\/api\/(?:saved\/)?/, '/api/saved/'));
  });
}

function replayDuration() { return replay.entries.at(-1)?.timestamp_s || 0; }
function replayControls() {
  const duration = replayDuration();
  if (replay.following) replay.time = duration;
  $('timeline').max = duration;
  $('timeline').value = replay.time;
  $('play').textContent = playing ? 'Ⅱ 일시정지' : '▶ 재생';
  $('play').disabled = replay.entries.length === 0;
  $('replay-clock').textContent = `${replay.time.toFixed(1)} / ${duration.toFixed(1)}초`;
  $('live-follow').setAttribute('aria-pressed', String(replay.following));
  const estimate = replay.entries.some((e) => e.timestamp_basis === 'latest_surviving_keyframe_estimate');
  const detail = replay.following ? '최신 지도 표시' : replay.applied === null ? '초기화 · 충분한 시차를 기다리는 구간' :
    `${playing ? '생성 과정 재생' : '기록 일시정지'} · 지도 ${replay.applied} · 표시 ${$('points').textContent}점`;
  $('replay-state').textContent = `${detail} · ${replay.entries.length}개 지도 갱신 기록${estimate ? ' · 이전 기록 시각은 마지막 키프레임 기준 추정' : ' · 입력 영상 시각 기준'}`;
  Object.assign($('replay-state').dataset, {time: replay.time, revision: replay.applied ?? 0,
    following: replay.following, playing, points: root?.visible === false ? 0 : cloud?.geometry.drawRange.count || 0});
  if (source.source === 'live' && lastLiveState.source === 'live') liveStatus(lastLiveState);
}
async function refreshTimeline(state, context) {
  if (state.backend !== 'orb_slam3_monocular') return;
  $('map-playback').hidden = false;
  const session = state.session_id;
  if (session !== replay.session) {
    replay.session = session;
    replay.entries = [];
    replay.cache.clear();
    replay.generation++;
    replay.following = true;
    replay.applied = null;
    replay.manifestRevision = null;
    renderTimes.length = 0;
    previousTime = 0;
    playing = false;
  }
  if (state.map_available && replay.manifestRevision !== state.revision) {
    const response = await source.read(`timeline?session=${encodeURIComponent(session)}`, context);
    if (!response) return;
    if (!response.ok) throw new Error('지도 재생 이력을 읽지 못했습니다.');
    const timeline = response.value;
    if (timeline.session_id === replay.session) {
      replay.entries = timeline.entries;
      replay.manifestRevision = state.revision;
    }
  }
  replayControls();
}
function frameAtReplayTime() {
  if (!run || !run.cameras.length) return;
  let index = 0;
  for (let i = 0; i < run.cameras.length; i++) if (timeOf(run.cameras[i]) <= replay.time) index = i;
  if (currentIndex !== index || !$('frame').getAttribute('src')) selectFrame(index);
  if ($('slam-preview').src !== $('frame').src) $('slam-preview').src = $('frame').src;
  $('slam-preview').alt = '선택한 지도 기록의 촬영 키프레임';
}
async function drawReplay() {
  if (source.source !== 'live' || replay.following || replay.loading) return;
  const context = source.context();
  const generation = replay.generation;
  const entry = replay.entries.findLast((e) => e.timestamp_s <= replay.time);
  if (!entry) {
    if (root) root.visible = false;
    $('points').textContent = '0';
    $('frame').removeAttribute('src');
    $('slam-preview').removeAttribute('src');
    $('timestamp').textContent = '초기화 대기';
    $('frame-name').textContent = '첫 유효 지도 이전';
    $('viewport').dataset.revision = '0';
    updateSurfacePreview();
    $('download').hidden = true;
    document.querySelector('.capture').hidden = true;
    showEmpty('첫 유효 지도 이전', '지도 생성 시간 슬라이더를 움직이거나 재생해 점이 생성되는 과정을 확인하세요.');
    replay.applied = null;
    replayControls();
    return;
  }
  if (replay.applied === entry.revision && root?.visible) { frameAtReplayTime(); return; }
  const task = {};
  replay.loading = task;
  try {
    let value = replay.cache.get(entry.revision);
    if (!value) {
      const response = await source.read(`run?revision=${String(entry.revision).padStart(6, '0')}&session=${encodeURIComponent(replay.session)}`, context);
      if (!response) return;
      if (!response.ok) throw new Error('저장된 지도 버전을 불러오지 못했습니다.');
      value = response.value;
    }
    if (!source.isCurrent(context) || generation !== replay.generation || replay.following) return;
    if (value.source !== 'live' || value.input_run_id !== replay.session) throw new Error('다른 세션의 지도 응답을 거부했습니다.');
    replay.cache.set(entry.revision, value);
    while (replay.cache.size > 3) replay.cache.delete(replay.cache.keys().next().value);
    applyRun(value);
    replay.applied = entry.revision;
    frameAtReplayTime();
    replayControls();
  } catch (error) {
    if (!source.isCurrent(context) || generation !== replay.generation) return;
    playing = false;
    $('replay-state').textContent = error.message;
  } finally { if (replay.loading === task) replay.loading = false; }
}
function seekReplay(time) {
  replay.following = false;
  replay.time = Math.max(0, Math.min(time, replayDuration()));
  replay.generation++;
  replayControls();
  drawReplay();
}
// Playback follows video time, independent of the browser's rendering frame rate.
setInterval(() => {
  const now = performance.now();
  if (playing && !replay.following) {
    replay.time = Math.min(replayDuration(), replay.time + (now - replay.lastTick) / 1000 * Number($('replay-speed').value));
    if (replay.time >= replayDuration()) playing = false;
    replayControls();
  }
  replay.lastTick = now;
  if (!replay.following) drawReplay();
}, 100);
function showState(state) {
  if (state.source === 'live') liveStatus(state);
  else {
    $('slam-panel').hidden = true;
    $('incremental-state').hidden = true;
  }
  if (state.backend === 'orb_slam3_monocular') {
    pollDelay = 250;
    $('slam-panel').hidden = false;
    const names = {2: '추적 중', 3: '재위치 탐색', 4: '추적 상실'};
    $('slam-tracking-state').textContent = names[state.tracking_state] || '초기화 중';
    const tracking = state.frame_tracking || {};
    $('slam-timing').textContent = `지도 점 ${tracking.tracked_features || 0}개 · 직전 프레임과 동일 ${tracking.reused_features || 0}개 · 추적 ${Number(state.tracking_ms || 0).toFixed(1)}ms · 재생 지연 ${Number(state.input_lag_seconds || 0).toFixed(2)}초`;
    $('slam-tracking-state').textContent = replay.following ? state.status === 'complete' ? '추적 종료' : state.status === 'failed' ? '추적 중단' : names[state.tracking_state] || '초기화 중' : '기록 재생';
    $('slam-preview-note').textContent = replay.following ?
      '노란 점: 직전 처리 프레임과 같은 지도 점 · 초록 점: 이번에 연결된 지도 점 · 숫자: 지도 점 ID' :
      '선택한 지도 버전의 촬영 키프레임입니다. 점군·경로는 그 시점에 실제로 저장된 지도 전체로 교체합니다.';
    if (replay.following && state.frame_id && state.frame_id !== previewFrame) {
      previewFrame = state.frame_id;
      $('slam-preview').src = `/api/live/slam-preview?frame=${encodeURIComponent(previewFrame)}&session=${encodeURIComponent(state.session_id)}`;
    }
    const pendingMap = state.map_available && state.map_id !== state.segment_id ? ' · 새 좌표계 준비, 마지막 지도 유지' : '';
    $('stream-state').textContent = `${state.status === 'complete' ? '영상 종료' : state.status === 'failed' ? '실행 실패' : '영상 추적'} · 처리 ${state.frames_received || 0}장 · 정상 추적 ${state.tracked_frames || 0}장 · 초기화 ${state.initializing_frames || 0}장 · 추적 상실 ${state.lost_tracking_frames || 0}장 · 지도 ${state.revision || 0}회 갱신${pendingMap}${state.input === 'browser_camera' ? ` · 입력 누락/대기 교체 ${state.frame_drops || 0}장 · 대기 ${state.queue_depth || 0}/1` : ''}`;
    $('incremental-state').hidden = false;
    const calibration = state.calibration_status || run?.slam?.calibration_status || '';
    $('incremental-state').textContent = `추적 P95 ${Number(state.tracking_p95_ms || 0).toFixed(1)}ms · 재생 지연 P95 ${Number(state.input_lag_p95_seconds || 0).toFixed(2)}초 · 단안 임의 단위${calibration.includes('fov_estimated') ? ' · 화각 추정/미보정' : calibration.includes('self_estimated') ? ' · 카메라 보정값 자체 추정' : ''}`;
    return;
  }
  const incremental = state.objects?.update_mode === 'incremental_keyframe';
  $('incremental-state').hidden = !incremental && !state.frame_tracking;
  $('incremental-state').textContent = incremental ?
    `증분 공간 지도 · 새 면 ${state.objects.new_objects || 0}개 · 유지 ${state.objects.retained_objects || 0}개 · 기존 면 관측 ${state.objects.reused_boundaries || 0}회 재사용 · 깊이 확인 대기 ${state.objects.pending_frames || 0}시점` :
    state.frame_tracking ? `프레임별 특징점 추적 중 · ${state.frame_tracking.tracked_features}점 이어받음 · 충분한 시차와 카메라 위치를 기다립니다.` : '';
  const mapAge = state.last_map_timestamp_s == null ? '' : ` · 지도 시각 ${state.last_map_timestamp_s.toFixed(1)}s`;
  const names = {idle: '영상 입력 전', starting: '연결 준비', receiving: '영상 수신', mapping: '지도 갱신', complete: '영상 종료 · 최종 지도', failed: '초기화/입력 실패', stopped_or_disconnected: '수신 중단'};
  $('stream-state').textContent = state.status === 'offline' ? '저장된 지도 · 실시간 수신 아님' :
    `${names[state.status] || state.status} · 갱신 ${state.revision || 0} · 건너뛴 프레임 ${state.frame_drops || 0}${mapAge}${state.mapping_state === 'processing' ? ' · 계산 중' : ''}${state.last_error ? ' · 일부 갱신 실패, 마지막 지도 유지' : ''}`;
}
async function poll() {
  const context = source.context();
  if (busy === context.generation) return;
  busy = context.generation;
  try {
    const result = await source.read('state', context);
    if (!result) return;
    if (!result.ok) throw new Error('지도 상태를 읽지 못했습니다.');
    const state = result.value;
    if (state.source !== context.source) throw new Error('선택한 지도와 응답의 출처가 다릅니다.');
    if (run && state.source === 'live' && state.session_id !== run.input_run_id) { clearMap(); revision = null; }
    await refreshTimeline(state, context);
    if (!source.isCurrent(context)) return;
    showState(state);
    $('error').hidden = true;
    if (!replay.following) return;
    if (!state.map_available) {
      const ended = (state.session_status || state.status) === 'complete';
      if (!run) showEmpty(state.source === 'saved' ? '저장된 지도가 없습니다' : state.status === 'failed' ? '3D 지도를 만들지 못했습니다' : ended ? '지도 초기화 전에 입력이 종료됐습니다' : state.session_id ? '3D 지도를 준비하고 있습니다' : '영상 입력을 기다리고 있습니다',
        state.source === 'saved' ? '저장 결과를 연결한 서버에서 이 탭을 열어 주세요.' : state.status === 'failed' ? '입력 상태를 확인한 뒤 다시 시작하세요.' : ended ? '영상 연결과 3D 생성은 별도입니다. 같은 공간을 보며 카메라 위치를 옮겨 다시 시험하세요.' : state.session_id ? '영상 수신과 지도 초기화는 별도 단계입니다. 충분한 시차와 특징점이 모이면 표시됩니다.' : '위에서 입력을 선택하고 시작하세요. 카메라 권한은 시작할 때 요청합니다.');
      return;
    }
    if (run && (state.status === 'offline' || (state.revision === revision && (!state.session_id || state.session_id === run.input_run_id)))) return;
    const query = state.session_id ? `?session=${encodeURIComponent(state.session_id)}&revision=${String(state.revision).padStart(6, '0')}` : '';
    const useDelta = run?.pose_source === 'orb_slam3' && run.source === state.source && run.input_run_id === state.session_id && run.revision < state.revision;
    const deltaQuery = useDelta ? `${query}&since_revision=${String(run.revision).padStart(6, '0')}` : query;
    const response = await source.read(`run${deltaQuery}`, context);
    if (!response) return;
    if (response.status === 503) { $('run-name').textContent = '영상 수신 중 · 충분한 시차와 관측점을 기다립니다.'; return; }
    if (!response.ok) throw new Error(`지도 읽기 실패 ${response.status}`);
    let value;
    try { value = applySceneDelta(run, response.value); }
    catch {
      const full = await source.read(`run${query}`, context);
      if (!full) return;
      if (!full.ok) throw new Error(`Map read failed ${full.status}`);
      value = full.value;
    }
    if (value.source !== state.source || (state.session_id && value.input_run_id !== state.session_id)) return;
    if (!replay.following || (state.backend === 'orb_slam3_monocular' && state.session_id !== replay.session)) return;
    applyRun(value);
    revision = run.revision ?? 0;
    if (state.source === 'live') { replay.applied = revision; replayControls(); liveStatus(state); }
    $('error').hidden = true;
  } catch (error) {
    if (!source.isCurrent(context)) return;
    $('error').hidden = false;
    $('error').textContent = `지도를 갱신하지 못했습니다. ${error.message}${run ? ' 마지막 유효 지도를 표시합니다.' : ''}`;
    if (!run) showEmpty('지도 서버에 연결하지 못했습니다', '연결이 복구되면 자동으로 다시 불러옵니다.');
  } finally {
    if (source.isCurrent(context)) { busy = null; pollTimer = setTimeout(poll, pollDelay); }
  }
}
$('play').addEventListener('click', () => {
  if (!replay.entries.length) return;
  if (!playing && (replay.following || replay.time >= replayDuration())) seekReplay(0);
  playing = !playing;
  replay.lastTick = performance.now();
  replayControls();
});
$('timeline').addEventListener('input', (e) => { playing = false; seekReplay(Number(e.target.value)); });
$('replay-reset').addEventListener('click', () => { playing = false; seekReplay(0); });
$('live-follow').addEventListener('click', () => {
  playing = false; replay.following = true; replay.generation++; revision = null; previewFrame = null; replayControls();
});
$('frame-timeline').addEventListener('input', (e) => { if (run) selectFrame(Number(e.target.value)); });
$('point-size').addEventListener('input', (e) => { if (cloud) cloud.material.size = Number(e.target.value); });
$('color-mode').addEventListener('change', showCloud);
$('error-limit').addEventListener('change', showCloud);
$('hologram-interval').addEventListener('input', (e) => setPointInterval(e.target.valueAsNumber));
$('hologram-interval-number').addEventListener('input', (e) => {
  if (!e.target.validity.badInput && !e.target.validity.stepMismatch) setPointInterval(e.target.valueAsNumber, true);
});
$('hologram-interval-number').addEventListener('change', (e) => { e.target.value = hologramInterval; });
$('hologram-interval-reset').addEventListener('click', () => setPointInterval(0));
$('hologram-interval-recommend').addEventListener('click', () => setPointInterval(spacingProfile.recommended));
$('hologram-confidence').addEventListener('input', (e) => setConfidence(e.target.valueAsNumber));
$('hologram-confidence-number').addEventListener('input', (e) => setConfidence(e.target.valueAsNumber));
$('hologram-confidence-number').addEventListener('change', (e) => { e.target.value = confidenceThreshold; });
$('surface-style').addEventListener('change', (e) => { surfaceStyle = e.target.value; schedulePointDisplay(); });
$('surface-wide').addEventListener('change', (e) => { surfaceWide = e.target.checked; schedulePointDisplay(); });
$('surface-connection').addEventListener('input', (e) => {
  surfaceConnection = e.target.valueAsNumber;
  $('surface-connection-value').textContent = `${surfaceConnection.toFixed(2)}×`;
  schedulePointDisplay();
});
for (const [id, update] of [
  ['surface-color', value => { surfaceColorThreshold = value; $('surface-color-value').textContent = `${value} ΔE`; }],
  ['surface-angle', value => { surfaceNormalAngle = value; $('surface-angle-value').textContent = `${value}°`; }],
]) $(id).addEventListener('input', e => { update(e.target.valueAsNumber); schedulePointDisplay(); });
$('show-path').addEventListener('change', (e) => { if (route) route.visible = selected.visible = e.target.checked; });
$('show-cameras').addEventListener('change', (e) => { if (frustums) frustums.visible = e.target.checked; });
$('show-grid').addEventListener('change', (e) => { if (grid) grid.visible = e.target.checked; });
document.querySelectorAll('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode, true)));
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => { if (run) setView(b.dataset.view); }));
$('reset').addEventListener('click', () => { if (run) setView('perspective', true); });
$('camera-view').addEventListener('click', () => { if (run) cameraView(); });
$('explore').addEventListener('click', () => {
  if (!run) return;
  if (navigating) { stopExplore(); return; }
  cameraView();
  navigating = true;
  controls.enabled = false;
  $('explore').setAttribute('aria-pressed', 'true');
  $('explore').textContent = '탐색 종료 · Esc';
  $('move-controls').hidden = false;
  $('viewer-hint').textContent = 'WASD 이동 · Q/E 위아래 · 드래그 시선 · Esc 종료';
  renderer.domElement.focus();
});
renderer.domElement.tabIndex = 0;
renderer.domElement.addEventListener('pointerdown', () => { if (navigating) dragging = true; });
window.addEventListener('pointerup', () => { dragging = false; });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!navigating || !dragging) return;
  const angle = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ');
  angle.y -= e.movementX * .003;
  angle.x = THREE.MathUtils.clamp(angle.x - e.movementY * .003, -1.55, 1.55);
  camera.quaternion.setFromEuler(angle);
});
window.addEventListener('keydown', (e) => {
  if (!navigating || /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
  if (e.key === 'Escape') { stopExplore(); return; }
  if ('wasdqe'.includes(e.key.toLowerCase())) { keys.add(e.key.toLowerCase()); e.preventDefault(); }
});
window.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => keys.clear());
document.querySelectorAll('[data-move]').forEach((button) => button.addEventListener('click', () => {
  if (!navigating) return;
  const d = button.dataset.move;
  const vector = d === 'up' || d === 'down' ? new THREE.Vector3(0, d === 'up' ? 1 : -1, 0) :
    d === 'left' || d === 'right' ? new THREE.Vector3(d === 'right' ? 1 : -1, 0, 0).applyQuaternion(camera.quaternion) :
    camera.getWorldDirection(new THREE.Vector3()).multiplyScalar(d === 'forward' ? 1 : -1);
  camera.position.addScaledVector(vector, run.scale.unit === 'm' ? .2 : radius * .08);
}));
new ResizeObserver(() => {
  const box = $('viewport').getBoundingClientRect();
  renderer.setSize(box.width, box.height);
  camera.aspect = box.width / Math.max(box.height, 1);
  camera.updateProjectionMatrix();
}).observe($('viewport'));
function animate(time) {
  requestAnimationFrame(animate);
  if (run && previousTime && document.visibilityState === 'visible') {
    renderTimes.push({dt: time - previousTime, revision});
    if (renderTimes.length > 600) renderTimes.shift();
    if (time - lastRenderReport > 1000 && renderTimes.length > 2) {
      const intervals = renderTimes.map((f) => f.dt).sort((a, b) => a - b);
      const p95 = intervals[Math.floor(intervals.length * .95)];
      const fps = 1000 / intervals[Math.floor(intervals.length * .5)];
      $('render-state').textContent = `화면 ${fps.toFixed(0)}fps · 프레임 간격 P95 ${p95.toFixed(1)}ms`;
      Object.assign($('render-state').dataset, {fps, p95Ms: p95, samples: intervals.length,
        firstRevision: renderTimes[0].revision, lastRevision: renderTimes.at(-1).revision});
      lastRenderReport = time;
    }
  }
  const dt = Math.min((time - previousTime) / 1000, .05);
  previousTime = time;
  if (run && navigating) {
    const forward = camera.getWorldDirection(new THREE.Vector3()), right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion);
    const move = new THREE.Vector3();
    if (keys.has('w')) move.add(forward);
    if (keys.has('s')) move.sub(forward);
    if (keys.has('d')) move.add(right);
    if (keys.has('a')) move.sub(right);
    if (keys.has('e')) move.y += 1;
    if (keys.has('q')) move.y -= 1;
    camera.position.addScaledVector(move.normalize(), (run.scale.unit === 'm' ? 1 : radius * .4) * dt);
  } else controls.update();
  if (navigating) $('explorer-position').textContent = `탐색 위치 ${camera.position.toArray().map((n) => n.toFixed(2)).join(' / ')} ${run.scale.unit === 'm' ? 'm' : '(임의 단위)'}`;
  objectLayer.render(time);
  renderer.render(scene, camera);
}
requestAnimationFrame(animate);
document.querySelectorAll('[data-source]').forEach((tab) => {
  tab.addEventListener('click', () => { location.hash = tab.dataset.source; });
  tab.addEventListener('keydown', (event) => {
    if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const next = event.key === 'Home' ? 'saved' : event.key === 'End' ? 'live' : source.source === 'saved' ? 'live' : 'saved';
    $(`tab-${next}`).focus();
    location.hash = next;
  });
});
window.addEventListener('hashchange', () => selectSource(location.hash === '#live' ? 'live' : 'saved'));
selectSource(source.source, true);
connectCapture({
  onStart(id) {
    lastLiveState = {source: 'live', session_id: id, session_status: 'starting', status: 'starting', map_available: false};
    if (source.source === 'live') selectSource('live', true);
  },
  onStatus(status) {
    captureState = status;
    $('live-tab-indicator').textContent = status.state === 'receiving' ? '수신 중' : ['requesting', 'connecting', 'stopping'].includes(status.state) ? '연결 중' : '';
    if (source.source === 'live') liveStatus(lastLiveState);
  },
}).then((handle) => { captureHandle = handle; captureHandle.update(lastLiveState); })
  .catch((error) => {
    captureState = {state: 'failed', message: `영상 입력 준비 실패: ${error.message}`};
    $('camera-status').textContent = captureState.message;
    $('camera-start').disabled = true;
    if (source.source === 'live') liveStatus(lastLiveState);
  });
