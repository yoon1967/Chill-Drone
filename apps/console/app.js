import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { PlanarObjects } from '/objects.js';
const $ = (id) => document.getElementById(id);
const number = (n) => Number(n).toLocaleString('ko-KR');
const timeOf = (c) => c.source_pts_s ?? c.timestamp_s;
let run, root, cloud, route, frustums, grid, selected;
let center, radius = 1, fullCenter, fullRadius = 1, direction;
let currentIndex = 0, playing = false, lastStep = 0, previousTime = 0, mode = 'objects';
let navigating = false, dragging = false, busy = false, revision = null;
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
  const colors = points.map((p) => {
    const colorMode = $('color-mode').value;
    if (colorMode === 'rgb') return p.slice(4, 7);
    const h = colorMode === 'error' ? Math.min(p[7] / 3, 1) :
      (p[2] - run.quality.bounds.min[1]) / Math.max(run.quality.bounds.max[1] - run.quality.bounds.min[1], .001);
    const c = new THREE.Color().setHSL((1 - h) * .58, .7, .6);
    return [c.r * 255, c.g * 255, c.b * 255];
  });
  cloud.geometry.dispose();
  cloud.geometry = geometry(points.map((p) => p.slice(1, 4)), colors);
  $('points').textContent = number(points.length);
}
function setMode(next) {
  mode = next;
  if (!run) return;
  cloud.material.transparent = next !== 'points';
  cloud.material.opacity = next !== 'points' && objectLayer.objects.length ? (next === 'hologram' ? .10 : .22) : 1;
  cloud.material.vertexColors = next !== 'hologram';
  cloud.material.color.set(next === 'hologram' ? 0x92f8fa : 0xffffff);
  cloud.material.needsUpdate = true;
  objectLayer.style(next);
  document.querySelectorAll('[data-mode]').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.mode === next)));
  $('mode-detail').textContent = {objects: '특징점이 지지하는 평면 객체 · 실제 영상 표면',
    hologram: '검출된 평면의 발광 면·격자·경계와 원본 점군', points: '원본 SfM 점군 · 점 표시 필터 적용'}[next];
  $('kind').textContent = {objects: 'PLANAR OBJECTS', hologram: 'HOLOGRAM · PLANAR OBJECTS', points: '원본 3D 점군'}[next];
}
function selectFrame(index) {
  currentIndex = Math.max(0, Math.min(index, run.cameras.length - 1));
  const c = run.cameras[currentIndex];
  $('timeline').value = currentIndex;
  $('frame').src = `/api/frame/${encodeURIComponent(c.frame_id)}${run.revision ? '?revision=' + String(run.revision).padStart(6, '0') : ''}`;
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
  const preserve = run && run.segment_id === value.segment_id;
  const frame = run?.cameras[currentIndex]?.frame_id;
  run = value;
  dispose();
  root = new THREE.Group();
  scene.add(root);
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
  for (const c of run.cameras) {
    const rot = rotation(c), origin = new THREE.Vector3(...c.position), d = radius * .025;
    const corners = [[-1, -.5625, 1.5], [1, -.5625, 1.5], [1, .5625, 1.5], [-1, .5625, 1.5]]
      .map((p) => new THREE.Vector3(...p).multiplyScalar(d).applyMatrix3(rot).add(origin));
    frustums.add(line([...corners, corners[0]], 0x6c8d89, .5));
    for (const p of corners) frustums.add(line([origin, p], 0x6c8d89, .4));
  }
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
  $('run-name').textContent = `${run.input_run_id} · ${timeOf(run.cameras[0]).toFixed(1)}–${timeOf(run.cameras.at(-1)).toFixed(1)}초 · ${run.stream ? '온라인 지도' : '저장된 지도'}`;
  $('run-date').textContent = new Date(run.created_utc).toLocaleDateString('ko-KR');
  $('frames').textContent = `${quality.registered_frames} / ${quality.input_frames}`;
  $('registration').textContent = `${Math.round(quality.registration_ratio * 100)}% 등록 · ${quality.excluded_frames.length}장 기하 제외${quality.result_status === 'partial' ? ' · 부분 복원' : ''}`;
  $('reprojection').textContent = `${quality.mean_reprojection_error_px.toFixed(2)} px`;
  $('elapsed').textContent = `${quality.processing_seconds.toFixed(1)} s`;
  $('scale-badge').textContent = run.scale.unit === 'm' ? '44cm 기준 · m' : '임의 단위 · 크기 미보정';
  $('scale-note').textContent = run.scale.unit === 'm' ? '44cm 기준 스케일을 모든 점·객체·경로에 동일 적용했습니다. 독립 치수 정확도와 중력 방향은 미검증입니다.' :
    '단안 영상의 임의 단위입니다. 좌표와 객체 크기를 cm·m로 해석할 수 없습니다. 실제 거리 센서나 실측 기준으로 스케일을 검증해야 합니다.';
  $('timeline').max = run.cameras.length - 1;
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
  $('download').href = run.stream ? '/api/run' : '/api/points.ply';
  document.querySelectorAll('footer a').forEach((a) => { a.hidden = Boolean(run.stream); });
}
function showState(state) {
  const incremental = state.objects?.update_mode === 'incremental_keyframe';
  $('incremental-state').hidden = !incremental && !state.frame_tracking;
  $('incremental-state').textContent = incremental ?
    `증분 공간 지도 · 새 면 ${state.objects.new_objects || 0}개 · 유지 ${state.objects.retained_objects || 0}개 · 기존 면 관측 ${state.objects.reused_boundaries || 0}회 재사용 · 깊이 확인 대기 ${state.objects.pending_frames || 0}시점` :
    state.frame_tracking ? `프레임별 특징점 추적 중 · ${state.frame_tracking.tracked_features}점 이어받음 · 충분한 시차와 카메라 위치를 기다립니다.` : '';
  const mapAge = state.last_map_timestamp_s == null ? '' : ` · 지도 시각 ${state.last_map_timestamp_s.toFixed(1)}s`;
  const names = {starting: '연결 준비', receiving: '영상 수신', mapping: '지도 갱신', complete: '영상 종료 · 최종 지도', failed: '초기화/입력 실패', stopped_or_disconnected: '수신 중단'};
  $('stream-state').textContent = state.status === 'offline' ? '저장된 지도 · 실시간 수신 아님' :
    `${names[state.status] || state.status} · 갱신 ${state.revision} · 건너뛴 프레임 ${state.frame_drops}${mapAge}${state.mapping_state === 'processing' ? ' · 계산 중' : ''}${state.last_error ? ' · 일부 갱신 실패, 마지막 지도 유지' : ''}`;
}
async function poll() {
  if (busy) return;
  busy = true;
  try {
    const state = await (await fetch('/api/state')).json();
    showState(state);
    if (run && (state.status === 'offline' || state.revision === revision)) return;
    const response = await fetch('/api/run');
    if (response.status === 503) { $('run-name').textContent = '영상 수신 중 · 충분한 시차와 관측점을 기다립니다.'; return; }
    if (!response.ok) throw new Error(`지도 읽기 실패 ${response.status}`);
    applyRun(await response.json());
    revision = run.revision ?? 0;
    $('error').hidden = true;
  } catch (error) { $('error').hidden = false; $('error').textContent = `지도를 갱신하지 못했습니다. ${error.message}`; }
  finally { busy = false; }
}
$('play').addEventListener('click', () => { playing = !playing; $('play').textContent = playing ? 'Ⅱ 일시정지' : '▶ 재생'; });
$('timeline').addEventListener('input', (e) => { if (run) selectFrame(Number(e.target.value)); });
$('point-size').addEventListener('input', (e) => { if (cloud) cloud.material.size = Number(e.target.value); });
$('color-mode').addEventListener('change', showCloud);
$('error-limit').addEventListener('change', showCloud);
$('show-path').addEventListener('change', (e) => { if (route) route.visible = selected.visible = e.target.checked; });
$('show-cameras').addEventListener('change', (e) => { if (frustums) frustums.visible = e.target.checked; });
$('show-grid').addEventListener('change', (e) => { if (grid) grid.visible = e.target.checked; });
document.querySelectorAll('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
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
  const dt = Math.min((time - previousTime) / 1000, .05);
  previousTime = time;
  if (run && playing && time - lastStep > 500) { selectFrame((currentIndex + 1) % run.cameras.length); lastStep = time; }
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
poll();
setInterval(poll, 2000);
