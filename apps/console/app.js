import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const number = (value) => value.toLocaleString('ko-KR');
let run, renderer, scene, camera, controls, cloud, route, frustums, grid, selected;
let currentIndex = 0, playing = false, lastStep = 0;
let center, radius, cameraDirection, fullCenter, fullRadius;
const size = new THREE.Vector2();

function showError(error) {
  $('error').hidden = false;
  $('error').textContent = `복원 화면을 열지 못했습니다. ${error.message} 로컬 실행 안내를 확인해 주세요.`;
  document.querySelector('.loading')?.remove();
}

function line(points, color, opacity = 1) {
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
}

function frameRotation(c) {
  const a = c.world_from_camera_rotation;
  return new THREE.Matrix3().set(...a[0], ...a[1], ...a[2]);
}

function showCloud() {
  const limit = Number($('error-limit').value);
  const mode = $('color-mode').value;
  const positions = [], colors = [];
  const bounds = run.quality.bounds;
  for (const p of run.points) {
    if (p[7] > limit) continue;
    positions.push(...p.slice(1, 4));
    const color = new THREE.Color();
    if (mode === 'rgb') color.setRGB(p[4] / 255, p[5] / 255, p[6] / 255, THREE.SRGBColorSpace);
    else {
      const value = mode === 'error' ? Math.min(p[7] / 3, 1) :
        (p[2] - bounds.min[1]) / Math.max(bounds.max[1] - bounds.min[1], 0.001);
      color.setHSL((1 - value) * 0.58, 0.68, 0.62);
    }
    colors.push(color.r, color.g, color.b);
  }
  cloud.geometry.dispose();
  cloud.geometry = new THREE.BufferGeometry();
  cloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  cloud.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  $('points').textContent = number(positions.length / 3);
}

function selectFrame(index) {
  currentIndex = Math.max(0, Math.min(index, run.cameras.length - 1));
  const c = run.cameras[currentIndex];
  $('timeline').value = currentIndex;
  $('frame').src = `/api/frame/${encodeURIComponent(c.frame_id)}`;
  $('timestamp').textContent = `${c.source_pts_s.toFixed(1)}s`;
  $('frame-name').textContent = c.frame_id;
  selected.position.fromArray(c.position);
}

function setView(view, fitAll = false) {
  const target = fitAll ? fullCenter : center;
  const distance = fitAll ? fullRadius : radius;
  const direction = view === 'front' ? new THREE.Vector3(0, 0, 1) :
    view === 'top' ? new THREE.Vector3(0.001, 1, 0.001) : cameraDirection.clone();
  camera.up.set(0, 1, 0);
  if (view === 'top') camera.up.set(0, 0, -1);
  camera.fov = 48;
  camera.updateProjectionMatrix();
  camera.position.copy(target).addScaledVector(direction.normalize(), distance * 2.6);
  controls.target.copy(target);
  controls.update();
  document.querySelectorAll('[data-view]').forEach((button) => button.classList.toggle('active', button.dataset.view === view));
}

function resize() {
  const box = $('viewport').getBoundingClientRect();
  renderer.setSize(box.width, box.height);
  camera.aspect = box.width / Math.max(box.height, 1);
  camera.updateProjectionMatrix();
  renderer.getSize(size);
}

function animate(time) {
  requestAnimationFrame(animate);
  if (playing && time - lastStep > 500) {
    selectFrame((currentIndex + 1) % run.cameras.length);
    lastStep = time;
  }
  controls.update();
  renderer.render(scene, camera);
}

async function main() {
  const response = await fetch('/api/run');
  if (!response.ok) throw new Error(`결과 읽기 실패 (${response.status})`);
  run = await response.json();
  if (run.schema_version !== 'phone-sfm-result/1.0' || !run.points?.length || !run.cameras?.length)
    throw new Error('지원하는 점군과 카메라 경로가 없습니다.');
  const q = run.quality;
  $('run-name').textContent = `${run.input_run_id} · 휴대폰 영상 ${run.cameras[0].source_pts_s.toFixed(1)}–${run.cameras.at(-1).source_pts_s.toFixed(1)}초 구간`;
  $('run-date').textContent = new Date(run.created_utc).toLocaleDateString('ko-KR');
  $('frames').textContent = `${q.registered_frames} / ${q.input_frames}`;
  $('registration').textContent = `${Math.round(q.registration_ratio * 100)}% 등록 · ${q.excluded_frames.length}장 제외`;
  $('reprojection').textContent = `${q.mean_reprojection_error_px.toFixed(2)} px`;
  $('elapsed').textContent = `${q.processing_seconds.toFixed(1)} s`;
  $('scale-badge').textContent = run.scale.unit === 'm' ? '44cm 기준 스케일' : '임의 스케일';
  $('scale-note').textContent = run.scale.unit === 'm' ?
    '같은 문 위쪽의 두 끝점 A–B를 44cm에 맞췄습니다. 격자와 경로는 같은 미터 좌표계를 사용합니다. 이 길이는 보정 기준이며 독립적인 정확도 검증값은 아닙니다.' :
    `스케일을 보정하지 못했습니다. 단위는 임의 스케일입니다. ${run.scale.reason || ''}`;

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor(0x101d22);
  $('viewport').replaceChildren(renderer.domElement);
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(48, 1, 0.01, 10000);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  cloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({
    vertexColors: true, size: 2.5, sizeAttenuation: false }));
  scene.add(cloud);
  showCloud();
  const bounds = new THREE.Box3().setFromBufferAttribute(cloud.geometry.getAttribute('position'));
  fullCenter = bounds.getCenter(new THREE.Vector3());
  fullRadius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 0.1);
  // Fit the central structure initially; the full unfiltered cloud stays rendered/exported.
  const quantiles = [1, 2, 3].map((axis) => run.points.map((p) => p[axis]).sort((a, b) => a - b));
  const focus = new THREE.Box3(new THREE.Vector3(...quantiles.map((a) => a[Math.floor(a.length * .02)])),
    new THREE.Vector3(...quantiles.map((a) => a[Math.floor(a.length * .98)])));
  center = focus.getCenter(new THREE.Vector3());
  radius = Math.max(focus.getSize(new THREE.Vector3()).length() / 2, 0.1);
  const positions = run.cameras.map((c) => new THREE.Vector3().fromArray(c.position));
  cameraDirection = positions.reduce((sum, p) => sum.add(p), new THREE.Vector3()).divideScalar(positions.length).sub(center);
  cameraDirection.normalize().y += 0.12;
  route = line(positions, 0xc6bb67, 0.85);
  scene.add(route);
  frustums = new THREE.Group();
  const d = radius * 0.035;
  for (const c of run.cameras) {
    const rot = frameRotation(c), origin = new THREE.Vector3().fromArray(c.position);
    const local = [[-1, -.5625, 1.5], [1, -.5625, 1.5], [1, .5625, 1.5], [-1, .5625, 1.5]];
    const corners = local.map((p) => new THREE.Vector3(...p).multiplyScalar(d).applyMatrix3(rot).add(origin));
    frustums.add(line([...corners, corners[0]], 0x6c8d89, 0.55));
    for (const p of corners) frustums.add(line([origin, p], 0x6c8d89, 0.45));
  }
  scene.add(frustums);
  selected = new THREE.Mesh(new THREE.SphereGeometry(radius * 0.012, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xe3ce71, depthTest: false }));
  selected.renderOrder = 2;
  scene.add(selected);
  const gridSize = run.scale.unit === 'm' ? Math.max(2, Math.ceil(fullRadius * 3)) : fullRadius * 3;
  grid = new THREE.Group();
  const plane = new THREE.GridHelper(gridSize, run.scale.unit === 'm' ? gridSize * 2 : 20, 0x34504e, 0x243b3f);
  plane.position.set(center.x, bounds.min.y - radius * 0.025, center.z);
  grid.add(plane, new THREE.AxesHelper(radius * 0.3));
  scene.add(grid);
  if (run.scale.anchors) {
    scene.add(line([new THREE.Vector3(...run.scale.anchors.A), new THREE.Vector3(...run.scale.anchors.B)], 0xc1e6a2));
  }
  $('timeline').max = run.cameras.length - 1;
  selectFrame(Math.floor(run.cameras.length / 2));
  setView('perspective');
  new ResizeObserver(resize).observe($('viewport'));
  resize();
  $('timeline').addEventListener('input', (e) => selectFrame(Number(e.target.value)));
  $('play').addEventListener('click', () => {
    playing = !playing;
    $('play').textContent = playing ? 'Ⅱ 일시정지' : '▶ 재생';
    $('play').setAttribute('aria-label', playing ? '촬영 경로 일시정지' : '촬영 경로 재생');
  });
  $('point-size').addEventListener('input', (e) => { cloud.material.size = Number(e.target.value); });
  $('color-mode').addEventListener('change', showCloud);
  $('error-limit').addEventListener('change', showCloud);
  $('show-path').addEventListener('change', (e) => { route.visible = e.target.checked; selected.visible = e.target.checked; });
  $('show-cameras').addEventListener('change', (e) => { frustums.visible = e.target.checked; });
  $('show-grid').addEventListener('change', (e) => { grid.visible = e.target.checked; });
  document.querySelectorAll('[data-view]').forEach((button) => button.addEventListener('click', () => setView(button.dataset.view)));
  $('reset').addEventListener('click', () => setView('perspective', true));
  $('camera-view').addEventListener('click', () => {
    const c = run.cameras[currentIndex], rot = frameRotation(c);
    camera.position.fromArray(c.position);
    camera.up.set(0, -1, 0).applyMatrix3(rot);
    camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(c.height / (2 * c.params[0])));
    camera.updateProjectionMatrix();
    controls.target.copy(camera.position).add(new THREE.Vector3(0, 0, 1).applyMatrix3(rot).multiplyScalar(radius));
    controls.update();
    document.querySelectorAll('[data-view]').forEach((button) => button.classList.remove('active'));
  });
  renderer.domElement.addEventListener('webglcontextlost', () => showError(new Error('그래픽 연결이 중단되었습니다. 페이지를 새로고침해 주세요.')));
  requestAnimationFrame(animate);
}

main().catch(showError);
