import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
import {cloudPly, cropBounds, cropCloud, editorCloud} from '/editor-data.js';

const $ = id => document.getElementById(id);
const query = new URLSearchParams(location.search);
const source = query.get('source');
const asset = query.get('asset');
const format = value => Number(value).toLocaleString('ko-KR');

function viewport(cloud) {
  const host = $('canvas');
  let renderer;
  try { renderer = new THREE.WebGLRenderer({antialias: true}); }
  catch { throw new Error('이 브라우저에서 3D 화면을 열 수 없습니다. WebGL을 지원하는 브라우저에서 다시 열어 주세요.'); }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x101d22);
  host.append(renderer.domElement);
  const scene = new THREE.Scene();
  const origin = new THREE.Vector3(...cloud.min).lerp(new THREE.Vector3(...cloud.max), .5);
  const radius = Math.max(new THREE.Vector3(...cloud.max).distanceTo(origin), .001);
  const toLocal = xyz => new THREE.Vector3(...xyz).sub(origin);
  const camera = new THREE.PerspectiveCamera(48, 1, radius * .001, radius * 200);
  camera.up.set(...cloud.up);
  if (!camera.up.lengthSq()) camera.up.set(0, 1, 0);
  camera.up.normalize();
  const controls = new OrbitControls(camera, renderer.domElement);
  const draw = () => renderer.render(scene, camera);
  controls.addEventListener('change', draw);

  const positions = new Float32Array(cloud.points.length * 3);
  const colors = new Float32Array(positions.length), color = new THREE.Color();
  const index = new THREE.BufferAttribute(new Uint32Array(cloud.points.length), 1).setUsage(THREE.DynamicDrawUsage);
  const indices = new Map();
  cloud.points.forEach((point, i) => {
    toLocal(point.slice(0, 3)).toArray(positions, i * 3);
    color.setRGB(point[3] / 255, point[4] / 255, point[5] / 255, THREE.SRGBColorSpace).toArray(colors, i * 3);
    indices.set(point, i);
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geometry.setIndex(index);
  scene.add(new THREE.Points(geometry, new THREE.PointsMaterial({size: 3, sizeAttenuation: false, vertexColors: true})));
  const path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(cloud.cameras.map(toLocal)),
    new THREE.LineBasicMaterial({color: 0xeac577}));
  scene.add(path, new THREE.AxesHelper(radius * .18));
  $('path').disabled = cloud.cameras.length < 2;
  $('path').addEventListener('change', () => { path.visible = $('path').checked; draw(); });
  const box = new THREE.Box3Helper(new THREE.Box3(), 0xe6be68);
  scene.add(box);

  function fit() {
    const distance = radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) / Math.min(1, camera.aspect);
    camera.position.set(.8, .65, 1).setLength(distance);
    controls.target.set(0, 0, 0);
    controls.update();
    draw();
  }

  new ResizeObserver(() => {
    if (!host.clientWidth || !host.clientHeight) return;
    camera.aspect = host.clientWidth / host.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(host.clientWidth, host.clientHeight, false);
    draw();
  }).observe(host);
  camera.aspect = host.clientWidth / Math.max(1, host.clientHeight);
  camera.updateProjectionMatrix();
  fit();
  $('fit').addEventListener('click', fit);

  return (points, bounds) => {
    points.forEach((point, i) => { index.array[i] = indices.get(point); });
    index.needsUpdate = true;
    geometry.setDrawRange(0, points.length);
    box.visible = Boolean(bounds);
    if (bounds) box.box.set(toLocal(bounds.min), toLocal(bounds.max));
    draw();
  };
}

function edit(cloud) {
  const count = $('point-count'), original = $('original'), keep = $('keep');
  const inputs = [...$('limits').querySelectorAll('input')];
  const history = [cloud.points];
  let cursor = 0, preview = cloud.points;
  $('workspace').hidden = false;
  const render = viewport(cloud);
  const sourceLabel = source === 'saved' ? '저장 복원' : '실시간 감독';
  const date = new Date(cloud.time);
  $('source-info').textContent = [sourceLabel, cloud.name,
    cloud.revision != null ? `버전 ${cloud.revision}` : '',
    cloud.time && Number.isFinite(date.getTime()) ? date.toLocaleString('ko-KR') : '',
    cloud.unit === 'm' ? '미터 단위' : '임의 단위',
    cloud.total > cloud.points.length ? `감독 화면 표본 ${format(cloud.points.length)} / ${format(cloud.total)}점` : '',
  ].filter(Boolean).join(' · ');

  function update() {
    const active = history[cursor];
    const limits = [0, 2, 4].map(i => [inputs[i].valueAsNumber, inputs[i + 1].valueAsNumber]);
    const bounds = cropBounds(cloud, limits);
    preview = bounds ? cropCloud(active, bounds, keep.value === 'inside') : active;
    render(original.checked ? cloud.points : preview, original.checked ? null : bounds);
    count.textContent = original.checked ? `원본 ${format(cloud.points.length)}점` : !bounds ? '0~100% 범위에서 최소값을 최대값 이하로 입력하세요.' :
      `적용된 점 ${format(active.length)}개 · 미리보기 ${format(preview.length)}개`;
    $('apply').disabled = original.checked || !bounds || !preview.length || preview.length === active.length;
    $('undo').disabled = cursor === 0;
    $('redo').disabled = cursor === history.length - 1;
  }

  function resetPreview() {
    inputs.forEach((input, i) => { input.value = i % 2 ? 100 : 0; });
    keep.value = 'inside';
    original.checked = false;
    update();
  }

  $('limits').addEventListener('input', update);
  [keep, original].forEach(control => control.addEventListener('change', update));
  $('apply').addEventListener('click', () => {
    history.splice(cursor + 1, history.length, preview);
    cursor++;
    resetPreview();
  });
  $('undo').addEventListener('click', () => { cursor--; resetPreview(); });
  $('redo').addEventListener('click', () => { cursor++; resetPreview(); });
  $('export').addEventListener('click', () => {
    const url = URL.createObjectURL(cloudPly(history[cursor], cloud.unit));
    const link = document.createElement('a');
    link.href = url;
    link.download = `${String(cloud.name).replace(/[^a-zA-Z0-9_-]/g, '_').slice(0, 80)}-edited.ply`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  $('export').hidden = false;
  update();
}

async function load() {
  if (source !== 'saved' && source !== 'supervisor') { $('sources').hidden = false; return; }
  const status = $('load-status');
  status.hidden = false;
  status.textContent = '점군을 불러오는 중입니다…';
  const savedQuery = asset ? `?${new URLSearchParams({asset})}` : '';
  if (asset && source === 'saved') $('back').href = `/${savedQuery}#saved`;
  try {
    const url = source === 'saved' ? `/api/saved/run${savedQuery}` : '/api/editor/supervisor';
    const response = await fetch(url, {cache: 'no-store', signal: AbortSignal.timeout(20000)});
    if (!response.ok) {
      const value = await response.json();
      const messages = {
        supervisor_gateway_disabled: '연결된 실시간 감독 게이트웨이가 없습니다. 통합 실행기에서 다시 열어 주세요.',
        supervisor_cloud_empty: '실시간 감독에 수신된 점군이 아직 없습니다.',
        supervisor_cloud_unavailable: '실시간 감독의 점군을 읽지 못했습니다. 게이트웨이 연결을 확인해 주세요.',
      };
      throw new Error(messages[value.error] || '저장된 점군을 읽지 못했습니다. 최근 복원에서 결과를 다시 열어 주세요.');
    }
    edit(editorCloud(await response.json(), source));
    status.hidden = true;
  } catch (error) {
    $('workspace').hidden = true;
    status.textContent = error instanceof DOMException ? '점군을 불러오지 못했습니다. 잠시 후 다시 시도해 주세요.' : error.message;
    $('sources').hidden = false;
  }
}

load();
