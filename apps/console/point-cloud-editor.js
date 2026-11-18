import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
import {modelEvents, requestModel} from './reconstruction-models.js';

const number = new Intl.NumberFormat('ko-KR');
const roomformModel = 'roomform';
const kinds = [
  {key: 'floor', color: '#94c5a3', opacity: .71},
  {key: 'wall', color: '#74cbd5', opacity: .71},
  {key: 'ceiling', color: '#e9bc73', opacity: .33},
];
const errors = {
  roomform_busy: '다른 지도를 복원하는 중입니다. 완료된 뒤 다시 시도하세요.',
  roomform_runtime_missing: '프로젝트 실행기로 Python 환경을 준비한 뒤 다시 시도하세요.',
  roomform_source_unavailable: '저장된 점군을 찾을 수 없습니다. 지도를 다시 선택하세요.',
  roomform_scene_changed: '저장 지도가 변경됐습니다. 목록에서 지도를 다시 선택하세요.',
  roomform_too_few_points: '이 지도에는 구조를 복원할 점이 충분하지 않습니다.',
  roomform_no_height: '점군에서 높이 범위를 확인할 수 없습니다.',
  roomform_no_surfaces: '이 점군에서 벽이나 바닥 면을 찾지 못했습니다.',
  roomform_out_of_memory: '복원에 사용할 메모리가 부족합니다. 다른 복원 작업을 끝낸 뒤 다시 시도하세요.',
};

function createViewport(host, cloud) {
  let renderer;
  try { renderer = new THREE.WebGLRenderer({antialias: true, stencil: true}); }
  catch { throw new Error('WebGL을 사용할 수 없어 3D 지도를 표시하지 못했습니다.'); }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.setClearColor(0x101921);
  const canvas = renderer.domElement;
  canvas.tabIndex = 0;
  canvas.setAttribute('aria-label', '3D 지도');
  host.replaceChildren(canvas);

  const scene = new THREE.Scene();
  const root = new THREE.Group();
  const shell = new THREE.Group();
  scene.add(root);
  root.add(shell);
  const min = new THREE.Vector3(...cloud.min), max = new THREE.Vector3(...cloud.max);
  const center = min.clone().lerp(max, .5);
  const radius = Math.max(center.distanceTo(max), .001);
  let fitWidth = radius * 2, fitHeight = radius * 2;
  const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, radius * .001, radius * 12);
  const controls = new OrbitControls(camera, canvas);
  const draw = () => renderer.render(scene, camera);
  controls.addEventListener('change', draw);

  const positions = new Float32Array(cloud.points.length * 3);
  const colors = new Float32Array(positions.length);
  const rgb = new THREE.Color();
  cloud.points.forEach(([x, y, z, r, g, b], i) => {
    const offset = i * 3;
    positions.set([x - center.x, y - center.y, z - center.z], offset);
    rgb.setRGB(r / 255, g / 255, b / 255, THREE.SRGBColorSpace).toArray(colors, offset);
  });
  const pointGeometry = new THREE.BufferGeometry()
    .setAttribute('position', new THREE.BufferAttribute(positions, 3))
    .setAttribute('color', new THREE.BufferAttribute(colors, 3));
  const points = new THREE.Points(pointGeometry, new THREE.PointsMaterial({
    size: 2.6, sizeAttenuation: false, vertexColors: true, transparent: true,
  }));
  root.add(points);

  function align(up) {
    const direction = new THREE.Vector3(...up);
    if (direction.lengthSq() === 0) direction.set(0, 1, 0);
    root.quaternion.setFromUnitVectors(direction.normalize(), new THREE.Vector3(0, 1, 0));
  }
  function fit(top = false) {
    camera.zoom = 1;
    camera.position.set(...(top ? [0, 1, .0001] : [.47, .70, -.53])).setLength(radius * 3);
    controls.target.set(0, 0, 0);
    controls.update();
    camera.updateMatrixWorld();
    const point = new THREE.Vector3();
    let left = Infinity, right = -Infinity, bottom = Infinity, topEdge = -Infinity;
    for (let offset = 0; offset < positions.length; offset += 3) {
      point.fromArray(positions, offset).applyQuaternion(root.quaternion).applyMatrix4(camera.matrixWorldInverse);
      left = Math.min(left, point.x); right = Math.max(right, point.x);
      bottom = Math.min(bottom, point.y); topEdge = Math.max(topEdge, point.y);
    }
    fitWidth = Math.max(right - left, radius * .01);
    fitHeight = Math.max(topEdge - bottom, radius * .01);
    point.set((left + right) / 2, (bottom + topEdge) / 2, 0).applyQuaternion(camera.quaternion);
    camera.position.add(point);
    controls.target.add(point);
    controls.update();
    resize();
  }
  function resize() {
    const width = host.clientWidth, height = host.clientHeight;
    if (!width || !height) return;
    const aspect = width / height, span = Math.max(fitHeight, fitWidth / aspect) * .56;
    camera.right = span * aspect;
    camera.left = -camera.right;
    camera.top = span;
    camera.bottom = -camera.top;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    draw();
  }
  const observer = new ResizeObserver(resize);
  observer.observe(host);
  align(cloud.up);
  fit();

  function clearShell() {
    for (const mesh of [...shell.children]) {
      mesh.geometry.dispose();
      mesh.material.dispose();
      shell.remove(mesh);
    }
  }

  return {
    fit,
    showResult(result) {
      if (result?.schema_version !== 'roomform-surface/1.0' || !result.surfaces) throw new Error('Roomform 결과를 읽을 수 없습니다.');
      const offset = new THREE.Vector3(...result.origin).sub(center).toArray();
      clearShell();
      for (const [index, kind] of kinds.entries()) {
        const surface = result.surfaces[kind.key];
        if (!surface) continue;
        const bytes = Uint8Array.from(atob(surface.positions), char => char.charCodeAt(0));
        const data = new DataView(bytes.buffer);
        if (data.byteLength !== surface.quads * 48) throw new Error('Roomform 면 데이터의 크기가 일치하지 않습니다.');
        const vertices = new Float32Array(data.byteLength / 4);
        for (let i = 0; i < vertices.length; i++) vertices[i] = data.getFloat32(i * 4, true) + offset[i % 3];
        const indices = new Uint32Array(surface.quads * 6);
        for (let i = 0; i < surface.quads; i++) {
          const j = i * 4;
          indices.set([j, j + 1, j + 2, j, j + 2, j + 3], i * 6);
        }
        const geometry = new THREE.BufferGeometry()
          .setAttribute('position', new THREE.BufferAttribute(vertices, 3))
          .setIndex(new THREE.BufferAttribute(indices, 1));
        const material = new THREE.MeshBasicMaterial({
          color: kind.color, opacity: kind.opacity, transparent: true, side: THREE.DoubleSide,
          depthTest: false, depthWrite: false, forceSinglePass: true,
          stencilWrite: true, stencilRef: index + 1, stencilFunc: THREE.NotEqualStencilFunc,
          stencilZPass: THREE.ReplaceStencilOp,
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = kind.key;
        mesh.renderOrder = index + 1;
        shell.add(mesh);
      }
      align(result.up);
      fit();
    },
    show(mode, overlay, ceiling) {
      shell.visible = mode === 'roomform';
      points.visible = !shell.visible || overlay;
      points.material.opacity = shell.visible ? .38 : 1;
      const roof = shell.getObjectByName('ceiling');
      if (roof) roof.visible = ceiling;
      canvas.setAttribute('aria-label', shell.visible ? 'Roomform 3D 구조 지도' : '원본 RGB 점군');
      host.dataset.view = mode;
      draw();
    },
    dispose() {
      observer.disconnect();
      controls.dispose();
      clearShell();
      pointGeometry.dispose();
      points.material.dispose();
      renderer.dispose();
      renderer.forceContextLoss();
      canvas.remove();
    },
  };
}

async function requestRoomform(query, {signal, method = 'GET', timeout = 15000} = {}) {
  const deadline = AbortSignal.timeout(timeout);
  const response = await fetch('/api/saved/roomform?' + new URLSearchParams(query), {
    cache: 'no-store',
    method,
    signal: signal ? AbortSignal.any([signal, deadline]) : deadline,
    ...(method === 'POST' ? {headers: {'X-Mapping-Client': '1'}} : {}),
  });
  const value = await response.json();
  if (!response.ok || value.status === 'failed') {
    throw new Error(errors[value.error] || 'Roomform 복원에 실패했습니다. 다시 시도하세요.', {cause: value.error});
  }
  return value;
}

function cancelRoomform(id) {
  return requestRoomform({action: 'cancel', id}, {method: 'POST', timeout: 60000});
}

export function connectRoomformViewer() {
  const $ = id => document.getElementById('editor-' + id);
  const panel = document.getElementById('point-editor');
  const placeholder = document.getElementById('edit-placeholder');
  let current, view, pending, result, mode = 'original';
  let activeJobId, cancellation;
  let modelRequest, modelStream, modelState;

  function showModel({status, percent = 0}) {
    const wasDownloading = modelState === 'downloading';
    modelState = status;
    const button = $('restore'), progress = $('progress');
    button.disabled = status === 'checking' || status === 'downloading';
    progress.hidden = status !== 'downloading';
    if (status === 'ready') {
      button.textContent = 'AI 복원하기';
      $('status').textContent = wasDownloading ? 'AI 복원모델 준비 완료' : '';
    } else if (status === 'downloading') {
      button.textContent = 'AI 복원모델 다운로드 중';
      progress.max = 100;
      progress.value = percent;
      $('status').textContent = `다운로드 중 ${percent}%`;
    } else if (status === 'checking') {
      button.textContent = 'AI 복원모델 확인 중';
      $('status').textContent = '';
    } else if (status === 'missing' || status === 'failed') {
      button.textContent = 'AI 복원모델 다운받기';
      $('status').textContent = status === 'failed' ? '다운로드에 실패했습니다. 인터넷과 저장 공간을 확인하세요.' : '';
    } else {
      button.textContent = 'AI 복원모델 다시 확인';
      $('status').textContent = '모델 상태를 확인할 수 없습니다.';
    }
  }

  function stopModelCheck() {
    modelRequest?.abort();
    modelStream?.close();
    modelRequest = modelStream = null;
  }

  async function checkModel(action) {
    stopModelCheck();
    const controller = new AbortController();
    modelRequest = controller;
    const active = () => modelRequest === controller;
    showModel({status: 'checking'});
    try {
      const state = await requestModel(roomformModel, action, controller.signal);
      if (!active()) return;
      showModel(state);
      if (state.status !== 'downloading') return;
      const stream = modelEvents(roomformModel);
      modelStream = stream;
      stream.onmessage = event => {
        if (!active()) return;
        const next = JSON.parse(event.data);
        showModel(next);
        if (next.status !== 'downloading') stopModelCheck();
      };
      stream.onerror = () => {
        if (!active()) return;
        stopModelCheck();
        showModel({status: 'blocked'});
      };
    } catch {
      if (active()) showModel({status: 'blocked'});
    }
  }

  function show(next) {
    mode = next;
    const restored = mode === 'roomform';
    $('original').setAttribute('aria-pressed', String(!restored));
    $('roomform-view').setAttribute('aria-pressed', String(restored));
    $('layers').hidden = !restored;
    $('ceiling-label').hidden = !result?.surfaces.ceiling;
    view.show(mode, $('overlay').checked, $('ceiling').checked);
  }

  function close() {
    stopModelCheck();
    pending?.abort();
    pending = null;
    if (activeJobId) {
      const id = activeJobId;
      activeJobId = null;
      cancellation = {id, done: cancelRoomform(id).then(() => true, () => false)};
    }
    current = result = null;
    view?.dispose();
    view = null;
    panel.hidden = true;
    placeholder.hidden = false;
  }

  $('restore').addEventListener('click', async () => {
    if (!current || pending) return;
    if (modelState !== 'ready') {
      checkModel(modelState === 'missing' || modelState === 'failed' ? 'download' : 'status');
      return;
    }
    const controller = new AbortController();
    const {signal} = controller;
    pending = controller;
    $('restore').disabled = true;
    $('restore').textContent = 'AI 복원 중';
    $('progress').hidden = false;
    $('progress').removeAttribute('value');
    try {
      if (cancellation) {
        const previous = cancellation;
        $('status').textContent = '이전 영상의 AI 복원 취소 중';
        if (!await previous.done) {
          signal.throwIfAborted();
          await cancelRoomform(previous.id);
        }
        if (cancellation === previous) cancellation = null;
      }
      signal.throwIfAborted();
      $('status').textContent = 'Roomform 준비 중';
      activeJobId ??= crypto.randomUUID().replaceAll('-', '');
      const query = {id: activeJobId};
      if (current.asset) query.asset = current.asset;
      let job = await requestRoomform(query, {signal, method: 'POST'});
      while (job.status === 'running' || job.status === 'cancelling') {
        signal.throwIfAborted();
        if (job.tiles_total) {
          $('progress').max = job.tiles_total;
          $('progress').value = job.tiles_done;
          $('status').textContent = '구조 복원 중 · ' + job.tiles_done + '/' + job.tiles_total + ' 구역';
        }
        await new Promise(resolve => setTimeout(resolve, 600));
        signal.throwIfAborted();
        job = await requestRoomform({id: job.id}, {signal});
      }
      if (job.status === 'complete' && !job.result) {
        // A cached POST returns job metadata only, like a newly started job.
        job = await requestRoomform({id: job.id}, {signal});
      }
      signal.throwIfAborted();
      activeJobId = null;
      if (job.status === 'cancelled') {
        $('status').textContent = 'AI 복원 취소됨';
        return;
      }
      view.showResult(job.result);
      result = job.result;
      $('roomform-view').hidden = false;
      $('status').textContent = result.scale_estimated ? '복원 완료 · 입력 크기 추정 사용' : '복원 완료';
      show('roomform');
    } catch (error) {
      if (!signal.aborted) {
        if (error.cause) activeJobId = null;
        if (error.cause === 'roomform_checkpoint_missing') modelState = 'missing';
        else $('status').textContent = error.name === 'TimeoutError'
          ? '응답이 지연됐습니다. AI 복원하기를 눌러 작업 상태를 다시 확인하세요.' : error.message;
      }
    } finally {
      if (pending === controller) {
        pending = null;
        if (modelState === 'missing') showModel({status: 'missing'});
        else {
          $('restore').disabled = false;
          $('restore').textContent = 'AI 복원하기';
          $('progress').hidden = true;
        }
      }
    }
  });
  $('original').addEventListener('click', () => { if (view) show('original'); });
  $('roomform-view').addEventListener('click', () => { if (result) show('roomform'); });
  $('fit').addEventListener('click', () => view?.fit());
  $('top').addEventListener('click', () => view?.fit(true));
  $('overlay').addEventListener('change', () => show(mode));
  $('ceiling').addEventListener('change', () => show(mode));

  return {
    get key() { return current?.record.key; },
    close,
    showOriginal() { if (view) show('original'); },
    open(record, cloud, asset) {
      close();
      panel.hidden = false;
      placeholder.hidden = true;
      try {
        view = createViewport($('viewport'), cloud);
      } catch (error) {
        close();
        throw error;
      }
      current = {record, asset};
      $('name').textContent = record.name;
      $('unit').textContent = '원본 점 ' + number.format(cloud.points.length) + '개 · ' +
        (cloud.unit === 'm' ? 'm' : '미확정 스케일');
      $('roomform-view').hidden = $('progress').hidden = true;
      $('status').textContent = '';
      $('overlay').checked = true;
      $('ceiling').checked = false;
      show('original');
      checkModel('status');
    },
  };
}
