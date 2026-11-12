import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
import {cloudPly, pointEdits} from './editor-data.js';

const number = new Intl.NumberFormat('ko-KR');

function createViewport(host, rectangle, cloud, edits, onSelect) {
  let renderer;
  try { renderer = new THREE.WebGLRenderer({antialias: true}); }
  catch { throw new Error('WebGL을 사용할 수 없어 점군을 표시하지 못했습니다.'); }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x152126);
  const canvas = renderer.domElement;
  canvas.tabIndex = 0;
  canvas.setAttribute('aria-label', '편집할 3D 점군');
  host.prepend(canvas);

  const scene = new THREE.Scene();
  const center = new THREE.Vector3(...cloud.min).lerp(new THREE.Vector3(...cloud.max), .5);
  const radius = Math.max(new THREE.Vector3(...cloud.max).distanceTo(center), .001);
  const camera = new THREE.PerspectiveCamera(46, 1, radius * .001, radius * 200);
  camera.up.set(...cloud.up);
  if (!camera.up.lengthSq()) camera.up.set(0, 1, 0);
  camera.up.normalize();
  const controls = new OrbitControls(camera, canvas);
  const draw = () => renderer.render(scene, camera);
  controls.addEventListener('change', draw);

  const positions = new Float32Array(cloud.points.length * 3);
  cloud.points.forEach((point, i) => {
    positions[i * 3] = point[0] - center.x;
    positions[i * 3 + 1] = point[1] - center.y;
    positions[i * 3 + 2] = point[2] - center.z;
  });
  const position = new THREE.BufferAttribute(positions, 3);
  const color = new THREE.Color();
  const colors = new THREE.BufferAttribute(new Float32Array(positions.length), 3).setUsage(THREE.DynamicDrawUsage);
  const index = new THREE.BufferAttribute(new Uint32Array(cloud.points.length), 1).setUsage(THREE.DynamicDrawUsage);
  const selectionIndex = new THREE.BufferAttribute(new Uint32Array(cloud.points.length), 1).setUsage(THREE.DynamicDrawUsage);
  const geometry = new THREE.BufferGeometry().setAttribute('position', position).setAttribute('color', colors).setIndex(index);
  const selectedGeometry = new THREE.BufferGeometry().setAttribute('position', position).setIndex(selectionIndex);
  geometry.setDrawRange(0, 0);
  selectedGeometry.setDrawRange(0, 0);
  const points = new THREE.Points(geometry, new THREE.PointsMaterial({size: 3, sizeAttenuation: false, vertexColors: true}));
  const highlight = new THREE.Points(selectedGeometry, new THREE.PointsMaterial({
    color: 0xffc471, size: 7, sizeAttenuation: false, depthWrite: false,
  }));
  highlight.renderOrder = -1;
  scene.add(points, highlight);

  function fit() {
    const distance = radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) / Math.min(1, camera.aspect);
    camera.position.set(.8, .65, 1).setLength(distance);
    controls.target.set(0, 0, 0);
    controls.update();
    draw();
  }
  function resize() {
    if (!host.clientWidth || !host.clientHeight) return;
    camera.aspect = host.clientWidth / host.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(host.clientWidth, host.clientHeight, false);
    draw();
  }
  const observer = new ResizeObserver(resize);
  observer.observe(host);

  let gesture;
  const pointer = event => {
    const bounds = canvas.getBoundingClientRect();
    return {
      x: Math.max(0, Math.min(bounds.width, event.clientX - bounds.left)),
      y: Math.max(0, Math.min(bounds.height, event.clientY - bounds.top)),
    };
  };
  function selectionRect(event) {
    const end = pointer(event);
    return {left: Math.min(gesture.x, end.x), top: Math.min(gesture.y, end.y),
      width: Math.abs(gesture.x - end.x), height: Math.abs(gesture.y - end.y)};
  }
  function cancelSelection() {
    const id = gesture?.id;
    gesture = null;
    rectangle.hidden = true;
    if (id !== undefined && canvas.hasPointerCapture(id)) canvas.releasePointerCapture(id);
  }
  canvas.addEventListener('pointerdown', event => {
    if (host.dataset.tool !== 'box' || !event.isPrimary || event.button !== 0) return;
    gesture = {...pointer(event), id: event.pointerId};
    canvas.focus({preventScroll: true});
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
  });
  canvas.addEventListener('pointermove', event => {
    if (gesture?.id !== event.pointerId) return;
    const bounds = selectionRect(event);
    Object.assign(rectangle.style, Object.fromEntries(Object.entries(bounds).map(([key, value]) => [key, `${value}px`])));
    rectangle.hidden = false;
  });
  canvas.addEventListener('pointerup', event => {
    if (gesture?.id !== event.pointerId) return;
    const {left, top, width, height} = selectionRect(event);
    cancelSelection();
    const bounds = canvas.getBoundingClientRect();
    const matrix = new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    const projected = new THREE.Vector3();
    const padding = Math.max(width, height) < 4 ? 5 : 0;
    edits.select(i => {
      projected.fromBufferAttribute(position, i).applyMatrix4(matrix);
      if (projected.z < -1 || projected.z > 1 || Math.abs(projected.x) > 1 || Math.abs(projected.y) > 1) return false;
      const x = (projected.x + 1) * bounds.width / 2, y = (1 - projected.y) * bounds.height / 2;
      return x >= left - padding && x <= left + width + padding && y >= top - padding && y <= top + height + padding;
    }, event.shiftKey);
    onSelect();
  });
  canvas.addEventListener('pointercancel', cancelSelection);
  canvas.addEventListener('lostpointercapture', cancelSelection);
  resize();
  fit();

  return {
    fit,
    setTool(tool) {
      cancelSelection();
      host.dataset.tool = tool;
      controls.enabled = tool === 'orbit';
    },
    update() {
      let visible = 0, highlighted = 0;
      for (let i = 0; i < cloud.points.length; i++) {
        if (edits.removed[i]) continue;
        index.array[visible++] = i;
        if (edits.selected[i]) selectionIndex.array[highlighted++] = i;
        color.setHex(edits.colors[i]).toArray(colors.array, i * 3);
      }
      colors.needsUpdate = index.needsUpdate = selectionIndex.needsUpdate = true;
      geometry.setDrawRange(0, visible);
      selectedGeometry.setDrawRange(0, highlighted);
      draw();
    },
    dispose() {
      cancelSelection();
      observer.disconnect();
      controls.dispose();
      for (const object of [points, highlight]) {
        object.geometry.dispose();
        object.material.dispose();
      }
      renderer.dispose();
      renderer.forceContextLoss();
      canvas.remove();
    },
  };
}

export function connectPointCloudEditor() {
  const $ = id => document.getElementById('editor-' + id);
  const panel = document.getElementById('point-editor');
  const placeholder = document.getElementById('edit-placeholder');
  const drafts = new Map();
  let current, view;

  function update() {
    const {edits} = current;
    $('count').textContent = `점 ${number.format(edits.count)}개 · 선택 ${number.format(edits.selectionCount)}개`;
    $('select-all').disabled = edits.selectionCount === edits.count;
    $('clear').disabled = $('paint').disabled = !edits.selectionCount;
    $('delete').disabled = !edits.selectionCount;
    $('keep').disabled = !edits.selectionCount || edits.selectionCount === edits.count;
    $('export').disabled = !edits.count;
    $('undo').disabled = !edits.canUndo;
    $('redo').disabled = !edits.canRedo;
    view.update();
  }

  function setTool(tool) {
    view.setTool(tool);
    $('orbit').setAttribute('aria-pressed', String(tool === 'orbit'));
    $('box').setAttribute('aria-pressed', String(tool === 'box'));
    $('guide').textContent = tool === 'box'
      ? '사각형 안의 앞뒤 점을 함께 선택합니다. Shift를 누르면 선택을 더합니다.'
      : '끌어서 회전 · 오른쪽 버튼을 끌어 이동 · 휠로 확대';
  }

  function change(edit) {
    const {key, edits} = current;
    if (edit(edits)) drafts.set(key, current);
    update();
  }

  $('orbit').addEventListener('click', () => setTool('orbit'));
  $('box').addEventListener('click', () => setTool('box'));
  $('fit').addEventListener('click', () => view.fit());
  $('select-all').addEventListener('click', () => { current.edits.select(() => true); update(); });
  $('clear').addEventListener('click', () => { current.edits.select(() => false); update(); });
  $('delete').addEventListener('click', () => change(edits => edits.remove(false)));
  $('keep').addEventListener('click', () => change(edits => edits.remove(true)));
  $('paint').addEventListener('click', () => change(edits => edits.paint(Number.parseInt($('color').value.slice(1), 16))));
  $('undo').addEventListener('click', () => { current.edits.undo(); update(); });
  $('redo').addEventListener('click', () => { current.edits.redo(); update(); });
  $('export').addEventListener('click', () => {
    const {cloud, edits, name} = current;
    const url = URL.createObjectURL(cloudPly(cloud.points, cloud.unit, edits));
    const link = document.createElement('a');
    link.href = url;
    link.download = `${name.replace(/[<>:"/\\|?*\x00-\x1f]/g, '_').slice(0, 80) || 'point-cloud'}-edited.ply`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  panel.addEventListener('keydown', event => {
    if (event.target.matches('input, textarea, select, [contenteditable]')) return;
    const key = event.key.toLowerCase();
    const modifier = event.ctrlKey || event.metaKey;
    const action = modifier ? (key === 'z' ? (event.shiftKey ? 'redo' : 'undo') : key === 'y' ? 'redo' : null)
      : {v: 'orbit', b: 'box', delete: 'delete', escape: 'clear'}[key];
    if (action && !$(action).disabled) {
      event.preventDefault();
      $(action).click();
    }
  });

  return {
    get key() { return current?.key; },
    hasDraft: key => drafts.has(key),
    open(record, cloud) {
      const next = drafts.get(record.key) || {...record, cloud, edits: pointEdits(cloud.points)};
      view?.dispose();
      view = current = null;
      panel.hidden = false;
      try { view = createViewport($('viewport'), $('selection-box'), next.cloud, next.edits, update); }
      catch (error) { placeholder.hidden = false; panel.hidden = true; throw error; }
      current = next;
      placeholder.hidden = true;
      $('name').textContent = current.name;
      $('unit').textContent = current.cloud.unit === 'm' ? '미터 단위' : '임의 단위';
      setTool('orbit');
      update();
    },
  };
}
