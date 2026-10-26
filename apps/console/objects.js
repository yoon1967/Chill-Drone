import * as THREE from 'three';

const $ = (id) => document.getElementById(id);
const vector = (p) => new THREE.Vector3(...p);
const svgNS = 'http://www.w3.org/2000/svg';
const isWide = (o) => o.shape === 'horizontal' || (!o.shape && o.width / o.height > 1.6) || o.plane_orientation === 'nonparallel';

function hologramMaterial(object) {
  return new THREE.ShaderMaterial({
    uniforms: {time: {value: 0}, tint: {value: new THREE.Color(isWide(object) ? 0xff8c80 : 0x72edff)},
      cells: {value: new THREE.Vector2(object.width, object.height).multiplyScalar(24 / Math.max(object.width, object.height))}},
    vertexShader: 'varying vec2 patchUv; void main() { patchUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
    fragmentShader: `varying vec2 patchUv;
      uniform float time;
      uniform vec3 tint;
      uniform vec2 cells;
      void main() {
        vec2 grid = fract(patchUv * cells);
        float line = max(step(.96, grid.x), step(.96, grid.y));
        float scan = .5 + .5 * sin(patchUv.y * 45.0 - time * 1.8);
        gl_FragColor = vec4(tint, .22 + .18 * line + .04 * scan);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`,
    side: THREE.DoubleSide, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
  });
}

export class PlanarObjects {
  constructor(renderer, camera, controls, onFrame, stopExplore) {
    Object.assign(this, {renderer, camera, controls, onFrame, stopExplore});
    this.objects = [];
    this.meshes = [];
    this.group = new THREE.Group();
    this.mode = 'objects';
    this.down = null;
    renderer.domElement.addEventListener('pointerdown', (e) => { this.down = [e.clientX, e.clientY]; });
    renderer.domElement.addEventListener('pointerup', (e) => {
      if (!this.down || Math.hypot(e.clientX - this.down[0], e.clientY - this.down[1]) > 5 || !this.group.visible) return;
      const bounds = renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2((e.clientX - bounds.left) / bounds.width * 2 - 1,
        -(e.clientY - bounds.top) / bounds.height * 2 + 1), camera);
      const hit = ray.intersectObjects(this.meshes.filter((mesh) => mesh.visible))[0];
      if (hit) this.select(hit.object.userData.objectId);
    });
    $('object-focus').addEventListener('click', () => this.focus());
    $('object-overview').addEventListener('click', () => this.focusAll());
    $('object-source').addEventListener('click', () => {
      const object = this.current();
      if (object) this.onFrame(object.source_frame_id, true);
    });
    $('object-evidence').addEventListener('change', () => this.style());
    $('show-objects').addEventListener('change', () => this.style());
  }

  current() { return this.objects.find((o) => o.object_id === this.selectedId); }

  apply(collection, run) {
    const oldSelection = this.selectedId;
    this.run = run;
    this.objects = [...(collection?.objects || [])].sort((a, b) => b.height / b.width > 1.6 && a.height / a.width <= 1.6 ? 1 :
      a.height / a.width > 1.6 && b.height / b.width <= 1.6 ? -1 : b.area - a.area);
    this.group = new THREE.Group();
    this.meshes = [];
    this.visuals = [];
    $('object-list').replaceChildren();
    $('object-labels').replaceChildren();
    $('object-count').textContent = `${this.objects.length}개`;
    $('object-status').textContent = this.objects.length ?
      `세로 ${this.objects.filter((o) => o.height / o.width > 1.6).length}개 · 가로 ${this.objects.filter((o) => o.width / o.height > 1.6).length}개 · 여러 시점에서 확인한 평면` :
      collection?.statistics?.reason === 'object_update_failed' ? '이번 지도에서 객체를 갱신하지 못했습니다.' : '영상 경계와 특징점이 충분한 평면을 기다립니다.';
    $('object-empty').hidden = Boolean(this.objects.length);
    const loader = new THREE.TextureLoader();
    for (const [index, o] of this.objects.entries()) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(o.corners.flat(), 3));
      g.setAttribute('uv', new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
      g.setIndex([0, 1, 2, 0, 2, 3]);
      g.computeVertexNormals();
      const query = run.stream ? `?revision=${String(run.revision).padStart(6, '0')}&session=${encodeURIComponent(run.stream.session_id)}` : '';
      const texture = loader.load(`/api/${run.source}/object-texture/${encodeURIComponent(o.texture_file)}${query}`, undefined, undefined, () => {
        if (this.run === run) $('object-status').textContent = '일부 영상 표면을 읽지 못했습니다. 경계와 특징점으로 확인하세요.';
      });
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = Math.min(8, this.renderer.capabilities.getMaxAnisotropy());
      const mesh = new THREE.Mesh(g, new THREE.MeshBasicMaterial({map: texture, side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1}));
      mesh.userData.objectId = o.object_id;
      mesh.name = o.object_id;
      const hologram = new THREE.Mesh(g.clone(), hologramMaterial(o));
      hologram.userData.objectId = o.object_id;
      hologram.renderOrder = 1;
      const outline = new THREE.Group();
      const thickness = Math.max(o.width, o.height) * .0028;
      for (let i = 0; i < 4; i++) {
        const a = vector(o.corners[i]), b = vector(o.corners[(i + 1) % 4]);
        const edge = new THREE.Mesh(new THREE.CylinderGeometry(thickness, thickness, a.distanceTo(b), 6),
          new THREE.MeshBasicMaterial({color: 0xbe75f5, depthTest: false}));
        edge.position.copy(a).add(b).multiplyScalar(.5);
        edge.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
        edge.renderOrder = 4;
        outline.add(edge);
        const corner = new THREE.Mesh(new THREE.SphereGeometry(thickness * 2.4, 10, 8),
          new THREE.MeshBasicMaterial({color: 0xe8b7ff, depthTest: false}));
        corner.position.copy(a);
        corner.renderOrder = 5;
        outline.add(corner);
      }
      const supportGeometry = new THREE.BufferGeometry();
      supportGeometry.setAttribute('position', new THREE.Float32BufferAttribute(o.support_points.flat(), 3));
      const support = new THREE.Points(supportGeometry, new THREE.PointsMaterial({color: 0xff725e, size: 5,
        sizeAttenuation: false, depthTest: false}));
      support.renderOrder = 6;
      const button = document.createElement('button');
      button.className = 'object-card';
      button.classList.toggle('wide-plane', isWide(o));
      button.setAttribute('aria-pressed', 'false');
      const title = document.createElement('strong');
      title.textContent = `${String(index + 1).padStart(2, '0')} · ${o.label}`;
      const detail = document.createElement('span');
      detail.textContent = `${this.dimensions(o)} · ${o.support_count}개 특징점${o.mapping_status ? ` · ${o.mapping_status === 'new' ? '새 면' : '유지한 면'}` : ''}`;
      button.append(title, detail);
      button.addEventListener('click', () => { this.select(o.object_id); this.focus(); });
      $('object-list').append(button);
      const label = document.createElement('button');
      label.className = 'object-label';
      label.classList.toggle('wide-plane', isWide(o));
      label.textContent = `${String(index + 1).padStart(2, '0')} ${o.label}`;
      label.addEventListener('click', () => this.select(o.object_id));
      $('object-labels').append(label);
      this.group.add(mesh, hologram, outline, support);
      this.meshes.push(mesh, hologram);
      this.visuals.push({object: o, mesh, hologram, outline, support, button, label});
    }
    this.select(this.objects.some((o) => o.object_id === oldSelection) ? oldSelection : this.objects[0]?.object_id);
    return this.group;
  }

  dimensions(object) {
    return `${object.width.toFixed(2)} × ${object.height.toFixed(2)} ${this.run.scale.unit === 'm' ? 'm' : '임의 단위'}`;
  }

  select(id) {
    this.selectedId = id;
    const o = this.current();
    $('object-detail').hidden = !o;
    if (o) {
      $('object-title').textContent = o.label;
      $('object-dimensions').textContent = this.dimensions(o);
      $('object-score').textContent = `${Math.round(o.geometric_score * 100)} / 100`;
      $('object-support').textContent = `${o.support_count}개 특징점 · ${o.evidence.length}개 시점`;
      $('object-residual').textContent = this.run.scale.unit === 'm' ? `${(o.plane_rms * 1000).toFixed(1)} mm` : `${o.plane_rms.toPrecision(2)} 임의 단위`;
      const index = this.run.cameras.findIndex((c) => c.frame_id === o.source_frame_id);
      if (index >= 0) this.onFrame(o.source_frame_id, false);
    }
    this.style();
  }

  style(mode = this.mode) {
    this.mode = mode;
    this.group.visible = $('show-objects').checked && mode !== 'points';
    for (const v of this.visuals || []) {
      const selected = v.object.object_id === this.selectedId;
      v.button.setAttribute('aria-pressed', String(selected));
      v.label.classList.toggle('selected', selected);
      v.mesh.visible = mode !== 'hologram';
      v.hologram.visible = mode === 'hologram';
      v.outline.visible = true;
      v.outline.traverse((edge) => { if (edge.material) edge.material.color.set(isWide(v.object) ?
        (selected ? 0xffad9f : 0xe66c65) : (selected ? 0xd898ff : 0x9a76bd)); });
      v.support.visible = selected && $('object-evidence').checked;
      v.label.hidden = !this.group.visible;
    }
    if (this.run) this.drawEvidence(this.frameIndex ?? 0);
  }

  focus() {
    const o = this.current();
    if (!o) return;
    this.stopExplore();
    const center = o.corners.map(vector).reduce((a, b) => a.add(b), new THREE.Vector3()).multiplyScalar(.25);
    const source = this.run.cameras.find((c) => c.frame_id === o.source_frame_id);
    if (source) this.camera.up.set(0, -1, 0).applyMatrix3(new THREE.Matrix3().set(...source.world_from_camera_rotation.flat()));
    const normal = vector(o.normal);
    const side = vector(o.corners[1]).sub(vector(o.corners[0])).normalize();
    this.camera.fov = 48;
    this.camera.updateProjectionMatrix();
    const diagonal = Math.hypot(o.width, o.height);
    const distance = diagonal / (2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov) / 2)) * 1.25;
    this.camera.position.copy(center).addScaledVector(normal, distance).addScaledVector(side, diagonal * .18);
    this.controls.target.copy(center);
    this.controls.update();
  }

  focusAll() {
    if (!this.objects.length) return;
    this.stopExplore();
    const bounds = new THREE.Box3().setFromPoints(this.objects.flatMap((o) => o.corners.map(vector)));
    const center = bounds.getCenter(new THREE.Vector3());
    const largest = [...this.objects].sort((a, b) => b.area - a.area)[0];
    const source = this.run.cameras.find((c) => c.frame_id === largest.source_frame_id);
    if (source) this.camera.up.set(0, -1, 0).applyMatrix3(new THREE.Matrix3().set(...source.world_from_camera_rotation.flat()));
    this.camera.fov = 48;
    this.camera.updateProjectionMatrix();
    const fov = Math.min(THREE.MathUtils.degToRad(48), 2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(24)) * this.camera.aspect));
    const distance = bounds.getSize(new THREE.Vector3()).length() / (2 * Math.sin(fov / 2)) * 1.1;
    this.camera.position.copy(center).addScaledVector(vector(largest.normal), distance);
    this.controls.target.copy(center);
    this.controls.update();
  }

  drawEvidence(index) {
    this.frameIndex = index;
    const overlay = $('object-overlay');
    overlay.replaceChildren();
    $('frame-proof').textContent = '';
    const o = this.current(), c = this.run?.cameras[index];
    if (!o || !c) return;
    overlay.setAttribute('viewBox', `0 0 ${c.width} ${c.height}`);
    const r = new THREE.Matrix3().set(...c.world_from_camera_rotation.flat()).transpose();
    const p = c.params;
    const simple = c.model.startsWith('SIMPLE') || c.model === 'RADIAL';
    const fx = p[0], fy = simple ? p[0] : p[1], cx = simple ? p[1] : p[2], cy = simple ? p[2] : p[3];
    const project = (xyz) => {
      const local = vector(xyz).sub(vector(c.position)).applyMatrix3(r);
      if (local.z <= 0) return null;
      let x = local.x / local.z, y = local.y / local.z;
      const r2 = x * x + y * y;
      if (c.model === 'SIMPLE_RADIAL' || c.model === 'RADIAL') {
        const radial = 1 + p[3] * r2 + (c.model === 'RADIAL' ? p[4] * r2 * r2 : 0);
        x *= radial; y *= radial;
      } else if (c.model === 'OPENCV') {
        const radial = 1 + p[4] * r2 + p[5] * r2 * r2;
        const xx = x * radial + 2 * p[6] * x * y + p[7] * (r2 + 2 * x * x);
        y = y * radial + p[6] * (r2 + 2 * y * y) + 2 * p[7] * x * y;
        x = xx;
      }
      return [fx * x + cx, fy * y + cy];
    };
    const corners = o.corners.map(project);
    if (corners.every(Boolean)) {
      const polygon = document.createElementNS(svgNS, 'polygon');
      polygon.setAttribute('points', corners.map((point) => point.join(',')).join(' '));
      polygon.setAttribute('class', 'evidence-boundary');
      polygon.classList.toggle('wide-plane', isWide(o));
      overlay.append(polygon);
    }
    if ($('object-evidence').checked) for (const xyz of o.support_points) {
      const pixel = project(xyz);
      if (!pixel || pixel[0] < 0 || pixel[0] > c.width || pixel[1] < 0 || pixel[1] > c.height) continue;
      const circle = document.createElementNS(svgNS, 'circle');
      circle.setAttribute('cx', pixel[0]); circle.setAttribute('cy', pixel[1]);
      circle.setAttribute('r', c.width / 250);
      circle.setAttribute('class', 'evidence-point');
      overlay.append(circle);
    }
    $('frame-proof').textContent = `${isWide(o) ? '빨간색' : '보라색'}: 추정 경계 · 주황색: 3D 지지점의 재투영`;
  }

  render(time = 0) {
    if (!this.run) return;
    const bounds = this.renderer.domElement.getBoundingClientRect();
    const occupied = [];
    const animate = !window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    for (const v of this.visuals || []) v.hologram.material.uniforms.time.value = animate ? time / 1000 : 0;
    for (const v of [...(this.visuals || [])].sort((a, b) => Number(b.object.object_id === this.selectedId) - Number(a.object.object_id === this.selectedId))) {
      const point = vector(v.object.corners[0]).lerp(vector(v.object.corners[1]), .5);
      const projected = point.project(this.camera);
      const visible = this.group.visible && projected.z > -1 && projected.z < 1 && Math.abs(projected.x) < .98 && Math.abs(projected.y) < .98;
      v.label.hidden = !visible;
      if (visible) {
        const x = (projected.x + 1) * bounds.width / 2;
        const y = (1 - projected.y) * bounds.height / 2;
        const box = {x: x - 55, y: y - 35, w: 110, h: 25};
        if (occupied.some((old) => Math.abs(box.x - old.x) < box.w && Math.abs(box.y - old.y) < box.h)) {
          v.label.hidden = true;
          continue;
        }
        occupied.push(box);
        v.label.style.left = `${x}px`;
        v.label.style.top = `${y}px`;
      }
    }
  }
}
