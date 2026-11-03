import * as THREE from '/vendor/three.module.js';

// Camera-aligned world axes. Selecting an axis preserves the orbit target and zoom.
export function connectAxisGizmo({host, camera, controls, enabled = () => true, beforeSelect = () => {}, up = [0, 1, 0]}) {
  const widget = document.createElement('div'); widget.className = 'axis-gizmo';
  widget.setAttribute('role', 'group'); widget.setAttribute('aria-label', 'X Y Z 축 시점 선택');
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('viewBox', '0 0 128 128'); svg.setAttribute('aria-hidden', 'true'); widget.append(svg);
  const abort = new AbortController();
  const axes = [];
  for (let axis = 0; axis < 3; axis++) for (const sign of [1, -1]) {
    const name = 'XYZ'[axis], vector = new THREE.Vector3().setComponent(axis, sign);
    const line = document.createElementNS(svg.namespaceURI, 'line');
    line.setAttribute('x1', '64'); line.setAttribute('y1', '64'); svg.append(line);
    const button = document.createElement('button'); button.type = 'button';
    button.className = `axis-gizmo-end axis-${name.toLowerCase()}`; button.dataset.axis = `${sign > 0 ? '+' : '-'}${name}`;
    button.textContent = sign > 0 ? name : '−' + name;
    button.setAttribute('aria-label', `${button.dataset.axis} 축에서 보기`); button.title = `${button.dataset.axis} 축에서 보기`;
    button.addEventListener('click', () => {
      if (!enabled()) return;
      beforeSelect();
      const damping = controls.enableDamping;
      controls.enableDamping = false;
      controls.update();
      controls.enableDamping = damping;
      const distance = Math.max(camera.position.distanceTo(controls.target), .01);
      camera.position.copy(controls.target).addScaledVector(vector, distance);
      // OrbitControls caches its world-up frame at construction. Keep that frame
      // fixed instead of rolling camera.up when selecting a vertical axis.
      camera.up.fromArray(up);
      if (Math.abs(vector.dot(camera.up)) > .99) {
        // A deterministic tiny tilt avoids the pole singularity and preserves the
        // expected top/bottom screen orientation without changing the orbit axis.
        camera.position.z += distance * up[1] * 1e-6;
        camera.position.sub(controls.target).setLength(distance).add(controls.target);
      }
      camera.lookAt(controls.target); controls.update();
      host.dataset.cameraAxis = button.dataset.axis;
      host.dataset.cameraDirection = camera.getWorldDirection(new THREE.Vector3()).toArray().join(',');
      update();
    }, {signal: abort.signal});
    widget.append(button); axes.push({vector, line, button, sign});
  }
  host.append(widget);
  const inverse = new THREE.Quaternion();
  const previous = new THREE.Quaternion(0, 0, 0, 0);
  let wasEnabled;
  function update() {
    const available = enabled();
    if (previous.equals(camera.quaternion) && available === wasEnabled) return;
    previous.copy(camera.quaternion); wasEnabled = available;
    host.dataset.cameraDirection = camera.getWorldDirection(new THREE.Vector3()).toArray().join(',');
    inverse.copy(camera.quaternion).invert();
    for (const {vector, line, button, sign} of axes) {
      const p = vector.clone().applyQuaternion(inverse);
      const axial = Math.hypot(p.x, p.y) < .15;
      const x = 64 + (axial ? sign * 18 : p.x * 43), y = 64 + (axial ? sign * 18 : -p.y * 43);
      line.setAttribute('x2', x); line.setAttribute('y2', y);
      line.style.stroke = {X: '#ec7775', Y: '#8bc96d', Z: '#7baaf2'}[button.dataset.axis[1]];
      line.style.opacity = sign > 0 ? '.8' : '.28';
      button.style.left = `${x}px`; button.style.top = `${y}px`;
      button.style.zIndex = String(Math.round((p.z + 1) * 10) + 1);
      button.style.opacity = p.z < -.2 ? '.55' : '1';
      button.disabled = !available;
    }
  }
  update();
  return {update, dispose() { abort.abort(); widget.remove(); }};
}
