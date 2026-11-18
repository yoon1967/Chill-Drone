import * as THREE from 'three';
import {sceneChanges, sceneToken} from './map-delta.js';
import {mappingSettings, rescuePointColor} from './mapping-settings.js';

const equal = (a, b) => a === b || Boolean(a && b && a.length === b.length && a.every((v, i) => v === b[i]));
const evidence = scene => scene.point_observations?.schema_version === 'point-observations/1.0' ?
  scene.point_observations.frame_index_by_point : null;
const pointAttributes = {position: 3, color: 3, pointScale: 1};
export const TOF_POINT_SIZE_MULTIPLIER = 1.4;

export function depthPointScale(tof, far) {
  if (tof) return TOF_POINT_SIZE_MULTIPLIER;
  return far ? .65 : 1;
}

export function depthPointMaterial(size) {
  const material = new THREE.PointsMaterial({vertexColors: true, sizeAttenuation: false, size});
  material.onBeforeCompile = shader => {
    shader.vertexShader = shader.vertexShader.replace('uniform float size;', 'uniform float size;\nattribute float pointScale;')
      .replace('gl_PointSize = size;', 'gl_PointSize = size * pointScale;');
  };
  return material;
}

// Dense slots keep one draw call. Deletion moves only the last slot; point IDs
// never depend on the native snapshot's changing order.
export class PointBuffer {
  constructor(capacity = 1024) {
    this.geometry = new THREE.BufferGeometry();
    this.slots = new Map(); this.ids = []; this.capacity = 0;
    this.grow(capacity);
    this.geometry.setDrawRange(0, 0);
  }
  grow(required) {
    if (required <= this.capacity) return;
    const capacity = Math.max(1024, 2 ** Math.ceil(Math.log2(required)));
    // Release the previous GPU allocations before replacing their attributes.
    if (this.capacity) this.geometry.dispose();
    for (const [name, size] of Object.entries(pointAttributes)) {
      const old = this.geometry.getAttribute(name);
      const attribute = new THREE.BufferAttribute(new Float32Array(capacity * size), size).setUsage(THREE.DynamicDrawUsage);
      if (old) attribute.array.set(old.array);
      this.geometry.setAttribute(name, attribute);
    }
    this.capacity = capacity;
  }
  write(attribute, slot, values) {
    const offset = slot * attribute.itemSize, next = values.map(Math.fround);
    if (next.every((v, i) => attribute.array[offset + i] === v)) return;
    attribute.array.set(next, offset);
    attribute.addUpdateRange(offset, attribute.itemSize);
    attribute.needsUpdate = true;
    if (attribute.updateRanges.length > 64) this.compactRanges();
  }
  upsert(id, position, color, pointScale = 1) {
    let slot = this.slots.get(id);
    if (slot === undefined) {
      slot = this.ids.length; this.grow(slot + 1);
      this.slots.set(id, slot); this.ids.push(id);
    }
    this.write(this.geometry.getAttribute('position'), slot, position);
    this.write(this.geometry.getAttribute('color'), slot, color);
    this.write(this.geometry.getAttribute('pointScale'), slot, [pointScale]);
    this.geometry.setDrawRange(0, this.ids.length);
  }
  remove(id) {
    const slot = this.slots.get(id);
    if (slot === undefined) return;
    const last = this.ids.pop(); this.slots.delete(id);
    if (last !== id) {
      this.ids[slot] = last; this.slots.set(last, slot);
      for (const [name, size] of Object.entries(pointAttributes)) {
        const attribute = this.geometry.getAttribute(name);
        this.write(attribute, slot, [...attribute.array.subarray(this.ids.length * size, this.ids.length * size + size)]);
      }
    }
    this.geometry.setDrawRange(0, this.ids.length);
  }
  bounds(bounds, interval = 0) {
    const box = new THREE.Box3(new THREE.Vector3(...bounds.min), new THREE.Vector3(...bounds.max));
    box.expandByScalar(interval / 2);
    this.geometry.boundingSphere = box.getBoundingSphere(new THREE.Sphere());
  }
  compactRanges() {
    // Many edits before a render must not create an unbounded range list.
    for (const name of Object.keys(pointAttributes)) {
      const attribute = this.geometry.getAttribute(name), ranges = attribute.updateRanges;
      if (ranges.length > 64) {
        const low = Math.min(...ranges.map(r => r.start));
        const high = Math.max(...ranges.map(r => r.start + r.count));
        attribute.clearUpdateRanges(); attribute.addUpdateRange(low, high - low);
      }
    }
  }
}

// Reference counts are necessary: removing one point must not remove a frame
// that still supports another point in the same cell.
class PointCells {
  constructor(interval) {
    this.interval = interval; this.cells = new Map(); this.members = new Map(); this.dirty = new Set();
    this.values = new Map(); this.supports = new Map(); this.maxSupport = 1; this.fallbackIds = new Set();
  }
  support(cell) {
    return Math.max(cell.frames.size, cell.fallbackMax);
  }
  uncount(cell) {
    const support = this.support(cell), count = (this.supports.get(support) || 1) - 1;
    if (count) this.supports.set(support, count); else this.supports.delete(support);
  }
  count(cell) {
    const support = this.support(cell);
    this.supports.set(support, (this.supports.get(support) || 0) + 1);
    this.maxSupport = Math.max(this.maxSupport, support);
  }
  invalidate(key) { this.values.delete(key); this.dirty.add(key); }
  remove(id) {
    const member = this.members.get(id);
    if (!member) return;
    const cell = this.cells.get(member.key);
    this.uncount(cell);
    cell.ids.delete(id);
    for (const frame of member.frames || []) {
      const count = cell.frames.get(frame) - 1;
      if (count) cell.frames.set(frame, count); else cell.frames.delete(frame);
    }
    if (cell.fallbacks.has(id)) {
      const value = cell.fallbacks.get(id), count = cell.fallbackCounts.get(value) - 1;
      if (count) cell.fallbackCounts.set(value, count); else cell.fallbackCounts.delete(value);
      cell.fallbacks.delete(id);
      if (!cell.fallbackCounts.has(cell.fallbackMax)) {
        cell.fallbackMax = 0;
        for (const n of cell.fallbackCounts.keys()) cell.fallbackMax = Math.max(cell.fallbackMax, n);
      }
    }
    this.members.delete(id);
    this.fallbackIds.delete(id);
    if (!cell.ids.size) this.cells.delete(member.key); else this.count(cell);
    this.invalidate(member.key);
  }
  upsert(id, row, observed, cameraCount) {
    this.remove(id);
    const indices = row.slice(1, 4).map(v => this.interval === 0 ? v : Math.sign(v) * Math.round(Math.abs(v) / this.interval));
    const key = indices.join(',');
    let cell = this.cells.get(key);
    if (cell) this.uncount(cell);
    if (!cell) {
      cell = {position: indices.map(v => this.interval === 0 || v === 0 ? v || 0 : v * this.interval),
        ids: new Set(), frames: new Map(), fallbacks: new Map(), fallbackCounts: new Map(), fallbackMax: 0};
      this.cells.set(key, cell);
    }
    const frames = Array.isArray(observed) && observed.every(i => Number.isInteger(i) && i >= 0 && i < cameraCount) ? new Set(observed) : null;
    cell.ids.add(id);
    for (const frame of frames || []) cell.frames.set(frame, (cell.frames.get(frame) || 0) + 1);
    if (!frames) {
      const value = Number.isFinite(row[8]) ? Math.max(0, Math.floor(row[8])) : 0;
      cell.fallbacks.set(id, value); cell.fallbackCounts.set(value, (cell.fallbackCounts.get(value) || 0) + 1);
      cell.fallbackMax = Math.max(cell.fallbackMax, value); this.fallbackIds.add(id);
    }
    this.members.set(id, {key, frames}); this.count(cell); this.invalidate(key);
  }
  value(key) {
    const cell = this.cells.get(key);
    if (!cell) return undefined;
    if (!this.values.has(key)) this.values.set(key, {position: cell.position, points: cell.ids.size,
      sourceIds: [...cell.ids], support: this.support(cell)});
    return this.values.get(key);
  }
  displayValue(key) {
    const cell = this.cells.get(key);
    return cell && {position: cell.position, support: this.support(cell)};
  }
  summary(scene, sourceCount) {
    if (!this.supports.has(this.maxSupport)) {
      this.maxSupport = 1;
      for (const support of this.supports.keys()) this.maxSupport = Math.max(this.maxSupport, support);
    }
    return {cellCount: this.cells.size, interval: this.interval, sourceCount, cameraCount: scene.cameras.length,
      lowerBound: this.fallbackIds.size > 0, basis: scene.point_observations?.basis || 'unknown', maxSupport: this.maxSupport};
  }
}

// The native scene remains immutable for export/replay. This index owns only
// display state and keeps the already allocated raw and quantized point buffers.
export class PointDisplay {
  constructor() {
    this.raw = new PointBuffer(); this.quantized = new PointBuffer();
    this.rows = new Map(); this.scene = null; this.version = 0;
    this.cells = null; this.grid = null; this.history = [];
  }
  color(row, scene, mode) {
    const c = new THREE.Color();
    if (mode === 'rgb' || mode === 'semantic' || (scene.rescue?.render_mode === 'points' && scene.rescue.point_labels?.[row[0]] === 1)) {
      const rgb = scene.rescue?.render_mode === 'points' ? rescuePointColor(row.slice(4,7),scene.rescue.point_labels?.[row[0]],scene.rescue,this.rescueStyle || mappingSettings()) : row.slice(4,7);
      c.setRGB(rgb[0]/255,rgb[1]/255,rgb[2]/255,THREE.SRGBColorSpace);
    }
    else {
      const bounds = scene.quality.bounds;
      const h = mode === 'error' ? Math.min(row[7] / 3, 1) : (row[2] - bounds.min[1]) / Math.max(bounds.max[1] - bounds.min[1], .001);
      c.setHSL((1 - h) * .58, .7, .6).convertSRGBToLinear();
    }
    return c.toArray();
  }
  update(scene, {errorLimit = Infinity, colorMode = 'rgb'} = {}) {
    this.rescueStyle = mappingSettings();
    const previous = this.scene, change = sceneChanges(scene);
    const incremental = previous && change?.previousToken === sceneToken(previous) && this.errorLimit === errorLimit;
    const touched = new Set(), removed = [], upserts = [];
    const observe = evidence(scene), oldObserve = previous && evidence(previous);
    let visited = 0;
    const set = row => {
      visited++; const id = row[0], old = this.rows.get(id);
      if (!(row[7] <= errorLimit)) {
        if (old) { this.rows.delete(id); removed.push(id); touched.add(id); }
      } else if (!equal(old, row)) { this.rows.set(id, row); upserts.push(row); touched.add(id); }
    };
    if (incremental) {
      for (const id of change.removed) if (this.rows.delete(id)) { removed.push(id); touched.add(id); }
      for (const row of change.upserts) set(row);
      for (const key of change.observationIds) {
        const id = this.rows.has(key) ? key : Number(key);
        if (this.rows.has(id)) touched.add(id);
      }
    } else {
      const present = new Set();
      for (const row of scene.points) { present.add(row[0]); set(row); }
      for (const id of this.rows.keys()) if (!present.has(id)) { this.rows.delete(id); removed.push(id); touched.add(id); }
      for (const [id] of this.rows) if (!equal(oldObserve?.[id], observe?.[id])) touched.add(id);
    }
    // Invalid legacy evidence can become valid when the camera range grows;
    // shrinking/relabeling the range requires every membership to be checked.
    if (previous && (Boolean(observe) !== Boolean(oldObserve) ||
        (this.cells && scene.cameras.length < previous.cameras.length))) {
      for (const id of this.rows.keys()) touched.add(id);
    } else if (this.cells && previous?.cameras.length !== scene.cameras.length) {
      for (const id of this.cells.fallbackIds) touched.add(id);
    }
    const recolor = this.colorMode !== colorMode || (scene.rescue?.render_mode === 'points' &&
      previous?.rescue?.point_revision !== scene.rescue.point_revision) || (colorMode === 'height' &&
      JSON.stringify(previous?.quality.bounds) !== JSON.stringify(scene.quality.bounds)) ||
      !equal(previous?.rescue?.point_tof, scene.rescue?.point_tof) ||
      !equal(previous?.rescue?.point_far, scene.rescue?.point_far);
    for (const id of removed) this.raw.remove(id);
    for (const row of recolor ? this.rows.values() : upserts) this.raw.upsert(row[0], row.slice(1, 4),
      this.color(row, scene, colorMode), depthPointScale(scene.rescue?.point_tof?.[row[0]] === true,
        scene.rescue?.point_far?.[row[0]] === true));
    if (this.cells) for (const id of touched) {
      const row = this.rows.get(id);
      if (row) this.cells.upsert(id, row, observe?.[id], scene.cameras.length); else this.cells.remove(id);
    }
    this.scene = scene; this.errorLimit = errorLimit; this.colorMode = colorMode;
    this.raw.bounds(scene.quality.bounds); this.raw.compactRanges();
    this.version++;
    this.history.push({version: this.version, ids: touched});
    if (this.history.length > 32) this.history.shift();
    this.grid = null;
    return {visited, changed: touched.size, rawPoints: this.rows.size};
  }
  points() { return [...this.rows.values()]; }
  display(interval, threshold, hologram) {
    if (!Number.isFinite(interval) || interval < 0) throw new RangeError('Invalid point interval');
    if (!Number.isInteger(threshold) || threshold < 1) throw new RangeError('Invalid confidence threshold');
    if (!hologram) return this.raw.geometry;
    if (!this.cells || this.cells.interval !== interval) {
      this.cells = new PointCells(interval); this.grid = null;
      const observed = evidence(this.scene);
      for (const [id, row] of this.rows) this.cells.upsert(id, row, observed?.[id], this.scene.cameras.length);
      // A settings change may invalidate every quantized slot.
      for (const id of [...this.quantized.ids]) this.quantized.remove(id);
    }
    const dirty = this.threshold === threshold ? this.cells.dirty : new Set(this.cells.cells.keys());
    if (this.threshold !== threshold) for (const key of this.quantized.ids) dirty.add(key);
    for (const key of dirty) {
      const cell = this.cells.displayValue(key);
      if (cell && cell.support >= threshold) this.quantized.upsert(key, cell.position, [1, 1, 1]);
      else this.quantized.remove(key);
    }
    this.cells.dirty.clear(); this.threshold = threshold;
    this.quantized.bounds(this.scene.quality.bounds, interval); this.quantized.compactRanges();
    return this.quantized.geometry;
  }
  gridSnapshot() {
    return {...this.gridSummary(), cells: [...this.cells.cells.keys()].map(key => this.cells.value(key))};
  }
  gridSummary() {
    return this.grid ??= this.cells.summary(this.scene, this.rows.size);
  }
  // A worker may skip several revisions. Merge changes since its last input,
  // falling back to a full snapshot when its bounded journal no longer covers it.
  workerUpdate(since = -1) {
    const reset = since < 0 || since > this.version || since < (this.history[0]?.version ?? this.version) - 1;
    const ids = reset ? new Set(this.rows.keys()) : new Set(this.history.filter(h => h.version > since).flatMap(h => [...h.ids]));
    const observed = evidence(this.scene), rows = [], removed = [], observations = {};
    for (const id of ids) {
      const row = this.rows.get(id);
      if (row) { rows.push(row); observations[id] = observed?.[id] ?? null; } else removed.push(id);
    }
    return {reset, base: since, version: this.version, points: rows, removed, observations,
      metadata: this.scene.point_observations ? {...this.scene.point_observations, frame_index_by_point: undefined} : null};
  }
  dispose() { this.raw.geometry.dispose(); this.quantized.geometry.dispose(); }
}
