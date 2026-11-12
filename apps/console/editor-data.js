const axes = [0, 1, 2];
const position = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
const rgb = value => Number.isFinite(value) ? Math.max(0, Math.min(255, Math.round(value))) : 128;

export function editorCloud(value) {
  const schema = value?.schema_version;
  if (!/^phone-sfm-result\/1\.[012]$/.test(schema))
    throw new Error('지원하는 점군 형식이 아닙니다.');
  const points = [];
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (const row of value.points || []) {
    if (!Array.isArray(row)) continue;
    const xyz = row.slice(1, 4);
    if (!position(xyz)) continue;
    points.push([...xyz, ...axes.map(axis => rgb(row[4 + axis]))]);
    for (const axis of axes) {
      min[axis] = Math.min(min[axis], xyz[axis]);
      max[axis] = Math.max(max[axis], xyz[axis]);
    }
  }
  if (!points.length) throw new Error('편집할 점군이 없습니다. 점군이 생성된 뒤 다시 열어 주세요.');
  return {points, min, max, cameras: (value.cameras || []).map(camera => camera?.position).filter(position),
    up: position(value.up) ? value.up : [0, 1, 0],
    name: value.input_run_id || '3d-map',
    unit: value.scale?.unit === 'm' ? 'm' : 'arbitrary'};
}

export function pointEdits(points) {
  const colors = Uint32Array.from(points, ([, , , r, g, b]) => (r << 16) | (g << 8) | b);
  const removed = new Uint8Array(points.length);
  const selected = new Uint8Array(points.length);
  const history = [];
  let cursor = 0;
  let count = points.length;
  let selectionCount = 0;

  function select(test, extend = false) {
    selectionCount = 0;
    for (let i = 0; i < selected.length; i++) {
      selected[i] = !removed[i] && ((extend && selected[i]) || test(i)) ? 1 : 0;
      selectionCount += selected[i];
    }
  }

  function apply(action, undo) {
    const {indices, previous, value, buffer} = action;
    indices.forEach((id, i) => { buffer[id] = undo ? (previous?.[i] ?? 0) : value; });
    if (buffer === removed) {
      count += undo ? indices.length : -indices.length;
      selected.fill(0);
      selectionCount = 0;
    }
  }

  function commit(action) {
    if (!action.indices.length) return false;
    history.splice(cursor);
    history.push(action);
    if (history.length > 30) history.shift();
    cursor = history.length;
    apply(action, false);
    return true;
  }

  return {
    colors, removed, selected, select,
    get count() { return count; },
    get selectionCount() { return selectionCount; },
    get canUndo() { return cursor > 0; },
    get canRedo() { return cursor < history.length; },
    remove(keepSelection) {
      if (!selectionCount) return false;
      const indices = new Uint32Array(keepSelection ? count - selectionCount : selectionCount);
      let at = 0;
      for (let i = 0; i < removed.length; i++)
        if (!removed[i] && Boolean(selected[i]) !== keepSelection) indices[at++] = i;
      return commit({indices, value: 1, buffer: removed});
    },
    paint(color) {
      const indices = [];
      for (let i = 0; i < selected.length; i++)
        if (selected[i] && colors[i] !== color) indices.push(i);
      const changed = Uint32Array.from(indices);
      return commit({indices: changed, previous: changed.map(id => colors[id]), value: color, buffer: colors});
    },
    undo() { if (cursor > 0) apply(history[--cursor], true); },
    redo() { if (cursor < history.length) apply(history[cursor++], false); },
  };
}

export function cloudPly(points, unit, edits) {
  const header = `ply\nformat binary_little_endian 1.0\ncomment unit ${unit === 'm' ? 'm' : 'arbitrary'}\n` +
    `element vertex ${edits.count}\nproperty double x\nproperty double y\nproperty double z\n` +
    'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n';
  const data = new DataView(new ArrayBuffer(edits.count * 27));
  let start = 0;
  points.forEach((point, index) => {
    if (edits.removed[index]) return;
    const color = edits.colors[index];
    for (const axis of axes) {
      data.setFloat64(start + axis * 8, point[axis], true);
      data.setUint8(start + 24 + axis, (color >> ((2 - axis) * 8)) & 255);
    }
    start += 27;
  });
  return new Blob([header, data], {type: 'application/octet-stream'});
}
