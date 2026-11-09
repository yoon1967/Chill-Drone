export function savedEditorUrl(asset) {
  const query = new URLSearchParams({source: 'saved'});
  if (asset) query.set('asset', asset);
  return `/editor.html?${query}`;
}

const axes = [0, 1, 2];
const position = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
const rgb = value => Number.isFinite(value) ? Math.max(0, Math.min(255, Math.round(value))) : 128;

export function editorCloud(value, source) {
  const supervisor = source === 'supervisor';
  const schema = value?.schema_version;
  if (supervisor ? schema !== 'gateway-map/0.1' : !/^phone-sfm-result\/1\.[012]$/.test(schema))
    throw new Error('지원하는 점군 형식이 아닙니다.');
  const offset = supervisor ? 0 : 1;
  const points = [];
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (const row of value.points || []) {
    if (!Array.isArray(row)) continue;
    const xyz = row.slice(offset, offset + 3);
    if (!position(xyz)) continue;
    points.push([...xyz, ...axes.map(axis => rgb(row[offset + 3 + axis]))]);
    for (const axis of axes) {
      min[axis] = Math.min(min[axis], xyz[axis]);
      max[axis] = Math.max(max[axis], xyz[axis]);
    }
  }
  if (!points.length) throw new Error('편집할 점군이 없습니다. 점군이 생성된 뒤 다시 열어 주세요.');
  return {points, min, max, cameras: (value.cameras || []).map(camera => supervisor ? camera : camera.position).filter(position),
    up: position(value.up) ? value.up : [0, 1, 0],
    name: (supervisor ? value.source?.input_run_id || value.source?.session_id : value.input_run_id) || '3d-map',
    revision: supervisor ? value.source?.revision : value.revision,
    total: supervisor ? value.points_total : points.length, time: supervisor ? value.received_at : value.created_utc,
    unit: value.scale?.unit === 'm' ? 'm' : 'arbitrary'};
}

export function cropBounds(cloud, limits) {
  if (limits.some(([min, max]) => !Number.isFinite(min) || !Number.isFinite(max) || min < 0 || max > 100 || min > max))
    return null;
  const coordinate = (axis, percent) => percent === 100 ? cloud.max[axis] :
    cloud.min[axis] + (cloud.max[axis] - cloud.min[axis]) * percent / 100;
  return {min: axes.map(axis => coordinate(axis, limits[axis][0])),
    max: axes.map(axis => coordinate(axis, limits[axis][1]))};
}

export function cropCloud(points, {min, max}, keepInside) {
  return points.filter(point => axes.every(axis => min[axis] <= point[axis] && point[axis] <= max[axis]) === keepInside);
}

export function cloudPly(points, unit) {
  const header = `ply\nformat binary_little_endian 1.0\ncomment unit ${unit === 'm' ? 'm' : 'arbitrary'}\n` +
    `element vertex ${points.length}\nproperty double x\nproperty double y\nproperty double z\n` +
    'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n';
  const data = new DataView(new ArrayBuffer(points.length * 27));
  points.forEach((point, index) => {
    const start = index * 27;
    for (const axis of axes) {
      data.setFloat64(start + axis * 8, point[axis], true);
      data.setUint8(start + 24 + axis, point[axis + 3]);
    }
  });
  return new Blob([header, data], {type: 'application/octet-stream'});
}
