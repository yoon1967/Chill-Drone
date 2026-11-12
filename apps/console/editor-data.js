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
  if (!points.length) throw new Error('표시할 점군이 없습니다. 점군이 생성된 뒤 다시 열어 주세요.');
  return {points, min, max, cameras: (value.cameras || []).map(camera => camera?.position).filter(position),
    up: position(value.up) ? value.up : [0, 1, 0],
    name: value.input_run_id || '3d-map',
    unit: value.scale?.unit === 'm' ? 'm' : 'arbitrary'};
}
