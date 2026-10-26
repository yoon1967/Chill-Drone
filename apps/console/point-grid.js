// Always use original coordinates so changing the interval never accumulates rounding.
// Equidistant values round away from zero; interval 0 restores the original positions.
export function updatePointGrid(geometry, source, interval) {
  if (!Number.isFinite(interval) || interval < 0) throw new RangeError('Invalid point interval');
  const position = geometry.getAttribute('position');
  if (position.array.length !== source.length) throw new RangeError('Point buffer size mismatch');
  if (interval === 0) position.array.set(source);
  else {
    for (let i = 0; i < source.length; i++) {
      const multiple = Math.round(Math.abs(source[i]) / interval);
      position.array[i] = multiple === 0 ? 0 : Math.sign(source[i]) * multiple * interval;
    }
  }
  position.needsUpdate = true;
  geometry.setDrawRange(0, source.length / 3);
  geometry.computeBoundingSphere();
}

// Sample queries against a full k-d tree: profile once per map, never on slider input.
export function pointSpacingProfile(points) {
  const xyz = points.map((p) => p.slice(1, 4)).filter((p) => p.every(Number.isFinite));
  const build = (indices, depth = 0) => {
    if (!indices.length) return null;
    const axis = depth % 3;
    indices.sort((a, b) => xyz[a][axis] - xyz[b][axis]);
    const mid = indices.length >> 1;
    return {index: indices[mid], axis, left: build(indices.slice(0, mid), depth + 1),
      right: build(indices.slice(mid + 1), depth + 1)};
  };
  const tree = build(xyz.map((_, i) => i));
  const nearest = (query) => {
    let best = Infinity;
    const visit = (node) => {
      if (!node) return;
      const p = xyz[node.index], delta = query[node.axis] - p[node.axis];
      const distance = query.reduce((sum, v, a) => sum + (v - p[a]) ** 2, 0);
      if (distance > 0 && distance < best) best = distance;
      visit(delta < 0 ? node.left : node.right);
      if (delta * delta < best) visit(delta < 0 ? node.right : node.left);
    };
    visit(tree);
    return Math.sqrt(best);
  };
  // Collapse coincident coordinates before searching (also avoids degenerate O(n²)).
  const queries = [...new Map(xyz.map((p) => [p.join(','), p])).values()];
  const distances = [];
  if (queries.length > 1) for (let i = 0, count = Math.min(256, queries.length); i < count; i++) {
    const distance = nearest(queries[Math.floor(i * queries.length / count)]);
    if (Number.isFinite(distance) && distance > 0) distances.push(distance);
  }
  distances.sort((a, b) => a - b);
  const median = distances.length ? distances[Math.floor(distances.length / 2)] : 0;
  const fallback = Math.max(...xyz.slice(0, 256).flat().map(Math.abs), 1) * .01;
  const base = 10 ** Math.floor(Math.log10(median || fallback));
  const step = Math.max(1e-12, base / 10);
  const recommended = Math.max(step, Math.round((median || fallback) / base) * base);
  const extent = (median || fallback) * 10;
  const decade = 10 ** Math.floor(Math.log10(extent));
  const max = [1, 2, 5, 10].find((n) => n * decade >= extent) * decade;
  return {step, max, recommended, median, samples: distances.length};
}

// Confidence is evaluated AFTER quantization, using distinct supporting frames.
export function pointGridCells(points, interval, observations, cameraCount) {
  if (!Number.isFinite(interval) || interval < 0) throw new RangeError('Invalid point interval');
  const metadata = observations?.schema_version === 'point-observations/1.0' ? observations : null;
  const cells = new Map();
  let lowerBound = false;
  for (const p of points) {
    const indices = p.slice(1, 4).map((v) => interval === 0 ? v : Math.sign(v) * Math.round(Math.abs(v) / interval));
    const key = indices.join(',');
    let cell = cells.get(key);
    if (!cell) {
      cell = {position: indices.map((v) => interval === 0 || v === 0 ? v || 0 : v * interval),
        frames: new Set(), fallback: 0, points: 0};
      cells.set(key, cell);
    }
    cell.points++;
    const frames = metadata?.frame_index_by_point?.[p[0]];
    if (Array.isArray(frames) && frames.every((i) => Number.isInteger(i) && i >= 0 && i < cameraCount)) {
      for (const frame of frames) cell.frames.add(frame);
    } else {
      // Legacy counts cannot be summed: their frames may overlap completely.
      cell.fallback = Math.max(cell.fallback, Number.isFinite(p[8]) ? Math.max(0, Math.floor(p[8])) : 0);
      lowerBound = true;
    }
  }
  const values = [...cells.values()].map((cell) => ({position: cell.position, points: cell.points,
    support: Math.max(cell.frames.size, cell.fallback)}));
  return {cells: values, lowerBound, basis: metadata?.basis || 'unknown',
    maxSupport: values.reduce((max, cell) => Math.max(max, cell.support), 1), sourceCount: points.length};
}

export function displayPointCells(geometry, grid, threshold) {
  if (!Number.isInteger(threshold) || threshold < 1) throw new RangeError('Invalid confidence threshold');
  const position = geometry.getAttribute('position');
  let count = 0;
  for (const cell of grid.cells) if (cell.support >= threshold) {
    position.array.set(cell.position, count * 3);
    count++;
  }
  // Draw range also handles zero trusted points without phantom points at the origin.
  geometry.setDrawRange(0, count);
  position.needsUpdate = true;
  geometry.computeBoundingSphere();
  return count;
}
