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
  geometry.computeBoundingSphere();
}
