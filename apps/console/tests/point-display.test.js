import test from 'node:test';
import assert from 'node:assert/strict';
import {PointBuffer, PointDisplay} from '../point-display.js';
import {WorkerPointState} from '../point-worker-state.js';
import {applySceneDelta} from '../map-delta.js';
import {pointGridCells, pointSpacingProfile} from '../point-grid.js';

const row = (id, x = id, y = 0, z = 0) => [id, x, y, z, 100, 150, 200, .5, 3];
const scene = (points, revision = 1) => ({pose_source: 'orb_slam3', input_run_id: 'synthetic', segment_id: 0,
  revision, points, cameras: Array.from({length: 4}, (_, i) => ({frame_id: String(i)})),
  quality: {bounds: {min: [-100, -100, -100], max: [100000, 100, 100]}},
  point_observations: {schema_version: 'point-observations/1.0', basis: 'keyframes',
    frame_index_by_point: Object.fromEntries(points.map(p => [p[0], [0, 1, 2]]))}});

function delta(previous, {upserts = [], removed = [], observations = {}, observationRemoved = [], cameras = previous.cameras} = {}) {
  const rows = new Map(previous.points.map(p => [p[0], p]));
  removed.forEach(id => rows.delete(id)); upserts.forEach(p => rows.set(p[0], p));
  const {points, point_observations, ...meta} = previous;
  return applySceneDelta(previous, {schema_version: 'phone-map-delta/1.0', base_revision: previous.revision,
    revision: previous.revision + 1, session_id: previous.input_run_id, segment_id: previous.segment_id,
    scene: {...meta, revision: previous.revision + 1, cameras}, points_upsert: upserts, points_removed: removed,
    point_order: [...rows.keys()].reverse(), observation_upserts: observations, observation_removed: observationRemoved,
    observation_metadata: {schema_version: 'point-observations/1.0', basis: 'keyframes'}});
}
const cells = grid => grid.cells.map(c => ({...c, sourceIds: [...c.sourceIds].sort((a, b) => a - b)}))
  .sort((a, b) => a.position.join(',').localeCompare(b.position.join(',')));
const positions = buffer => Object.fromEntries(buffer.ids.map((id, i) => [id,
  [...buffer.geometry.getAttribute('position').array.slice(i * 3, i * 3 + 3)]]));
const clearRanges = buffer => ['position', 'color'].forEach(name => buffer.geometry.getAttribute(name).clearUpdateRanges());

test('point buffers reuse allocations and update only changed slots, including deletion compaction', () => {
  const buffer = new PointBuffer();
  assert.equal(buffer.geometry.drawRange.count, 0);
  buffer.upsert(10, [.1, 2, 3], [1, 0, 0]); buffer.upsert(20, [4, 5, 6], [0, 1, 0]);
  const geometry = buffer.geometry, attribute = geometry.getAttribute('position'), color = geometry.getAttribute('color');
  clearRanges(buffer);
  const version = attribute.version;
  buffer.upsert(10, [.1, 2, 3], [1, 0, 0]);
  assert.equal(attribute.version, version, 'float32 rounding must not dirty unchanged coordinates');
  buffer.upsert(20, [4, 5, 7], [0, 1, 0]);
  assert.deepEqual(attribute.updateRanges, [{start: 3, count: 3}]);
  assert.equal(color.updateRanges.length, 0);
  buffer.remove(10);
  assert.deepEqual(positions(buffer), {20: [4, 5, 7]});
  assert.equal(buffer.geometry, geometry); assert.equal(geometry.getAttribute('position'), attribute);
  assert.equal(geometry.drawRange.count, 1);
  buffer.remove(20); assert.equal(geometry.drawRange.count, 0);
});

test('capacity grows geometrically, preserves live data and bounds pending update ranges', () => {
  const buffer = new PointBuffer(), geometry = buffer.geometry;
  let released = 0; geometry.addEventListener('dispose', () => released++);
  for (let i = 0; i < 3000; i++) buffer.upsert(i, [i, i + .5, 0], [1, 0, 0]);
  assert.equal(buffer.geometry, geometry); assert.equal(buffer.capacity, 4096); assert.equal(released, 2);
  assert.deepEqual(positions(buffer)[100], [100, 100.5, 0]);
  for (const name of ['position', 'color']) assert.ok(geometry.getAttribute(name).updateRanges.length <= 64);
});

test('one changed point in a large map visits one row and keeps existing buffers and native order', () => {
  const display = new PointDisplay(), before = scene(Array.from({length: 50000}, (_, i) => row(i)));
  display.update(before);
  const geometry = display.raw.geometry, attribute = geometry.getAttribute('position'); clearRanges(display.raw);
  const next = delta(before, {upserts: [row(100, 100.25)]}), original = JSON.stringify(next);
  const stats = display.update(next);
  assert.equal(stats.visited, 1); assert.equal(stats.changed, 1);
  assert.equal(display.raw.geometry, geometry); assert.equal(geometry.getAttribute('position'), attribute);
  assert.deepEqual(attribute.updateRanges, [{start: 300, count: 3}]);
  assert.equal(positions(display.raw)[100][0], 100.25);
  assert.equal(next.points[0][0], 49999); assert.equal(JSON.stringify(next), original);
});

test('incremental grid matches full aggregation after moves, deletes and observation-only changes', () => {
  const display = new PointDisplay();
  let current = scene([row(0, .1), row(1, .2), row(2, 2), row(3, 3)]);
  display.update(current); display.display(1, 3, true);
  const check = () => {
    const actual = display.gridSnapshot(), expected = pointGridCells(display.points(), 1, current.point_observations, current.cameras.length);
    assert.deepEqual(cells(actual), cells(expected));
    for (const field of ['sourceCount', 'cameraCount', 'lowerBound', 'basis', 'maxSupport']) assert.equal(actual[field], expected[field]);
  };
  check();
  const edits = [
    {observations: {0: [0], 1: [0, 1]}},
    {upserts: [row(0, 2.1), row(4, .3)], observations: {4: [1, 2, 3]}},
    {removed: [1], observationRemoved: ['1']},
    {observations: {0: [3], 2: [0, 2], 4: [1]}},
    {upserts: [row(4, -1.6)], observations: {3: [0, 0, 1]}},
    {removed: [0, 2, 3], observationRemoved: ['0', '2', '3']},
  ];
  for (const edit of edits) {
    current = delta(current, edit); display.update(current); display.display(1, 3, true); check();
    const expected = display.gridSnapshot().cells.filter(c => c.support >= 3).length;
    assert.equal(display.quantized.geometry.drawRange.count, expected);
  }
});

test('shared-frame reference counts survive deletion, and confidence/mode/interval changes retain original points', () => {
  const display = new PointDisplay(), first = scene([row(0, .1), row(1, .2), row(2, 2.2)]);
  display.update(first); display.display(1, 3, true);
  const next = delta(first, {removed: [0], observationRemoved: ['0']}); display.update(next);
  display.display(1, 3, true);
  assert.equal(display.gridSnapshot().cells.find(c => c.position[0] === 0).support, 3);
  assert.equal(display.display(1, 100, true).drawRange.count, 0);
  assert.equal(display.display(1, 1, true).drawRange.count, 2);
  assert.equal(display.display(0, 3, false), display.raw.geometry);
  assert.equal(positions(display.raw)[1][0], Math.fround(.2));
  display.display(.01, 3, true);
  assert.equal(display.gridSnapshot().cells.length, 2);
});

test('filters, corrected full snapshots and color changes refresh only their affected state', () => {
  const display = new PointDisplay(), first = scene([row(0), row(1)]);
  display.update(first); display.display(1, 3, true);
  const position = display.raw.geometry.getAttribute('position'); clearRanges(display.raw);
  display.update(first, {colorMode: 'error'});
  assert.equal(position.updateRanges.length, 0, 'color changes must not upload positions');
  const next = structuredClone(first); next.revision = 2; next.points[0][1] = 9; next.points[1][7] = 5;
  display.update(next, {errorLimit: 1}); display.display(1, 3, true);
  assert.equal(display.rows.size, 1); assert.equal(positions(display.raw)[0][0], 9);
  display.update(next, {errorLimit: 999}); assert.equal(display.rows.size, 2);
  display.update(next, {errorLimit: 0}); assert.equal(display.raw.geometry.drawRange.count, 0);
});

test('legacy and invalid evidence are revalidated as camera ranges and metadata change', () => {
  const display = new PointDisplay(), first = scene([row(0, .1), row(1, .2)]);
  first.point_observations.frame_index_by_point[0] = [0, 7];
  display.update(first); display.display(1, 3, true);
  assert.equal(display.gridSnapshot().lowerBound, true);
  let next = delta(first, {cameras: Array.from({length: 8}, (_, i) => ({frame_id: String(i)}))});
  display.update(next); display.display(1, 3, true);
  assert.equal(display.gridSnapshot().lowerBound, false);
  next = delta(next, {cameras: first.cameras}); display.update(next); display.display(1, 3, true);
  assert.equal(display.gridSnapshot().lowerBound, true);
  const legacy = {...next, revision: next.revision + 1, point_observations: undefined};
  display.update(legacy); display.display(1, 3, true);
  assert.equal(display.gridSnapshot().lowerBound, true);
  assert.equal(display.gridSnapshot().cells[0].support, 3);
});

test('workers receive coalesced changes and recover from journal gaps or an invalid base', () => {
  const display = new PointDisplay(), worker = new WorkerPointState();
  let current = scene([row(0), row(1), row(2)]); display.update(current);
  worker.apply(display.workerUpdate()); const base = worker.version;
  current = delta(current, {removed: [1], observationRemoved: ['1'], upserts: [row(3)], observations: {3: [0, 3]}}); display.update(current);
  current = delta(current, {upserts: [row(0, 8)], observations: {0: [1]}}); display.update(current);
  const update = display.workerUpdate(base), result = worker.apply(update);
  assert.equal(update.reset, false); assert.equal(update.points.length, 2); assert.deepEqual(update.removed, [1]);
  assert.deepEqual(result.points.sort((a, b) => a[0] - b[0]), [...current.points].sort((a, b) => a[0] - b[0]));
  assert.deepEqual(result.observations, current.point_observations);
  assert.throws(() => worker.apply(update), /base mismatch/);
  for (let i = 0; i < 35; i++) { current = delta(current); display.update(current); }
  const full = display.workerUpdate(base); assert.equal(full.reset, true); worker.apply(full);
  assert.equal(worker.rows.size, 3);
});

test('dense legacy cells use the maximum surviving track count after deleting its owner', () => {
  const display = new PointDisplay(), first = scene(Array.from({length: 2000}, (_, i) => row(i, 0)));
  first.point_observations = undefined; first.points[0][8] = 100;
  display.update(first); display.display(1, 3, true);
  assert.equal(display.gridSummary().maxSupport, 100);
  const next = {...first, revision: 2, points: first.points.slice(1)};
  display.update(next); display.display(1, 3, true);
  assert.equal(display.gridSummary().maxSupport, 3);
  assert.equal(display.gridSummary().cellCount, 1);
  assert.equal(display.quantized.geometry.drawRange.count, 1);
});

test('spacing runs in its worker with exact complete-map results after sparse updates', async () => {
  const display = new PointDisplay(), messages = [];
  const before = scene([row(0, 0), row(1, 1), row(2, 3)]); display.update(before);
  globalThis.self = {postMessage: message => messages.push(message)};
  try {
    await import('../point-spacing-worker.js');
    const update = display.workerUpdate(); self.onmessage({data: {update}});
    assert.deepEqual(messages[0].profile, pointSpacingProfile(before.points));
    const after = delta(before, {upserts: [row(1, 2)]}); display.update(after);
    self.onmessage({data: {update: display.workerUpdate(update.version)}});
    assert.deepEqual(messages[1].profile, pointSpacingProfile(after.points));
  } finally { delete globalThis.self; }
});
