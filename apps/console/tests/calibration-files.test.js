import test from 'node:test';
import assert from 'node:assert/strict';
import {feedCalibrationFiles, validateCalibrationFiles} from '../calibration-files.js';

const photo = (name, size = 1000) => ({name, type: 'image/jpeg', size});
const videoFile = () => ({name: 'board.mp4', type: 'video/mp4', size: 1000});
const canvas = () => ({width: 0, height: 0, toBlob(callback) { callback(new Blob(['jpeg'], {type: 'image/jpeg'})); }});

test('file choice accepts one video or multiple photos and rejects mixed or undersized sets', () => {
  assert.equal(validateCalibrationFiles([videoFile()]).kind, 'video');
  assert.equal(validateCalibrationFiles([photo('a.jpg'), photo('b.jpg')]).kind, 'photos');
  assert.throws(() => validateCalibrationFiles([photo('a.jpg')]), /2~120장/);
  assert.throws(() => validateCalibrationFiles([photo('a.jpg'), videoFile()]), /2~120장/);
  assert.throws(() => validateCalibrationFiles([{name: 'a.gif', type: 'image/gif', size: 10}]), /2~120장/);
  assert.equal(validateCalibrationFiles([photo('more.jpg')], {additional: true}).kind, 'photos');
});

test('photos are normalized to one resolution before feeding the existing frame API', async t => {
  const original = globalThis.createImageBitmap;
  t.after(() => { if (original) globalThis.createImageBitmap = original; else delete globalThis.createImageBitmap; });
  let closed = 0, count = 0;
  globalThis.createImageBitmap = async file => ({width: file.width, height: file.height, close() { closed++; }});
  const selected = validateCalibrationFiles([
    {...photo('a.jpg'), width: 2560, height: 1920}, {...photo('b.jpg'), width: 800, height: 600}]);
  const surface = canvas(), dimensions = [];
  const size = await feedCalibrationFiles(selected, {canvas: surface,
    context: {drawImage(image, x, y, width, height) { dimensions.push([width, height]); }},
    onFrame: async blob => { assert.equal(blob.type, 'image/jpeg'); return {accepted: ++count}; },
    onProgress() {}});
  assert.deepEqual(size, [1280, 960]); assert.deepEqual(dimensions, [[1280, 960], [1280, 960]]);
  assert.equal(count, 2); assert.equal(closed, 2);
});

test('video is sampled in browser and its object URL is released', async t => {
  const create = URL.createObjectURL, revoke = URL.revokeObjectURL;
  let released = false, count = 0;
  URL.createObjectURL = () => 'blob:test'; URL.revokeObjectURL = () => { released = true; };
  t.after(() => { URL.createObjectURL = create; URL.revokeObjectURL = revoke; });
  const handlers = new Map(), video = {duration: 1, videoWidth: 960, videoHeight: 720, readyState: 1,
    addEventListener(name, callback) { handlers.set(name, callback); }, removeEventListener(name) { handlers.delete(name); },
    set currentTime(value) { this.time = value; queueMicrotask(() => handlers.get('seeked')?.()); },
    removeAttribute() {}, load() {}};
  await feedCalibrationFiles(validateCalibrationFiles([videoFile()]), {canvas: canvas(), context: {drawImage() {}},
    video, onFrame: async () => ({accepted: ++count}), onProgress() {}});
  assert.equal(count, 30); assert.equal(released, true);
});
