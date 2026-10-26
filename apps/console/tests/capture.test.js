import test from 'node:test';
import assert from 'node:assert/strict';
import { connectCapture, captureConfiguration } from '../capture.js';

const deferred = () => { let resolve; const promise = new Promise((r) => { resolve = r; }); return {promise, resolve}; };
const reply = (value, ok = true) => ({ok, json: async () => value});
function stream(label = 'Test camera') {
  const track = {handlers: {}, label, readyState: 'live', stops: 0,
    addEventListener(name, handler) { this.handlers[name] = handler; },
    stop() { this.stops++; this.readyState = 'ended'; },
    getSettings() { return {frameRate: 30}; }};
  return {track, getVideoTracks: () => [track], getTracks: () => [track]};
}

// Synthetic media adapter: these tests never request a physical camera or Docker.
function fixture(t, {denied = false, enabled = true, devices = [], getMedia, fetcher, encode} = {}) {
  const originals = new Map(['document', 'navigator', 'window', 'fetch'].map((key) => [key, Object.getOwnPropertyDescriptor(globalThis, key)]));
  t.after(() => { for (const [key, descriptor] of originals) { if (descriptor) Object.defineProperty(globalThis, key, descriptor); else delete globalThis[key]; } });
  let callback = 0;
  const callbacks = new Map(), windowEvents = {}, deviceEvents = {};
  const element = () => ({handlers: {}, dataset: {}, files: [], value: '', readyState: 2, videoWidth: 960, videoHeight: 540,
    addEventListener(name, handler) { this.handlers[name] = handler; },
    removeEventListener(name, handler) { if (this.handlers[name] === handler) delete this.handlers[name]; },
    querySelector() { return {}; }, replaceChildren(...items) { this.options = items; },
    removeAttribute() {}, load() {}, pause() {}, async play() {},
    requestVideoFrameCallback(fn) { callbacks.set(++callback, fn); return callback; },
    cancelVideoFrameCallback(id) { callbacks.delete(id); }});
  const elements = new Map(), get = (id) => { if (!elements.has(id)) elements.set(id, element()); return elements.get(id); };
  get('camera-fov').value = '60'; get('input-source').value = 'camera';
  const media = stream();
  const requests = [], statuses = [], starts = [], constraints = [];
  let permissions = 0;
  const values = {
    document: {getElementById: get, createElement: (kind) => kind === 'canvas' ? {
      getContext: () => ({drawImage() {}}), toBlob: (done) => encode ? encode(done) : done(new Blob(['synthetic']))
    } : element()},
    navigator: {mediaDevices: {
      async enumerateDevices() { return devices; }, addEventListener(name, fn) { deviceEvents[name] = fn; },
      async getUserMedia(config) { permissions++; constraints.push(config); if (denied) throw new DOMException('denied', 'NotAllowedError'); return getMedia ? getMedia() : media; }
    }},
    window: {addEventListener(name, fn) { windowEvents[name] = fn; }},
    fetch: async (url, options) => {
      requests.push({url, options});
      const value = url === '/api/live' ? {enabled, demo_available: false} :
        url === '/api/live/start' ? {session_id: 'live_test', session_status: 'starting'} :
        url.startsWith('/api/live/state') ? {session_id: 'live_test', status: 'receiving', transport_ready: true} :
        url.startsWith('/api/live/frame') ? {frame_drops: 0, queue_depth: 1} : {status: 'stopping'};
      return fetcher ? fetcher(url, options, value) : reply(value);
    },
  };
  for (const [key, value] of Object.entries(values)) Object.defineProperty(globalThis, key, {configurable: true, value});
  return {get, media, requests, statuses, starts, constraints, windowEvents, deviceEvents,
    permissions: () => permissions,
    async deliver(data) { const [id, fn] = callbacks.entries().next().value; callbacks.delete(id); return fn(0, data); },
    connect: () => connectCapture({onStart: (id) => starts.push(id), onStatus: (status) => statuses.push(status)})};
}

test('page/device enumeration is lazy; denial never starts SLAM and can be retried', async (t) => {
  const f = fixture(t, {denied: true}); await f.connect();
  assert.equal(f.permissions(), 0);
  await f.get('camera-start').handlers.click();
  assert.equal(f.permissions(), 1);
  assert.equal(f.requests.some((r) => r.url === '/api/live/start'), false);
  assert.equal(f.statuses.at(-1).state, 'denied');
  assert.match(f.get('camera-status').textContent, /권한이 거부/);
  assert.equal(f.get('camera-start').disabled, false);
});

test('selected video device is requested exactly, microphone stays off, device list refreshes', async (t) => {
  const devices = [{kind: 'audioinput', deviceId: 'mic'}, {kind: 'videoinput', deviceId: 'camera-a', label: 'Built-in'}];
  const f = fixture(t, {devices}); await f.connect();
  assert.equal(f.get('camera-device').options.length, 2);
  f.get('camera-device').value = 'camera-a';
  await f.get('camera-start').handlers.click();
  assert.deepEqual(f.constraints[0].video.deviceId, {exact: 'camera-a'});
  assert.equal(f.constraints[0].audio, false);
  assert.equal(f.get('camera-device').disabled, true);
  assert.match(f.get('camera-input-info').textContent, /960×540.*미보정/);
  devices.length = 0; await f.deviceEvents.devicechange();
  assert.equal(f.get('camera-device').value, '');
  assert.match(f.get('camera-device-note').textContent, /목록이 없습니다/);
});

test('device loss stops only the owned session and frees camera until backend finalizes', async (t) => {
  const f = fixture(t); const handle = await f.connect();
  await f.get('camera-start').handlers.click();
  assert.deepEqual(f.starts, ['live_test']);
  await f.media.track.handlers.ended();
  assert.equal(f.statuses.at(-1).state, 'disconnected');
  assert.equal(f.media.track.stops, 1);
  const stop = f.requests.find((r) => r.url === '/api/live/stop');
  assert.deepEqual(JSON.parse(stop.options.body), {session_id: 'live_test'});
  assert.equal(f.get('camera-start').disabled, true);
  handle.update({session_id: 'live_test', session_status: 'complete'});
  assert.equal(f.get('camera-start').disabled, false);
});

test('cancel pending permission; late old stream is released without touching a new session', async (t) => {
  const first = deferred(), oldMedia = stream('old'), newMedia = stream('new');
  let calls = 0;
  const f = fixture(t, {getMedia: () => ++calls === 1 ? first.promise : newMedia}); await f.connect();
  const pending = f.get('camera-start').handlers.click();
  assert.equal(f.get('camera-stop').disabled, false);
  await f.get('camera-stop').handlers.click();
  assert.match(f.get('camera-status').textContent, /연결을 취소/);
  await f.get('camera-start').handlers.click();
  first.resolve(oldMedia); await pending;
  assert.equal(oldMedia.track.stops, 1);
  assert.equal(newMedia.track.stops, 0);
  assert.equal(f.get('input-preview').srcObject, newMedia);
  assert.equal(f.starts.length, 1);
});

test('cancel pending server start; late session is stopped once without rendering it', async (t) => {
  const start = deferred(), requested = deferred();
  const f = fixture(t, {fetcher: (url, options, value) => {
    if (url === '/api/live/start') { requested.resolve(); return start.promise; }
    return reply(value);
  }});
  await f.connect();
  const pending = f.get('camera-start').handlers.click(); await requested.promise;
  await f.get('camera-stop').handlers.click();
  assert.equal(f.media.track.stops, 1);
  assert.equal(f.get('camera-start').disabled, true);
  start.resolve(reply({session_id: 'late', session_status: 'starting'})); await pending;
  assert.equal(f.starts.length, 0);
  assert.equal(f.requests.filter((r) => r.url === '/api/live/stop').length, 1);
  assert.deepEqual(JSON.parse(f.requests.at(-1).options.body), {session_id: 'late'});
});

test('server failure releases capture even if no further frame callback arrives', async (t) => {
  const f = fixture(t); const handle = await f.connect();
  await f.get('camera-start').handlers.click();
  handle.update({session_id: 'live_test', session_status: 'failed', last_error: 'tracker stopped'});
  assert.equal(f.media.track.stops, 1);
  assert.equal(f.get('input-preview').srcObject, null);
  assert.equal(f.get('camera-start').disabled, false);
  assert.equal(f.statuses.at(-1).state, 'failed');
  assert.match(f.get('camera-status').textContent, /tracker stopped/);
});

test('frame metadata preserves media timestamps and capture sequence gaps; duplicate time is ignored', async (t) => {
  const f = fixture(t); await f.connect(); await f.get('camera-start').handlers.click();
  await f.deliver({mediaTime: 12, presentedFrames: 100});
  await f.deliver({mediaTime: 12, presentedFrames: 101});
  await f.deliver({mediaTime: 12.1, presentedFrames: 103});
  const frames = f.requests.filter((r) => r.url.startsWith('/api/live/frame'));
  assert.equal(frames.length, 2);
  assert.equal(frames[0].options.headers['X-Frame-Timestamp'], '0');
  assert.equal(frames[1].options.headers['X-Frame-Sequence'], '3');
  assert.ok(Math.abs(Number(frames[1].options.headers['X-Frame-Timestamp']) - .1) < 1e-8);
});

test('one encoding/upload at a time; canceled encoding never uploads into the next session', async (t) => {
  const encoded = deferred();
  const f = fixture(t, {getMedia: () => stream(), encode: (done) => encoded.promise.then(done)}); const handle = await f.connect();
  await f.get('camera-start').handlers.click();
  const oldFrame = f.deliver({mediaTime: 1, presentedFrames: 1});
  await f.deliver({mediaTime: 2, presentedFrames: 2});
  await f.get('camera-stop').handlers.click();
  handle.update({session_id: 'live_test', session_status: 'complete'});
  await f.get('camera-start').handlers.click();
  encoded.resolve(new Blob(['late'])); await oldFrame;
  assert.equal(f.requests.filter((r) => r.url.startsWith('/api/live/frame')).length, 0);
  assert.equal(f.statuses.at(-1).state, 'receiving');
});

test('frame upload failure releases media and stops session; pagehide is also scoped', async (t) => {
  const f = fixture(t, {fetcher: (url, options, value) => reply(url.startsWith('/api/live/frame') ? {error: 'session gone'} : value, !url.startsWith('/api/live/frame'))});
  await f.connect(); await f.get('camera-start').handlers.click();
  await f.deliver({mediaTime: 1, presentedFrames: 1});
  assert.equal(f.media.track.stops, 1);
  assert.equal(f.statuses.at(-1).state, 'disconnected');
  f.windowEvents.pagehide();
  assert.equal(f.requests.filter((r) => r.url === '/api/live/stop').length, 1);
});

test('pagehide frees camera and sends one keepalive stop for its session', async (t) => {
  const f = fixture(t); await f.connect(); await f.get('camera-start').handlers.click();
  f.windowEvents.pagehide(); f.windowEvents.pagehide();
  assert.equal(f.media.track.stops, 1);
  assert.equal(f.requests.at(-1).options.keepalive, true);
  assert.deepEqual(JSON.parse(f.requests.at(-1).options.body), {session_id: 'live_test'});
});

test('invalid calibration JSON is rejected before camera permission; ratio mismatch releases camera before SLAM', async (t) => {
  const f = fixture(t); await f.connect();
  f.get('camera-calibration').files = [{size: 10, text: async () => '{bad'}];
  await f.get('camera-start').handlers.click();
  assert.equal(f.permissions(), 0);
  f.get('camera-calibration').files = [{size: 10, text: async () => JSON.stringify({width: 480, height: 640, fx: 400, fy: 400, cx: 240, cy: 320})}];
  await f.get('camera-start').handlers.click();
  assert.equal(f.media.track.stops, 1);
  assert.equal(f.requests.some((r) => r.url === '/api/live/start'), false);
  assert.match(f.get('camera-status').textContent, /가로세로 비율/);
});

test('record-only server disables new input without asking for a camera', async (t) => {
  const f = fixture(t, {enabled: false}); await f.connect();
  assert.equal(f.permissions(), 0);
  assert.equal(f.get('camera-start').disabled, true);
  assert.equal(f.get('camera-device').disabled, true);
});

test('capture configuration caps long side, preserves portrait orientation and validates calibration', () => {
  assert.deepEqual(captureConfiguration(1920, 1080, 30, 60), {width: 960, height: 540, fps: 30, fov: 60, camera: null});
  assert.equal(captureConfiguration(720, 1280, 30, 60).height, 960);
  for (const args of [[0, 540, 30, 60], [200, 100, 30, 60], [960, 540, 120, 60], [960, 540, 30, NaN]]) {
    assert.throws(() => captureConfiguration(...args));
  }
  const camera = {width: 1920, height: 1080, fx: 1000, fy: 1000, cx: 960, cy: 540, distortion: [0, 0, 0, 0]};
  assert.equal(captureConfiguration(960, 540, 30, NaN, camera).camera, camera);
  for (const bad of [{...camera, fx: '1000'}, {...camera, cy: 2000}, {...camera, distortion: [NaN, 0, 0, 0]}, []]) {
    assert.throws(() => captureConfiguration(960, 540, 30, 60, bad));
  }
});
