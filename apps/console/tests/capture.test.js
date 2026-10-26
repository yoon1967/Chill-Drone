import test from 'node:test';
import assert from 'node:assert/strict';
import { connectCapture } from '../capture.js';

// A synthetic media adapter verifies permission timing and disconnect cleanup without a real camera.
function fixture(t, {denied = false, enabled = true} = {}) {
  const originals = new Map(['document', 'navigator', 'window', 'fetch'].map((key) => [key, Object.getOwnPropertyDescriptor(globalThis, key)]));
  t.after(() => { for (const [key, descriptor] of originals) { if (descriptor) Object.defineProperty(globalThis, key, descriptor); else delete globalThis[key]; } });
  const element = () => ({handlers: {}, dataset: {}, files: [], value: 'camera', readyState: 1, videoWidth: 960, videoHeight: 540,
    addEventListener(name, handler) { this.handlers[name] = handler; }, querySelector() { return {}; }, removeAttribute() {},
    load() {}, pause() {}, async play() {}, requestVideoFrameCallback() { return 1; }, cancelVideoFrameCallback() {}});
  const elements = new Map();
  const get = (id) => { if (!elements.has(id)) elements.set(id, element()); return elements.get(id); };
  get('camera-fov').value = '60';
  let permissions = 0, stopped = 0;
  const requests = [], statuses = [], starts = [];
  const track = {handlers: {}, addEventListener(name, handler) { this.handlers[name] = handler; }, stop() { stopped++; }, getSettings() { return {frameRate: 30}; }};
  const values = {
    document: {getElementById: get, createElement: () => ({getContext: () => ({})})},
    navigator: {mediaDevices: {async getUserMedia() { permissions++; if (denied) throw new DOMException('denied', 'NotAllowedError'); return {getVideoTracks: () => [track], getTracks: () => [track]}; }}},
    window: {addEventListener() {}},
    fetch: async (url, options) => {
      requests.push({url, options});
      const value = url === '/api/live' ? {enabled, demo_available: false} :
        url === '/api/live/start' ? {session_id: 'live_test', session_status: 'starting'} :
        url.startsWith('/api/live/state') ? {session_id: 'live_test', status: 'receiving', transport_ready: true} : {status: 'stopping'};
      return {ok: true, json: async () => value};
    },
  };
  for (const [key, value] of Object.entries(values)) Object.defineProperty(globalThis, key, {configurable: true, value});
  return {get, track, requests, statuses, starts, permissions: () => permissions, stopped: () => stopped,
    connect: () => connectCapture({onStart: (id) => starts.push(id), onStatus: (status) => statuses.push(status)})};
}

test('opening the page does not request permission; denial does not start SLAM', async (t) => {
  const f = fixture(t, {denied: true}); await f.connect();
  assert.equal(f.permissions(), 0);
  await f.get('camera-start').handlers.click();
  assert.equal(f.permissions(), 1);
  assert.equal(f.requests.some((r) => r.url === '/api/live/start'), false);
  assert.equal(f.statuses.at(-1).state, 'denied');
  assert.match(f.get('camera-status').textContent, /권한이 거부/);
  assert.equal(f.get('camera-start').disabled, false);
});

test('allowed media starts one session; device loss stops that session and releases media', async (t) => {
  const f = fixture(t); const handle = await f.connect();
  await f.get('camera-start').handlers.click();
  assert.deepEqual(f.starts, ['live_test']);
  assert.equal(f.get('camera-stop').disabled, false);
  await f.track.handlers.ended();
  assert.equal(f.statuses.at(-1).state, 'disconnected');
  assert.equal(f.stopped(), 1);
  const stop = f.requests.find((r) => r.url === '/api/live/stop');
  assert.deepEqual(JSON.parse(stop.options.body), {session_id: 'live_test'});
  assert.equal(f.get('camera-start').disabled, true); // Backend still finalizing.
  handle.update({session_id: 'live_test', session_status: 'complete'});
  assert.equal(f.get('camera-start').disabled, false);
});

test('record-only server disables new input without asking for a camera', async (t) => {
  const f = fixture(t, {enabled: false}); await f.connect();
  assert.equal(f.permissions(), 0);
  assert.equal(f.get('camera-start').disabled, true);
  assert.equal(f.statuses.at(-1).state, 'disabled');
});
