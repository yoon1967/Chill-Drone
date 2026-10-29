import test from 'node:test';
import assert from 'node:assert/strict';
import { connectionStatus, monitorConnections } from '../map-source.js';

const active = {source: 'live', session_id: 'current', status: 'receiving', input: 'tello_camera',
  input_origin: 'tello_sdk_camera', transport_ready: true, transport_received: 10, map_available: true, tracking_state: 2};

test('unreachable server does not accuse the drone; a responding error is distinct', () => {
  const down = connectionStatus({reachable: false, state: active});
  assert.equal(down.server.text, '연결 실패');
  for (const key of ['drone', 'video', 'map']) assert.equal(down[key].text, '확인 불가');
  assert.equal(connectionStatus({responseOK: false}).server.text, '응답 오류');
  const failed = connectionStatus({state: {...active, status: 'failed', last_error: 'tello_sdk_unavailable_check_wifi'}});
  assert.equal(failed.server.text, '연결됨'); assert.equal(failed.drone.text, '연결 실패');
  assert.equal(failed.video.text, '입력 없음');
});

test('video, tracking and completed records cannot imply a current connection', () => {
  assert.equal(connectionStatus({state: active}).video.text, '수신 중');
  const stalled = connectionStatus({state: {...active, tracking_state: 3}, fresh: false});
  assert.equal(stalled.video.text, '수신 지연'); assert.equal(stalled.map.text, '추적 상실');
  const stopped = connectionStatus({state: {...active, status: 'complete'}});
  assert.equal(stopped.drone.text, '종료'); assert.equal(stopped.video.text, '종료');
  assert.equal(stopped.map.text, '기록 보존');
  assert.equal(connectionStatus({state: {...active, input_origin: 'gateway_reported_fake'}}).drone.text, '검증용 입력');
  assert.equal(connectionStatus({state: active, capture: {kind: 'tello'}}).map.text, '해당 없음');
});

test('new starts ignore the previous run and preflight failures belong to SLAM', () => {
  const state = {...active, status: 'failed', last_error: 'tello_sdk_unavailable_check_wifi'};
  const pending = connectionStatus({state, capture: {state: 'connecting', kind: 'tello-map'}});
  assert.equal(pending.drone.text, '연결 중'); assert.equal(pending.video.text, '대기');
  const missing = connectionStatus({state, capture: {state: 'disconnected', kind: 'tello-map', error: 'slam_image_missing'}});
  assert.equal(missing.map.text, '처리 실패'); assert.equal(missing.drone.text, '확인 전');
  const otherSession = connectionStatus({state: active, capture: {session: 'old', error: 'slam_image_missing'}});
  assert.equal(otherSession.map.text, '생성 중');
  const lostVideo = connectionStatus({state: {...active, status: 'failed', last_error: 'tello_video_timeout_check_wifi_and_udp'}});
  assert.equal(lostVideo.video.text, '수신 실패'); assert.notEqual(lostVideo.drone.text, '연결됨');
});

test('read-only monitor recovers, detects stalled frames and survives invalid responses', async (t) => {
  const original = ['window', 'location'].map(key => [key, Object.getOwnPropertyDescriptor(globalThis, key)]);
  t.after(() => { for (const [key, value] of original) value ? Object.defineProperty(globalThis, key, value) : delete globalThis[key]; });
  const handlers = new Map();
  Object.defineProperty(globalThis, 'window', {configurable: true, value: {
    addEventListener: (name, fn) => handlers.set(name, fn), removeEventListener: name => handlers.delete(name)}});
  Object.defineProperty(globalThis, 'location', {configurable: true, value: {hash: '#live'}});
  const nodes = new Map(), element = id => { if (!nodes.has(id)) nodes.set(id, {dataset: {}}); return nodes.get(id); };
  let next, now = 0, fail = false, invalid = false;
  const urls = [];
  const monitor = monitorConnections({element, clock: () => now, schedule: fn => { next = fn; }, cancel() {},
    fetcher: async (url, options) => {
      urls.push(url); assert.equal(options.method, undefined); // Never start, heartbeat or stop a session.
      if (fail) throw new Error('offline');
      return {ok: true, json: async () => invalid ? {wrong: true} : active};
    }});
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(element('connection-server').textContent, '연결됨');
  now = 6000; await next(); assert.equal(element('connection-video').textContent, '수신 지연');
  fail = true; await next(); assert.equal(element('connection-drone').textContent, '확인 불가');
  fail = false; invalid = true; await next(); assert.equal(element('connection-server').textContent, '응답 오류');
  invalid = false; await next(); assert.equal(element('connection-server').textContent, '연결됨');
  location.hash = '#saved'; handlers.get('hashchange')(); assert.equal(element('connection-monitor').hidden, true);
  assert.ok(urls.every(url => url === '/api/live/state')); monitor.close();
});
