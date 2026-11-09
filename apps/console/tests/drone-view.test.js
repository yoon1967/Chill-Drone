import test from 'node:test';
import assert from 'node:assert/strict';
import {connectDrones, renderDroneStages, wifiSignal} from '../drone-view.js';

function fixture(t, options = {}) {
  const original = Object.getOwnPropertyDescriptor(globalThis, 'document');
  t.after(() => original ? Object.defineProperty(globalThis, 'document', original) : delete globalThis.document);
  const elements = new Map();
  function node() {
    return {hidden: false, disabled: false, children: [], attrs: {}, handlers: {},
      addEventListener(name, handler) { this.handlers[name] = handler; },
      setAttribute(name, value) { this.attrs[name] = value; },
      replaceChildren() { this.children = []; }, append(...values) { this.children.push(...values); }};
  }
  function get(id) { if (!elements.has(id)) elements.set(id, node()); return elements.get(id); }
  get('drone-resource-panel').hidden = true;
  Object.defineProperty(globalThis, 'document', {configurable: true, value: {createElement: node}});
  const requests = [], timers = [], previewTimers = [];
  const state = {connected: true, selected: {ssid: 'TELLO-test'}, resources: [
    {name: '<sensor>', value: '<script>', status: 'unconfirmed'}], video: {note: 'SDK camera'}};
  let fail = false, scanResult = null;
  const handle = connectDrones({...options, element: get, schedule: fn => timers.push(fn),
    schedulePreview: (fn, delay) => { const timer = {fn, delay, cancelled: false}; previewTimers.push(timer); return timer; },
    cancelPreview: timer => { timer.cancelled = true; }, fetcher: async (url, options) => {
    requests.push({url, options});
    if (fail) throw new Error('offline');
    return {ok: true, json: async () => url.endsWith('/scan') ? scanResult || {items: [
      {id: 'TELLO-test', ssid: 'TELLO-test', kind: 'wifi', signal_pct: 75, drone_candidate: true},
      {id: 'home', ssid: '<Home>', kind: 'wifi', signal_pct: 45}]} : structuredClone(state)};
  }});
  return {handle, get, requests, timers, previewTimers, state, setScan: value => { scanResult = value; }, setFail: () => { fail = true; }};
}
const settle = () => new Promise(resolve => setImmediate(resolve));

test('denied and empty scans show useful guidance without selectable fallback entries', async t => {
  const f = fixture(t);
  await settle();
  await f.handle.action('scan');
  f.get('drone-list').children[0].handlers.click();
  f.setScan({items: [], warning: 'wifi_location_permission_required'});
  await f.handle.action('scan');
  assert.equal(f.get('drone-list').children.length, 0);
  assert.equal(f.get('drone-connect').disabled, true);
  assert.match(f.get('drone-status').textContent, /Windows 설정.*위치 서비스/);
  assert.match(f.get('drone-wifi-scan-status').textContent, /Windows 설정.*위치 서비스/);
  assert.equal(f.get('drone-wifi-empty').hidden, false);
  f.setScan({items: [], warning: null});
  await f.handle.action('scan');
  assert.match(f.get('drone-status').textContent, /검색된 Wi-Fi가 없습니다/);
  assert.match(f.get('drone-wifi-scan-status').textContent, /검색된 Wi-Fi가 없습니다/);
});

test('live reconstruction content stays hidden until mapping starts; saved tab remains usable', () => {
  const elements = {'reconstruction-stage': {}};
  const render = (source, state, step = 2) => renderDroneStages(source, state, id => elements[id], step);
  render('live', {}); assert.equal(elements['reconstruction-stage'].hidden, true);
  render('live', {connected: true}); assert.equal(elements['reconstruction-stage'].hidden, true);
  render('live', {connected: true, mapping_active: true}); assert.equal(elements['reconstruction-stage'].hidden, false);
  for (const step of [0, 1]) {
    render('live', {connected: true, mapping_active: true}, step);
    assert.equal(elements['reconstruction-stage'].hidden, true);
  }
  render('live', {finalizing: true}); assert.equal(elements['reconstruction-stage'].hidden, false);
  render('live', {mapping_session_id: 'finished', map_available: true}); assert.equal(elements['reconstruction-stage'].hidden, false);
  render('live', {connected: true, mapping_session_id: null}); assert.equal(elements['reconstruction-stage'].hidden, true);
  render('saved', {}); assert.equal(elements['reconstruction-stage'].hidden, false);
});

test('scan reveals a Wi-Fi list, a single click selects, double click connects only once', async t => {
  const f = fixture(t);
  f.state.connected = false;
  await settle();
  assert.equal(f.get('drone-wifi-status').hidden, true);
  assert.equal(f.get('drone-wifi-signal').hidden, true);
  await f.handle.action('scan');
  assert.equal(f.get('drone-wifi-results').hidden, false);
  assert.equal(f.get('drone-wifi-empty').hidden, true);
  assert.equal(f.get('drone-wifi-status').hidden, false);
  assert.equal(f.get('drone-wifi-signal').hidden, false);
  const rows = f.get('drone-list').children;
  assert.equal(rows.length, 2);
  assert.equal(rows[1].children[0].textContent, '<Home>');
  rows[0].handlers.click();
  assert.equal(f.requests.some(r => r.url.endsWith('/connect')), false);
  assert.equal(rows[0].attrs['aria-selected'], 'true');
  rows[0].handlers.dblclick(); rows[0].handlers.dblclick();
  await settle();
  const connects = f.requests.filter(r => r.url.endsWith('/connect'));
  assert.equal(connects.length, 1);
  assert.deepEqual(JSON.parse(connects[0].options.body), {id: 'TELLO-test'});
});

test('explicit current-network action works independently of the scanned Wi-Fi list', async t => {
  const f = fixture(t);
  await settle();
  f.get('drone-current-connect').handlers.click();
  await settle();
  const request = f.requests.find(r => r.url.endsWith('/connect'));
  assert.deepEqual(JSON.parse(request.options.body), {id: 'current-network'});
});

test('keyboard connection and live signal changes remain separate from SDK state', async t => {
  const f = fixture(t);
  await settle(); await f.handle.action('scan');
  f.get('drone-list').children[0].handlers.keydown({key: 'Enter', preventDefault() {}});
  await settle();
  assert.ok(f.requests.some(r => r.url.endsWith('/connect')));
  assert.equal(f.get('drone-workspace').open, true);
  f.state.wifi = {status: 'ok', ssid: 'TELLO-test', connected: true, signal_pct: 83, age_s: 1};
  await f.timers.shift()();
  assert.match(f.get('drone-wifi-signal').textContent, /83%.*강함/);
  f.state.wifi.signal_pct = 18;
  await f.timers.shift()();
  assert.match(f.get('drone-wifi-signal').textContent, /18%.*약함/);
  f.state.wifi = {status: 'unavailable', connected: null};
  await f.timers.shift()();
  assert.match(f.get('drone-wifi-signal').textContent, /확인 불가/);
  assert.doesNotMatch(f.get('drone-list').children[0].signalLabel.textContent, /Wi-Fi 연결됨/);
  assert.equal(wifiSignal({status: 'ok', connected: true, signal_pct: 0}), '0% · 약함');
  assert.match(wifiSignal({status: 'stale', signal_pct: 90}), /오래된/);
});

test('connected state never opens the panel or probes resources until details is clicked', async t => {
  const f = fixture(t);
  await settle();
  assert.equal(f.get('drone-resource-panel').hidden, true);
  assert.equal(f.get('drone-details').disabled, false);
  assert.equal(f.requests.some(r => r.url.endsWith('explore')), false);
  f.get('drone-details').handlers.click();
  await settle();
  assert.equal(f.get('drone-resource-panel').hidden, false);
  assert.equal(f.get('drone-details').attrs['aria-expanded'], 'true');
  assert.equal(f.get('drone-details').textContent, '상세 정보 접기');
  const request = f.requests.find(r => r.url.endsWith('explore'));
  assert.equal(request.options.headers['X-Mapping-Client'], '1');
  assert.equal(f.get('drone-resources').children[0].children[0].textContent, '<sensor>');
  assert.match(f.get('drone-resources').children[0].children[1].textContent, /미확인.*<script>/);
  f.get('drone-details').handlers.click();
  await settle();
  assert.equal(f.get('drone-resource-panel').hidden, true);
  assert.equal(f.get('drone-details').textContent, '자세히 보기');
  await f.timers.shift()();
  assert.equal(f.get('drone-resource-panel').hidden, true);
  f.get('drone-details').handlers.click();
  await settle();
  assert.equal(f.get('drone-resource-panel').hidden, false);
  assert.equal(f.requests.some(r => r.url.endsWith('preview/stop')), false);
  assert.equal(f.get('drone-camera-panel').hidden, false);
});

test('disconnect hides the resource panel and late image load cannot reopen it', async t => {
  const f = fixture(t);
  await settle();
  await f.handle.action('explore');
  f.state.connected = false;
  await f.timers.shift()();
  assert.equal(f.get('drone-resource-panel').hidden, true);
  assert.equal(f.get('drone-camera-panel').hidden, true);
  assert.equal(f.get('drone-details').disabled, true);
  f.get('drone-resource-image').handlers.load();
  assert.equal(f.get('drone-resource-image').hidden, true);
});

test('camera preview refreshes after each image and stops after disconnection', async t => {
  const f = fixture(t);
  f.state.owner = 'video-input';
  await settle();
  const image = f.get('drone-resource-image');
  const first = image.src;
  assert.match(first, /preview\.jpg\?t=/);
  image.handlers.load();
  assert.equal(f.previewTimers.at(-1).delay, 100);
  const timer = f.previewTimers.at(-1);
  timer.fn();
  assert.notEqual(image.src, undefined);
  image.handlers.error();
  assert.equal(f.previewTimers.at(-1).delay, 500);
  f.state.connected = false;
  await f.timers.shift()();
  assert.equal(f.previewTimers.at(-1).cancelled, true);
  const after = f.previewTimers.length;
  image.handlers.load();
  assert.equal(f.previewTimers.length, after);
  assert.equal(image.hidden, true);
});

test('camera space appears only after connection; explicit disconnect pins the mapping session', async t => {
  const f = fixture(t);
  f.state.connected = false;
  await settle();
  await f.timers.shift()();
  assert.equal(f.get('drone-camera-panel').hidden, true);
  f.state.connected = true;
  f.state.owner = 'video-input';
  f.state.mapping_active = true;
  f.state.mapping_session_id = 'drone-session';
  f.state.disconnect_available = true;
  await f.timers.shift()();
  assert.equal(f.get('drone-camera-panel').hidden, false);
  assert.equal(f.get('drone-map-start').disabled, true);
  assert.equal(f.get('drone-disconnect').disabled, false);
  f.get('drone-disconnect').handlers.click();
  await settle();
  const request = f.requests.find(r => r.url.endsWith('/disconnect'));
  assert.deepEqual(JSON.parse(request.options.body), {session_id: 'drone-session'});
});

test('gateway mode shows the control gateway link and never offers a second direct connection', async t => {
  const f = fixture(t);
  Object.assign(f.state, {owner: 'gateway', selected: null, disconnect_available: false,
    gateway: {reachable: true, connected: true, battery_pct: 87.4, video_available: false}});
  await settle();
  for (const id of ['drone-wifi-results', 'drone-current-network', 'drone-details', 'drone-video-start'])
    assert.equal(f.get(id).hidden, true, id);
  assert.equal(f.get('drone-link-state').textContent, '관제 게이트웨이 연결됨 · 배터리 87% · 영상 꺼짐');
  assert.equal(f.get('drone-disconnect').textContent, '지도 마무리');
  assert.equal(f.get('drone-camera-panel').hidden, false);
  assert.equal(f.get('drone-map-start').disabled, true);   // 게이트웨이 영상이 꺼져 있으면 지도를 시작하지 않는다
  assert.match(f.get('drone-flow-status').textContent, /관제 화면에서 드론 영상을 켜면/);
  assert.match(f.get('drone-image-status').textContent, /관제 화면에서 봅니다/);
  f.state.gateway.video_available = true;
  await f.timers.shift()();
  assert.equal(f.get('drone-map-start').disabled, false);
  assert.match(f.get('drone-link-state').textContent, /영상 켜짐/);
  Object.assign(f.state, {connected: false, mapping_active: true, disconnect_available: true,
    gateway: {reachable: true, connected: false, battery_pct: null, video_available: false}});
  await f.timers.shift()();
  assert.equal(f.get('drone-link-state').textContent, '관제 게이트웨이 · 드론 연결 전');
  assert.match(f.get('drone-flow-status').textContent, /지도 마무리를 눌러 지도를 마무리하세요/);
  Object.assign(f.state, {mapping_active: false, disconnect_available: false, gateway: {reachable: false}});
  await f.timers.shift()();
  assert.equal(f.get('drone-link-state').textContent, '관제 게이트웨이 연결 안 됨');
  assert.match(f.get('drone-flow-status').textContent, /관제 게이트웨이에 연결할 수 없습니다/);
  delete f.state.owner; delete f.state.gateway;   // 직접 연결 모드는 그대로
  Object.assign(f.state, {connected: true, selected: {ssid: 'TELLO-test'}});
  await f.timers.shift()();
  assert.equal(f.get('drone-wifi-results').hidden, false);
  assert.equal(f.get('drone-current-network').hidden, false);
  assert.equal(f.get('drone-disconnect').textContent, '드론 연결 해제');
  assert.equal(f.get('drone-link-state').textContent, '연결됨 · TELLO-test');
});

test('server outage disables connection actions and reports unknown drone state', async t => {
  const f = fixture(t);
  await settle();
  f.setFail();
  await f.timers.shift()();
  assert.equal(f.get('drone-details').disabled, true);
  assert.match(f.get('drone-status').textContent, /상태를 확인하지 못했습니다/);
});

test('mapping handoff keeps the camera visible and reports preparation and failure in place', async t => {
  let finish;
  const f = fixture(t, {onMapStart: () => new Promise(resolve => { finish = resolve; })});
  await settle();
  const starting = f.get('drone-map-start').handlers.click();
  assert.equal(f.get('drone-map-start').disabled, true);
  assert.match(f.get('drone-flow-status').textContent, /연결하고 있습니다/);
  f.state.connected = false;
  f.state.mapping_active = true;
  f.state.disconnect_available = true;
  await f.timers.shift()();
  assert.equal(f.get('drone-camera-panel').hidden, false);
  f.handle.mappingStatus({kind: 'tello-map', state: 'failed', message: '영상 입력 실패 · 다시 시도하세요.'});
  assert.match(f.get('drone-flow-status').textContent, /영상 입력 실패/);
  finish(); await starting;
  assert.equal(f.get('drone-camera-panel').hidden, false);
});
