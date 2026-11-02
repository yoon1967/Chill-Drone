import test from 'node:test';
import assert from 'node:assert/strict';
import {connectDrones, renderDroneStages} from '../drone-view.js';

function fixture(t) {
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
  const requests = [], timers = [];
  const state = {connected: true, selected: {ssid: 'TELLO-test'}, resources: [
    {name: '<sensor>', value: '<script>', status: 'unconfirmed'}], video: {note: 'SDK camera'}};
  let fail = false;
  const handle = connectDrones({element: get, schedule: fn => timers.push(fn), fetcher: async (url, options) => {
    requests.push({url, options});
    if (fail) throw new Error('offline');
    return {ok: true, json: async () => structuredClone(state)};
  }});
  return {handle, get, requests, timers, state, setFail: () => { fail = true; }};
}
const settle = () => new Promise(resolve => setImmediate(resolve));

test('live reconstruction content stays hidden until mapping starts; saved tab remains usable', () => {
  const stage = {};
  const render = (source, state) => renderDroneStages(source, state, () => stage);
  render('live', {}); assert.equal(stage.hidden, true);
  render('live', {connected: true}); assert.equal(stage.hidden, true);
  render('live', {connected: true, mapping_active: true}); assert.equal(stage.hidden, false);
  render('live', {finalizing: true}); assert.equal(stage.hidden, false);
  render('live', {mapping_session_id: 'finished', map_available: true}); assert.equal(stage.hidden, false);
  render('live', {connected: true, mapping_session_id: null}); assert.equal(stage.hidden, true);
  render('saved', {}); assert.equal(stage.hidden, false);
  render('live', {}); assert.equal(stage.hidden, true);
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
  const request = f.requests.find(r => r.url.endsWith('explore'));
  assert.equal(request.options.headers['X-Mapping-Client'], '1');
  assert.equal(f.get('drone-resources').children[0].children[0].textContent, '<sensor>');
  assert.match(f.get('drone-resources').children[0].children[1].textContent, /미확인.*<script>/);
  f.get('drone-details').handlers.click();
  await settle();
  assert.equal(f.get('drone-resource-panel').hidden, true);
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

test('server outage disables connection actions and reports unknown drone state', async t => {
  const f = fixture(t);
  await settle();
  f.setFail();
  await f.timers.shift()();
  assert.equal(f.get('drone-details').disabled, true);
  assert.match(f.get('drone-status').textContent, /상태를 확인하지 못했습니다/);
});
