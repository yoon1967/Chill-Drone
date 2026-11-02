import test from 'node:test';
import assert from 'node:assert/strict';
import { MapSource } from '../map-source.js';
import { connectImport, importMessage, openUrl, savedPhase, processingSteps } from '../import-view.js';

test('import binding applies to saved reads and never live input', async () => {
  const urls = [];
  const source = new MapSource('saved', async url => { urls.push(url); return {ok: true, status: 200, json: async () => ({})}; });
  source.asset = 'import_one';
  await source.read('run?revision=000001', source.context());
  source.select('live');
  await source.read('state', source.context());
  assert.deepEqual(urls, ['/api/saved/run?revision=000001&asset=import_one', '/api/live/state']);
});

test('saved workflow separates options, computation, ready maps and failed input', () => {
  assert.equal(savedPhase({status: 'awaiting_options'}), 'settings');
  for (const status of ['uploading', 'inspecting', 'running', 'cancelling'])
    assert.equal(savedPhase({status}), 'processing');
  for (const status of ['ready', 'complete']) assert.equal(savedPhase({status}), 'ready');
  for (const status of ['failed', 'cancelled']) assert.equal(savedPhase({status}), 'input');
});

test('stage display marks only observed earlier stages complete and does not invent percentage', () => {
  const steps = processingSteps({status: 'running', stage: 'reconstruction'});
  assert.deepEqual(steps.map(step => step.state), ['done', 'done', 'done', 'active', 'waiting', 'waiting']);
  assert.ok(processingSteps({status: 'complete', stage: 'finished'}).every(step => step.state === 'done'));
  assert.ok(processingSteps({status: 'running', stage: 'unknown'}).every(step => step.state === 'waiting'));
  assert.equal(processingSteps({status: 'failed', stage: 'features'}).filter(step => step.state === 'active').length, 0);
});

test('opening an already ready record navigates once without starting another status poll', async t => {
  const originals = new Map();
  function replace(name, value) {
    originals.set(name, Object.getOwnPropertyDescriptor(globalThis, name));
    Object.defineProperty(globalThis, name, {configurable: true, value, writable: true});
  }
  t.after(() => {
    for (const [name, descriptor] of originals)
      descriptor ? Object.defineProperty(globalThis, name, descriptor) : delete globalThis[name];
  });
  const nodes = new Map(), assignments = [], requests = [];
  function node() {
    return {dataset: {}, children: [], handlers: {}, value: '', checked: false,
      setAttribute() {}, removeAttribute() {}, pause() {}, load() {}, focus() {},
      addEventListener(name, handler) { this.handlers[name] = handler; },
      append(...children) { this.children.push(...children); },
      replaceChildren(...children) { this.children = children; }};
  }
  const get = id => { if (!nodes.has(id)) nodes.set(id, node()); return nodes.get(id); };
  get('saved-steps').children = Array.from({length: 4}, node);
  replace('document', {body: {dataset: {}}, getElementById: get, createElement: node});
  replace('window', {addEventListener() {}});
  replace('location', {hash: '#saved', search: '', assign: url => assignments.push(url)});
  replace('Option', class { constructor(text, value) { this.text = text; this.value = value; } });
  const record = {id: 'import_ready', status: 'ready', summary: {kind: 'result', name: 'map', points: 2, keyframes: 1, revisions: 0}};
  replace('fetch', async url => {
    requests.push(url);
    return {ok: true, json: async () => url.endsWith('/library') ? {items: [record]} : record};
  });
  connectImport();
  await new Promise(setImmediate);
  get('import-library').value = record.id;
  await get('import-local').handlers.click();
  await new Promise(setImmediate);
  assert.deepEqual(assignments, ['/?asset=import_ready#saved']);
  assert.equal(requests.filter(url => url.includes('/status')).length, 0);
});

test('open link and import status keep reconstruct and view separate', () => {
  assert.equal(openUrl('a&b'), '/?asset=a%26b#saved');
  assert.match(importMessage({status: 'ready'}), /지도를 열/);
  assert.match(importMessage({status: 'running', stage: 'features'}), /매칭/);
  assert.match(importMessage({status: 'failed', error: 'frame_dimensions_differ'}), /해상도/);
});
