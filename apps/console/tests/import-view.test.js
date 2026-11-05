import test from 'node:test';
import assert from 'node:assert/strict';
import { MapSource } from '../map-source.js';
import { connectImport, importMessage, openUrl, savedPhase, processingSteps } from '../import-view.js';

test('fast reconstruction shows frame work instead of offline SIFT stages', () => {
  const job = {status: 'running', stage: 'tracking', options: {mode: 'depth_rgbd'}, progress: {processed: 12, points: 3000}};
  assert.match(importMessage(job), /카메라 위치·방향 추적 중 · 12프레임 처리 · 3000점/);
  const steps = processingSteps(job);
  assert.equal(steps.length, 6);
  assert.deepEqual(steps.map(s => s.state), ['done', 'active', 'waiting', 'waiting', 'waiting', 'waiting']);
  assert.ok(steps.every(s => !s.label.includes('특징점')));
  const integration = {...job, stage: 'integration', progress: {integrated_frames: 3, integration_total: 8}};
  assert.match(importMessage(integration), /전체 경로 지도 통합 중 · 3 \/ 8관측/);
  assert.equal(processingSteps(integration)[4].state, 'active');
});

test('live model preparation and per-frame stages use the same reconstruction steps', () => {
  for (const [stage, index] of [['semantic_model', 0], ['pose_wait', 1], ['depth', 2], ['semantics', 3], ['presentation', 3], ['saving', 5]]) {
    const steps = processingSteps({status: 'running', stage, options: {mode: 'depth_rgbd'}});
    assert.equal(steps[index].state, 'active');
    assert.equal(steps.filter(step => step.state === 'active').length, 1);
  }
});

test('feature extraction shows observed frame counts and switches to matching', () => {
  const job = {status: 'running', stage: 'features', feature_progress: {phase: 'extraction', completed: 12, total: 30}};
  assert.match(importMessage(job), /12 \/ 총 30프레임 완료/);
  assert.equal(processingSteps(job)[2].label, importMessage(job));
  job.feature_progress.completed = null;
  assert.match(importMessage(job), /진행 수 확인 중/);
  job.feature_progress = {phase: 'matching', completed: 30, total: 30};
  assert.match(importMessage(job), /매칭·결과 정리 중 · 총 30프레임 추출 완료/);
  job.stage = 'reconstruction';
  assert.match(importMessage(job), /3D 지도 복원 중/);
});

test('every processing stage describes its work and preserves observed counts', () => {
  const message = (stage, progress) => importMessage({status: 'running', stage, progress});
  assert.match(message('input', {phase: 'decode', completed: 120, total: 834, selected: 4}), /120 \/ 총 834프레임 읽음 · 복원용 4프레임 저장/);
  assert.match(message('input', {phase: 'copy_frames', completed: 4, total: 28}), /4 \/ 총 28장 변환/);
  assert.match(message('input', {phase: 'verify_source', selected: 28}), /원본 파일 변경 여부 검증/);
  assert.match(message('reconstruction', {stage: 'mapping'}), /카메라 위치 등록·3D 점 생성/);
  assert.match(message('reconstruction', {stage: 'export'}), /점 색상 추출/);
  assert.match(message('preview', {phase: 'projections'}), /1 \/ 2장 저장/);
  assert.match(message('finalization'), /체크섬/);
  assert.match(importMessage({status: 'inspecting', stage: 'inspect'}), /해상도·프레임 순서/);
  const job = {status: 'running', stage: 'features', feature_progress: {
    phase: 'matching', completed: 28, total: 28, pairs_completed: 52, pairs_total: 102}};
  assert.match(importMessage(job), /52 \/ 총 102쌍 기하 검증 완료/);
  job.feature_progress.phase = 'export';
  assert.match(importMessage(job), /결과 정리 중/);
  job.feature_progress.phase = 'validation';
  assert.match(importMessage(job), /결과 검증 중/);
});

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

test('opening a completed depth record navigates to its final map once without another status poll', async t => {
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
  replace('document', {body: {dataset: {}}, getElementById: get, createElement: node});
  replace('window', {addEventListener() {}});
  replace('location', {hash: '#saved', search: '', assign: url => assignments.push(url)});
  replace('Option', class { constructor(text, value) { this.text = text; this.value = value; } });
  const record = {id: 'import_ready', status: 'complete', options: {mode: 'depth_rgbd'}, summary: {kind: 'result', name: 'map', points: 2, keyframes: 1, revisions: 0}};
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

test('a completed map can be reopened after replaying its reconstruction repeatedly', async t => {
  const originals = new Map();
  function replace(name, value) {
    originals.set(name, Object.getOwnPropertyDescriptor(globalThis, name));
    Object.defineProperty(globalThis, name, {configurable: true, value, writable: true});
  }
  t.after(() => {
    for (const [name, descriptor] of originals)
      descriptor ? Object.defineProperty(globalThis, name, descriptor) : delete globalThis[name];
  });
  const nodes = new Map(), handlers = {}, assignments = [];
  function node() {
    return {dataset: {}, children: [], handlers: {}, value: '', checked: false,
      setAttribute() {}, removeAttribute() {}, pause() {}, load() {}, focus() {},
      addEventListener(name, handler) { this.handlers[name] = handler; },
      append(...children) { this.children.push(...children); },
      replaceChildren(...children) { this.children = children; }};
  }
  const get = id => { if (!nodes.has(id)) nodes.set(id, node()); return nodes.get(id); };
  const classes = new Set();
  const body = {dataset: {}, classList: {
    toggle(name, enabled) { enabled ? classes.add(name) : classes.delete(name); },
    remove(name) { classes.delete(name); },
  }};
  const record = {id: 'import_ready', status: 'complete', history_available: true, options: {mode: 'depth_rgbd'},
    summary: {kind: 'result', name: 'map', points: 2, keyframes: 1, revisions: 0}};
  replace('document', {body, getElementById: get, createElement: node});
  replace('window', {addEventListener(name, handler) { handlers[name] = handler; }});
  replace('location', {hash: '#saved', search: '?asset=import_ready', assign: url => assignments.push(url)});
  replace('Option', class { constructor(text, value) { this.text = text; this.value = value; } });
  replace('fetch', async url => ({ok: true, json: async () => url.endsWith('/library') ? {items: []} : record}));
  connectImport();
  await new Promise(setImmediate);
  handlers['saved-map-ready']();
  assert.equal(body.dataset.savedPhase, 'result');
  for (let visit = 0; visit < 2; visit++) {
    get('saved-replay').handlers.click();
    await new Promise(setImmediate);
    assert.equal(body.dataset.savedPhase, 'processing');
    assert.equal(get('import-open').href, '/?asset=import_ready#saved');
    get('import-open').handlers.click?.({button: 0, preventDefault() {}});
    await new Promise(setImmediate);
    assert.equal(body.dataset.savedPhase, 'result', 'same-page map links must leave reconstruction replay');
    assert.equal(get('import-details').open, false);
    assert.equal(get('progressive-workspace').hidden, true);
    assert.equal(classes.has('progressive-processing'), false);
  }
  assert.deepEqual(assignments, [], 'the already loaded completed map does not need another navigation');
});
