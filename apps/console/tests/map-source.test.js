import test from 'node:test';
import assert from 'node:assert/strict';
import { MapSource, captureError } from '../map-source.js';

const deferred = () => { let resolve; const promise = new Promise((done) => { resolve = done; }); return {promise, resolve}; };
const response = (value, status = 200) => ({ok: status < 400, status, json: async () => value});

test('saved response arriving after live selection cannot replace the live map', async () => {
  const pending = deferred(), urls = [];
  const source = new MapSource('saved', (url) => { urls.push(url); return pending.promise; });
  const old = source.context(), read = source.read('run', old);
  source.select('live');
  assert.equal(old.signal.aborted, true);
  pending.resolve(response({source: 'saved'})); // Even transports ignoring cancellation are rejected.
  assert.equal(await read, null);
  assert.deepEqual(urls, ['/api/saved/run']);
});

test('switching live → saved → live also discards an older live response body', async () => {
  const body = deferred(), entered = deferred();
  const source = new MapSource('live', async () => ({ok: true, status: 200, json: () => { entered.resolve(); return body.promise; }}));
  const read = source.read('run?session=old&revision=000001', source.context());
  await entered.promise;
  source.select('saved'); source.select('live');
  body.resolve({source: 'live', input_run_id: 'old'});
  assert.equal(await read, null);
});

test('a session change is retried from state and never treated as another source', async () => {
  const source = new MapSource('live', async (url) => {
    assert.equal(url, '/api/live/run?session=old&revision=000001');
    return response({error: 'session_changed'}, 409);
  });
  assert.equal(await source.read('run?session=old&revision=000001', source.context()), null);
  assert.equal(source.source, 'live');
});

test('current connection failures remain visible; obsolete failures are ignored', async () => {
  const pending = deferred();
  const source = new MapSource('saved', async () => { await pending.promise; throw new Error('offline'); });
  const read = source.read('state', source.context());
  source.select('live'); pending.resolve();
  assert.equal(await read, null);
  await assert.rejects(source.read('state', source.context()), /offline/);
  assert.throws(() => source.select('unknown'));
});

test('permission, unavailable device and busy device have actionable messages', () => {
  assert.equal(captureError({name: 'NotAllowedError'}).state, 'denied');
  assert.match(captureError({name: 'SecurityError'}).message, /권한/);
  assert.match(captureError({name: 'NotFoundError'}).message, /찾지 못/);
  assert.match(captureError({name: 'NotReadableError'}).message, /다른 앱/);
});

test('SLAM startup failures explain the required local preparation', () => {
  assert.match(captureError(new Error('slam_image_missing')).message, /docker build.*ssafy-orbslam3:local/);
  assert.match(captureError(new Error('slam_docker_unavailable')).message, /Docker Desktop.*Linux/);
});
