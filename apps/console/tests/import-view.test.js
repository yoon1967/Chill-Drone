import test from 'node:test';
import assert from 'node:assert/strict';
import { MapSource } from '../map-source.js';
import { importMessage, openUrl } from '../import-view.js';

test('import binding applies to saved reads and never live input', async () => {
  const urls = [];
  const source = new MapSource('saved', async url => { urls.push(url); return {ok: true, status: 200, json: async () => ({})}; });
  source.asset = 'import_one';
  await source.read('run?revision=000001', source.context());
  source.select('live');
  await source.read('state', source.context());
  assert.deepEqual(urls, ['/api/saved/run?revision=000001&asset=import_one', '/api/live/state']);
});

test('open link and import status keep reconstruct and view separate', () => {
  assert.equal(openUrl('a&b'), '/?asset=a%26b#saved');
  assert.match(importMessage({status: 'ready'}), /지도를 열/);
  assert.match(importMessage({status: 'running', stage: 'features'}), /매칭/);
  assert.match(importMessage({status: 'failed', error: 'frame_dimensions_differ'}), /해상도/);
});
