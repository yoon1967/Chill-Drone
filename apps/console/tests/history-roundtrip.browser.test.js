import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {setTimeout as delay} from 'node:timers/promises';

const url = process.env.MAPPING_TEST_URL;
const source = process.env.MAPPING_HISTORY_SOURCE_ID;

test('reconstruction archives round-trip through the import API and open in the map console',
  {skip: !url || !source, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE
      ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE)
      : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage();
      const errors = [];
      page.on('pageerror', error => errors.push(error.message));
      const headers = {'X-Mapping-Client': '1'};
      const get = path => page.request.get(url + path);
      const post = (path, data) => page.request.post(url + path, {headers, data});
      const json = async pending => {
        const response = await pending;
        assert.ok(response.ok(), `${response.status()} ${response.url()}`);
        return response.json();
      };
      const waitForReady = async job => {
        for (let attempt = 0; job.status === 'inspecting' && attempt < 40; attempt++) {
          await delay(125);
          job = await json(get('/api/import/status?id=' + job.id));
        }
        assert.ok(['ready', 'complete'].includes(job.status), job.error || job.status);
        return job;
      };
      const original = await waitForReady(await json(post('/api/import/local', {local_id: source})));
      const originalScene = await json(get('/api/saved/run?asset=' + original.id));
      assert.ok(originalScene.points.length > 0);
      const archive = await get('/api/import/history.zip?id=' + original.id);
      assert.equal(archive.status(), 200);
      const bytes = await archive.body();
      assert.ok(bytes.length > 0);

      const name = 'reconstruction-history.zip';
      const imported = await json(post('/api/import/create', {name}));
      assert.notEqual(imported.id, original.id);
      await json(post(`/api/import/upload?id=${imported.id}&path=${name}`, bytes));
      const ready = await waitForReady(await json(post('/api/import/inspect?id=' + imported.id, {})));
      assert.equal(ready.status, 'ready');
      assert.equal(ready.history_available, true);
      const restored = await json(get('/api/saved/run?asset=' + imported.id));
      assert.deepEqual(restored.points, originalScene.points);
      assert.deepEqual(restored.cameras, originalScene.cameras);
      const history = await json(get('/api/import/history?id=' + imported.id));
      assert.equal(history.count, ready.summary.history_frames);
      assert.ok(history.count > 0);

      await page.goto(url + '/reconstruct.html?asset=' + imported.id);
      await page.locator('#progressive-viewport canvas').waitFor();
      await page.getByRole('button', {name: '점군 편집'}).click();
      await page.locator('#editor-viewport canvas').waitFor();
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
