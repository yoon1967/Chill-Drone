import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {readFile} from 'node:fs/promises';

const url = process.env.MAPPING_TEST_URL;
const source = process.env.MAPPING_HISTORY_SOURCE_ID;

test('downloaded reconstruction ZIPs reopen their map and replay through the initial drop zone and picker',
  {skip: !url || !source, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}, acceptDownloads: true});
      const errors = [], reconstructionRequests = [];
      page.on('pageerror', error => errors.push(error.message));
      page.on('request', request => {
        if (new URL(request.url()).pathname === '/api/import/start') reconstructionRequests.push(request.url());
      });
      const api = async path => {
        const response = await page.request.get(url + path);
        assert.equal(response.status(), 200);
        return response.json();
      };
      await page.goto(url + '/#saved');
      const library = await api('/api/import/library');
      const sourceRecord = library.items.find(item => (item.local_id || item.id) === source);
      assert.ok(sourceRecord, 'the requested history source exists in the library');
      await page.locator('button.saved-record').filter({hasText: sourceRecord.summary?.name || sourceRecord.name}).click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
      let asset = new URL(page.url()).searchParams.get('asset');
      const original = await api('/api/saved/run?asset=' + asset);
      assert.ok(original.points.length > 0);
      for (const input of ['drop', 'picker']) {
        await page.locator('#saved-replay').click();
        await page.locator('#progressive-frame:not([hidden])').waitFor();
        const pending = page.waitForEvent('download');
        await page.locator('#progressive-download').click();
        const download = await pending;
        assert.equal(download.suggestedFilename(), 'reconstruction-history.zip');
        const bytes = await readFile(await download.path());
        assert.ok(bytes.length > 0);
        await page.goto(url + '/#saved');
        assert.match(await page.locator('#import-file').getAttribute('accept'), /\.zip/);
        if (input === 'drop') {
          const transfer = await page.evaluateHandle(data => {
            const transfer = new DataTransfer();
            transfer.items.add(new File([new Uint8Array(data)], 'reconstruction-history.zip', {type: 'application/zip'}));
            return transfer;
          }, [...bytes]);
          await page.locator('#import-drop').dispatchEvent('drop', {dataTransfer: transfer});
          await transfer.dispose();
        } else {
          await page.setViewportSize({width: 390, height: 1000});
          await page.locator('#import-file').setInputFiles({name: 'reconstruction-history.zip', mimeType: 'application/zip', buffer: bytes});
        }
        await page.waitForURL('**/?asset=import_*#saved');
        const imported = new URL(page.url()).searchParams.get('asset');
        assert.notEqual(imported, asset, 'the archive is opened in its own imported record');
        asset = imported;
        await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
        await page.waitForFunction(() => document.querySelector('#frame').naturalWidth > 0);
        const status = await api('/api/import/status?id=' + asset);
        assert.equal(status.status, 'ready');
        assert.equal(status.history_available, true);
        assert.equal(status.summary.history_frames, 2);
        const restored = await api('/api/saved/run?asset=' + asset);
        assert.deepEqual(restored.points, original.points);
        assert.deepEqual(restored.cameras, original.cameras);
        assert.equal(await page.locator('#viewport canvas').isVisible(), true);
        assert.equal(await page.locator('#import-start').isVisible(), false);
        assert.equal((await api('/api/import/history?id=' + asset)).count, 2);
        await page.locator('#saved-replay').click();
        await page.locator('#progressive-frame:not([hidden])').waitFor();
        await page.locator('#progressive-seek').fill('1');
        await page.locator('#progressive-seek-go').click();
        await page.waitForFunction(() => document.querySelector('#progressive-frame').src.includes('frame_0000001.jpg'));
        assert.equal(await page.locator('#progressive-frame').evaluate(image => image.naturalWidth), 32);
        assert.match(await page.locator('#progressive-frame-time').textContent(), /1\.00/);
        assert.equal(await page.locator('#progressive-replay-state').textContent(), '저장된 복원 기록 재생');
        assert.equal(await page.locator('#progressive-viewport canvas').isVisible(), true);
        await page.locator('#import-open').click();
        await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
        assert.equal(await page.locator('#map-content').isVisible(), true);
      }
      assert.deepEqual(reconstructionRequests, [], 'opening an exported archive never starts reconstruction again');
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
