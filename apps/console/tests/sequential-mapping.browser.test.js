import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
const video = process.env.MAPPING_SEQUENTIAL_VIDEO;
test('real saved reconstruction waits for preparation and presentation, then reuses its model',
  {skip: !url || !video, timeout: 240000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge',
      args: ['--disable-background-timer-throttling', '--disable-renderer-backgrounding']});
    try {
      let page;
      const errors = [];
      for (let iteration = 0; iteration < 3; iteration++) {
        if (page) await page.close();
        page = await browser.newPage({viewport: {width: 1440, height: 1000}});
        page.on('pageerror', e => errors.push(e.message));
        await page.goto(url + '/#saved');
        await page.locator('#import-file').setInputFiles(video);
        await page.waitForFunction(() => document.body.dataset.savedPhase === 'settings');
        await page.locator('#import-confirm').check();
        let held = null, release = null;
        const intercepted = new Promise(resolve => { release = resolve; });
        await page.route('**/api/import/presented?*', async route => {
          if (!held) { held = route; release(); }
          else await route.continue();
        });
        const startResponse = page.waitForResponse(r => r.url().includes('/api/import/start?'));
        const started = Date.now();
        await page.locator('#import-start').click();
        const response = await startResponse;
        const initial = await response.json();
        assert.equal(response.status(), 202, JSON.stringify(initial));
        const endpoint = `${url}/api/import/status?id=${initial.id}`;
        if (!iteration) {
          let preparing;
          for (let attempt = 0; attempt < 200; attempt++) {
            const value = await (await page.request.get(endpoint)).json();
            if (value.progress?.stage === 'model') { preparing = value; break; }
            assert.notEqual(value.status, 'failed', JSON.stringify(value));
            await page.waitForTimeout(50);
          }
          assert.ok(preparing?.progress, 'a model preparation state was observed');
          assert.equal(preparing.progress.processed, 0);
          assert.equal(preparing.progress.timestamp_s, 0);
          assert.deepEqual(await page.locator('#saved-preview').evaluate(v => [v.paused, v.currentTime]), [true, 0]);
        }
        await intercepted;
        const waiting = await (await page.request.get(endpoint)).json();
        assert.equal(waiting.progress.revision, 1);
        await page.waitForTimeout(400);
        const stillWaiting = await (await page.request.get(endpoint)).json();
        assert.equal(stillWaiting.progress.revision, 1, 'next frame waits for the current displayed frame acknowledgement');
        assert.equal(stillWaiting.progress.timestamp_s, 0);
        if (iteration === 1) {
          await page.locator('#import-cancel').click();
          let cancelled;
          for (let attempt = 0; attempt < 200; attempt++) {
            cancelled = await (await page.request.get(endpoint)).json();
            if (cancelled.status === 'cancelled') break;
            await page.waitForTimeout(50);
          }
          assert.equal(cancelled.status, 'cancelled');
          assert.equal(cancelled.progress.processed, 1, 'cancellation does not admit the next frame');
          await held.continue();
          await page.unroute('**/api/import/presented?*');
          continue;
        }
        await held.continue();
        await page.unroute('**/api/import/presented?*');
        await page.waitForFunction(() => document.querySelector('#progressive-state').textContent.includes('복원 완료'), null, {timeout: 100000});
        const job = await (await page.request.get(endpoint)).json();
        assert.equal(job.status, 'complete');
        assert.equal(job.progress.processing_policy, 'sequential');
        assert.equal(job.progress.dropped, 0);
        if (iteration) assert.ok(job.progress.model_preparation_ms < 1000, 'the next reconstruction reuses the prepared model');
        const history = await (await page.request.get(`${url}/api/import/history?id=${initial.id}`)).json();
        assert.equal(history.count, job.progress.processed);
        assert.ok(history.entries.every((e, i, entries) => !i || e.timestamp_s > entries[i - 1].timestamp_s));
        console.log(JSON.stringify({iteration, id: initial.id, elapsedMs: Date.now() - started,
          processed: job.progress.processed, registered: job.progress.registered, points: job.progress.points,
          modelPreparationMs: job.progress.model_preparation_ms}));
      }
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });

test('original preview stops and resets before model preparation',
  {skip: !url || !video, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage();
      let preparing = null;
      await page.route('**/api/import/status?*', route => preparing ? route.fulfill({json: preparing}) : route.continue());
      await page.route('**/api/import/start?*', route => {
        const id = new URL(route.request().url()).searchParams.get('id');
        preparing = {id, status: 'running', stage: 'model', options: {mode: 'depth_rgbd'},
          progress: {stage: 'model', processed: 0, timestamp_s: 0}};
        return route.fulfill({status: 202, json: preparing});
      });
      await page.goto(url + '/#saved');
      await page.locator('#import-file').setInputFiles(video);
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'settings');
      await page.locator('#saved-preview').evaluate(async element => {
        element.muted = true;
        element.currentTime = .5;
        await element.play();
      });
      assert.equal(await page.locator('#saved-preview').evaluate(element => element.paused), false);
      await page.locator('#import-confirm').check();
      await page.locator('#import-start').click();
      await page.waitForFunction(() => document.querySelector('#progressive-state').textContent.includes('모델 준비'));
      assert.deepEqual(await page.locator('#saved-preview').evaluate(element => [element.paused, element.currentTime]), [true, 0]);
      assert.equal(await page.locator('#saved-preview-wrap').isVisible(), false);
    } finally { await browser.close(); }
  });
