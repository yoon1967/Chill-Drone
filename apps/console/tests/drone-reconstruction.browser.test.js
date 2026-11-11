import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('drone recordings select sensor poses and a depth model, then ordinary video restores the pose picker',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage();
      const errors = [], checked = [];
      page.on('pageerror', error => errors.push(error.message));
      let job, started;
      await page.route('**/api/import/drone-recording', async route => {
        const session = route.request().postDataJSON().session_id;
        job = {id: 'import_1234567890123456', name: session, status: 'awaiting_options',
          summary: {kind: 'drone_recording', width: 960, height: 720}};
        await route.fulfill({json: job});
      });
      await page.route('**/api/import/status?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/start?*', route => {
        started = route.request().postDataJSON();
        job = {...job, status: 'cancelled'};
        return route.fulfill({json: job});
      });
      await page.route('**/api/reconstruction-models/status?*', route => {
        checked.push(new URL(route.request().url()).searchParams.get('id'));
        return route.fulfill({json: {status: 'ready'}});
      });
      await page.goto(url + '/reconstruct.html');
      await page.locator('#drone-recordings-list button').first().waitFor();
      await page.locator('#drone-recordings-list button').first().click();
      await page.locator('#drone-pose-hint').waitFor();
      assert.equal(await page.locator('#pose-model-field').isVisible(), false);
      await page.selectOption('#depth-model', 'depth-anything-v2-metric-indoor-small');
      await page.waitForFunction(() => !document.getElementById('start-reconstruction').disabled);
      assert.deepEqual(checked, ['depth-anything-v2-metric-indoor-small']);
      await page.locator('#start-reconstruction').click();
      await page.waitForFunction(() => document.querySelector('main').dataset.state === 'cancelled');
      assert.equal(started.depth_model, 'depth-anything-v2-metric-indoor-small');
      assert.equal(started.causal_video, false);
      assert.equal('pose_model' in started, false);

      await page.route('**/api/import/create', route => route.fulfill({json: {id: job.id}}));
      await page.route('**/api/import/upload?*', route => route.fulfill({json: {status: 'uploaded'}}));
      await page.route('**/api/import/inspect?*', route => {
        job = {...job, name: 'ordinary.mp4', status: 'awaiting_options', summary: {kind: 'video', width: 960, height: 720}};
        return route.fulfill({json: job});
      });
      await page.locator('#change').click();
      await page.setInputFiles('#video', {name: 'ordinary.mp4', mimeType: 'video/mp4', buffer: Buffer.from('test upload')});
      await page.locator('#pose-model-field').waitFor();
      assert.equal(await page.locator('#drone-pose-hint').isVisible(), false);
      assert.equal(await page.locator('#start-reconstruction').isDisabled(), true);
      await page.setViewportSize({width: 390, height: 844});
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), true);
      assert.deepEqual(errors, []);
    } finally {
      await browser.close();
    }
  });

test('completion uploads saved coordinates even when the preview has the same point revision',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage();
      const id = 'import_1234567890123456';
      let status = 'running';
      const rescue = {map_id: 0, render_mode: 'points', point_revision: 1, point_labels: [0], people: []};
      await page.addInitScript(job => sessionStorage.setItem('mapping-video-reconstruction', job), id);
      await page.route('**/api/import/status?*', route => route.fulfill({json: {
        id, name: 'drone', status, options: {mode: 'depth_rgbd'}, summary: {kind: 'drone_recording'},
      }}));
      await page.route('**/api/import/preview?*', route => route.fulfill({json: {
        revision: 1, frame_id: 'frame_0000001.jpg', frame_number: 1, timestamp_s: 0, map_id: 0,
        points: [[1, 2, 3, 255, 0, 0]], rescue, cameras: [], processed: 1, registered: 1,
      }}));
      await page.route('**/api/import/preview-frame?*', route => route.fulfill({contentType: 'image/png',
        body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')}));
      await page.route('**/api/saved/run?*', route => route.fulfill({json: {
        schema_version: 'phone-sfm-result/1.2', scale: {unit: 'm'}, segment_id: 0,
        points: [[0, 1, -2, -3, 255, 0, 0]], rescue, cameras: [],
      }}));
      await page.goto(url + '/reconstruct.html');
      await page.evaluate(async () => {
        const {Points, Vector3} = await import('/vendor/three.module.js');
        Points.prototype.onBeforeRender = function (_renderer, scene) {
          if (!this.geometry.drawRange.count) return;
          window.renderedPoint = {
            sceneScale: scene.scale.toArray(),
            position: new Vector3().fromBufferAttribute(this.geometry.getAttribute('position'), 0)
              .applyMatrix4(this.matrixWorld).toArray(),
          };
        };
      });
      const preview = await page.waitForFunction(() => window.renderedPoint?.sceneScale[1] === -1 && window.renderedPoint.position);
      assert.deepEqual(await preview.jsonValue(), [1, -2, -3]);
      status = 'complete';
      await page.locator('main[data-state="complete"]').waitFor();
      const result = await page.waitForFunction(() => window.renderedPoint?.sceneScale[1] === 1 && window.renderedPoint.position);
      assert.deepEqual(await result.jsonValue(), [1, -2, -3]);
    } finally {
      await browser.close();
    }
  });
