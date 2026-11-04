import test from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const url = process.env.MAPPING_TEST_URL;

test('calibration opens inside the tab panel, changes steps, and returns to the map',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true,
      ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
    try {
      const page = await browser.newPage();
      const errors = [], mutations = [];
      page.on('pageerror', error => errors.push(error.message));
      page.on('request', request => { if (request.method() === 'POST') mutations.push(request.url()); });
      await page.route('**/api/drones/state', route => route.fulfill({json: {connected: true}}));
      await page.goto(new URL('/#live', url).href);
      await page.waitForFunction(() => document.body.dataset.mapSource === 'live');
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 900});
        await page.locator('#drone-open').click();
        await page.locator('#calibration-open').click();
        assert.equal(await page.locator('#calibration-workspace').isVisible(), true);
        assert.equal(await page.locator('#calibration-method').isVisible(), true);
        assert.equal(await page.locator('#calibration-close').isVisible(), true);
        assert.equal(await page.locator('#camera-actions').isVisible(), false);
        assert.equal(await page.locator('.source-bar').isVisible(), false);
        assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        await page.locator('#calibration-method-file').click();
        assert.equal(await page.locator('#calibration-files').isVisible(), true);
        await page.locator('#calibration-media-next').click();
        assert.equal(await page.locator('#calibration-media-error').isVisible(), true);
        await page.locator('#calibration-media-back').click();
        await page.locator('#calibration-method-live').click();
        assert.equal(await page.locator('#calibration-source').isVisible(), true);
        await page.locator('#calibration-media-next').click();
        assert.equal(await page.locator('#calibration-setup').isVisible(), true);
        await page.locator('#calibration-close').click();
        assert.equal(await page.locator('#calibration-workspace').isVisible(), false);
        assert.equal(await page.locator('#calibration-open').isVisible(), true);
        assert.equal(await page.locator('#calibration-open').evaluate(node => document.activeElement === node), true);
        await page.locator('#tab-saved').click();
        assert.equal(await page.locator('#import-panel').isVisible(), true);
        await page.locator('#tab-live').click();
      }
      assert.deepEqual(errors, []);
      assert.deepEqual(mutations, [], 'opening the wizard must not start calibration or camera input');
    } finally {
      await browser.close();
    }
  });

test('drone calibration preview advances while collection status is waiting and stops on close',
  {skip: !url, timeout: 20000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true,
      ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
    let releaseState;
    const delayedState = new Promise(resolve => { releaseState = resolve; });
    try {
      const page = await browser.newPage();
      let previews = 0, inFlight = 0, maxInFlight = 0, statusRequests = 0, statusResponses = 0;
      const state = {id: 'preview-test', source: 'tello', status: 'collecting', accepted: 0,
        coverage: {ready: false, checks: {}}, reason: 'checkerboard_not_found', size: null};
      await page.route('**/api/drones/state', route => route.fulfill({json: {connected: false}}));
      await page.route('**/api/calibration**', async route => {
        const path = new URL(route.request().url()).pathname;
        if (path.endsWith('/preview')) {
          maxInFlight = Math.max(maxInFlight, ++inFlight);
          await new Promise(resolve => setTimeout(resolve, 25));
          await route.fulfill({contentType: 'image/svg+xml', body: '<svg xmlns="http://www.w3.org/2000/svg" width="32" height="24"><rect width="32" height="24" fill="green"/></svg>'});
          inFlight--; previews++;
          return;
        }
        if (path.endsWith('/state')) { statusRequests++; await delayedState; statusResponses++; }
        return route.fulfill({json: path === '/api/calibration' ? {enabled: true, busy: false} : state});
      });
      await page.goto(new URL('/#live', url).href);
      await page.locator('#calibration-open').click();
      await page.locator('#calibration-method-live').click();
      await page.locator('#calibration-source').selectOption('tello');
      await page.locator('#calibration-media-next').click();
      await page.locator('#calibration-confirm').check();
      await page.locator('#calibration-start').click();
      await page.waitForFunction(() => performance.getEntriesByType('resource').filter(entry =>
        entry.name.includes('/api/calibration/preview?')).length >= 4);
      assert.ok(previews >= 4);
      assert.equal(statusRequests, 1);
      assert.equal(statusResponses, 0, 'preview must not wait for calibration status/analysis');
      assert.equal(maxInFlight, 1, 'slow images must not build an overlapping request queue');
      assert.equal(await page.locator('#calibration-preview').isVisible(), true);
      await page.locator('#calibration-close').click();
      releaseState();
      await page.waitForTimeout(300);
      const stopped = previews;
      await page.waitForTimeout(300);
      assert.equal(previews, stopped);
    } finally { releaseState(); await browser.close(); }
  });
