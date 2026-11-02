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
      await page.goto(new URL('/#live', url).href);
      await page.waitForFunction(() => document.body.dataset.mapSource === 'live');
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 900});
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
