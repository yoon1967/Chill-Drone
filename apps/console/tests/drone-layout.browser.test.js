import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('drone connection card stays open after mapping startup fails', {skip: !url, timeout: 30000}, async () => {
  const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
    createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
  const browser = await chromium.launch({headless: true,
    ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage();
    page.setDefaultTimeout(5000);
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    const state = {connected: true, owner: 'video-input', disconnect_available: true};
    await page.route('**/api/live/state*', route => route.fulfill({json: {
      source: 'live', status: 'idle', session_status: 'idle', session_id: null, map_available: false}}));
    await page.route('**/api/drones/**', route => {
      if (route.request().url().includes('preview.jpg')) return route.fulfill({status: 404, body: ''});
      const value = route.request().url().endsWith('/scan') ? {items: [
        {id: 'wifi-a', ssid: 'Wi-Fi A', kind: 'wifi'}]} : state;
      return route.fulfill({json: value});
    });
    await page.route('**/api/live/tello/start', route => route.fulfill({status: 409,
      json: {error: 'slam_docker_unavailable'}}));
    await page.goto(new URL('/#live', url).href);
    await page.waitForFunction(() => document.body.dataset.mapSource === 'live');
    for (const width of [1440, 390]) {
      await page.setViewportSize({width, height: 900});
      await page.locator('#drone-open').click();
      await page.locator('#drone-scan').click();
      await page.locator('#drone-list button').waitFor();
      assert.equal(await page.locator('#drone-list button').count(), 1);
      assert.equal(await page.locator('#drone-current-connect').isVisible(), true);
      if (!await page.locator('#profile-manager').evaluate(node => node.open))
        await page.locator('#profile-manager > summary').click();
      await page.locator('#profile-confirm-live').check();
      await page.locator('#drone-map-start').click();
      await page.waitForFunction(() => document.querySelector('#drone-flow-status').textContent.includes('Docker'), null, {timeout: 5000})
        .catch(async error => { throw new Error(await page.locator('#drone-status').textContent() + ' / ' + await page.locator('#drone-flow-status').textContent() + ' / ' + await page.locator('#camera-status').textContent(), {cause: error}); });
      assert.equal(await page.locator('#drone-workspace').evaluate(node => node.open), true);
      assert.equal(await page.locator('#drone-camera-panel').isVisible(), true);
      const card = await page.locator('#drone-workspace').boundingBox();
      const content = await page.locator('.drone-panel').boundingBox();
      assert.ok(content.x >= card.x && content.x + content.width <= card.x + card.width + 1);
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      await page.locator('#drone-open').click();
    }
    assert.deepEqual(errors, []);
  } finally { await browser.close(); }
});
