import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('matching camera cards expand inline and stay open after mapping startup fails', {skip: !url, timeout: 30000}, async () => {
  const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
    createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
  const browser = await chromium.launch({headless: true,
    ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage();
    page.setDefaultTimeout(5000);
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    const connections = [];
    const state = {connected: true, owner: 'video-input', disconnect_available: true,
      resources: [{name: '배터리', status: 'observed', value: 69, unit: '%'}]};
    await page.route('**/api/live/state*', route => route.fulfill({json: {
      source: 'live', status: 'idle', session_status: 'idle', session_id: null, map_available: false}}));
    await page.route('**/api/drones/**', route => {
      if (route.request().url().endsWith('/connect')) connections.push(route.request().postDataJSON());
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
      const fresh = await page.locator('#calibration-open').boundingBox();
      const existing = await page.locator('#drone-open').boundingBox();
      assert.ok(Math.abs(fresh.width - existing.width) < 1);
      assert.ok(Math.abs(fresh.height - existing.height) < 1);
      if (width > 600) assert.ok(Math.abs(fresh.y - existing.y) < 1);
      else assert.ok(existing.y >= fresh.y + fresh.height);
      const top = await page.locator('.source-bar').boundingBox();
      assert.ok(top.height < (width > 600 ? 100 : 180));
      await page.locator('.connection-details > summary').click();
      assert.equal(await page.locator('#connection-note').isVisible(), true);
      await page.locator('.connection-details > summary').click();
      await page.locator('#drone-open').click();
      await page.locator('#drone-scan').click();
      await page.locator('#drone-list button').waitFor();
      assert.equal(await page.locator('#drone-list button').count(), 1);
      const before = connections.length;
      await page.locator('#drone-list button').click();
      await page.locator('#drone-list button').press('Enter');
      await page.waitForFunction(() => !document.querySelector('#drone-details').disabled);
      assert.equal(connections.length, before + 1);
      assert.equal(connections.at(-1).id, 'wifi-a');
      await page.locator('#drone-details').click();
      await page.locator('#drone-resources dt').waitFor();
      assert.equal(await page.locator('#drone-details').textContent(), '상세 정보 접기');
      const heading = await page.locator('.drone-resource-heading h2').boundingBox();
      const refresh = await page.locator('#drone-resource-refresh').boundingBox();
      assert.ok(Math.abs((heading.y + heading.height / 2) - (refresh.y + refresh.height / 2)) < 2);
      await page.locator('#drone-details').click();
      assert.equal(await page.locator('#drone-resource-panel').isVisible(), false);
      assert.equal(await page.locator('#drone-details').textContent(), '자세히 보기');
      assert.equal(await page.locator('#drone-current-connect').isVisible(), true);
      assert.equal(await page.locator('#profile-enabled-live').isChecked(), false);
      await page.locator('#drone-map-start').click();
      await page.waitForFunction(() => document.querySelector('#drone-flow-status').textContent.includes('Docker'), null, {timeout: 5000})
        .catch(async error => { throw new Error(await page.locator('#drone-status').textContent() + ' / ' + await page.locator('#drone-flow-status').textContent() + ' / ' + await page.locator('#camera-status').textContent(), {cause: error}); });
      assert.equal(await page.locator('#drone-workspace').evaluate(node => node.open), true);
      assert.equal(await page.locator('#drone-camera-panel').isVisible(), true);
      const card = await page.locator('#camera-actions').boundingBox();
      const content = await page.locator('.drone-panel').boundingBox();
      assert.ok(content.x >= card.x && content.x + content.width <= card.x + card.width + 1);
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      await page.locator('#drone-open').click();
    }
    assert.deepEqual(errors, []);
  } finally { await browser.close(); }
});
