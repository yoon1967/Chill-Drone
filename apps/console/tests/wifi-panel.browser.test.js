import test from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const url = process.env.MAPPING_TEST_URL;

test('Wi-Fi panel stays visible outside the disclosure before search, on failure, and with results',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true,
      ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
    try {
      const page = await browser.newPage();
      const errors = [];
      page.on('pageerror', error => errors.push(error.message));
      await page.route('**/api/drones/state', route => route.fulfill({json: {connected: false}}));
      let scan = {items: [], warning: 'wifi_location_permission_required'};
      await page.route('**/api/drones/scan', route => route.fulfill({json: scan}));
      await page.goto(new URL('/#live', url).href);
      await page.waitForFunction(() => document.body.dataset.mapSource === 'live');
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 900});
        assert.equal(await page.locator('#drone-workspace').evaluate(node => node.open), false);
        assert.equal(await page.locator('#drone-wifi-results').isVisible(), true);
        assert.equal(await page.locator('#calibration-open').isVisible(), false);
        assert.equal(await page.locator('#drone-scan').isVisible(), true);
        assert.equal(await page.locator('#drone-workspace #drone-wifi-results').count(), 0);
        assert.equal(await page.locator('#drone-scan').count(), 1);
        scan = {items: [], warning: 'wifi_location_permission_required'};
        await page.locator('#drone-scan').click();
        await page.waitForFunction(() => document.getElementById('drone-wifi-scan-status').textContent.includes('Windows 설정'));
        assert.equal(await page.locator('#drone-wifi-empty').isVisible(), true);
        scan = {items: [{id: 'Home', ssid: 'Home', kind: 'wifi', signal_pct: 85},
          {id: 'TELLO-test', ssid: 'TELLO-test', kind: 'wifi', signal_pct: 50, drone_candidate: true}]};
        await page.locator('#drone-scan').click();
        await page.waitForFunction(() => document.querySelectorAll('#drone-list .wifi-row').length === 2);
        assert.equal(await page.locator('#drone-wifi-empty').isVisible(), false);
        await page.getByRole('option').filter({hasText: 'TELLO-test'}).click();
        assert.equal(await page.locator('#drone-connect').isEnabled(), true);
        assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        await page.locator('#tab-saved').click();
        assert.equal(await page.locator('#drone-wifi-results').isVisible(), false);
        await page.locator('#tab-live').click();
      }
      assert.deepEqual(errors, []);
    } finally {
      await browser.close();
    }
  });
