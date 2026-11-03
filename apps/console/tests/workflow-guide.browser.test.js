import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('side guide gates future steps, returns to earlier saved/live stages, and works on narrow screens',
  {skip: !url, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true,
      ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
    try {
      const page = await browser.newPage(), errors = [], posts = [];
      page.setDefaultTimeout(7000);
      page.on('pageerror', error => errors.push(error.message));
      page.on('request', request => { if (request.method() === 'POST') posts.push(request.url()); });
      let drone = {connected: false};
      await page.route('**/api/drones/state', route => route.fulfill({json: drone}));
      await page.route('**/api/import/library', route => route.fulfill({json: {items: []}}));
      await page.goto(new URL('/#live', url).href);
      await page.waitForFunction(() => document.body.dataset.mapSource === 'live');
      assert.equal(await page.locator('#calibration-open').isVisible(), true);
      assert.equal(await page.locator('#reconstruction-stage').isVisible(), false);
      assert.equal(await page.locator('#live-map-placeholder').count(), 0);
      await page.locator('#drone-open').click();
      assert.equal(await page.locator('#drone-wifi-results').isVisible(), true);
      await page.locator('#workflow-toggle').hover();
      assert.equal(await page.locator('#workflow-menu').isVisible(), true);
      assert.equal(await page.locator('#workflow-list button').nth(3).isDisabled(), true);
      drone = {connected: true};
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('2.'));
      assert.equal(await page.locator('#reconstruction-stage').isVisible(), false);
      assert.equal(await page.locator('#drone-wifi-results').isVisible(), true);
      await page.mouse.move(1000, 20);
      await page.waitForFunction(() => document.getElementById('workflow-guide').dataset.open === 'false');
      assert.equal(await page.locator('#calibration-open').isVisible(), true);
      drone = {connected: true, mapping_active: true};
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('3.'));
      assert.equal(await page.locator('#reconstruction-stage').isVisible(), true);
      drone = {connected: false, can_save: true};
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('4.'));
      await page.locator('#workflow-toggle').hover();
      await page.locator('#workflow-list button').first().click();
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('1.'));
      await page.waitForTimeout(3200);
      assert.equal(await page.locator('#reconstruction-stage').isVisible(), false);
      assert.equal(await page.locator('#drone-wifi-results').isVisible(), true);
      assert.equal(await page.locator('#workflow-list button').nth(3).isDisabled(), true);
      await page.locator('#tab-saved').click();
      await page.locator('#workflow-toggle').hover();
      assert.match(await page.locator('#workflow-title').textContent(), /저장 지도/);
      assert.equal(await page.locator('#workflow-list button').nth(3).isDisabled(), true);
      // A loaded map enters the result view through the existing result-ready event.
      await page.evaluate(() => window.dispatchEvent(new Event('saved-map-ready')));
      await page.locator('#saved-current').click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
      await page.locator('#workflow-toggle').hover();
      await page.locator('#workflow-list button').first().click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'input');
      assert.equal(await page.locator('#saved-input').isVisible(), true);
      assert.equal(await page.locator('#workflow-list button').nth(3).isDisabled(), true);
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 900});
        await page.locator('#saved-title').click();
        await page.mouse.move(width - 5, 5);
        await page.waitForFunction(() => document.getElementById('workflow-guide').dataset.open === 'false');
        await page.locator('#workflow-toggle').click();
        assert.equal(await page.locator('#workflow-menu').isVisible(), true);
        assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        await page.locator('#workflow-toggle').press('Escape');
        assert.equal(await page.locator('#workflow-toggle').getAttribute('aria-expanded'), 'false');
      }
      assert.deepEqual(posts, [], 'guide navigation must not start, stop, or cancel a job');
      let job = {id: 'guide-test', status: 'awaiting_options', stage: 'confirm',
        summary: {kind: 'video', name: 'Synthetic video', width: 640, height: 480, frames: 100, duration_s: 20}};
      await page.route('**/api/import/library', route => route.fulfill({json: {items: [], initial: job.id}}));
      await page.route('**/api/import/status?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/start?*', route => {
        job = {...job, status: 'running', stage: 'features'};
        return route.fulfill({json: job});
      });
      await page.reload();
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('2.'));
      await page.locator('#import-confirm').check();
      await page.locator('#import-start').click();
      await page.waitForFunction(() => document.getElementById('workflow-current').textContent.startsWith('3.'));
      await page.locator('#workflow-toggle').hover();
      await page.locator('#workflow-list button').nth(1).click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'settings');
      assert.equal(await page.locator('#import-options').isVisible(), true);
      assert.equal(await page.locator('#import-start').isVisible(), false);
      assert.equal(await page.locator('#import-sample-fps').isDisabled(), true);
      await page.waitForTimeout(1000);
      assert.equal(await page.locator('#import-options').isVisible(), true, 'polling preserves earlier settings view');
      await page.locator('#workflow-toggle').hover();
      await page.locator('#workflow-list button').first().click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'input');
      assert.equal(await page.locator('#saved-input').isVisible(), true);
      assert.equal(posts.filter(path => path.includes('/cancel')).length, 0);
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
