import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('drone mapping starts with calibration off, a profile, or JSON without an extra confirmation',
  {skip: !url, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true,
      ...(process.env.MAPPING_BROWSER_CHANNEL ? {channel: process.env.MAPPING_BROWSER_CHANNEL} : {})});
    const camera = {width: 960, height: 720, fx: 700, fy: 701, cx: 480, cy: 360, distortion: [0, 0, 0, 0]};
    const profile = {id: 'test-camera', name: 'Test camera', version: 1, camera_sha256: 'test-hash',
      width: 960, height: 720, camera_identity: 'Test drone', input_modes: ['tello', 'file'], camera};
    try {
      for (const mode of ['none', 'profile', 'upload', 'disabled']) {
        const page = await browser.newPage({viewport: {width: mode === 'upload' ? 390 : 1440, height: 1000}});
        page.setDefaultTimeout(7000);
        const errors = [], starts = [];
        let active = false;
        page.on('pageerror', error => errors.push(error.message));
        await page.route('**/api/live', route => route.fulfill({json: {
          enabled: true, tello_map_enabled: true, tello_camera: camera}}));
        await page.route('**/api/camera-profiles', route => route.fulfill({json: {
          profiles: [profile], defaults: {tello: profile.id}, warnings: []}}));
        await page.route('**/api/drones/**', route => route.fulfill({json: {
          connected: true, owner: 'video-input', disconnect_available: true}}));
        await page.route('**/api/live/tello/start', async route => {
          starts.push(route.request().postDataJSON()); active = true;
          await route.fulfill({json: {session_id: 'test-session', session_status: 'starting'}});
        });
        await page.route('**/api/live/state*', route => route.fulfill({json: active ? {
          source: 'live', session_id: 'test-session', session_status: 'receiving', status: 'receiving',
          transport_ready: true, input: 'tello_camera', map_available: false} : {
          source: 'live', session_status: 'idle', status: 'idle', map_available: false}}));
        await page.route('**/api/live/tracking', route => route.fulfill({json: {
          session_status: 'complete', status: 'complete', input: 'tello_camera'}}));
        await page.route('**/api/live/stop', route => route.fulfill({json: {session_status: 'complete'}}));
        await page.goto(new URL('/#live', url).href);
        await page.locator('#drone-open').click();
        await page.waitForFunction(() => document.querySelector('#input-source').value === 'tello-map');
        assert.equal(await page.locator('#profile-enabled-live').isVisible(), true);
        assert.equal(await page.locator('#profile-enabled-live').isChecked(), false);
        assert.equal(await page.locator('#profile-live-options').isVisible(), false);
        assert.equal(await page.locator('#profile-confirm-live').count(), 0);
        if (mode !== 'none') {
          await page.locator('#profile-enabled-live').check();
          assert.equal(await page.locator('#profile-select-live').isVisible(), true);
          assert.equal(await page.locator('#camera-calibration').isVisible(), true);
          if (mode === 'profile') await page.locator('#profile-select-live').selectOption(profile.id);
          else await page.locator('#camera-calibration').setInputFiles({name: 'camera.json', mimeType: 'application/json',
            buffer: Buffer.from(JSON.stringify(camera))});
          if (mode === 'disabled') await page.locator('#profile-enabled-live').uncheck();
          if (mode === 'profile') {
            await page.locator('#profile-detail-live > summary').click();
            assert.equal(await page.locator('#profile-values-live dt').count(), 8);
            assert.match(await page.locator('#profile-values-live').textContent(), /fx 700.000.*cy 360.000/);
            await page.locator('#profile-detail-live > summary').click();
          }
        }
        assert.equal(await page.locator('#drone-map-start').isEnabled(), true);
        await page.locator('#drone-map-start').click();
        await page.waitForFunction(() => document.querySelector('#camera-status').textContent.includes('영상 연결과 SLAM') ||
          document.querySelector('#camera-status').textContent.includes('드론 영상 수신') ||
          document.querySelector('#camera-status').textContent.includes('종료')).catch(async error => {
            throw new Error(mode + ': ' + await page.locator('#camera-status').textContent() +
              ' / starts=' + starts.length, {cause: error});
          });
        assert.equal(starts.length, 1, mode + ' must reach the mapping start API');
        if (mode === 'profile') assert.equal(starts[0].profile_selection.id, profile.id);
        else assert.equal(starts[0].profile_selection, undefined);
        if (mode === 'upload') assert.equal(starts[0].camera.fy, camera.fy);
        assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        assert.deepEqual(errors, []);
        await page.close();
      }
    } finally { await browser.close(); }
  });
