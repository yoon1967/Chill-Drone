import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('top and bottom axis selection preserve the orbit frame during drag, pan and zoom', {skip: !url, timeout: 30000}, async () => {
  const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
  const browser = await chromium.launch({headless: true, channel: 'msedge'});
  try {
    const page = await browser.newPage();
    await page.goto(url + '/#saved');
    for (const up of [1, -1]) {
      await page.evaluate(async up => {
        const THREE = await import('/vendor/three.module.js');
        const {OrbitControls} = await import('/vendor/OrbitControls.js');
        const {connectAxisGizmo} = await import('/axis-gizmo.js');
        window.axisProbe?.dispose();
        const host = document.createElement('div'); host.id = 'axis-probe';
        host.style.cssText = 'position:fixed;left:0;top:0;width:400px;height:400px;z-index:10000;background:#142326';
        const canvas = document.createElement('canvas'); canvas.style.cssText = 'width:400px;height:400px'; host.append(canvas); document.body.append(host);
        const camera = new THREE.PerspectiveCamera(48, 1, .01, 1000); camera.up.set(0, up, 0); camera.position.set(4, 3, 6);
        const controls = new OrbitControls(camera, canvas); controls.target.set(1, 1, 1); controls.update();
        const gizmo = connectAxisGizmo({host, camera, controls, up: [0, up, 0]});
        window.axisProbe = {camera, controls, dispose() { gizmo.dispose(); controls.dispose(); host.remove(); }};
      }, up);
      for (const axis of ['+Y', '-Y', '+X']) {
        await page.locator(`#axis-probe [data-axis="${axis}"]`).click();
        const snapshot = () => page.evaluate(() => {
          const {camera, controls} = window.axisProbe;
          return {up: camera.up.toArray(), position: camera.position.toArray(), target: controls.target.toArray(), distance: camera.position.distanceTo(controls.target), quaternion: camera.quaternion.toArray()};
        });
        const before = await snapshot();
        assert.deepEqual(before.up, [0, up, 0], `${axis} must not change the orbit controller's world-up basis`);
        await page.mouse.move(140, 220); await page.mouse.down(); await page.mouse.move(190, 250, {steps: 8}); await page.mouse.up();
        const rotated = await snapshot();
        assert.notDeepEqual(rotated.quaternion, before.quaternion, 'drag changes camera orientation');
        assert.ok(Math.abs(rotated.distance - before.distance) < 1e-7, 'rotation preserves zoom');
        rotated.target.forEach((value, i) => assert.ok(Math.abs(value - before.target[i]) < 1e-10, 'rotation preserves the orbit center'));
        assert.deepEqual(rotated.up, [0, up, 0]);
        assert.ok(rotated.position.every(Number.isFinite));
        await page.mouse.move(140, 220); await page.mouse.down({button: 'right'}); await page.mouse.move(180, 235, {steps: 5}); await page.mouse.up({button: 'right'});
        const panned = await snapshot();
        assert.notDeepEqual(panned.target, rotated.target, 'right-drag pans the target');
        assert.ok(Math.abs(panned.distance - rotated.distance) < 1e-7);
        await page.mouse.wheel(0, 160); await page.waitForTimeout(80);
        assert.ok((await snapshot()).distance > panned.distance, 'wheel zoom works after axis selection');
      }
    }
  } finally { await browser.close(); }
});

test('restored maps use interactive world axes and never start retired hologram analysis', {skip: !url, timeout: 30000}, async () => {
  const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
  const browser = await chromium.launch({headless: true, channel: 'msedge'});
  try {
    const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
    const errors = [], workers = []; page.on('pageerror', e => errors.push(e.message)); page.on('worker', worker => workers.push(worker.url()));
    const fixture = {source: 'saved', schema_version: 'phone-sfm-result/1.2', input_run_id: 'axis-test', created_utc: '2026-11-03T00:00:00Z',
      pose_source: 'orb_slam3_depth', scale: {unit: 'arbitrary'}, segment_id: 0,
      points: [[0, 0, 0, 1, 255, 0, 0, 0, 1], [1, 1, 0, 1, 0, 255, 0, 0, 1], [2, 0, 1, 1, 0, 0, 255, 0, 1]],
      cameras: [0, 1].map(i => ({frame_id: `frame_${i}.jpg`, timestamp_s: i, position: [i, 0, 0], width: 32, height: 32, params: [20, 20, 16, 16], world_from_camera_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]})),
      quality: {result_status: 'complete', input_frames: 2, registered_frames: 2, registration_ratio: 1, excluded_frames: [], processing_seconds: 1, mean_reprojection_error_px: null, bounds: {min: [0, 0, 1], max: [1, 1, 1]}}};
    await page.route('**/api/saved/state*', route => route.fulfill({json: {source: 'saved', status: 'offline', map_available: true}}));
    await page.route('**/api/saved/run*', route => route.fulfill({json: fixture}));
    await page.route('**/api/import/status?*', route => route.fulfill({json: {id: 'import_1111111111111111', status: 'complete', summary: {kind: 'result', name: 'axis-test', media: []}}}));
    await page.goto(url + '/?asset=import_1111111111111111&mode=hologram#saved');
    await page.locator('#viewport .axis-gizmo button:not(:disabled)').first().waitFor();
    assert.equal(await page.locator('.view-buttons,[data-mode="hologram"],#reset,#progressive-fit').count(), 0);
    for (const axis of ['+X', '+Y', '+Z', '-X', '-Y', '-Z']) {
      await page.locator(`#viewport [data-axis="${axis}"]`).click();
      await page.waitForTimeout(80);
      const actual = (await page.locator('#viewport').getAttribute('data-camera-direction')).split(',').map(Number);
      const expected = [0, 0, 0]; expected['XYZ'.indexOf(axis[1])] = axis[0] === '+' ? -1 : 1;
      actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < .0001, `${axis} points at the orbit target`));
    }
    const before = await page.locator('#viewport [data-axis="+X"]').getAttribute('style');
    const canvas = await page.locator('#viewport canvas').boundingBox();
    await page.mouse.move(canvas.x + canvas.width / 2, canvas.y + canvas.height / 2);
    await page.mouse.down(); await page.mouse.move(canvas.x + canvas.width / 2 + 120, canvas.y + canvas.height / 2 + 60, {steps: 8}); await page.mouse.up();
    await page.waitForTimeout(150);
    assert.notEqual(await page.locator('#viewport [data-axis="+X"]').getAttribute('style'), before, 'axis widget follows orbit rotation');
    assert.equal(await page.locator('#hologram-controls').isVisible(), false);
    assert.deepEqual(workers.filter(worker => /point-spacing|surface/.test(worker)), []);
    assert.deepEqual(errors, []);
  } finally { await browser.close(); }
});
