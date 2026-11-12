import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('top and bottom axis selection preserve the orbit frame during drag, pan and zoom', {skip: !url, timeout: 30000}, async () => {
  const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
  const browser = await chromium.launch({headless: true, channel: 'msedge'});
  try {
    const page = await browser.newPage();
    await page.goto(url + '/reconstruct.html');
    await page.addStyleTag({content: `
      #axis-probe .axis-gizmo { position: absolute; top: 0; right: 0; width: 128px; height: 128px; }
      #axis-probe .axis-gizmo svg { position: absolute; width: 100%; pointer-events: none; }
      #axis-probe .axis-gizmo button { position: absolute; transform: translate(-50%, -50%); min-height: 20px; padding: 0 4px; }
    `});
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
