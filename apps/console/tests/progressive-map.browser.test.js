import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('reconstruction has no axis widget and completed maps reopen after replay on desktop and mobile',
  {skip: !url, timeout: 30000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      const errors = []; page.on('pageerror', e => errors.push(e.message));
      page.on('console', message => { if (message.text().includes('Shader Error')) errors.push(message.text()); });
      const job = {id: 'import_1234567890123456', name: 'test', status: 'running', stage: 'mapping', history_available: true,
        options: {mode: 'depth_rgbd'}, progress: {processed: 1, points: 3}};
      const completed = {source: 'saved', schema_version: 'phone-sfm-result/1.2', input_run_id: 'replay-test',
        created_utc: '2026-11-03T00:00:00Z', pose_source: 'orb_slam3_depth', scale: {unit: 'arbitrary'}, segment_id: 0,
        points: [[0, 0, 0, 1, 255, 0, 0, 0, 1], [1, 1, 0, 1, 0, 255, 0, 0, 1], [2, 0, 1, 1, 0, 0, 255, 0, 1]],
        rescue: {render_mode: 'points', point_labels: [0, 0, 0], point_tof: [true, false, true], people: []},
        cameras: [{frame_id: 'frame_0000001.jpg', timestamp_s: .2, position: [0, 0, 0], width: 32, height: 32,
          params: [20, 20, 16, 16], world_from_camera_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
        quality: {result_status: 'complete', input_frames: 1, registered_frames: 1, registration_ratio: 1,
          excluded_frames: [], processing_seconds: 1, mean_reprojection_error_px: null,
          bounds: {min: [0, 0, 0], max: [1, 1, 1]}}};
      await page.route('**/api/saved/state*', route => route.fulfill({json: {source: 'saved', status: 'offline', map_available: true}}));
      await page.route('**/api/saved/run*', route => route.fulfill({json: completed}));
      await page.route('**/api/import/library', route => route.fulfill({json: {initial: job.id, items: []}}));
      await page.route('**/api/import/status?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/history?*', route => route.fulfill({json: {entries: [
        {revision: 1, timestamp_s: 0}, {revision: 2, timestamp_s: .1}, {revision: 3, timestamp_s: .15},
        {revision: 4, timestamp_s: .18}, {revision: 5, timestamp_s: .2}, {revision: 6, timestamp_s: 2}]}}));
      await page.route('**/api/import/history-frame?*', route => route.fulfill({json: {revision: 6, frame_id: 'frame_0000061.jpg', frame_number: 61, timestamp_s: 2, tracking: 'lost', points: [], cameras: [], processed: 2, registered: 1, dropped: 0, latency_ms: 80, processing_ms: 70}}));
      const mapped = {revision: 5, frame_id: 'frame_0000002.jpg', frame_number: 2, timestamp_s: .2, tracking: 'tracking', pose_confirmed: true, pose_position: [0, 0, 0], pose_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], map_id: 0, pose_map_id: 0, points: [[0, 0, 2, 255, 0, 0], [1, 0, 2, 0, 255, 0], [0, 1, 2, 0, 0, 255]], cameras: [[0, 0, 0]], processed: 2, registered: 1, dropped: 0, latency_ms: 80, processing_ms: 70,
        rescue: {map_id: 0, render_mode: 'points', point_revision: 1, point_labels: [0, 0, 0], point_tof: [true, false, true], people: []}};
      let preview = {...mapped, revision: 1, frame_id: 'frame_0000001.jpg', frame_number: 1, timestamp_s: 0,
        tracking: 'initializing', pose_confirmed: false, points: [], cameras: [], processed: 1, registered: 0, rescue: null};
      await page.route('**/api/import/preview?*', route => route.fulfill({json:
        Number(new URL(route.request().url()).searchParams.get('revision')) === preview.revision ? {unchanged: true} : preview}));
      await page.route('**/api/import/preview-frame?*', route => route.fulfill({contentType: 'image/png', body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')}));
      await page.goto(url + '/#saved');
      await page.evaluate(async () => {
        const {OrbitControls} = await import('/vendor/OrbitControls.js');
        const update = OrbitControls.prototype.update;
        OrbitControls.prototype.update = function (...args) {
          const result = update.apply(this, args);
          if (this.domElement?.parentElement?.id === 'progressive-viewport') {
            window.reconstructionView = {position: this.object.position.toArray(), target: this.target.toArray(),
              rotation: this.object.quaternion.toArray()};
          }
          return result;
        };
      });
      const canvas = page.locator('#progressive-viewport canvas');
      const readView = () => page.evaluate(() => window.reconstructionView);
      const delta = (a, b) => a.map((value, i) => value - b[i]);
      const near = (a, b) => Math.hypot(...delta(a, b)) < 1e-7;
      async function pan() {
        await canvas.scrollIntoViewIfNeeded();
        await page.waitForFunction(() => window.reconstructionView);
        const before = await readView();
        const {x, y, width, height} = await canvas.boundingBox();
        await page.mouse.move(x + width / 2, y + height / 2);
        await page.mouse.down({button: 'right'});
        await page.mouse.move(x + width / 2 + 55, y + height / 2 + 30, {steps: 4});
        await page.mouse.up({button: 'right'});
        const after = await readView();
        assert.ok(Math.hypot(...delta(after.position, before.position)) > .01, 'right drag moves the viewing camera');
        assert.ok(near(delta(after.position, before.position), delta(after.target, before.target)), 'position and target move together');
        assert.ok(near(after.rotation, before.rotation), 'right drag preserves viewing direction');
        return after;
      }
      await page.waitForFunction(() => document.querySelector('#progressive-count').textContent.includes('지도 반영 0'));
      assert.equal(await page.locator('#progressive-frame').isVisible(), false);
      preview = {...preview, revision: 2, timestamp_s: .1, front_tof: {distance_m: null, used: false, surface_supported: false},
        frame_preview: {space: 'camera', tof_verified: false, points: mapped.points}};
      await page.locator('#progressive-frame:not([hidden])').waitFor();
      const small = Number(await page.locator('#progressive-viewport').getAttribute('data-frame-point-size'));
      assert.equal(small, 2);
      const pannedPreview = await pan();
      preview = {...preview, revision: 3, timestamp_s: .15, tracking: 'lost',
        front_tof: {distance_m: .8, surface_supported: true},
        frame_preview: {space: 'camera', tof_verified: true, points: mapped.points}};
      await page.locator('#progressive-viewport[data-frame-point-size="2.8"] canvas').waitFor();
      await page.locator('#progressive-frame:not([hidden])').waitFor();
      assert.match(await page.locator('#progressive-map-detail').textContent(), /카메라 기준 임시 점군/);
      assert.match(await page.locator('#progressive-count').textContent(), /지도 반영 0/);
      assert.ok(near((await readView()).position, pannedPreview.position), 'a new temporary point cloud preserves the pan');
      preview = {...mapped, revision: 4, timestamp_s: .18, tracking: 'pose_predicted', pose_confirmed: false};
      await page.locator('#progressive-pose').filter({hasText: /선형 이동 기반 자세 예측/}).waitFor();
      const pannedMap = await pan();
      preview = mapped;
      await page.locator('#progressive-pose').filter({hasText: /위치·방향 확정/}).waitFor();
      await page.locator('#progressive-viewport[data-frame-preview=""][data-point-count="3"]').waitFor();
      assert.ok(near((await readView()).position, pannedMap.position), 'a new revision of the same map preserves the pan');
      assert.equal(await page.locator('#progressive-viewport').getAttribute('data-verified-points'), '2');
      assert.match(await page.locator('#progressive-frame').getAttribute('src'), /frame_0000002/);
      await page.waitForFunction(() => document.querySelector('#progressive-count').textContent.includes('지도 반영 1'));
      assert.equal(await page.locator('#progressive-viewport canvas').isVisible(), true);
      assert.equal(await page.locator('#progressive-download').isVisible(), false, 'a full archive needs its completed result');
      assert.equal(await page.locator('#progressive-viewport .axis-gizmo').count(), 0, 'reconstruction progress has no axis widget');
      assert.match(await page.locator('#progressive-pose').textContent(), /위치·방향 확정/);
      const controlStyles = await page.locator('#progressive-workspace').evaluate(panel =>
        ['progressive-play', 'progressive-follow', 'progressive-speed', 'progressive-seek', 'progressive-seek-go'].map(id => {
          const style = getComputedStyle(panel.querySelector('#' + id));
          return {radius: style.borderRadius, font: style.fontFamily, size: style.fontSize};
        }));
      assert.ok(controlStyles.every(style => style.radius === '8px' && style.size === '12px' && style.font === controlStyles[0].font));
      const video = await page.locator('.progressive-video').boundingBox(), map = await page.locator('.progressive-map').boundingBox();
      assert.ok(map.x > video.x && Math.abs(map.y - video.y) < 5, 'desktop panels are side by side');
      assert.equal(await page.locator('#map-content').isVisible(), false, 'the old result viewer stays separate');
      await page.locator('#progressive-seek').fill('2');
      await page.locator('#progressive-seek-go').click();
      await page.waitForFunction(() => document.querySelector('#progressive-frame').src.includes('frame_0000061'));
      assert.equal(await page.locator('#progressive-viewport .axis-gizmo').count(), 0, 'historical reconstruction playback has no axis widget');
      await page.waitForTimeout(900);
      assert.match(await page.locator('#progressive-frame').getAttribute('src'), /frame_0000061/, 'new status polling preserves historical playback');
      await page.locator('#progressive-follow').click();
      await page.waitForFunction(() => document.querySelector('#progressive-frame').src.includes('frame_0000002'));
      await page.setViewportSize({width: 390, height: 900});
      const mobileVideo = await page.locator('.progressive-video').boundingBox(), mobileMap = await page.locator('.progressive-map').boundingBox();
      assert.ok(mobileMap.y > mobileVideo.y);
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      job.status = 'complete'; job.stage = 'finished';
      job.summary = {kind: 'result', name: 'test', points: 3, keyframes: 1, revisions: 0};
      await page.waitForFunction(() => document.querySelector('#progressive-state').textContent.includes('복원 완료'));
      assert.ok(page.url().endsWith('#saved'), 'completion does not replace the live preview');
      assert.equal(await page.locator('#import-open').isVisible(), true);
      assert.equal(await page.locator('#progressive-download').isVisible(), true);
      await page.locator('#import-open').click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 1000});
        await page.locator('#saved-replay').click();
        await page.locator('#progressive-frame:not([hidden])').waitFor();
        assert.equal(await page.locator('#progressive-viewport .axis-gizmo').count(), 0);
        await page.locator('#import-open').click();
        await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
        await page.locator('#viewport canvas').waitFor({state: 'visible'});
        assert.equal(await page.locator('#map-content').isVisible(), true);
        assert.equal(await page.locator('#viewport canvas').isVisible(), true);
        assert.equal(await page.locator('#viewport .axis-gizmo button').count(), 6, 'completed maps keep their axis controls');
        assert.equal(await page.locator('#progressive-viewport canvas').count(), 0, 'leaving replay releases its renderer');
      }
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
