import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('live reconstruction shows actual model preparation, then themed controls and camera stop during replay',
  {skip: !url, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({headless: true, channel: 'msedge'});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      const stops = [], errors = [], presented = [], imageRequests = [];
      let uploaded = 0;
      let active = false, lifecycle = 'receiving';
      const job = {id: 'import_1234567890123456', status: 'running', stage: 'model', history_available: true,
        options: {render_window_s: null}};
      const preparing = () => ['model', 'semantic_model'].includes(job.stage);
      const state = () => active ? {source: 'live', session_id: 'test-live', session_status: preparing() ? 'starting' : lifecycle,
        status: preparing() ? 'starting' : lifecycle, backend: 'orb_slam3_depth', input: 'browser_camera', transport_ready: !preparing(),
        can_accept_frame: !preparing(), map_available: false, depth_job_id: job.id, depth_job: job} :
        {source: 'live', session_status: 'idle', status: 'idle', map_available: false};
      const mapped = {revision: 2, frame_id: 'frame_0000002.jpg', frame_number: 2, timestamp_s: 2,
        tracking: 'tracking', pose_confirmed: true, pose_position: [0, 0, 0],
        pose_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], map_id: 0, pose_map_id: 0,
        points: [[0, 0, 2, 255, 0, 0]], cameras: [[0, 0, 0]], processed: 1, registered: 1,
        processing_ms: 40, latency_ms: 50, render_window_s: null};
      const snapshot = {...mapped, revision: 1, frame_id: 'frame_0000001.jpg', frame_number: 1,
        timestamp_s: 1, tracking: 'initializing', pose_confirmed: false, map_id: null,
        points: [], cameras: [], registered: 0, needs_presentation: true};
      page.on('pageerror', error => errors.push(error.message));
      await page.addInitScript(() => {
        // A real browser MediaStream with deterministic pixels, independent of
        // Windows camera drivers. This test exercises UI preparation and stop.
        navigator.mediaDevices.getUserMedia = async () => {
          const canvas = document.createElement('canvas'); canvas.width = 960; canvas.height = 540;
          const context = canvas.getContext('2d'); let frame = 0;
          const paint = () => { context.fillStyle = `rgb(${frame++ % 255},100,140)`; context.fillRect(0, 0, 960, 540); };
          paint(); window.testCaptureTimer = setInterval(paint, 33);
          const stream = canvas.captureStream(30); window.testTracks = stream.getTracks(); return stream;
        };
      });
      await page.route('**/api/live', route => route.fulfill({json: {enabled: true}}));
      await page.route('**/api/drones/state*', route => route.fulfill({json: {connected: false}}));
      await page.route('**/api/tello', route => route.fulfill({json: {enabled: false}}));
      // An existing selected saved map must not navigate this isolated live test
      // away from its mocked camera session during application startup.
      await page.route('**/api/import/library*', route => route.fulfill({json: {items: [], initial: null}}));
      await page.route('**/api/live/start', route => {
        assert.equal(route.request().postDataJSON().sample_fps, 2.5);
        active = true; return route.fulfill({json: state()});
      });
      await page.route('**/api/live/state*', route => route.fulfill({json: state()}));
      await page.route('**/api/live/tracking*', route => route.fulfill({json: state()}));
      await page.route('**/api/live/frame?*', route => {
        uploaded++;
        return route.fulfill({json: state()});
      });
      await page.route('**/api/live/stop', route => {
        stops.push(route.request().postDataJSON()); lifecycle = 'stopping';
        return route.fulfill({json: state()});
      });
      await page.route('**/api/import/history?*', route => route.fulfill({json: {
        entries: [{revision: snapshot.revision, timestamp_s: snapshot.timestamp_s}]}}));
      await page.route('**/api/import/history-frame?*', route => route.fulfill({json: snapshot}));
      await page.route('**/api/import/preview?*', route => route.fulfill({json:
        preparing() ? {waiting: true, revision: 0} : Number(new URL(route.request().url()).searchParams.get('revision')) === snapshot.revision
          ? {revision: snapshot.revision, unchanged: true} : snapshot}));
      await page.route('**/api/import/presented?*', route => {
        const {revision} = route.request().postDataJSON();
        presented.push(revision);
        return route.fulfill({json: {revision}});
      });
      await page.route('**/api/import/preview-frame?*', route => {
        imageRequests.push(new URL(route.request().url()).searchParams.get('name'));
        return route.fulfill({contentType: 'image/png',
          body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')});
      });
      await page.goto(url + '/#live');
      // This test mocks browser capture, independently of an enabled Tello input.
      await page.locator('#input-source').evaluate(select => { select.value = 'camera'; select.dispatchEvent(new Event('change', {bubbles: true})); });
      await page.locator('#rescue-settings summary').click();
      await page.locator('#rescue-sample-fps').fill('2.5');
      await page.waitForFunction(() => !document.querySelector('#camera-start').disabled);
      // The legacy input panel is tucked inside the result workspace. Exercise
      // its real capture handler, then use the visible live reconstruction UI.
      await page.locator('#camera-start').evaluate(button => button.click());
      for (const [stage, title] of [['model', '깊이 모델 준비 중'], ['semantic_model', '사람·공간 인식 모델 준비 중']]) {
        job.stage = stage;
        await page.waitForFunction(title => document.querySelector('#live-progressive-workspace h2')?.textContent === title, title, {timeout: 8000}).catch(async error => {
          const status = await page.evaluate(() => ({hash: location.hash, camera: document.querySelector('#camera-status')?.textContent,
            input: document.querySelector('#input-state')?.textContent, title: document.querySelector('#live-progressive-workspace h2')?.textContent}));
          throw new Error(`${error.message}: ${JSON.stringify({active, errors, status})}`);
        });
        assert.equal(await page.locator('#live-progressive-state').textContent(), title);
        assert.equal(await page.locator('#input-state').textContent(), '모델 준비 중');
        assert.equal(await page.locator('#map-state').textContent(), '모델 준비 중');
        assert.equal(await page.locator('#tracking-state').textContent(), '모델 준비 대기');
        assert.equal(await page.locator('#live-tab-indicator').textContent(), '모델 준비 중');
        await page.waitForFunction(() => document.querySelector('#connection-map').textContent === '모델 준비 중');
        assert.match(await page.locator('#connection-note').textContent(), /모델 준비 중/);
        assert.equal(await page.locator('#live-progressive-frame').isVisible(), false);
        assert.equal(await page.locator('#input-preview').getAttribute('hidden'), '');
      }
      assert.equal(uploaded, 0);
      job.stage = 'tracking';
      await page.waitForFunction(() => document.querySelector('#live-progressive-count')?.textContent.includes('지도 반영 0'));
      await page.waitForFunction(() => document.querySelector('#camera-status')?.dataset.sent >= 1);
      assert.ok(uploaded >= 1);
      assert.deepEqual(presented, [1]);
      assert.deepEqual(imageRequests, []);
      assert.equal(await page.locator('#live-progressive-frame').isVisible(), false);
      assert.equal(await page.locator('#live-progressive-play').isDisabled(), true);
      Object.assign(snapshot, mapped, {needs_presentation: false});
      await page.locator('#live-progressive-frame:not([hidden])').waitFor();
      assert.equal(imageRequests[0], mapped.frame_id);
      assert.equal(await page.locator('#live-progressive-viewport').getAttribute('data-point-count'), '1');
      await page.waitForFunction(() => !document.querySelector('#live-progressive-workspace h2').textContent.includes('모델 준비'));
      assert.doesNotMatch(await page.locator('#live-progressive-video-empty').textContent(), /인식 모델 준비 중/);
      await page.waitForFunction(() => window.testTracks?.some(track => track.readyState === 'live'));
      assert.equal(await page.locator('#live-depth-stop').isVisible(), true);
      assert.match(await page.locator('#live-progressive-map-detail').textContent(), /전체 누적 지도/);
      for (const width of [1440, 390]) {
        await page.setViewportSize({width, height: 1000});
        const styles = await page.locator('#live-progressive-workspace').evaluate(panel =>
          ['live-progressive-play', 'live-progressive-follow', 'live-progressive-speed', 'live-progressive-seek', 'live-progressive-seek-go'].map(id => {
            const style = getComputedStyle(panel.querySelector('#' + id));
            return {radius: style.borderRadius, font: style.fontFamily, size: style.fontSize};
          }));
        assert.ok(styles.every(style => style.radius === '8px' && style.size === '12px' && style.font === styles[0].font));
        assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      }
      // A browser can load new JS from a server that still runs old Python.
      job.options.render_window_s = 10;
      Object.assign(snapshot, {revision: 3, timestamp_s: 24.688, tracking: 'lost', pose_confirmed: false,
        points: [], map_points: 0, render_window_s: 10});
      await page.waitForFunction(() => document.querySelector('#live-depth-error').textContent.includes('최근 10초'));
      await page.waitForFunction(() => document.querySelector('#live-progressive-viewport').dataset.retainingMap === 'true');
      await page.locator('#live-progressive-map-empty').waitFor({state: 'hidden'});
      assert.equal(await page.locator('#live-progressive-viewport').getAttribute('data-point-count'), '1');
      assert.match(await page.locator('#live-progressive-map-detail').textContent(), /마지막 유효 지도/);
      assert.match(await page.locator('#live-depth-error').textContent(), /실행 CMD를 다시/);
      job.options.render_window_s = null;
      Object.assign(snapshot, {revision: 4, points: [[0, 0, 2, 255, 0, 0]], map_points: 1, render_window_s: null});
      await page.locator('#live-depth-error').waitFor({state: 'hidden'});
      await page.locator('#live-progressive-map-empty').waitFor({state: 'hidden'});
      await page.locator('#live-progressive-seek-go').click();
      assert.equal(await page.locator('#live-progressive-follow').getAttribute('aria-pressed'), 'false');
      await page.locator('#live-depth-stop').click();
      await page.waitForFunction(() => window.testTracks.every(track => track.readyState === 'ended'));
      assert.deepEqual(stops, [{session_id: 'test-live', reason: 'user_stop'}]);
      assert.equal(await page.locator('#input-preview').evaluate(video => video.srcObject), null);
      assert.equal(await page.locator('#live-depth-stop').isDisabled(), true);
      lifecycle = 'complete'; job.status = 'complete';
      await page.waitForFunction(() => document.querySelector('#live-depth-stop').textContent === '촬영 종료됨');
      assert.equal(await page.locator('#live-progressive-workspace').isVisible(), true);
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
