import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const url = process.env.MAPPING_TEST_URL;
test('live mapping uses sequential pose/depth, waits for display, and integrates the complete recorded path',
  {skip: !url || !process.env.MAPPING_LIVE_DEPTH_VIDEO, timeout: 240000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ? createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('../node_modules/playwright/index.mjs');
    const browser = await chromium.launch({headless: true, channel: 'msedge',
      args: ['--disable-background-timer-throttling', '--disable-renderer-backgrounding']});
    let session;
    const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
    const errors = []; page.on('pageerror', error => errors.push(error.message));
    const status = async () => (await page.request.get(`${url}/api/live/state?session=${session}`)).json();
    try {
      await page.goto(url + '/#live');
      const response = await page.request.post(url + '/api/live/start', {
        headers: {'X-Mapping-Client': '1'}, data: {width: 640, height: 360, fps: 15, fov: 60}});
      const started = await response.json();
      assert.equal(response.status(), 201, JSON.stringify(started));
      session = started.session_id;
      assert.equal(started.backend, 'orb_slam3_depth');
      let state;
      for (let attempt = 0; attempt < 600; attempt++) {
        state = await status();
        assert.notEqual(state.session_status, 'failed', JSON.stringify(state));
        if (state.transport_ready) break;
        assert.equal(state.depth_job.progress?.processed || 0, 0);
        await page.waitForTimeout(100);
      }
      assert.equal(state.transport_ready, true);
      await page.locator('#live-progressive-workspace').waitFor({state: 'visible'});
      let held = null, release;
      const intercepted = new Promise(resolve => { release = resolve; });
      await page.route('**/api/import/presented?*', route => {
        if (!held) { held = route; release(); }
        else return route.continue();
      });
      const begin = Date.now();
      const feeding = page.evaluate(async session => {
        const video = document.createElement('video'); video.muted = true; video.preload = 'auto';
        video.src = '/api/source-video';
        await new Promise((resolve, reject) => { video.addEventListener('loadeddata', resolve, {once: true}); video.addEventListener('error', reject, {once: true}); });
        const canvas = document.createElement('canvas'); canvas.width = 640; canvas.height = 360;
        const context = canvas.getContext('2d');
        for (let index = 0; index < 180; index++) {
          const stamp = index / 15;
          if (index) await new Promise(resolve => { video.addEventListener('seeked', resolve, {once: true}); video.currentTime = stamp; });
          context.drawImage(video, 0, 0, 640, 360);
          const body = await new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', .85));
          const response = await fetch('/api/live/frame?session=' + session, {method: 'POST',
            headers: {'X-Mapping-Client': '1', 'Content-Type': 'image/jpeg', 'X-Frame-Timestamp': String(stamp), 'X-Frame-Sequence': String(index)}, body});
          if (!response.ok) throw new Error(await response.text());
          await response.json();
        }
        video.removeAttribute('src'); video.load();
      }, session);
      await intercepted;
      const first = await status();
      assert.equal(first.depth_job.progress.processed, 1);
      await page.waitForTimeout(250);
      assert.equal((await status()).transport_received, 1);
      await held.continue(); await page.unroute('**/api/import/presented?*');
      await feeding;
      const stop = await page.request.post(url + '/api/live/stop', {headers: {'X-Mapping-Client': '1'}, data: {session_id: session, reason: 'user_stop'}});
      assert.equal(stop.status(), 202);
      for (let attempt = 0; attempt < 400; attempt++) {
        state = await status();
        if (state.session_status === 'failed' || (state.session_status === 'complete' && state.depth_job.status === 'complete')) break;
        await page.waitForTimeout(100);
      }
      assert.equal(state.session_status, 'complete', JSON.stringify(state));
      assert.equal(state.depth_job.status, 'complete', JSON.stringify(state.depth_job));
      assert.equal(state.depth_job.summary.recording.finalized, true);
      assert.equal(state.depth_job.summary.media.length, 1);
      assert.deepEqual(state.depth_job.summary.warnings, []);
      assert.equal(state.depth_job.progress.dropped, 0);
      assert.ok(state.depth_job.progress.registered > 5);
      const result = await page.request.get(`${url}/api/live/run?session=${session}`);
      assert.equal(result.status(), 200);
      const scene = await result.json();
      assert.equal(scene.pose_source, 'orb_slam3_depth');
      assert.equal(scene.input_run_id, session);
      assert.equal(scene.depth_mapping.preview_window_applied, false);
      assert.equal(scene.depth_mapping.integrated_frames, state.depth_job.progress.registered);
      await page.locator('#session-download').waitFor({state: 'visible'});
      const archive = await page.request.get(`${url}/api/live/map.zip?session=${session}`);
      assert.equal(archive.status(), 200);
      assert.match(archive.headers()['content-type'], /zip/);
      await archive.dispose();
      await page.locator('#live-depth-open').waitFor({state: 'visible'});
      await page.locator('#live-depth-open').click();
      await page.waitForFunction(() => document.body.dataset.savedPhase === 'result');
      await page.locator('#viewport canvas').waitFor({state: 'visible'});
      assert.deepEqual(errors, []);
      console.log(JSON.stringify({session, elapsedMs: Date.now() - begin, processed: state.depth_job.progress.processed,
        registered: state.depth_job.progress.registered, finalPoints: scene.points.length}));
    } finally {
      if (session) await page.request.post(url + '/api/live/stop', {headers: {'X-Mapping-Client': '1'}, data: {session_id: session, reason: 'user_stop'}}).catch(() => {});
      await browser.close();
    }
  });
