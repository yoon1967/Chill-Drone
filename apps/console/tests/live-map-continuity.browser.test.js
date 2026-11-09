import test from 'node:test';
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {createRequire} from 'node:module';
import {createInterface} from 'node:readline';
import {fileURLToPath} from 'node:url';

test('live cloud survives empty/error results and unaligned segments remain accessible with a fitted camera',
  {skip: !process.env.MAPPING_TEST_PYTHON || !process.env.MAPPING_PLAYWRIGHT_MODULE, timeout: 60000}, async () => {
    const root = fileURLToPath(new URL('../../../', import.meta.url));
    const fixture = spawn(process.env.MAPPING_TEST_PYTHON, ['-B', 'tests/integration/rescue_browser_fixture.py'],
      {cwd: root, stdio: ['pipe', 'pipe', 'inherit']});
    let browser;
    try {
      const lines = createInterface({input: fixture.stdout});
      const info = await new Promise((resolve, reject) => {
        lines.once('line', line => { try { resolve(JSON.parse(line)); } catch (error) { reject(error); } });
        fixture.once('exit', code => reject(new Error(`fixture exited ${code}`)));
      });
      const {chromium} = createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE);
      browser = await chromium.launch({channel: 'msedge', headless: true});
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      const errors = []; page.on('pageerror', error => errors.push(error.message));
      await page.addInitScript(() => { window.renderCount = 0; window.fitTargets = []; });
      await page.route('**/progressive-map.js', async route => {
        const source = await (await route.fetch()).text();
        await route.fulfill({contentType: 'text/javascript', body: source
          .replace('renderer.render(scene, camera);', 'renderer.render(scene, camera); window.renderCount++;')
          .replace('mapTarget.copy(center);', 'window.fitTargets.push(center.toArray()); mapTarget.copy(center);')});
      });
      const job = {id: 'import_1234567890123456', status: 'running', stage: 'mapping', name: 'continuity',
        options: {mode: 'depth_rgbd'}, progress: {processed: 1, points: 3}, history_available: true};
      const first = {revision: 1, frame_id: 'frame_0000001.jpg', frame_number: 1, timestamp_s: 1,
        tracking: 'tracking', map_id: 0, submaps: 1, processed: 1, registered: 1, processing_ms: 50, latency_ms: 80,
        points: [[0, 0, 2, 255, 0, 0], [1, 0, 2, 0, 255, 0], [0, 1, 2, 0, 0, 255]], map_points: 3, cameras: [[0, 0, 0]]};
      const second = {...first, revision: 3, frame_id: 'frame_0000003.jpg', timestamp_s: 3, map_id: 1, submaps: 2,
        points: first.points.map(p => [p[0] + 1000, ...p.slice(1)]), cameras: [[1000, 0, 0]]};
      let snapshot = first, fail = false, imageFail = false;
      const entries = [first];
      await page.route('**/api/import/library', route => route.fulfill({json: {initial: job.id, items: []}}));
      await page.route('**/api/import/status?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/history?*', route => route.fulfill({json: {entries}}));
      await page.route('**/api/import/history-frame?*', route => route.fulfill({json: first}));
      await page.route('**/api/import/preview?*', route => fail ? route.fulfill({status: 503, json: {error: 'temporary'}}) :
        route.fulfill({json: Number(new URL(route.request().url()).searchParams.get('revision')) === snapshot.revision ? {unchanged: true} : snapshot}));
      await page.route('**/api/import/preview-frame?*', route => imageFail ? route.fulfill({status: 503, body: 'temporary'}) : route.fulfill({contentType: 'image/png',
        body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')}));
      await page.goto(info.url + '/#saved');
      const host = page.locator('#progressive-viewport');
      await page.waitForFunction(() => document.querySelector('#progressive-viewport').dataset.pointCount === '3');
      const geometry = await host.getAttribute('data-point-geometry-id');
      snapshot = {...first, revision: 2, frame_id: 'frame_0000002.jpg', timestamp_s: 2, tracking: 'lost', points: [], map_points: 0};
      await page.waitForFunction(() => document.querySelector('#progressive-viewport').dataset.retainingMap === 'true');
      assert.equal(await host.getAttribute('data-point-count'), '3');
      assert.match(await page.locator('#progressive-map-detail').textContent(), /마지막 유효 지도/);
      fail = true;
      const before = await page.evaluate(() => window.renderCount);
      await page.waitForTimeout(600);
      assert.ok(await page.evaluate(() => window.renderCount) > before + 5, 'the real renderer keeps drawing during input/API stalls');
      assert.equal(await host.getAttribute('data-point-count'), '3');
      fail = false; snapshot = second; entries.push(second);
      await page.waitForFunction(() => document.querySelector('#progressive-viewport').dataset.displayedMap === '1');
      assert.equal(await host.getAttribute('data-point-geometry-id'), geometry, 'segment switches reuse bounded GPU storage');
      assert.ok((await page.evaluate(() => window.fitTargets.at(-1)))[0] > 1000, 'a new coordinate system refits the camera');
      await page.locator('#progressive-segments:not([hidden])').waitFor();
      await page.locator('#progressive-segment').selectOption('0');
      await page.waitForFunction(() => document.querySelector('#progressive-viewport').dataset.displayedMap === '0');
      assert.equal(await host.getAttribute('data-point-count'), '3', 'the earlier map remains available');
      await page.locator('#progressive-segment').selectOption('latest');
      await page.waitForFunction(() => document.querySelector('#progressive-viewport').dataset.displayedMap === '1');
      imageFail = true; snapshot = {...second, revision: 4, frame_id: 'frame_0000004.jpg', timestamp_s: 4};
      await page.waitForFunction(() => document.querySelector('#progressive-map-detail').textContent.includes('재시도'));
      assert.equal(await host.getAttribute('data-point-count'), '3');
      assert.ok(!(await page.locator('#progressive-frame').getAttribute('src')).includes('frame_0000004'), 'a broken image cannot change the matched map/frame');
      imageFail = false;
      await page.waitForFunction(() => document.querySelector('#progressive-frame').src.includes('frame_0000004'));
      await page.setViewportSize({width: 390, height: 900});
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      assert.equal(await page.locator('#progressive-segment').isVisible(), true);
      assert.deepEqual(errors, []);
    } finally {
      if (browser) await browser.close();
      fixture.stdin.end('\n');
      if (fixture.exitCode === null) await new Promise(resolve => fixture.once('exit', resolve));
    }
  });
