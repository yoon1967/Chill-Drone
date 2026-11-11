import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {spawn} from 'node:child_process';
import {mkdtemp, mkdir, writeFile, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join, resolve, dirname} from 'node:path';
import {once} from 'node:events';
import {createInterface} from 'node:readline';

test('capture library selects actual video and sensor assets separately from saved reconstructions',
  {skip: !process.env.MAPPING_PLAYWRIGHT_MODULE, timeout: 45000}, async () => {
    const {chromium} = createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE);
    const root = await mkdtemp(join(tmpdir(), 'mapping-capture-test-'));
    const write = (path, data) => writeFile(path, JSON.stringify(data));
    async function capture(name, {sync = 'complete', active = false, missing = false} = {}) {
      const folder = join(root, name);
      await mkdir(join(folder, 'recordings'), {recursive: true});
      await mkdir(join(folder, 'frames'));
      await write(join(folder, 'session.json'), {schema_version: 'mapping-session/1.0', input: 'tello_camera',
        session_id: name, started_at: '2026-11-11T08:00:00+00:00', recording: {finalized: !active},
        recordings: [0, 1].map(id => ({id, index_path: `recordings/part_${id}.jsonl`,
          playback: {path: `recordings/part_${id}.mkv`, duration_s: 10}}))});
      for (const id of [0, 1]) {
        await write(join(folder, `recordings/part_${id}.jsonl`), {timestamp_s: id});
        await writeFile(join(folder, `recordings/part_${id}.mkv`), 'fixture');
      }
      await write(join(folder, 'sensor.jsonl'), {event: 'sample'});
      if (!missing) await write(join(folder, 'video_sensor_sync.json'), {schema_version: 'video-sensor-sync/1.0', status: sync});
      await write(join(folder, 'camera.json'), {width: 960});
      await write(join(folder, 'settings.json'), {});
      await write(join(folder, 'frames.jsonl'), {private: true});
    }
    let browser, server;
    try {
      await capture('live_complete');
      await capture('live_partial', {sync: 'partial'});
      await capture('live_active', {active: true});
      await capture('live_missing', {missing: true});
      const python = process.env.MAPPING_TEST_PYTHON || resolve('.venv/Scripts/python.exe');
      server = spawn(python, ['-B', '-u', '-c', [
        'from functools import partial', 'import sys',
        'from apps.console.serve import ViewerServer,ViewerHandler,Timeline,ObservationCache',
        'from modules.mapping.data.session_import import ImportManager',
        'manager=ImportManager(sys.argv[1])',
        "server=ViewerServer(('127.0.0.1',0),partial(ViewerHandler,imports=manager,timeline=Timeline(),observations=ObservationCache()))",
        'print(server.server_port,flush=True)', 'server.serve_forever()',
      ].join('\n'), root], {stdio: ['ignore', 'pipe', 'pipe']});
      const lines = createInterface({input: server.stdout});
      const [line] = await Promise.race([once(lines, 'line'), once(server, 'exit').then(() => { throw Error('test server stopped'); })]);
      const url = `http://127.0.0.1:${Number(line)}`;
      browser = await chromium.launch({channel: 'msedge', headless: true});
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      page.setDefaultTimeout(8000);
      const errors = [];
      page.on('pageerror', error => errors.push(error.message));
      let starts = 0;
      page.on('request', request => { if (/\/api\/import\/(start|create|local|inspect|drone-recording)(\?|$)/.test(request.url())) starts++; });
      await page.goto(url + '/reconstruct.html');
      await page.locator('#captures-list button').first().waitFor();
      assert.equal(await page.locator('#recent-list button').count(), 0);
      assert.equal(await page.locator('#recent-empty').textContent(), '복원 기록이 없습니다.');
      assert.equal(await page.locator('#captures-list button').count(), 4);
      let folderOpened = 0;
      await page.route('**/api/import/capture-folder', route => {
        folderOpened++;
        assert.deepEqual(route.request().postDataJSON(), {});
        return route.fulfill({json: {folder: root, opened: true}});
      });
      await page.getByRole('button', {name: '촬영 저장 위치 ↗'}).click();
      assert.equal(folderOpened, 1);
      assert.equal(await page.locator('#captures-root').textContent(), root);
      const left = await page.locator('#recent').boundingBox(), right = await page.locator('#captures').boundingBox();
      assert.ok(Math.abs(left.width - right.width) < 1 && right.x > left.x + left.width);
      assert.equal(await page.getByRole('button', {name: /^live_active/}).isDisabled(), true);
      assert.equal(await page.getByRole('button', {name: /^live_missing/}).isDisabled(), true);
      await page.evaluate(() => {
        document.addEventListener('mapping-capture-selected', event => { window.captureSelection = event.detail; });
      });
      await page.getByRole('button', {name: /^live_complete/}).click();
      await page.locator('#capture-selection').waitFor();
      const bundle = await page.evaluate(() => window.captureSelection);
      assert.equal(bundle.session_id, 'live_complete');
      assert.equal(bundle.parts.length, 2);
      assert.equal(bundle.files.filter(file => file.role === 'video').length, 2);
      assert.equal(bundle.files.some(file => file.path.includes('frames')), false);
      for (const file of bundle.files) assert.equal((await page.request.get(url + file.url)).status(), 200);
      assert.equal(await page.locator('#capture-warning').isVisible(), false);
      await page.locator('#capture-files').evaluate(node => { node.parentElement.open = true; });
      assert.ok((await page.locator('#capture-files').textContent()).includes('video_sensor_sync.json'));
      assert.equal(starts, 0, 'selection must not open or start a reconstruction');
      await page.getByRole('button', {name: /^live_partial/}).click();
      await page.locator('#capture-warning').waitFor();
      assert.ok((await page.locator('#capture-warning').textContent()).includes('불완전'));
      await page.locator('#capture-clear').click();
      assert.equal(await page.locator('#drop').isVisible(), true);
      assert.equal(await page.evaluate(() => window.captureSelection), null);
      await capture('live_automatic');
      await page.getByRole('button', {name: /^live_automatic/}).waitFor({timeout: 8000});
      await page.setViewportSize({width: 390, height: 844});
      const mobileLeft = await page.locator('#recent').boundingBox(), mobileRight = await page.locator('#captures').boundingBox();
      assert.ok(mobileRight.y >= mobileLeft.y + mobileLeft.height);
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
      // The ordinary single-video path must still reach its existing model picker.
      const job = {id: 'import_1234567890123456', name: 'ordinary.mp4', status: 'awaiting_options', summary: {kind: 'video'}};
      await page.route('**/api/import/create', route => route.fulfill({json: job}));
      await page.route('**/api/import/upload?*', route => route.fulfill({json: {status: 'uploaded'}}));
      await page.route('**/api/import/inspect?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/status?*', route => route.fulfill({json: job}));
      await page.route('**/api/import/cancel?*', route => route.fulfill({json: {status: 'cancelled'}}));
      await page.locator('#video').setInputFiles({name: job.name, mimeType: 'video/mp4', buffer: Buffer.from('fixture')});
      await page.locator('main[data-state="awaiting_options"]').waitFor();
      assert.equal(await page.locator('#model-picker').isVisible(), true);
      assert.equal(await page.locator('#capture-selection').isVisible(), false);
      await page.locator('#change').click();
      await page.locator('#drop').waitFor();
      assert.deepEqual(errors, []);
      if (process.env.MAPPING_CAPTURE_SCREENSHOT) {
        await page.setViewportSize({width: 1440, height: 1000});
        await page.screenshot({path: process.env.MAPPING_CAPTURE_SCREENSHOT, fullPage: true});
      }
    } finally {
      await browser?.close();
      if (server && server.exitCode === null) { server.kill(); await once(server, 'exit'); }
      assert.equal(dirname(resolve(root)), resolve(tmpdir()));
      assert.ok(root.includes('mapping-capture-test-'));
      await rm(root, {recursive: true, force: true});
    }
  });
