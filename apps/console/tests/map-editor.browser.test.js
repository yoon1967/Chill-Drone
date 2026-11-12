import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';

const gateway = process.env.MAPPING_CONTROLLER_URL;

test('controller restores a saved point cloud with Roomform inside the map console and retains the supervisor view',
  {skip: !gateway, timeout: 45000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({channel: 'msedge', headless: true});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 900}});
      page.setDefaultTimeout(8000);
      const errors = [];
      page.context().on('page', tab => tab.on('pageerror', error => errors.push(error.message)));
      page.on('pageerror', error => errors.push(error.message));
      const job = {id: 'import_1234567890123456', name: '복도.mp4', summary: {kind: 'video', duration_s: 4}, status: 'uploading'};
      const saved = {id: 'import_6543210987654321', name: '이전 지도', status: 'complete'};
      const checkpoint = {id: 'import_4444444444444444', name: '저장 회차', status: 'ready'};
      const points = [[0, 0, 2, 255, 0, 0], [1, 0, 2, 0, 255, 0], [0, 1, 2, 0, 0, 255]];
      const acknowledgements = [];
      const pose = 'orb-slam3', depth = 'metric-video-depth-anything-small';
      let installed = false, starts = 0, finishDownload;
      const downloadDone = new Promise(resolve => { finishDownload = resolve; });
      let roomformInstalled = false, roomformDownloads = 0, finishRoomformDownload;
      const roomformDownloadDone = new Promise(resolve => { finishRoomformDownload = resolve; });
      await page.route('**/api/reconstruction-models', route => route.fulfill({json: {models: [
        {id: pose, kind: 'pose', name: 'ORB-SLAM3', size_bytes: 145250924, vram_gb: 0},
        {id: depth, kind: 'depth', name: 'Metric Video Depth Anything Small', size_bytes: 116444063, vram_gb: 2},
      ]}}));
      await page.route('**/api/reconstruction-models/status?*', route => {
        const key = new URL(route.request().url()).searchParams.get('id');
        if (key === 'roomform') return route.fulfill({json: {status: roomformInstalled ? 'ready' : 'missing'}});
        return route.fulfill({json: {status: key === pose || installed ? 'ready' : 'missing'}});
      });
      await page.route('**/api/reconstruction-models/download', route => {
        const {id} = route.request().postDataJSON();
        if (id === 'roomform') {
          roomformDownloads++;
          return route.fulfill({json: {status: 'downloading', method: 'transfer', percent: 27}});
        }
        assert.equal(id, depth);
        return route.fulfill({json: {status: 'downloading', method: 'transfer', percent: 41}});
      });
      await page.route('**/api/reconstruction-models/events?*', async route => {
        if (new URL(route.request().url()).searchParams.get('id') === 'roomform') {
          if (roomformDownloads === 1) return route.fulfill({contentType: 'text/event-stream',
            body: 'data: {"status":"failed","error":"model_download_failed"}\n\n'});
          await roomformDownloadDone;
          roomformInstalled = true;
          return route.fulfill({contentType: 'text/event-stream', body: 'data: {"status":"ready","percent":100}\n\n'});
        }
        await downloadDone;
        installed = true;
        return route.fulfill({contentType: 'text/event-stream', body: 'data: {"status":"ready","percent":100}\n\n'});
      });
      await page.route('**/api/import/reconstruction-library', route => route.fulfill({json: {items: [saved]}}));
      await page.route('**/api/import/editable-maps', route => route.fulfill({json: {items: [
        {key: saved.id, name: saved.name, asset: saved.id},
        {key: 'saved_checkpoint', name: checkpoint.name, local_id: 'saved_checkpoint'},
      ]}}));
      const packed = Buffer.alloc(48);
      [0, 0, -2, 1, 0, -2, 1, 1, -2, 0, 1, -2].forEach((value, i) => packed.writeFloatLE(value, i * 4));
      const quad = {quads: 1, positions: packed.toString('base64')};
      const shell = {schema_version: 'roomform-surface/1.0', origin: [0, 0, 0], up: [0, 1, 0],
        scale_estimated: true, surfaces: {wall: quad, floor: quad, ceiling: quad}};
      const roomformStarts = [], roomformCancels = [];
      let roomformDone = false, releaseFirstStart;
      const firstStartResponse = new Promise(resolve => { releaseFirstStart = resolve; });
      await page.route('**/api/import/thumbnail?*', route => route.fulfill({contentType: 'image/png',
        body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')}));
      await page.route('**/api/saved/roomform?*', async route => {
        const request = route.request(), query = new URL(request.url()).searchParams;
        const id = query.get('id');
        if (request.method() === 'POST') {
          assert.equal(request.headers()['x-mapping-client'], '1');
          assert.match(id, /^[0-9a-f]{32}$/);
          if (query.get('action') === 'cancel') {
            roomformCancels.push(id);
            await route.fulfill({json: {id, status: 'cancelled'}});
            releaseFirstStart();
            return;
          }
          assert.equal(query.get('asset'), saved.id);
          roomformStarts.push(id);
          if (roomformStarts.length === 1) {
            await firstStartResponse;
            await route.abort().catch(() => {});
            return;
          }
          return route.fulfill({status: 202, json: {id, status: 'running', tiles_done: 1, tiles_total: 2}});
        }
        assert.equal(id, roomformStarts.at(-1));
        return route.fulfill({json: roomformDone ? {id, status: 'complete', result: shell} :
          {id, status: 'running', tiles_done: 1, tiles_total: 2}});
      });
      await page.route('**/api/import/local', route => route.fulfill({json:
        route.request().postDataJSON().local_id === 'saved_checkpoint' ? checkpoint : saved}));
      await page.route('**/api/import/create', route => {
        job.status = 'uploading';
        job.progress = {};
        return route.fulfill({json: job});
      });
      await page.route('**/api/import/upload?*', route => {
        assert.equal(new URL(route.request().url()).searchParams.get('path'), job.name);
        return route.fulfill({json: {status: 'uploaded'}});
      });
      await page.route('**/api/import/inspect?*', route => {
        assert.deepEqual(route.request().postDataJSON(), {});
        job.status = 'awaiting_options';
        return route.fulfill({json: job});
      });
      await page.route('**/api/import/start?*', route => {
        const options = route.request().postDataJSON();
        assert.deepEqual([options.pose_model, options.depth_model, options.causal_video], [pose, depth, true]);
        assert.equal(options.sample_fps, undefined);
        starts++;
        job.status = 'running'; job.stage = 'model';
        return route.fulfill({json: job});
      });
      await page.route('**/api/import/status?*', route => route.fulfill({json:
        new URL(route.request().url()).searchParams.get('id') === saved.id ? saved : job}));
      await page.route('**/api/import/preview?*', route => route.fulfill({json:
        Number(new URL(route.request().url()).searchParams.get('revision')) === 1 ? {unchanged: true} : {
        revision: 1, frame_id: 'frame_0000001.jpg', frame_number: 1, timestamp_s: 1,
        tracking: 'pose_held', map_id: 0, points, cameras: [[0, 0, 0]], needs_presentation: false,
      }}));
      await page.route('**/api/import/preview-frame?*', route => route.fulfill({contentType: 'image/png',
        body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWQ0AAAAASUVORK5CYII=', 'base64')}));
      await page.route('**/api/import/presented?*', route => {
        acknowledgements.push(route.request().url());
        return route.fulfill({json: {revision: 1}});
      });
      await page.context().route('**/api/saved/run?*', route => route.fulfill({json: {
        schema_version: 'phone-sfm-result/1.2', scale: {unit: 'arbitrary'},
        quality: {result_status: 'partial', coverage: {map_count: 2, held_pose_frames: 1}},
        points: [...points, [1, 1, 2, 255, 255, 255]].map(([x, y, z, ...rgb], index) => [index, x, -y, -z, ...rgb]),
      }}));
      await page.route('**/api/import/cancel?*', route => {
        job.status = 'cancelled';
        return route.fulfill({json: job});
      });
      await page.goto(gateway + '/#/map');
      const frame = page.frameLocator('.console-frame');
      await frame.getByRole('button', {name: '영상 선택'}).waitFor();
      await frame.locator('main').evaluate(async () => {
        const {OrbitControls} = await import('/vendor/OrbitControls.js');
        const update = OrbitControls.prototype.update;
        OrbitControls.prototype.update = function (...args) {
          const updated = update.apply(this, args);
          const camera = this.object;
          if (this.domElement.closest('#editor-viewport')) return updated;
          window.mappingView = {orthographic: camera.isOrthographicCamera, up: camera.up.toArray(),
            z: camera.position.z, targetZ: this.target.z};
          return updated;
        };
      });
      assert.equal(await frame.locator('#import-panel, #workflow-guide, #rescue-settings').count(), 0);
      assert.equal(await page.locator('#app .strip, #app .controls, .console-view, .console-tools').count(), 0);
      const top = await page.locator('.top').boundingBox();
      const surface = await page.locator('.console-frame').boundingBox();
      assert.equal(surface.width, (await page.locator('#app').boundingBox()).width);
      assert.ok(Math.abs(surface.y - top.height) <= 1 && surface.y + surface.height >= 899);
      const url = new URL(await page.locator('.console-frame').getAttribute('src'));
      const console = url.origin;
      assert.equal(url.pathname, '/reconstruct.html');
      for (const legacy of ['/index.html', '/editor.html?source=supervisor', '/api/editor/supervisor'])
        assert.equal((await page.request.get(console + legacy)).status(), 404);
      assert.doesNotMatch(await (await page.request.get(console + '/')).text(), /공간 복원실|editor\.html/);
      const consoleFrame = page.frames().find(item => item.url().startsWith(console));
      const restoreTab = frame.getByRole('tab', {name: '지도 복원'});
      const editTab = frame.getByRole('tab', {name: '지도 편집'});
      const tabs = frame.getByRole('tablist');
      await page.setViewportSize({width: 460, height: 420});
      await page.waitForFunction(() => document.documentElement.scrollHeight > innerHeight + 80);
      const startTabs = await tabs.boundingBox();
      await page.mouse.move(40, startTabs.y + startTabs.height + 40);
      await page.mouse.wheel(0, 80);
      await page.waitForFunction(() => scrollY === 80);
      assert.equal((await page.locator('.top').boundingBox()).y, top.y - 80);
      assert.equal((await tabs.boundingBox()).y, startTabs.y - 80);
      assert.equal(await consoleFrame.evaluate(() => scrollY), 0);
      await page.evaluate(() => scrollTo(0, 0));
      await page.setViewportSize({width: 1440, height: 900});
      const tabPosition = await tabs.boundingBox();
      assert.equal(await restoreTab.getAttribute('aria-selected'), 'true');
      await editTab.click();
      await consoleFrame.waitForFunction(() => document.getElementById('restore-panel').getBoundingClientRect().right <= 1);
      assert.deepEqual(await tabs.boundingBox(), tabPosition);
      assert.equal(await editTab.getAttribute('aria-selected'), 'true');
      assert.equal(await frame.getByRole('button', {name: '영상 선택'}).count(), 0);
      await frame.getByRole('button', {name: saved.name}).locator('img').click();
      const editor = frame.locator('#editor-viewport');
      await editor.and(frame.locator('[data-view="original"]')).waitFor();
      assert.equal(page.context().pages().length, 1);
      assert.equal(await frame.locator('#editor-name').textContent(), saved.name);
      assert.match(await frame.locator('#editor-unit').textContent(), /원본 점 4개/);
      assert.equal(await frame.locator('#editor-box, #editor-delete, #editor-paint, #editor-export').count(), 0);
      const ai = frame.getByRole('button', {name: 'AI 복원하기'});
      const downloadAI = frame.getByRole('button', {name: 'AI 복원모델 다운받기'});
      await downloadAI.click();
      await frame.locator('#editor-status').filter({hasText: '다운로드에 실패했습니다'}).waitFor();
      await downloadAI.click();
      await frame.locator('#editor-progress[value="27"]').waitFor();
      assert.equal(roomformStarts.length, 0);
      finishRoomformDownload();
      const firstStart = page.waitForRequest(request => request.method() === 'POST' &&
        request.url().includes('/api/saved/roomform?') && !request.url().includes('action=cancel'));
      await ai.click();
      await firstStart;
      async function switchAwayDuringRestore() {
        const cancelled = page.waitForResponse(response => response.url().includes('/api/saved/roomform?action=cancel&') && response.ok());
        await frame.getByRole('button', {name: checkpoint.name}).click();
        await cancelled;
        await frame.locator('#editor-name').filter({hasText: checkpoint.name}).waitFor();
        await editor.and(frame.locator('[data-view="original"]')).waitFor();
        await frame.getByRole('button', {name: saved.name}).click();
        await frame.locator('#editor-name').filter({hasText: saved.name}).waitFor();
        assert.equal(await frame.getByRole('button', {name: 'AI 복원 결과'}).count(), 0);
      }
      await switchAwayDuringRestore();
      await ai.click();
      await frame.locator('#editor-progress[value="1"]').waitFor();
      await switchAwayDuringRestore();
      assert.deepEqual(roomformCancels, roomformStarts);
      await ai.click();
      await frame.locator('#editor-progress[value="1"]').waitFor();
      roomformDone = true;
      await editor.and(frame.locator('[data-view="roomform"]')).waitFor();
      await frame.getByLabel('천장 표시').check();
      await frame.getByRole('button', {name: '원본 점군'}).click();
      await editor.and(frame.locator('[data-view="original"]')).waitFor();
      await frame.getByRole('button', {name: 'AI 복원 결과'}).click();
      await editor.and(frame.locator('[data-view="roomform"]')).waitFor();
      assert.equal(new Set(roomformStarts).size, 3);
      assert.equal(await ai.isVisible(), true);
      await frame.getByRole('button', {name: checkpoint.name}).click();
      await frame.locator('#editor-name').filter({hasText: checkpoint.name}).waitFor();
      await editor.and(frame.locator('[data-view="original"]')).waitFor();
      await ai.waitFor();
      assert.equal(roomformDownloads, 2);
      assert.equal(roomformCancels.length, 2);
      assert.equal(await frame.getByRole('button', {name: 'AI 복원 결과'}).count(), 0);
      await frame.getByRole('button', {name: saved.name}).click();
      await frame.locator('#editor-name').filter({hasText: saved.name}).waitFor();
      assert.match(await frame.locator('#editor-unit').textContent(), /원본 점 4개/);
      await editTab.press('ArrowLeft');
      await consoleFrame.waitForFunction(() =>
        document.activeElement.id === 'restore-tab' &&
        document.getElementById('edit-panel').getBoundingClientRect().left >= document.documentElement.clientWidth - 1);
      assert.deepEqual(await tabs.boundingBox(), tabPosition);
      const dragPane = async (direction, panel) => {
        const {x, y, width} = await frame.locator('#workspace-panels').boundingBox();
        const fromX = x + width * (direction < 0 ? .75 : .25);
        await page.mouse.move(fromX, y + 10);
        await page.mouse.down();
        await page.mouse.move(fromX + direction * width / 4, y + 10, {steps: 6});
        const offset = await frame.locator(panel).evaluate(node => node.getBoundingClientRect().x);
        assert.ok(offset * direction > width / 5, 'the panel follows the pointer');
        assert.deepEqual(await tabs.boundingBox(), tabPosition);
        await page.mouse.up();
      };
      await dragPane(-1, '#restore-panel');
      await frame.getByRole('tabpanel', {name: '지도 편집'}).waitFor();
      await consoleFrame.waitForFunction(() => Math.abs(document.getElementById('edit-panel').getBoundingClientRect().left) < 1);
      await dragPane(1, '#edit-panel');
      await frame.getByRole('tabpanel', {name: '지도 복원'}).waitFor();
      await frame.getByRole('button', {name: /이전 지도/}).click();
      await frame.locator('main[data-state="partial"]').waitFor();
      assert.equal(await frame.locator('#file-name').textContent(), saved.name);
      await frame.getByRole('button', {name: '지도 편집에서 열기'}).click();
      await frame.getByRole('tabpanel', {name: '지도 편집'}).waitFor();
      assert.match(await frame.locator('#editor-unit').textContent(), /원본 점 4개/);
      await restoreTab.click();
      await frame.locator('#change').click();
      const video = {name: job.name, mimeType: 'video/mp4', buffer: Buffer.from('fixture')};
      await frame.locator('#drop').evaluate((drop, name) => {
        const transfer = new DataTransfer();
        transfer.items.add(new File(['fixture'], name, {type: 'video/mp4'}));
        drop.dispatchEvent(new DragEvent('drop', {bubbles: true, cancelable: true, dataTransfer: transfer}));
      }, video.name);
      const restore = frame.getByRole('button', {name: '영상 복원하기'});
      const selectModels = async () => {
        await frame.getByLabel('카메라 위치 모델', {exact: true}).selectOption(pose);
        await frame.getByLabel('영상 깊이 모델', {exact: true}).selectOption(depth);
      };
      await restore.waitFor();
      assert.equal(await restore.isDisabled(), true);
      await selectModels();
      assert.match(await frame.locator('#depth-model option:checked').textContent(), /116\.4 MB.*2 GB/);
      assert.equal(await restore.isDisabled(), true);
      assert.equal(starts, 0);
      await frame.getByRole('button', {name: '모델 다운받기'}).click();
      await frame.locator('#depth-model-progress[value="41"]').waitFor();
      assert.equal(await restore.isDisabled(), true);
      finishDownload();
      await frame.locator('#depth-model-status').filter({hasText: '모델 준비 완료'}).waitFor();
      await restore.click();
      await frame.getByText('깊이 모델 준비 중', {exact: true}).waitFor();
      assert.equal(starts, 1);
      assert.equal(await frame.getByRole('button', {name: '3D 지도 만들기'}).count(), 0);
      assert.equal(await frame.locator('#file-name').textContent(), job.name);
      await editTab.click();
      job.stage = 'initial_pose';
      await consoleFrame.waitForFunction(() => document.getElementById('status').textContent === '카메라의 초기 위치와 방향 찾는 중');
      await restoreTab.click();
      await frame.getByText('카메라의 초기 위치와 방향 찾는 중', {exact: true}).waitFor();
      job.stage = 'mapping'; job.progress = {completed_timestamp_s: 1, alignment: 'pose_held'};
      await frame.getByText('마지막 위치를 기준으로 지도에 합치는 중', {exact: true}).waitFor();
      await frame.locator('#progressive-tracking-note').filter({hasText: '마지막으로 확인한 위치에 합쳤습니다.'}).waitFor();
      await frame.locator('progress[value="22"]').waitFor();
      await page.getByRole('link', {name: '실시간 감독', exact: true}).click();
      job.stage = 'integration'; job.progress = {integrated_frames: 1, integration_total: 2};
      await page.getByRole('link', {name: '지도 콘솔', exact: true}).click();
      assert.equal(page.frames().find(item => item.url().startsWith(console)), consoleFrame);
      await frame.locator('progress[value="94"]').waitFor();
      await frame.locator('#progressive-viewport[data-point-count="3"] canvas').waitFor();
      await frame.locator('#progressive-frame').waitFor();
      const previewView = await frame.locator('main').evaluate(() => window.mappingView);
      assert.ok(previewView.orthographic && previewView.z > previewView.targetZ);
      assert.deepEqual(previewView.up, [0, 1, 0]);
      assert.deepEqual(acknowledgements, []);
      job.status = 'complete'; job.history_available = true;
      await frame.locator('main[data-state="partial"] #progressive-viewport[data-point-count="4"]').waitFor();
      const resultView = await frame.locator('main').evaluate(() => window.mappingView);
      assert.ok(resultView.orthographic && resultView.z > resultView.targetZ);
      assert.deepEqual(resultView.up, previewView.up);
      assert.equal(resultView.targetZ, previewView.targetZ);
      assert.match(await frame.locator('#notice').textContent(), /2개 구간.*실제 거리 측정/);
      assert.match(await frame.locator('#notice').textContent(), /1개 장면은 마지막으로 확인한 위치에 합쳤습니다/);
      assert.equal(await frame.locator('#current-frame').isVisible(), false);
      assert.match(await frame.locator('#download').getAttribute('href'), /history\.zip/);
      await frame.locator('#change').click();
      await frame.locator('#video').setInputFiles(video);
      await selectModels();
      await restore.click();
      await frame.locator('main[data-state="running"]').waitFor();
      await frame.locator('#cancel').click();
      await frame.locator('main[data-state="cancelled"]').waitFor();
      await frame.locator('#change').click();
      await frame.locator('#video').setInputFiles(video);
      await selectModels();
      await restore.click();
      await frame.locator('main[data-state="running"]').waitFor();
      job.status = 'failed'; job.error = 'no_valid_depth_map';
      await frame.locator('main[data-state="failed"]').waitFor();
      assert.match(await frame.locator('#notice').textContent(), /촬영한 영상/);

      const snapshot = {schema_version: 'gateway-map/0.1', kind: 'snapshot', upstream: 'ok', source_url: console,
        source: {kind: 'live', session_id: `live_fixture_${Date.now()}`, revision: 7}, scale: {unit: 'arbitrary'},
        points: [[0, 0, 0, 255, 0, 0], [1, 1, 0, 0, 255, 0], [2, 1, 1, 0, 0, 255]], points_total: 6,
        cameras: [[0, 0, 0], [0, 1, 0]], up: [0, 1, 0]};
      await page.evaluate(() => {
        const draw = window.PointView.prototype._draw;
        window.PointView.prototype._draw = function () {
          draw.call(this);
          if (this.n) window.supervisorView = structuredClone(this.view);
        };
      });
      await page.getByRole('link', {name: '실시간 감독', exact: true}).click();
      const demo = {...snapshot, source: {...snapshot.source, revision: 6, pose_source: 'fake'}};
      assert.equal((await page.request.post(gateway + '/api/map', {data: demo})).status(), 201);
      await page.getByText(/점 3개.*전체 6개/).waitFor();
      assert.equal(await page.getByRole('link', {name: '3D 지도 편집 ↗', exact: true}).count(), 0);
      assert.equal((await page.request.post(gateway + '/api/map', {data: snapshot})).status(), 201);
      const supervisorCanvas = page.locator('.map3d-host canvas');
      await supervisorCanvas.waitFor();
      await page.waitForFunction(() => window.supervisorView);
      const before = await page.evaluate(() => window.supervisorView);
      const {x, y, width, height} = await supervisorCanvas.boundingBox();
      await page.mouse.move(x + width / 2, y + height / 2);
      await page.mouse.down({button: 'right'});
      await page.mouse.move(x + width / 2 + 40, y + height / 2 + 25, {steps: 4});
      await page.mouse.up({button: 'right'});
      await page.waitForFunction(({target}) => window.supervisorView.target.some((value, i) => value !== target[i]), before);
      const after = await page.evaluate(() => window.supervisorView);
      assert.deepEqual([after.yaw, after.pitch, after.dist], [before.yaw, before.pitch, before.dist]);
      assert.equal(await supervisorCanvas.evaluate(node => node.dispatchEvent(new Event('contextmenu', {cancelable: true}))), false);
      assert.equal(await page.locator('a[href*="editor.html"], a[href$="/#live"]').count(), 0);
      const latest = await (await page.request.get(gateway + '/api/map')).json();
      assert.equal(latest.source.revision, 7);
      assert.deepEqual(latest.points, snapshot.points);
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
