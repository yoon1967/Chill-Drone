import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {readFile} from 'node:fs/promises';
import {basename, dirname} from 'node:path';

const gateway = process.env.MAPPING_CONTROLLER_URL;
const video = process.env.MAPPING_EDITOR_VIDEO;

test('combined controller keeps the mapping frame and opens independent saved and supervisor edits',
  {skip: !gateway || !video, timeout: 65000}, async () => {
    const {chromium} = process.env.MAPPING_PLAYWRIGHT_MODULE ?
      createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE) : await import('playwright');
    const browser = await chromium.launch({channel: 'msedge', headless: true});
    try {
      const page = await browser.newPage({viewport: {width: 1440, height: 900}});
      const errors = [];
      page.context().on('page', tab => tab.on('pageerror', error => errors.push(error.message)));
      page.on('pageerror', error => errors.push(error.message));
      await page.goto(gateway + '/#/map');
      const frame = page.frameLocator('.console-frame');
      await frame.locator('#import-drop').waitFor();
      await frame.locator('.saved-record').first().waitFor();
      await page.locator('body.console-landing').waitFor();
      assert.equal(await frame.locator('#rescue-settings').isVisible(), false);
      assert.equal(await page.locator('.console-view > header').isVisible(), false);
      const top = await page.locator('body > .top').boundingBox();
      const drop = await frame.locator('#import-drop').boundingBox();
      const recent = await frame.locator('#saved-library-section').boundingBox();
      assert.ok(drop.y >= top.y + top.height && drop.height >= 225 && drop.y + drop.height < recent.y);
      const console = new URL(await page.locator('.console-frame').getAttribute('src')).origin;
      const initial = await (await page.request.get(console + '/api/import/video-folder')).json();
      await frame.locator('#video-folder-change').click();
      await frame.locator('#video-folder-input').fill('relative-path');
      await frame.getByRole('button', {name: '저장', exact: true}).click();
      await frame.getByText('이 PC에 이미 있는 폴더의 전체 경로를 입력하세요.').waitFor();
      await frame.locator('#video-folder-input').fill(dirname(video));
      await frame.getByRole('button', {name: '저장', exact: true}).click();
      await frame.getByText('영상 폴더 위치를 저장했습니다.').waitFor();
      await page.reload();
      await frame.locator('#video-folder-path').getByText(dirname(video), {exact: true}).waitFor();
      await frame.locator('#video-folder-change').click();
      await frame.locator('#video-folder-reset').click();
      await frame.locator('#video-folder-path').getByText(initial.folder, {exact: true}).waitFor();

      await frame.getByRole('button', {name: '실시간 지도', exact: true}).click();
      await frame.locator('#connection-monitor').waitFor();
      await page.locator('body:not(.console-landing) .controls').waitFor();
      const consoleFrame = page.frames().find(item => item.url().startsWith(console));
      await page.getByRole('link', {name: '실시간 감독', exact: true}).click();
      await page.getByRole('link', {name: '지도 콘솔', exact: true}).click();
      assert.equal(page.frames().find(item => item.url().startsWith(console)), consoleFrame);
      assert.ok(consoleFrame.url().endsWith('#live'));
      await frame.getByRole('tab', {name: '저장 지도'}).click();

      const bytes = (await readFile(video)).toString('base64');
      await frame.locator('#import-drop').evaluate((drop, {bytes, name}) => {
        const buffer = Uint8Array.from(atob(bytes), char => char.charCodeAt(0));
        const transfer = new DataTransfer();
        transfer.items.add(new File([buffer], name, {type: 'video/mp4'}));
        drop.dispatchEvent(new DragEvent('drop', {bubbles: true, cancelable: true, dataTransfer: transfer}));
      }, {bytes, name: basename(video)});
      await frame.locator('#import-start').waitFor();
      assert.match(await frame.locator('#import-info').textContent(), /\d+×\d+/);
      await frame.locator('#saved-new').click();
      await frame.locator('.saved-record').filter({hasText: 'saved_editor_fixture'}).click();
      const savedPopup = page.waitForEvent('popup');
      await frame.locator('#result-editor').click();
      const saved = await savedPopup;
      await saved.locator('#workspace').waitFor();
      assert.match(await saved.locator('#source-info').textContent(), /저장 복원/);
      await saved.close();

      const snapshot = {schema_version: 'gateway-map/0.1', kind: 'snapshot', upstream: 'ok', source_url: console,
        source: {kind: 'live', session_id: 'live_fixture', revision: 7}, scale: {unit: 'arbitrary'},
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
      await page.getByText(/버전 6 · 점/).waitFor();
      assert.equal(await page.getByRole('link', {name: '3D 지도 편집 ↗', exact: true}).count(), 0);
      assert.equal((await page.request.post(gateway + '/api/map', {data: snapshot})).status(), 201);
      const canvas = page.locator('.map3d-host canvas');
      await canvas.waitFor();
      await page.waitForFunction(() => window.supervisorView);
      const before = await page.evaluate(() => window.supervisorView);
      const {x, y, width, height} = await canvas.boundingBox();
      await page.mouse.move(x + width / 2, y + height / 2);
      await page.mouse.down({button: 'right'});
      await page.mouse.move(x + width / 2 + 40, y + height / 2 + 25, {steps: 4});
      await page.mouse.up({button: 'right'});
      await page.waitForFunction(({target}) => window.supervisorView.target.some((value, i) => value !== target[i]), before);
      const after = await page.evaluate(() => window.supervisorView);
      assert.deepEqual([after.yaw, after.pitch, after.dist], [before.yaw, before.pitch, before.dist]);
      assert.equal(await canvas.evaluate(node => node.dispatchEvent(new Event('contextmenu', {cancelable: true}))), false);
      await page.getByText(/버전 7 · 점/).waitFor();
      const livePopup = page.waitForEvent('popup');
      await page.getByRole('link', {name: '3D 지도 편집 ↗', exact: true}).click();
      const editor = await livePopup;
      await editor.locator('#workspace').waitFor();
      assert.match(await editor.locator('#source-info').textContent(), /실시간 감독.*버전 7.*3 \/ 6점/);
      await editor.getByLabel('X 최대 퍼센트').fill('50');
      assert.match(await editor.locator('#point-count').textContent(), /미리보기 2개/);
      await editor.locator('#apply').click();
      await editor.locator('#undo').click();
      assert.match(await editor.locator('#point-count').textContent(), /적용된 점 3개/);
      await editor.locator('#redo').click();
      assert.match(await editor.locator('#point-count').textContent(), /적용된 점 2개/);
      const downloadPromise = editor.waitForEvent('download');
      await editor.locator('#export').click();
      const download = await downloadPromise;
      const chunks = [];
      for await (const chunk of await download.createReadStream()) chunks.push(chunk);
      const file = Buffer.concat(chunks);
      const headerEnd = file.indexOf('end_header\n') + 'end_header\n'.length;
      assert.match(file.subarray(0, headerEnd).toString(), /element vertex 2/);
      assert.equal(file.length - headerEnd, 54);
      assert.deepEqual((await (await page.request.get(gateway + '/api/map')).json()).points, snapshot.points);
      assert.deepEqual(errors, []);
    } finally { await browser.close(); }
  });
