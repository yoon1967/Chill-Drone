import test from 'node:test';
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {createRequire} from 'node:module';
import {createInterface} from 'node:readline';
import {fileURLToPath} from 'node:url';

test('bounded point map, red person voxels, editable appearance and archived replay render in the real console',
  {skip: !process.env.MAPPING_TEST_PYTHON || !process.env.MAPPING_PLAYWRIGHT_MODULE, timeout: 90000}, async () => {
    const root = fileURLToPath(new URL('../../../', import.meta.url));
    const fixture = spawn(process.env.MAPPING_TEST_PYTHON, ['-B', 'tests/integration/rescue_browser_fixture.py'],
      {cwd: root, stdio: ['pipe', 'pipe', 'inherit']});
    let browser;
    try {
      const lines = createInterface({input: fixture.stdout});
      const info = await new Promise((resolve, reject) => {
        lines.once('line', line => { try { resolve(JSON.parse(line)); } catch (e) { reject(e); } });
        fixture.once('exit', code => reject(new Error(`fixture exited ${code}`)));
      });
      const {chromium} = createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE);
      browser = await chromium.launch({channel: 'msedge', headless: true});
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      const errors = []; page.on('pageerror', error => errors.push(error.message));
      await page.goto(info.url + '/#saved');
      await page.waitForFunction(() => Number(document.getElementById('viewport').dataset.rescueVoxels) > 0);
      assert.equal(await page.locator('#viewport canvas').isVisible(), true);
      assert.equal(Number(await page.locator('#viewport').getAttribute('data-rescue-triangles')), 0);
      assert.ok(Number(await page.locator('#viewport').getAttribute('data-visible-points')) > 0);
      await page.locator('#rescue-settings summary').click();
      if (process.env.MAPPING_RESCUE_SCREENSHOT) await page.screenshot({path: process.env.MAPPING_RESCUE_SCREENSHOT, fullPage: true});
      assert.equal(await page.locator('#rescue-person-color').inputValue(), '#ff3030');
      await page.locator('#rescue-sample-fps').fill('2.5');
      assert.equal(await page.locator('#import-sample-fps').inputValue(), '2.5');
      assert.equal(await page.locator('#rescue-sampling-note').textContent(), '초당 2.5프레임 목표 · 약 0.4초마다 1장');
      await page.locator('#rescue-sample-fps').fill('16');
      assert.equal(await page.locator('#rescue-sample-fps').evaluate(element => element.checkValidity()), false);
      assert.equal(await page.evaluate(async () => { try { (await import('/mapping-settings.js')).mappingOptions(); return false; } catch { return true; } }), true);
      await page.locator('#rescue-sample-fps').fill('2.5');
      await page.locator('#rescue-person-color').fill('#00ccff');
      await page.locator('#rescue-person-outline').uncheck();
      await page.locator('#rescue-point-budget').selectOption('4000');
      await page.locator('#rescue-point-budget').dispatchEvent('input');
      await page.reload();
      await page.waitForFunction(() => Number(document.getElementById('viewport').dataset.rescueVoxels) > 0);
      await page.locator('#rescue-settings summary').click();
      assert.equal(await page.locator('#rescue-person-color').inputValue(), '#00ccff');
      assert.equal(await page.locator('#rescue-sample-fps').inputValue(), '2.5');
      await page.locator('#import-mode').evaluate(element => { element.value='sfm'; element.dispatchEvent(new Event('change')); });
      await page.locator('#import-sample-fps').evaluate(element => { element.value='30'; element.dispatchEvent(new Event('input')); });
      assert.equal(await page.locator('#import-sample-fps').getAttribute('max'), '30');
      assert.equal(await page.locator('#rescue-sample-fps').inputValue(), '2.5');
      await page.locator('#import-mode').evaluate(element => { element.value='depth_rgbd'; element.dispatchEvent(new Event('change')); });
      assert.equal(await page.locator('#import-sample-fps').inputValue(), '2.5');
      assert.equal(await page.locator('#rescue-person-outline').isChecked(), false);
      assert.equal(await page.locator('#rescue-point-budget').inputValue(), '4000');
      const library = await (await page.request.get(info.url + '/api/import/library')).json();
      const id = library.initial || library.items[0]?.id;
      assert.ok(id, JSON.stringify(library));
      const history = await (await page.request.get(`${info.url}/api/import/history?id=${id}`)).json();
      assert.equal(history.count, 5);
      const replay = await (await page.request.get(`${info.url}/api/import/history-frame?id=${id}&revision=1`)).json();
      assert.ok(replay.rescue.people[0].voxels.length > 0);
      assert.equal(replay.rescue.render_mode, 'points');
      assert.equal(replay.rescue.mesh, undefined);
      assert.ok(replay.points.length <= replay.rescue.point_fusion.point_budget + 2000);
      assert.equal(replay.rescue.point_labels.length, replay.points.length);
      const archive = await page.request.get(`${info.url}/api/import/history.zip?id=${id}`);
      assert.equal(archive.status(), 200);
      const created = await (await page.request.post(info.url + '/api/import/create', {
        headers: {'X-Mapping-Client': '1'}, data: {name: 'rescue-roundtrip.zip'}})).json();
      const importedId = created.id;
      const upload = await page.request.post(`${info.url}/api/import/upload?id=${importedId}&path=rescue-roundtrip.zip`, {
        headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/octet-stream'}, data: await archive.body()});
      assert.equal(upload.status(), 200);
      await page.request.post(`${info.url}/api/import/inspect?id=${importedId}`, {headers: {'X-Mapping-Client': '1'}, data: {}});
      let inspected;
      for (let attempt = 0; attempt < 200; attempt++) {
        inspected = await (await page.request.get(`${info.url}/api/import/status?id=${importedId}`)).json();
        assert.notEqual(inspected.status, 'failed', JSON.stringify(inspected));
        if (inspected.status === 'ready') break;
        await page.waitForTimeout(50);
      }
      assert.equal(inspected.status, 'ready', JSON.stringify(inspected));
      const reopened = await (await page.request.get(`${info.url}/api/import/history-frame?id=${importedId}&revision=1`)).json();
      const reopenedStatus = await (await page.request.get(`${info.url}/api/import/status?id=${importedId}`)).json();
      assert.ok(reopened.rescue, JSON.stringify({reopened, reopenedStatus}));
      assert.deepEqual(reopened.rescue.people, replay.rescue.people);
      await page.locator('#saved-replay').click();
      await page.waitForFunction(() => Number(document.getElementById('progressive-viewport').dataset.rescueVoxels) > 0);
      assert.equal(await page.locator('#progressive-viewport canvas').isVisible(), true);
      assert.ok(Number(await page.locator('#progressive-viewport').getAttribute('data-point-count')) > 0);
      assert.ok(Number(await page.locator('#progressive-viewport').getAttribute('data-point-buffer-capacity')) <= 2**Math.ceil(Math.log2(Math.max(1024,replay.rescue.point_fusion.point_budget+2000))));
      const buffer = await page.locator('#progressive-viewport').getAttribute('data-point-attribute-id');
      await page.locator('#progressive-seek').fill('0.2');
      await page.locator('#progressive-seek-go').click();
      await page.waitForFunction(() => document.getElementById('progressive-viewport').dataset.pointRevision === '2');
      assert.equal(await page.locator('#progressive-viewport').getAttribute('data-point-attribute-id'), buffer);
      await page.setViewportSize({width: 390, height: 844});
      assert.equal(await page.locator('#rescue-sample-fps').isVisible(), true);
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
      assert.equal(await page.locator('#rescue-person-color').isVisible(), true);
      assert.deepEqual(errors, []);
    } finally {
      await browser?.close(); fixture.stdin.end('\n');
      await new Promise(resolve => { if (fixture.exitCode !== null) resolve(); else fixture.once('exit', resolve); });
    }
  });
