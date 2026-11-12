import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {spawn} from 'node:child_process';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import {join, resolve} from 'node:path';
import {once} from 'node:events';
import {createInterface} from 'node:readline';

test('supervisor saves cumulative live captures through the actual cross-origin console API',
  {skip: !process.env.MAPPING_CHECKPOINT_INTEGRATION, timeout: 45000}, async () => {
    const {chromium} = createRequire(import.meta.url)(process.env.MAPPING_PLAYWRIGHT_MODULE);
    let browser, python;
    let state;
    const staticRoot = resolve('apps/controller');
    const gateway = createServer(async (request, response) => {
      if (request.url === '/api/drone/state') {
        response.setHeader('Content-Type', 'application/json'); return response.end(JSON.stringify(state));
      }
      if (request.url.startsWith('/api/detections')) {
        response.setHeader('Content-Type', 'application/json'); return response.end('{"findings":[]}');
      }
      const name = request.url.split('?')[0].slice(1) || 'index.html';
      if (!/^[a-zA-Z0-9.-]+$/.test(name)) { response.statusCode = 404; return response.end(); }
      try {
        const data = await readFile(join(staticRoot, name));
        response.setHeader('Content-Type', name.endsWith('.js') ? 'text/javascript' : name.endsWith('.css') ? 'text/css' : 'text/html');
        response.end(data);
      } catch { response.statusCode = 404; response.end(); }
    });
    await new Promise(resolve => gateway.listen(0, '127.0.0.1', resolve));
    const origin = `http://127.0.0.1:${gateway.address().port}`;
    try {
      python = spawn(process.env.MAPPING_TEST_PYTHON || resolve('.venv/Scripts/python.exe'), ['-B', '-u', '-c', [
        'import sys,threading', 'from functools import partial', "sys.path.insert(0,'tests/integration')",
        'from test_live_checkpoint import LiveCheckpointTests',
        'from apps.console.serve import ViewerHandler,ViewerServer,Timeline,ObservationCache',
        'from modules.mapping.data.session_import import ImportManager', 'from types import SimpleNamespace',
        'case=LiveCheckpointTests();case.setUp()', 'imports=ImportManager(case.root.parent)',
        "server=ViewerServer(('127.0.0.1',0),partial(ViewerHandler,live=SimpleNamespace(current=case.session),imports=imports,tello_map=SimpleNamespace(gateway=sys.argv[1]),timeline=Timeline(),observations=ObservationCache()))",
        'threading.Thread(target=server.serve_forever,daemon=True).start()', 'print(server.server_port,flush=True)',
        'try:', ' for command in sys.stdin:',
        "  if command.strip()=='append': case.append_map(3);case.append_video(3);print('appended',flush=True)",
        "  if command.strip()=='quit': break",
        'finally:', ' server.shutdown();server.server_close();imports.shutdown();case.doCleanups()',
      ].join('\n'), origin], {stdio: ['pipe', 'pipe', 'pipe']});
      const lines = createInterface({input: python.stdout});
      const [line] = await once(lines, 'line');
      const consoleUrl = `http://127.0.0.1:${Number(line)}`;
      state = {schema_version: '0.1.0', type: 'drone_state', source: 'fake', connected: true,
        mode: 'grounded', authority: 'operator', allowed: [], warnings: [], preflight: [], battery_pct: 80,
        temp_high_c: 50, height_m: 0, attitude: {roll_rad: 0, pitch_rad: 0, yaw_rad: 0},
        video: {available: true, enabled: true, on: true}, mapping_console_url: consoleUrl,
        map: {alive: true, upstream: 'ok', source_url: consoleUrl, session_status: 'receiving', revision: 2, points: 2, unit: 'm'}};
      browser = await chromium.launch({channel: 'msedge', headless: true});
      const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
      page.setDefaultTimeout(10000);
      const errors = [], commands = [];
      page.on('pageerror', error => errors.push(error.message));
      await page.routeWebSocket('**/ws/drone', socket => {
        socket.send(JSON.stringify(state));
        socket.onMessage(message => commands.push(JSON.parse(message)));
      });
      await page.goto(origin + '/#/live');
      const button = page.getByRole('button', {name: '지도 저장', exact: true});
      await button.waitFor();
      await page.waitForFunction(() => !document.querySelector('[data-live-map-save]').disabled);
      const box = await button.boundingBox(), panel = await button.locator('..').locator('..').boundingBox();
      assert.ok(box.y + box.height <= panel.y + panel.height, 'save button is inside the bottom of the map card');
      await button.click();
      await page.locator('.live-map-save').getByText('지도·영상·센서 자료 저장 완료', {exact: false}).waitFor();
      let saved = (await (await page.request.get(consoleUrl + '/api/live/checkpoint-status')).json()).latest;
      const first = saved.path;
      assert.equal((await readFile(join(first, 'sensor.jsonl'), 'utf8')).trim().split('\n').length, 2);
      const nextLine = once(lines, 'line'); python.stdin.write('append\n'); await nextLine;
      await page.getByRole('button', {name: '지도 저장', exact: true}).click();
      await page.waitForFunction(previous => !document.querySelector('.live-map-save').textContent.includes(previous), first);
      await page.locator('.live-map-save').getByText('지도·영상·센서 자료 저장 완료', {exact: false}).waitFor();
      saved = (await (await page.request.get(consoleUrl + '/api/live/checkpoint-status')).json()).latest;
      assert.notEqual(saved.path, first);
      assert.equal((await readFile(join(saved.path, 'sensor.jsonl'), 'utf8')).trim().split('\n').length, 3);
      assert.equal((await readFile(join(first, 'sensor.jsonl'), 'utf8')).trim().split('\n').length, 2);
      const captureId = saved.id;
      const library = await (await page.request.get(consoleUrl + '/api/import/capture-library')).json();
      assert.equal(library.items.find(item => item.session_id === captureId).selectable, true);
      const bundle = await (await page.request.get(consoleUrl + '/api/import/capture-bundle?session_id=' + captureId)).json();
      assert.equal(bundle.files.some(file => file.path === 'video_sensor_sync.json'), true);
      assert.equal(bundle.files.some(file => file.path.includes('frames')), false);
      const source = (await (await page.request.get(consoleUrl + '/api/live/checkpoint-status')).json());
      assert.equal(source.available, true);
      assert.equal(commands.some(command => command.type === 'cmd'), false, 'saving must not stop video, land or send a flight command');
      await page.getByRole('button', {name: '2D 경로'}).click();
      assert.equal(await page.getByRole('button', {name: '지도 저장', exact: true}).isVisible(), true);
      await page.setViewportSize({width: 390, height: 844});
      assert.equal(await page.getByRole('button', {name: '지도 저장', exact: true}).isVisible(), true);
      assert.deepEqual(errors, []);
      if (process.env.MAPPING_CHECKPOINT_SCREENSHOT) {
        await page.setViewportSize({width: 1440, height: 1000});
        await page.screenshot({path: process.env.MAPPING_CHECKPOINT_SCREENSHOT, fullPage: true});
      }
      const originalScene = join(saved.path, 'reconstruction/scene.json');
      const beforeEdit = await readFile(originalScene);
      await page.getByRole('link', {name: '지도 콘솔', exact: true}).click();
      const frame = page.frameLocator('.console-frame');
      await frame.getByRole('tab', {name: '지도 편집'}).click();
      const checkpoint = frame.getByRole('button', {name: new RegExp(captureId)});
      await checkpoint.click();
      await frame.locator('#editor-viewport canvas').waitFor().catch(async error => {
        throw new Error(await frame.locator('#edit-load-status').textContent(), {cause: error});
      });
      await frame.getByRole('button', {name: '전체 선택'}).click();
      await frame.getByRole('button', {name: '선택에 적용'}).click();
      assert.deepEqual(await readFile(originalScene), beforeEdit, 'editor leaves the saved map on disk intact');
      const alias = await (await page.request.get(consoleUrl + '/api/import/editable-maps')).json();
      assert.equal(alias.items.find(item => item.key === captureId).asset?.startsWith('import_'), true);
      await frame.getByRole('button', {name: '새로고침', exact: true}).click();
      assert.equal(await checkpoint.getAttribute('aria-pressed'), 'true');
      assert.deepEqual(errors, []);
    } finally {
      await browser?.close();
      if (python && python.exitCode === null) { python.stdin.write('quit\n'); await once(python, 'exit'); }
      await new Promise(resolve => gateway.close(resolve));
    }
  });
