import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const code = readFileSync(new URL('../../controller/live-save.js', import.meta.url), 'utf8');
const tick = async () => { for (let i = 0; i < 10; i++) await Promise.resolve(); };
function client(fetcher) {
  const context = {window: {}, URL, AbortSignal, setTimeout: () => 1, clearTimeout: () => {}};
  vm.runInNewContext(code, context);
  return context.window.createLiveMapSave({fetcher});
}
const response = (value, ok = true) => ({ok, json: async () => value});

test('save client sends only checkpoint requests and supports cumulative repeated saves', async () => {
  let latest = null, requests = 0;
  const calls = [];
  const save = client(async (url, options) => {
    calls.push(url);
    if (options.method === 'POST') {
      requests++;
      assert.deepEqual(JSON.parse(options.body), {session_id: 'live_one'});
      assert.equal(options.headers['X-Mapping-Client'], '1');
      latest = {id: 'saved_' + requests, status: 'saving'};
      return response(latest);
    }
    return response({available: true, session_id: 'live_one', latest});
  });
  save.setTarget('http://127.0.0.1:8790'); await tick();
  assert.equal(save.state().available, true);
  await save.save(); await tick();
  assert.equal(save.state().busy, true);
  await save.save();
  assert.equal(requests, 1);
  latest = {...latest, status: 'complete', path: 'C:/captures/one'};
  await save.refresh();
  assert.equal(save.state().path, latest.path);
  await save.save(); await tick();
  assert.equal(requests, 2);
  assert.ok(calls.every(url => /\/api\/live\/checkpoint(?:-status)?$/.test(url)));
  save.dispose();
});

test('save errors stay visible until another attempt or a new server result', async () => {
  let fail = true;
  const save = client(async (url, options) => options.method === 'POST' ?
    response(fail ? {error: 'checkpoint_map_not_ready'} : {id: 'saved_ok', status: 'saving'}, !fail) :
    response({available: true, session_id: 'live_one', latest: null}));
  save.setTarget('http://127.0.0.1:8790'); await tick();
  await save.save(); await tick();
  assert.ok(save.state().message.includes('지도가 준비된 뒤'));
  assert.equal(save.state().busy, false);
  fail = false;
  await save.save(); await tick();
  assert.equal(save.state().message.includes('준비된 뒤'), false);
  save.dispose();
});

test('server loss disables new saves, then refresh recovers without a flight command', async () => {
  let online = false;
  const save = client(async () => { if (!online) throw Error('offline'); return response({available: true}); });
  save.setTarget('http://127.0.0.1:8790'); await tick();
  assert.equal(save.state().available, false);
  assert.ok(save.state().message.includes('연결할 수 없습니다'));
  online = true; await save.refresh();
  assert.equal(save.state().available, true);
  assert.equal(save.state().message.includes('연결할 수 없습니다'), false);
  save.dispose();
});
