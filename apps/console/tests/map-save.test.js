import test from 'node:test';
import assert from 'node:assert/strict';
import {mapDownload, saveShortcut, connectMapSave} from '../map-save.js';
import {MapSource} from '../map-source.js';

test('history export pins the session and imported asset without limiting the displayed revision', () => {
  const source = new MapSource('live');
  const run = {source: 'live', points: [[1]], revision: 7, stream: {session_id: 'session A'}};
  assert.equal(mapDownload(source, run), '/api/live/map.zip?session=session+A');
  source.select('saved'); source.asset = 'asset A';
  assert.throws(() => mapDownload(source, run));
  assert.equal(mapDownload(source, {...run, source: 'saved'}), '/api/saved/map.zip?session=session+A&asset=asset%20A');
  assert.throws(() => mapDownload(source, {source: 'saved', points: []}));
});

test('save shortcut supports Windows and Mac and leaves text editing alone', () => {
  assert.equal(saveShortcut({ctrlKey: true, key: 's'}), true);
  assert.equal(saveShortcut({metaKey: true, key: 'S'}), true);
  assert.equal(Boolean(saveShortcut({key: 's'})), false);
  assert.equal(saveShortcut({ctrlKey: true, key: 's', target: {tagName: 'INPUT'}}), false);
  assert.equal(saveShortcut({ctrlKey: true, key: 's', target: {isContentEditable: true}}), false);
});

test('failed download reports an error and does not create a broken archive', async () => {
  const elements = new Map(['map-save', 'map-save-status'].map(id => [id, {addEventListener() {}}]));
  let downloads = 0;
  const handle = connectMapSave({source: new MapSource('saved'), scene: () => ({source: 'saved', points: [[1]]}),
    element: id => elements.get(id), target: {addEventListener() {}}, fetcher: async () => ({ok: false}), download() { downloads++; }});
  await handle.save();
  assert.equal(downloads, 0);
  assert.match(elements.get('map-save-status').textContent, /저장할 수 없습니다/);
  assert.equal(elements.get('map-save').disabled, false);
});

test('hidden save and shortcut cannot download before manual disconnect and finalization', async () => {
  const elements = new Map(['map-save', 'map-save-status', 'map-save-bar'].map(id => [id, {addEventListener() {}}]));
  let ready = false, requests = 0;
  const handle = connectMapSave({source: new MapSource('live'), scene: () => ({source: 'live', points: [[1]]}),
    allowed: () => ready, element: id => elements.get(id), target: {addEventListener() {}},
    fetcher: async () => { requests++; return {ok: true, blob: async () => ({})}; }, download() {}});
  assert.equal(elements.get('map-save-bar').hidden, true);
  assert.equal(elements.get('map-save').disabled, true);
  await handle.save();
  assert.equal(requests, 0);
  ready = true; handle.update();
  assert.equal(elements.get('map-save-bar').hidden, false);
  await handle.save();
  assert.equal(requests, 1);
  ready = false; handle.update();
  assert.equal(elements.get('map-save-bar').hidden, true);
});
