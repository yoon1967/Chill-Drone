import test from 'node:test';
import assert from 'node:assert/strict';
import { connectProfiles, selectionFor, profileDescription } from '../profile-view.js';

const row = {id: 'profile_test', version: 1, camera_sha256: 'hash', width: 640, height: 480,
  name: 'Camera A', camera_identity: 'device A', image_geometry: 'full frame', camera: {}, readonly: false};
test('profile selection requires a matching original resolution and explicit confirmation', () => {
  assert.throws(() => selectionFor(row, false, 640, 480), /확인란/);
  assert.throws(() => selectionFor(row, true, 1280, 960), /해상도/);
  assert.deepEqual(selectionFor(row, true, 640, 480), {id: 'profile_test', version: 1, camera_sha256: 'hash', confirmed: true});
  assert.equal(selectionFor(null, false), null);
});
test('external quality is labelled as file information rather than a new measurement', () => {
  assert.match(profileDescription({...row, camera: {calibration_summary: {views: 20, validation_rms_px: .5}}}), /파일 기록: 20장/);
  assert.match(profileDescription(row), /별도로 확인/);
});

const deferred = () => { let resolve; const promise = new Promise(r => { resolve = r; }); return {resolve, promise}; };
function fixture(t, {defaultRequest} = {}) {
  const originals = new Map(['document', 'fetch', 'Option'].map(k => [k, Object.getOwnPropertyDescriptor(globalThis, k)]));
  t.after(() => { for (const [key, value] of originals) { if (value) Object.defineProperty(globalThis, key, value); else delete globalThis[key]; } });
  const camera = {width: 640, height: 480, fx: 600, fy: 601, cx: 320, cy: 240, distortion: [0, 0, 0, 0],
    calibration_summary: {views: 20, validation_rms_px: .5}};
  const cam = {...row, id: 'cam', camera}, cam2 = {...row, id: 'cam2', name: 'Camera B', camera};
  const drone = {...row, id: 'drone', name: 'Drone A', camera};
  cam.input_modes = cam2.input_modes = ['camera', 'file']; drone.input_modes = ['tello', 'file'];
  const data = {profiles: [cam, cam2, drone], defaults: {camera: 'cam', tello: 'drone', file: null}, warnings: []};
  const requests = [], elements = new Map();
  const get = id => {
    if (!elements.has(id)) elements.set(id, {value: '', textContent: '', checked: false, disabled: false, hidden: false, open: false,
      handlers: {}, files: [], attrs: {}, classList: {add() {}, remove() {}},
      addEventListener(name, fn) { this.handlers[name] = fn; }, setAttribute(name, value) { this.attrs[name] = value; },
      replaceChildren(...children) { this.options = children; }, focus() { this.focused = true; }});
    return elements.get(id);
  };
  for (const id of ['profile-tools', 'profile-import', 'profile-register']) get(id).hidden = true;
  Object.defineProperty(globalThis, 'document', {configurable: true, value: {getElementById: get}});
  Object.defineProperty(globalThis, 'Option', {configurable: true, value: class { constructor(label, value) { this.label = label; this.value = value; } }});
  Object.defineProperty(globalThis, 'fetch', {configurable: true, value: async (url, options = {}) => {
    const body = options.body ? JSON.parse(options.body) : null; requests.push({url, body});
    if (url.endsWith('/default')) {
      if (defaultRequest) return defaultRequest(body, data);
      data.defaults[body.mode] = body.id;
    } else if (body) {
      if (!body.confirmed) return {ok: false, json: async () => ({error: 'confirm_camera_profile_binding'})};
      const stored = {...body.camera, camera_identity: body.camera_identity, profile: {...row, id: 'new', name: body.name}};
      data.profiles.push({...stored.profile, camera: stored, camera_identity: body.camera_identity,
        image_geometry: body.image_geometry, width: camera.width, height: camera.height, input_modes: body.input_modes});
      return {ok: true, json: async () => stored};
    }
    return {ok: true, json: async () => structuredClone(data)};
  }});
  const handle = connectProfiles();
  return {get, data, requests, handle, camera, async change(id, value) { get(id).value = value; return get(id).handlers.change(); }};
}

test('basic view separates execution choice from hidden management and calibration details', async t => {
  const f = fixture(t); await f.handle.ready;
  assert.equal(f.get('profile-select-live').value, 'cam');
  assert.equal(f.get('profile-tools').hidden, true); assert.equal(f.get('profile-import').hidden, true);
  assert.doesNotMatch(f.get('profile-info-live').textContent, /RMS|버전|fx/);
  assert.match(f.get('profile-values-live').textContent, /검증 RMS 0.500px|검증 RMS 0.5px/);
  f.get('profile-manage-live').handlers.click();
  await f.change('profile-library', 'cam2');
  assert.equal(f.get('profile-select-live').value, 'cam');
  f.get('profile-confirm-live').checked = true;
  f.get('profile-use').handlers.click();
  assert.equal(f.get('profile-select-live').value, 'cam2'); assert.equal(f.get('profile-confirm-live').checked, false);
  assert.equal(f.get('profile-tools').hidden, true);
});

test('remember checkbox scopes save and removal to the actual live or file input', async t => {
  const f = fixture(t); await f.handle.ready;
  assert.equal(f.get('profile-remember-live').checked, true);
  f.get('profile-remember-live').checked = false; await f.get('profile-remember-live').handlers.change();
  assert.equal(f.data.defaults.camera, null); assert.equal(f.data.defaults.tello, 'drone');
  f.handle.syncLiveKind('tello-map'); assert.equal(f.get('profile-select-live').value, 'drone');
  f.handle.setContext('saved'); await f.change('profile-select-file', 'cam2');
  f.get('profile-remember-file').checked = true; await f.get('profile-remember-file').handlers.change();
  assert.deepEqual(f.requests.filter(r => r.url.endsWith('/default')).map(r => r.body), [{mode: 'camera', id: null}, {mode: 'file', id: 'cam2'}]);
  assert.equal(f.get('profile-select-live').value, 'drone');
});

test('failed preference save restores the checkbox without changing the selected profile', async t => {
  const f = fixture(t, {defaultRequest: async () => ({ok: false, json: async () => ({error: 'disk_full'})})});
  await f.handle.ready; f.get('profile-remember-live').checked = false;
  await f.get('profile-remember-live').handlers.change();
  assert.equal(f.get('profile-remember-live').checked, true); assert.equal(f.get('profile-select-live').value, 'cam');
  assert.match(f.get('profile-status').textContent, /disk_full/);
});

test('late default save cannot select a profile in a different live mode', async t => {
  const pending = deferred();
  const f = fixture(t, {defaultRequest: () => pending.promise}); await f.handle.ready;
  await f.change('profile-select-live', 'cam2'); f.get('profile-remember-live').checked = true;
  const saving = f.get('profile-remember-live').handlers.change();
  f.handle.syncLiveKind('tello-map');
  pending.resolve({ok: true, json: async () => ({defaults: {camera: 'cam2', tello: 'drone', file: null}})});
  await saving;
  assert.equal(f.get('profile-select-live').value, 'drone'); assert.equal(f.get('profile-remember-live').checked, true);
  assert.deepEqual(f.requests.at(-1).body, {mode: 'camera', id: 'cam2'});
});

test('running live capture blocks management apply and preference edits, while file management stays usable', async t => {
  const f = fixture(t); await f.handle.ready; f.handle.lockLive(true);
  await f.change('profile-library', 'cam2'); f.get('profile-use').handlers.click();
  assert.equal(f.get('profile-select-live').value, 'cam'); assert.equal(f.get('profile-remember-live').disabled, true);
  f.get('profile-remember-live').checked = false; await f.get('profile-remember-live').handlers.change();
  assert.equal(f.requests.some(r => r.url.endsWith('/default')), false);
  f.handle.setContext('saved'); f.get('profile-use').handlers.click();
  assert.equal(f.get('profile-select-file').value, 'cam2');
});

test('basic JSON registration supplies visible metadata defaults and never applies the new profile implicitly', async t => {
  const f = fixture(t); await f.handle.ready;
  await f.get('profile-file').handlers.change({target: {files: [{name: 'My camera.json', size: 300, text: async () => JSON.stringify(f.camera)}]}});
  assert.equal(f.get('profile-import').hidden, false); assert.equal(f.get('profile-advanced').open, false);
  assert.doesNotMatch(f.get('profile-file-info').textContent, /fx|fy/);
  assert.match(f.get('profile-registration-summary').textContent, /파일 복원만/);
  f.get('profile-name').value = 'Front camera'; f.get('profile-name').handlers.input();
  assert.equal(f.get('profile-identity').value, 'Front camera');
  f.get('profile-register-confirm').checked = true; await f.get('profile-save').handlers.click();
  const stored = f.requests.find(r => r.body?.name === 'Front camera').body;
  assert.equal(stored.camera_identity, 'Front camera'); assert.match(stored.image_geometry, /가져온 보정 파일/);
  assert.deepEqual(stored.input_modes, ['file']); assert.equal(stored.confirmed, true);
  assert.equal(f.get('profile-import').hidden, true); assert.equal(f.get('profile-select-live').value, 'cam');
});

test('tutorial result opens the collapsed import form and preserves measured metadata', async t => {
  const f = fixture(t); await f.handle.ready;
  await f.handle.prepareCamera({...f.camera, camera_identity: 'Measured drone'}, {mode: 'tello', name: 'New calibration'});
  assert.equal(f.get('profile-import').hidden, false); assert.equal(f.get('profile-register').hidden, false);
  assert.equal(f.get('profile-name').value, 'New calibration'); assert.equal(f.get('profile-identity').value, 'Measured drone');
  assert.match(f.get('profile-registration-summary').textContent, /드론 영상 수신/);
  assert.equal(f.get('profile-register-confirm').checked, false);
});

test('closing or switching drawers preserves a draft, but an invalid replacement cannot save the old camera', async t => {
  const f = fixture(t); await f.handle.ready;
  await f.handle.prepareCamera({...f.camera, camera_identity: 'Draft camera'}, {name: 'Draft'});
  f.get('profile-import-close').handlers.click(); f.get('profile-import-open').handlers.click();
  assert.equal(f.get('profile-name').value, 'Draft'); assert.equal(f.get('profile-register').hidden, false);
  f.get('profile-manage-live').handlers.click(); assert.equal(f.get('profile-import').hidden, true);
  f.get('profile-import-open').handlers.click(); assert.equal(f.get('profile-tools').hidden, true);
  assert.equal(f.get('profile-identity').value, 'Draft camera');
  await f.get('profile-file').handlers.change({target: {files: [{name: 'bad.json', size: 2, text: async () => '{}'}]}});
  assert.equal(f.get('profile-register').hidden, true);
  await f.get('profile-save').handlers.click();
  assert.equal(f.requests.some(r => r.body?.camera), false);
});
