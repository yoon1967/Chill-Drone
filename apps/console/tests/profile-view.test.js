import test from 'node:test';
import assert from 'node:assert/strict';
import { selectionFor, profileDescription } from '../profile-view.js';

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
