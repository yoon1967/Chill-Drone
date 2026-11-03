import test from 'node:test';
import assert from 'node:assert/strict';
import {ProgressiveReplay} from '../progressive-replay.js';

test('seek selects the frame at or before the requested time and plays from there', () => {
  const replay = new ProgressiveReplay();
  replay.setEntries([0, 1, 2].map((timestamp_s, i) => ({timestamp_s, revision: i + 1})));
  replay.seek(.5); assert.equal(replay.selected().revision, 1);
  replay.play(100); replay.tick(1100); assert.equal(replay.selected().revision, 2);
  replay.pause(); replay.tick(5000); assert.equal(replay.time, 1.5);
  replay.setEntries([0, 1, 2, 3].map((timestamp_s, i) => ({timestamp_s, revision: i + 1})));
  assert.equal(replay.time, 1.5, 'processing new frames does not move a historical cursor');
  replay.follow(); assert.equal(replay.time, 3);
  replay.seek(-100); assert.equal(replay.time, 0);
  replay.seek(100); assert.equal(replay.time, 3);
  replay.play(0); replay.rate = 4; replay.tick(1000);
  assert.equal(replay.time, 3); assert.equal(replay.playing, false);
});
