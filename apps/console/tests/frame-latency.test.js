import test from 'node:test';
import assert from 'node:assert/strict';
import { FrameLatency } from '../frame-latency.js';

const scene = (session = 'live_test', revision = 1) => ({source: 'live', input_run_id: session,
  pose_source: 'orb_slam3', revision,
  cameras: [{frame_id: 'a', timestamp_s: .1}, {frame_id: 'b', timestamp_s: .2}, {frame_id: 'c', timestamp_s: .3}],
  point_observations: {schema_version: 'point-observations/1.0', frame_index_by_point: {'50': [0, 1], '51': [0, 1]}}});

test('each supported input is measured once at display, including full mapping delay', () => {
  const latency = new FrameLatency(); latency.start('live_test');
  latency.record({session: 'live_test', pts: .1, capturedAtMs: 100, encodedAtMs: 110, ackAtMs: 125});
  latency.record({session: 'live_test', pts: .2, capturedAtMs: 200, encodedAtMs: 210, ackAtMs: 240});
  assert.equal(latency.present(scene(), 1100, {fetchMs: 20, applyMs: 5}), 2);
  assert.equal(latency.present(scene('live_test', 2), 1200), 0);
  const summary = latency.summary();
  assert.equal(summary.meanMs, 950); assert.equal(summary.p95Ms, 1000); assert.equal(summary.maxMs, 1000);
  assert.equal(summary.encodeMeanMs, 10); assert.equal(summary.uploadMeanMs, 22.5);
  assert.equal(summary.fetchMeanMs, 20); assert.equal(summary.applyMeanMs, 5);
  assert.equal(summary.measuredKeyframes, 2);
});

test('unconfirmed frames, saved results, stale sessions and negative times create no samples', () => {
  const latency = new FrameLatency(); latency.start('live_test');
  latency.record({session: 'live_test', pts: .3, capturedAtMs: 100}); // not supporting the displayed points
  assert.equal(latency.present(scene(), 1000), 0);
  latency.record({session: 'live_test', pts: .1, capturedAtMs: 100});
  assert.equal(latency.present({...scene(), source: 'saved'}, 1000), 0);
  assert.equal(latency.present(scene('other'), 1000), 0);
  assert.equal(latency.present(scene(), 90), 0);
  latency.start('new_session');
  assert.equal(latency.present(scene(), 1000), 0);
  assert.equal(latency.summary().samples, 0);
});

test('bounded frame history reports expired marks and cannot fabricate old delays', () => {
  const latency = new FrameLatency(2); latency.start('live_test');
  for (let i = 1; i <= 3; i++) latency.record({session: 'live_test', pts: i / 10, capturedAtMs: i * 100});
  latency.record({session: 'other', pts: .1, capturedAtMs: 0});
  assert.equal(latency.present(scene(), 1000), 1);
  assert.equal(latency.summary().expiredFrameMarks, 1);
  assert.equal(latency.summary().submittedFrames, 3);
  assert.equal(latency.report().measurements[0].frame, 'b');
  assert.equal(latency.summary().encodeMeanMs, null);
});

test('points hidden by display filters cannot create a presentation sample', () => {
  const latency = new FrameLatency(); latency.start('live_test');
  latency.record({session: 'live_test', pts: .1, capturedAtMs: 100});
  assert.equal(latency.present(scene(), 1000, {}, []), 0);
  assert.equal(latency.present(scene(), 1100, {}, [50]), 1);
});
