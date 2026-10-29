import test from 'node:test';
import assert from 'node:assert/strict';
import { trackingDescription, connectTrackingPreview } from '../tracking-preview.js';

test('live receive latency and stale/failure states do not use file replay lag', () => {
  const state = {tracking_state: 2, last_map_timestamp_s: 1.2, receive_to_track_ms: 37,
    tracker_age_s: .1, input_lag_seconds: 0, frame_tracking: {tracked_features: 10}};
  assert.match(trackingDescription(state).detail, /수신→추적 37ms/);
  assert.equal(trackingDescription({...state, input_state: 'reconnecting'}).stale, true);
  assert.match(trackingDescription({...state, tracker_age_s: 2}).label, /마지막 처리/);
  assert.match(trackingDescription({...state, session_status: 'failed'}).label, /지도 보존/);
});

test('preview continues without map revisions and pins images to their exact frame/session', async () => {
  let selected = 'live', follow = true, next, frame = 'orb_00000001.jpg';
  const elements = new Map();
  const element = id => { if (!elements.has(id)) elements.set(id, {dataset: {}}); return elements.get(id); };
  const source = {context: () => ({source: selected}), isCurrent: c => c.source === selected,
    read: async () => ({ok: true, value: {session_id: 'one', frame_id: frame, preview_ready: true}})};
  connectTrackingPreview({source, following: () => follow, element, schedule: fn => next = fn});
  await new Promise(setImmediate);
  assert.match(element('slam-preview').src, /frame=orb_00000001.jpg&session=one/);
  frame = 'orb_00000002.jpg'; await next();
  assert.match(element('slam-preview').src, /00000002/);
  follow = false; frame = 'orb_00000003.jpg'; await next();
  assert.match(element('slam-preview').src, /00000002/);
  selected = 'saved'; await next();
  assert.match(element('slam-preview').src, /00000002/);
});
