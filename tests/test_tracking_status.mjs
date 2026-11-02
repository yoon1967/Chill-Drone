import assert from 'node:assert/strict';
import test from 'node:test';
import { liveTrackingNotices, trackingDescription } from '../apps/console/tracking-preview.js';

const receiving = {
  status: 'receiving', input_state: 'receiving', tracker_age_s: 0.3,
  map_available: true, map_id: 0, segment_id: 0,
  segments: [{map_id: 0}], tracking_state: 2,
};

test('continuous video with lost map tracking is identified separately', () => {
  const lost = {...receiving, tracking_state: 3};
  const notice = liveTrackingNotices(lost);
  assert.match(notice.tracking.title, /영상 수신 중/);
  assert.match(notice.tracking.detail, /기존 지도와 연결되지 않아/);
  assert.match(trackingDescription(lost).label, /지도 갱신 대기/);
  assert.equal(liveTrackingNotices(receiving).tracking, null);
});

test('stale input is not described as continuing video reception', () => {
  const lost = {...receiving, tracking_state: 3, tracker_age_s: 2};
  assert.doesNotMatch(liveTrackingNotices(lost).tracking.title, /영상 수신 중/);
});

test('new native map is distinct from relocalization and remains marked', () => {
  const initializing = {...receiving, tracking_state: 0, map_id: 1};
  assert.match(liveTrackingNotices(initializing).tracking.title, /새 지도 초기화/);
  const newMap = {...receiving, map_id: 1, segment_id: 1,
    segments: [{map_id: 0}, {map_id: 1}]};
  assert.equal(liveTrackingNotices(newMap).tracking, null);
  assert.match(liveTrackingNotices(newMap).segment.detail, /정렬되지 않았습니다/);
  assert.match(liveTrackingNotices({...newMap, tracking_state: 3}).tracking.title, /추적 중단/);
});
