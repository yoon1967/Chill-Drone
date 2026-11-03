import test from 'node:test';
import assert from 'node:assert/strict';
import {liveStep, canVisitStep} from '../workflow-guide.js';

test('live stages follow observed connection, running map and finalized result', () => {
  assert.equal(liveStep({}), 0);
  assert.equal(liveStep({connected: true}), 1);
  assert.equal(liveStep({connected: true, mapping_active: true}), 2);
  assert.equal(liveStep({can_save: true}), 3);
  assert.equal(liveStep({finalizing: true}), 3);
  assert.equal(liveStep({mapping_session_id: 'one', map_available: true}), 3);
});

test('guide allows returning to previous steps and rejects skipping ahead or invalid steps', () => {
  assert.equal(canVisitStep(0, 3), false);
  assert.equal(canVisitStep(3, 0), true);
  assert.equal(canVisitStep(2, 2), true);
  for (const target of [-1, NaN, 1.5, '1']) assert.equal(canVisitStep(3, target), false);
});
