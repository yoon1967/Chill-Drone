import test from 'node:test';
import assert from 'node:assert/strict';

test('previous default upgrades once while person preferences and subsequent choices survive', async () => {
  let saved={point_budget:8000, sample_fps:2.5, person_color:'#00ccff', person_outline:false};
  globalThis.localStorage={getItem:()=>JSON.stringify(saved),setItem:(_,value)=>{saved=JSON.parse(value);}};
  try {
    const first=await import('../mapping-settings.js?budget-upgrade');
    assert.equal(first.mappingOptions().point_budget,32000);
    assert.equal(first.mappingOptions().sample_fps,2.5);
    assert.equal(first.mappingOptions().person_color,'#00ccff');
    assert.equal(first.mappingOptions().person_outline,false);
    saved.point_budget=8000;
    const next=await import('../mapping-settings.js?budget-reload');
    assert.equal(next.mappingOptions().point_budget,8000);
    assert.equal(next.mappingOptions().sample_fps,2.5);
  } finally {delete globalThis.localStorage;}
});
