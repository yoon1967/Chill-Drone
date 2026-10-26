import test from 'node:test';
import assert from 'node:assert/strict';
import {pointGridCells, pointSpacingProfile} from './point-grid.js';
import {buildPointSurfaces, IncrementalPointSurfaces} from './point-surfaces.js';
import {applySceneDelta} from './map-delta.js';

const metadata = points => ({schema_version:'point-observations/1.0',
  frame_index_by_point:Object.fromEntries(points.map(p => [p[0],[0,1,2]]))});
const row = (id, xyz, rgb=[160,170,180]) => [id,...xyz,...rgb,.5,3];
const plane = (width=8, height=8, offset=0, rgb) => Array.from({length:width*height}, (_,i) =>
  row(offset+i,[i%width,Math.floor(i/width),0],rgb));
function build(points, options={}, interval=0, observations=metadata(points)) {
  const grid=pointGridCells(points,interval,observations,Math.max(3,points.length*3));
  return buildPointSurfaces(points,grid,observations,{spacing:1,...options});
}
const triples = flat => Array.from({length:flat.length/3}, (_,i) => flat.slice(i*3,i*3+3));
const pairs = flat => Array.from({length:flat.length/2}, (_,i) => flat.slice(i*2,i*2+2));

test('three non-collinear points form a face without a density or region-size gate', () => {
  const points=[row(0,[0,0,0]),row(1,[1,0,0]),row(2,[0,1,0])], before=JSON.stringify(points);
  const model=build(points);
  assert.equal(model.triangles.length,3);
  assert.equal(model.strongTriangles.length,3);
  assert.equal(model.pointOnly,0);
  assert.equal(JSON.stringify(points),before);
});

test('color boundaries separate adjacent regions', () => {
  const points=plane(12,6).map(p => [...p.slice(0,4),...(p[1]<6 ? [235,30,20] : [20,40,230]),...p.slice(7)]);
  const model=build(points,{colorThreshold:20});
  assert.ok(model.triangles.length>30);
  for (const [a,b] of pairs(model.edges)) assert.equal(points[a][1]<6,points[b][1]<6);
  assert.ok(model.groups>=2);
});

test('gradually bent surfaces remain connected across different plane directions', () => {
  const points=Array.from({length:18*6}, (_,i) => {
    const angle=(i%18)*Math.PI/36;
    return row(i,[12*Math.sin(angle),Math.floor(i/18),12*(1-Math.cos(angle))]);
  });
  const model=build(points,{normalAngle:35},.25);
  assert.ok(model.triangles.length>100);
  const connected=new Set([0]);
  for (let pass=0; pass<points.length; pass++) for (const [a,b] of pairs(model.edges)) {
    if (connected.has(a)) connected.add(b);
    if (connected.has(b)) connected.add(a);
  }
  assert.ok(connected.has(17),'surface should connect directions spanning 85 degrees');
});

test('normal-angle tolerance controls local graph boundaries', () => {
  const points=Array.from({length:12*6}, (_,i) => {
    const angle=(i%12)*Math.PI/24;
    return row(i,[8*Math.sin(angle),Math.floor(i/12),8*(1-Math.cos(angle))]);
  });
  assert.ok(build(points,{normalAngle:10}).candidateEdges<build(points,{normalAngle:85}).candidateEdges);
});

test('two points and collinear neighborhoods retain lines without invented faces', () => {
  const pair=build([row(0,[0,0,0]),row(1,[1,0,0])]);
  assert.equal(pair.edges.length,2);
  assert.equal(pair.triangles.length,0);
  const line=build(Array.from({length:12}, (_,i) => row(i,[i,0,0])));
  assert.ok(line.edges.length>=20);
  assert.equal(line.triangles.length,0);
});

test('nearby colors can be enabled explicitly with the color tolerance', () => {
  const points=[row(0,[0,0,0],[100,100,100]),row(1,[1,0,0],[145,145,145])];
  assert.equal(build(points,{colorThreshold:5}).edges.length,0);
  assert.equal(build(points,{colorThreshold:30}).edges.length,2);
});

test('large unsupported gaps are not bridged by matching colors', () => {
  const points=[...plane(5,5),...plane(5,5,25).map(p => [p[0],p[1]+30,...p.slice(2)])];
  const model=build(points);
  assert.ok(model.triangles.length>0);
  for (const [a,b] of pairs(model.edges)) assert.equal(a<25,b<25);
});

test('shared observations control brightness, not the existence of a geometric candidate', () => {
  const points=plane(5,5), observations=metadata(points);
  for (const p of points) observations.frame_index_by_point[p[0]]=[p[0]*3,p[0]*3+1,p[0]*3+2];
  const model=build(points,{},0,observations);
  assert.ok(model.weakTriangles.length>0);
  assert.equal(model.strongTriangles.length,0);
  assert.equal(model.triangles.length,model.weakTriangles.length);
});

test('missing or single-frame observations and high confidence thresholds leave points only', () => {
  const points=plane(4,4), observations=metadata(points);
  assert.equal(build(points,{},0,null).edges.length,0);
  for (const p of points) observations.frame_index_by_point[p[0]]=[0];
  assert.equal(build(points,{threshold:1},0,observations).edges.length,0);
  const empty=build(points,{threshold:100});
  assert.equal(empty.visible,0);
  assert.equal(empty.edges.length,0);
});

test('grid cells with conflicting original colors remain uncertain', () => {
  const points=[row(0,[0,0,0],[255,0,0]),row(1,[.1,0,0],[0,0,255]),
    row(2,[2,0,0],[255,0,0]),row(3,[0,2,0],[255,0,0])];
  const model=build(points,{colorThreshold:15},1);
  assert.equal(model.triangles.length,0);
  assert.equal(model.pointOnly,1);
});

test('confidence and shared observations use distinct frames after grid merging', () => {
  const points=Array.from({length:9}, (_,i) => row(i,[[0,0,0],[2,0,0],[0,2,0]][Math.floor(i/3)]));
  const observations=metadata(points);
  for (const p of points) observations.frame_index_by_point[p[0]]=[p[0]%3];
  const model=build(points,{},1,observations);
  assert.equal(model.visible,3);
  assert.equal(model.strongTriangles.length,3);
});

test('coincident coordinates and dense buckets retain bounded work and valid indices', () => {
  const points=Array.from({length:400}, (_,i) => row(i,[i/10000,(i%13)/10000,(i%7)/10000]));
  const model=build(points);
  assert.ok(model.neighborTests<=model.sampled*27*model.bucketLimit);
  assert.ok(model.candidateEdges<=model.sampled*model.neighborLimit/2);
  assert.ok(model.triangles.length<=model.sampled*model.neighborLimit);
  for (const id of [...model.edges,...model.triangles]) assert.ok(Number.isInteger(id) && id>=0 && id<model.sampled);
  assert.equal(build([row(0,[0,0,0]),row(1,[0,0,0])]).edges.length,0);
});

test('triangles are unique, respect edge incidence, and partition brightness consistently', () => {
  const points=plane(), model=build(points), incidence=new Map();
  const triangles=triples(model.triangles);
  assert.equal(new Set(triangles.map(t => [...t].sort((a,b) => a-b).join(','))).size,triangles.length);
  for (const [a,b,c] of triangles) for (const pair of [[a,b],[b,c],[c,a]]) {
    const key=pair.sort((a,b) => a-b).join(','); incidence.set(key,(incidence.get(key)||0)+1);
  }
  assert.ok([...incidence.values()].every(n => n<=2));
  assert.equal(model.strongTriangles.length+model.weakTriangles.length,model.triangles.length);
  assert.equal(model.strongEdges.length+model.weakEdges.length,model.edges.length);
  assert.deepEqual(build(points),model);
});

test('sampling bounds the overlay while preserving the visible point count', () => {
  const model=build(plane(),{maxPoints:12});
  assert.equal(model.visible,64);
  assert.equal(model.sampled,12);
  assert.ok(model.pointOnly>=52);
  assert.throws(() => build(plane(),{colorThreshold:0}),RangeError);
  assert.throws(() => build(plane(),{normalAngle:NaN}),RangeError);
});

test('presorted spacing profile agrees with brute-force nearest neighbors', () => {
  const points=Array.from({length:80}, (_,i) => row(i,[Math.sin(i*17)*5,Math.cos(i*13)*3,i/31]));
  const distances=points.map(p => Math.min(...points.filter(q => p!==q).map(q => Math.hypot(...p.slice(1,4).map((v,a) => v-q[a+1]))))).sort((a,b) => a-b);
  assert.ok(Math.abs(pointSpacingProfile(points).median-distances[40])<1e-12);
});

const context = revision => ({session:'synthetic',segment:0,poseSource:'orb_slam3',revision,frameKeys:['a','b','c']});
function increment(engine,points,revision=1,{interval=1,options={},observations=metadata(points),ctx={}}={}) {
  const grid=pointGridCells(points,interval,observations,points.length*3+3);
  return engine.update(points,grid,observations,{spacing:1,...options},{...context(revision),...ctx});
}
const islands = () => [...plane(6,6),...plane(6,6,36).map(p => [p[0],p[1]+200,...p.slice(2)])];
const meshKeys = engine => [...engine.patches.values()].flatMap(m => triples(m.triangles).map(t => t.map(i => m.nodeKeys[i]).sort().join('|'))).sort();

test('identical revisions reuse every patch without geometry work', () => {
  const e=new IncrementalPointSurfaces(),points=islands();
  const first=increment(e,points), models=[...e.patches.values()], second=increment(e,points,2);
  assert.ok(first.stats.faces>0);
  assert.equal(second.patches.length,0);
  assert.equal(second.stats.remeshedBlocks,0);
  assert.equal(second.stats.analyzedNodes,0);
  assert.equal(second.stats.retainedFaces,first.stats.faces);
  assert.deepEqual([...e.patches.values()],models);
});

test('local additions preserve distant geometry and agree with fresh blocked reconstruction', () => {
  const e=new IncrementalPointSurfaces({motionFraction:0,colorTolerance:0}),points=islands();
  increment(e,points);
  const distant=e.patches.get('2,0,0'), next=[...points,row(99,[6,2,0])];
  const result=increment(e,next,2), full=new IncrementalPointSurfaces({motionFraction:0,colorTolerance:0});
  increment(full,next,2);
  assert.equal(e.patches.get('2,0,0'),distant);
  assert.ok(result.stats.reusedBlocks>0);
  assert.ok(result.stats.retainedFaces>0);
  assert.deepEqual(meshKeys(e),meshKeys(full));
});

test('deletions invalidate incident faces rather than leaving ghost geometry', () => {
  const e=new IncrementalPointSurfaces({motionFraction:0}),points=islands();
  increment(e,points);
  const next=points.filter(p => p[0]!==7), result=increment(e,next,2);
  assert.ok(result.stats.removedFaces>0);
  for (const model of e.patches.values()) assert.ok(!model.nodeKeys.includes('g:1,1,0'));
  const full=new IncrementalPointSurfaces({motionFraction:0}); increment(full,next,2);
  assert.deepEqual(meshKeys(e),meshKeys(full));
});

test('observation-only changes update brightness without triangulation', () => {
  const e=new IncrementalPointSurfaces(),points=plane(5,5), observations=metadata(points);
  for (const p of points) observations.frame_index_by_point[p[0]]=[p[0]*3,p[0]*3+1,p[0]*3+2];
  const first=increment(e,points,1,{observations,ctx:{frameKeys:[]}});
  const second=increment(e,points,2,{ctx:{frameKeys:[]}});
  assert.ok(first.stats.weakFaces>0);
  assert.equal(second.stats.remeshedBlocks,0);
  assert.equal(second.stats.weakFaces,0);
  assert.ok(second.stats.evidenceBlocks>0);
});

test('camera reindexing with the same physical frames does not invalidate evidence', () => {
  const e=new IncrementalPointSurfaces(),points=plane(4,4);
  increment(e,points,1);
  const result=increment(e,points,2,{ctx:{frameKeys:['c','a','b']}});
  assert.equal(result.stats.observationCells,0);
  assert.equal(result.patches.length,0);
});

test('small original-coordinate motion updates vertices while retaining topology', () => {
  const e=new IncrementalPointSurfaces(),points=plane(5,5);
  increment(e,points,1,{interval:0});
  const next=points.map(p => [p[0],p[1]+.01,...p.slice(2)]), result=increment(e,next,2,{interval:0});
  assert.equal(result.stats.remeshedBlocks,0);
  assert.ok(result.stats.positionBlocks>0);
  const model=[...e.patches.values()][0];
  assert.ok(model.positions.some(v => Math.abs(v-.01)<1e-10));
});

test('motion accumulates against the last reconstructed reference', () => {
  const e=new IncrementalPointSurfaces(),points=plane(4,4);
  increment(e,points,1);
  for (const shift of [.05,.1,.15]) {
    const next=points.map(p => [p[0],p[1]+shift,...p.slice(2)]);
    assert.equal(increment(e,next,2).stats.remeshedBlocks,0);
  }
  const next=points.map(p => [p[0],p[1]+.25,...p.slice(2)]);
  assert.ok(increment(e,next,3).stats.remeshedBlocks>0);
});

test('settings, map changes, rewinds and periodic reconciliation rebuild all blocks', () => {
  const points=plane(4,4), e=new IncrementalPointSurfaces({reconcileEvery:2});
  increment(e,points,1); increment(e,points,2);
  assert.equal(increment(e,points,3).stats.reason,'periodic');
  assert.equal(increment(e,points,2).stats.reason,'rewind');
  assert.equal(increment(e,points,3,{ctx:{segment:1}}).stats.reason,'coordinate_system');
  assert.equal(increment(e,points,4,{options:{colorThreshold:40}}).stats.reason,'coordinate_system');
  assert.equal(increment(e,points,5,{options:{colorThreshold:45}}).stats.reason,'settings');
});

test('a patch at a halo boundary is invalidated by newly added support', () => {
  const points=plane(4,4),e=new IncrementalPointSurfaces({motionFraction:0,colorTolerance:0});
  increment(e,points);
  const next=[...points,row(99,[90,0,0])], updated=increment(e,next,2);
  assert.ok(updated.stats.remeshedBlocks>0);
  const full=new IncrementalPointSurfaces({motionFraction:0,colorTolerance:0}); increment(full,next,2);
  assert.deepEqual(meshKeys(e),meshKeys(full));
});

test('incremental settings reject invalid radii before spatial loops', () => {
  const e=new IncrementalPointSurfaces();
  assert.throws(() => increment(e,plane(),1,{options:{connection:0}}),RangeError);
  assert.throws(() => new IncrementalPointSurfaces({blockScale:0}),RangeError);
});

test('map deltas preserve point order, observations and previous snapshots', () => {
  const previous={pose_source:'orb_slam3',revision:1,input_run_id:'synthetic',segment_id:0,
    points:[row(0,[0,0,0]),row(1,[1,0,0])],point_observations:{frame_index_by_point:{0:[0,1],1:[1,2]}}};
  const before=JSON.stringify(previous), payload={schema_version:'phone-map-delta/1.0',base_revision:1,revision:2,
    session_id:'synthetic',segment_id:0,scene:{pose_source:'orb_slam3',revision:2,input_run_id:'synthetic',segment_id:0},
    points_removed:[0],points_upsert:[row(2,[0,1,0])],point_order:[2,1],observation_removed:['0'],
    observation_upserts:{2:[0,2]},observation_metadata:{schema_version:'point-observations/1.0'}};
  const result=applySceneDelta(previous,payload);
  assert.deepEqual(result.points.map(p => p[0]),[2,1]);
  assert.deepEqual(result.point_observations.frame_index_by_point,{1:[1,2],2:[0,2]});
  assert.equal(JSON.stringify(previous),before);
  assert.throws(() => applySceneDelta({...previous,revision:0},payload),/base mismatch/);
  assert.throws(() => applySceneDelta(previous,{...payload,point_order:[1,1]}),/point order/);
  assert.equal(applySceneDelta(previous,previous),previous);
});

test('worker resynchronizes geometry after the UI skips an obsolete result', async () => {
  const messages=[];
  globalThis.self={postMessage:message => messages.push(message)};
  try {
    await import('./point-surface-worker.js');
    const points=plane(4,4), observations=metadata(points), grid=pointGridCells(points,1,observations,3);
    const send=(id,ack) => self.onmessage({data:{id,ack,points,grid,observations,settings:{spacing:1},context:context(id)}});
    await send(1,0); await send(2,0); await send(3,2);
    assert.ok(messages[0].result.patches.length>0);
    assert.ok(messages[1].result.reset);
    assert.ok(messages[1].result.patches.length>0,'missed acknowledgement requires a complete cached geometry snapshot');
    assert.equal(messages[2].result.patches.length,0);
  } finally { delete globalThis.self; }
});

test('block ownership prevents duplicate triangles at shared boundaries', () => {
  const points=plane(20,6).map(p => [p[0],p[1]+70,...p.slice(2)]), e=new IncrementalPointSurfaces();
  increment(e,points);
  const keys=meshKeys(e);
  assert.ok(e.patches.size>1);
  assert.equal(new Set(keys).size,keys.length);
});
