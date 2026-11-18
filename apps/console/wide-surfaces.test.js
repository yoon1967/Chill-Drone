import test from 'node:test';
import assert from 'node:assert/strict';
import {WidePointSurfaces,projectToView} from './wide-surfaces.js';
import {pointGridCells} from './point-grid.js';
import {buildPointSurfaces} from './point-surfaces.js';

const green=[70,100,70],rotation=[[1,0,0],[0,1,0],[0,0,1]];
const camera=(frame_id,x=0)=>({frame_id,width:240,height:160,params:[180,180,120,80,0,0,0,0],
  cam_from_world_rotation_raw:rotation,cam_from_world_translation_raw:[x,0,0]});
const context={session:'test',segment:0,poseSource:'orb_slam3',revision:1,frameKeys:['a','b','c'],
  displayRotation:rotation,cameras:[camera('a'),camera('b',.1),camera('c',-.1)]};
const points=()=>Array.from({length:30},(_,i)=>[i,-2+(i%6)*.8,-1.5+Math.floor(i/6)*.75,5,...green,.5,3]);
const observations=rows=>({schema_version:'point-observations/1.0',frame_index_by_point:Object.fromEntries(rows.map(p=>[p[0],[0,1,2]]))});
const image=(pixel=()=>green)=>{const data=new Uint8ClampedArray(240*160*4);for(let y=0;y<160;y++)for(let x=0;x<240;x++)data.set([...pixel(x,y),255],(y*240+x)*4);return {data,width:240,height:160};};
const settings={threshold:3,colorThreshold:21,spacing:.025};
async function update(engine,rows=points(),ctx={},options={}) {
  const obs=observations(rows),grid=pointGridCells(rows,.1,obs,3);
  return engine.update(rows,grid,obs,{...settings,...options},{...context,...ctx});
}

test('sparse same-color support creates a broad surface beyond local edge distances',async()=>{
  const rows=points(),before=JSON.stringify(rows),obs=observations(rows),grid=pointGridCells(rows,.1,obs,3);
  assert.equal(buildPointSurfaces(rows,grid,obs,settings).triangles.length,0);
  const engine=new WidePointSurfaces({loadImage:async()=>image()}),result=await update(engine,rows);
  assert.equal(result.stats.widePlanes,1);assert.ok(result.stats.wideQuads>0);
  assert.equal(result.stats.wideFaces,result.stats.wideQuads*2);
  assert.ok(Math.abs([...engine.patches.values()][0].quadAreas.reduce((s,a)=>s+a,0)-12)<1e-8);
  assert.equal(result.stats.wideWeakFaces,0);assert.equal(JSON.stringify(rows),before);
  for(const model of engine.patches.values())assert.ok(model.positions.every(Number.isFinite));
});

test('a disconnected image-color gap is left open inside the point hull',async()=>{
  const engine=new WidePointSurfaces({loadImage:async()=>image(x=>x>114&&x<126?[170,30,30]:green)});
  const result=await update(engine);assert.ok(result.stats.wideFaces>0);
  for(const model of engine.patches.values())for(let i=0;i<model.triangles.length;i+=3) {
    const xyz=model.triangles.slice(i,i+3).map(id=>model.positions.slice(id*3,id*3+3));
    const center=[0,1,2].map(a=>xyz.reduce((s,p)=>s+p[a],0)/3);
    assert.ok(Math.abs(center[0])>.15,'image gap must not be bridged');
  }
});

test('one confirming view yields faint candidates; no available images yields none',async()=>{
  const one=new WidePointSurfaces({loadImage:async url=>image(()=>url.includes('/a?')?green:[180,30,30])});
  const result=await update(one);assert.ok(result.stats.wideFaces>0);
  assert.equal(result.stats.wideWeakFaces,result.stats.wideFaces);
  const missing=new WidePointSurfaces({loadImage:async()=>{throw new Error('missing');}});
  assert.equal((await update(missing)).stats.wideFaces,0);
});

test('lighting variation is accepted while a neutral-colored foreground is excluded',async()=>{
  const engine=new WidePointSurfaces({loadImage:async()=>image((x,y)=>x>108&&x<132&&y>65&&y<95?[210,210,210]:x<120?[50,75,50]:[110,150,110])});
  const result=await update(engine);assert.ok(result.stats.wideFaces>0);
  assert.ok([...engine.patches.values()][0].quadAreas.reduce((s,a)=>s+a,0)>6);
  for(const m of engine.patches.values())for(let i=0;i<m.triangles.length;i+=3) {
    const center=[0,1,2].map(a=>m.triangles.slice(i,i+3).reduce((s,id)=>s+m.positions[id*3+a],0)/3);
    assert.ok(Math.abs(center[0])>.25||Math.abs(center[1])>.3);
  }
});

test('unchanged frames reuse geometry and decoded images; disabling removes all broad patches',async()=>{
  let loads=0;const e=new WidePointSurfaces({loadImage:async()=>{loads++;return image();}});
  const first=await update(e),models=[...e.patches.values()],again=await update(e,points(),{revision:2});
  assert.equal(again.patches.length,0);assert.equal(again.stats.wideRetainedFaces,first.stats.wideFaces);
  assert.deepEqual([...e.patches.values()],models);assert.equal(loads,3);
  const off=await update(e,points(),{revision:3},{wideSurfaces:false});assert.equal(off.stats.wideFaces,0);
  assert.equal(off.removed.length,1);
});

test('same-color parallel planes at different depths stay separate',async()=>{
  const rows=[...points(),...points().map(p=>[p[0]+100,p[1]+7,p[2],p[3]+1,...p.slice(4)])];
  const cameras=[...context.cameras].map(c=>({...c,width:800,params:[100,100,400,80]}));
  const e=new WidePointSurfaces({loadImage:async()=>image()});
  const r=await update(e,rows,{cameras});assert.equal(r.stats.widePlanes,2);
  for(const m of e.patches.values())assert.ok(Math.max(...m.positions.filter((_,i)=>i%3===2))-Math.min(...m.positions.filter((_,i)=>i%3===2))<1e-6);
});

test('removing spatial support shrinks the mesh without ghost faces',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image()});await update(e);
  const remaining=points().filter(p=>p[1]<.5),r=await update(e,remaining,{revision:2});
  assert.ok(r.stats.wideFaces>0);assert.ok(r.stats.wideRemovedFaces>0);
  for(const m of e.patches.values())for(let i=0;i<m.positions.length;i+=3)assert.ok(m.positions[i]<=.5);
  const empty=await update(e,remaining.slice(0,3),{revision:3});
  assert.equal(empty.stats.wideFaces,0);assert.equal(e.patches.size,0);assert.ok(empty.removed.length>0);
});

test('confidence, missing observations, line support and coordinate resets are respected',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image()});
  assert.equal((await update(e,points(),{}, {threshold:4})).stats.widePlanes,0);
  const line=points().map((p,i)=>[p[0],i*.1,0,5,...p.slice(4)]);
  assert.equal((await update(e,line)).stats.widePlanes,0);
  await update(e);await update(e,points(),{segment:1,revision:1});
  assert.ok([...e.patches.keys()].every(k=>k.startsWith('wide:')));
  const obs={},grid=pointGridCells(points(),.1,obs,3);
  assert.equal((await e.update(points(),grid,obs,settings,context)).stats.widePlanes,0);
});

test('projection reverses display rotation, applies lens distortion and rejects points behind camera',()=>{
  assert.deepEqual(projectToView([0,0,5],camera('a'),rotation),[.5,.5]);
  assert.deepEqual(projectToView([0,0,-5],camera('a'),[[1,0,0],[0,-1,0],[0,0,-1]]),[.5,.5]);
  assert.equal(projectToView([0,0,-5],camera('a'),rotation),null);
  const c=camera('a');c.params[4]=.1;
  assert.ok(projectToView([1,0,5],c,rotation)[0]>projectToView([1,0,5],camera('a'),rotation)[0]);
});

const uneven=()=>[-2,-1.9,-1.7,-1.4,-.8,.6,2].flatMap(x=>[-1.5,-1.1,0,1.5].map(y=>[x,y,5]))
  .map((xyz,id)=>[id,...xyz,...green,.5,3]);

test('variable rectangle boundaries are anchored to actual projected features',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image()}),rows=uneven(),r=await update(e,rows);
  assert.ok(r.stats.wideQuads>1);assert.ok(r.stats.wideQuads<=192);
  const m=[...e.patches.values()][0],plane=e.planes[0];
  assert.ok(Math.max(...m.quadAreas)>Math.min(...m.quadAreas)*1.2);
  const projected=rows.map(p=>[plane.u,plane.v].map(axis=>axis.reduce((s,v,i)=>s+v*(p[i+1]-plane.center[i]),0)));
  for(const bounds of m.quadBounds)for(let a=0;a<4;a++)
    assert.ok(projected.some(p=>Math.abs(p[a%2]-bounds[a])<1e-8),'every cut must use a detected feature coordinate');
  for(let i=0;i<m.quads.length;i+=4) {
    const q=m.quads.slice(i,i+4).map(id=>m.positions.slice(id*3,id*3+3));
    const a=q[1].map((v,j)=>v-q[0][j]),b=q[3].map((v,j)=>v-q[0][j]);
    assert.ok(Math.abs(a.reduce((s,v,j)=>s+v*b[j],0))<1e-8);
  }
});

test('mixed rectangle sizes share subdivided edges without false internal borders',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image()});await update(e,uneven());
  const m=[...e.patches.values()][0];
  for(let i=0;i<m.boundary.length;i+=2) {
    const [a,b]=m.boundary.slice(i,i+2).map(id=>m.positions.slice(id*3,id*3+3));
    assert.ok((Math.abs(Math.abs(a[0])-2)<1e-8&&Math.abs(a[0]-b[0])<1e-8)||
      (Math.abs(Math.abs(a[1])-1.5)<1e-8&&Math.abs(a[1]-b[1])<1e-8),'interior T junction is not a surface boundary');
  }
});

test('moving the outer feature coordinates changes rectangle bounds',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image()});await update(e,uneven());
  const next=uneven().map(p=>p[1]===2?[p[0],1.6,...p.slice(2)]:p);
  const r=await update(e,next,{revision:2});assert.ok(r.patches.length>0);
  for(const m of e.patches.values())for(let i=0;i<m.positions.length;i+=3)assert.ok(m.positions[i]<=1.6+1e-8);
});

test('interior image pixels prevent large rectangles from covering a small color hole',async()=>{
  const e=new WidePointSurfaces({loadImage:async()=>image((x,y)=>x>=90&&x<=94&&y>=65&&y<=70?[180,20,20]:green)});
  const r=await update(e);assert.ok(r.stats.wideQuads>0);
  const x=(92.5-120)*5/180,y=(67.5-80)*5/180;
  for(const m of e.patches.values())for(let i=0;i<m.quads.length;i+=4) {
    const q=m.quads.slice(i,i+4).map(id=>m.positions.slice(id*3,id*3+3));
    assert.ok(x<Math.min(...q.map(p=>p[0]))||x>Math.max(...q.map(p=>p[0]))||
      y<Math.min(...q.map(p=>p[1]))||y>Math.max(...q.map(p=>p[1])));
  }
});

test('the patch budget coarsens dense supported regions instead of truncating coverage',async()=>{
  const rows=Array.from({length:2400},(_,id)=>[id,-2+(id%60)*4/59,-1.5+Math.floor(id/60)*3/39,5,...green,.5,3]);
  const e=new WidePointSurfaces({loadImage:async()=>image()}),r=await update(e,rows);
  assert.ok(r.stats.wideQuads<=192);assert.ok(r.stats.wideQuads>1);
  const m=[...e.patches.values()][0];assert.ok(Math.abs(m.quadAreas.reduce((s,a)=>s+a,0)-12)<1e-7);
});

test('image requests and cached masks stay isolated between saved and live sources',async()=>{
  const urls=[],e=new WidePointSurfaces({loadImage:async url=>{urls.push(url);return image();}});
  await update(e,points(),{source:'saved'});
  const live=await update(e,points(),{source:'live'});
  assert.equal(urls.length,6);assert.ok(live.stats.wideQuads>0);
  assert.ok(urls.slice(0,3).every(url=>url.startsWith('/api/saved/frame/')&&!url.includes('revision=')));
  assert.ok(urls.slice(3).every(url=>url.startsWith('/api/live/frame/')&&url.includes('revision=000001')&&url.includes('session=test')));
});
