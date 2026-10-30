import test from 'node:test';
import assert from 'node:assert/strict';
import {connectCalibration, coverageGuide, calibrationMessage} from '../calibration-view.js';

const deferred = () => {let resolve; const promise = new Promise(r=>{resolve=r;}); return {resolve, promise};};
const state = (id='calibration_test') => ({id, source:'camera', status:'collecting', accepted:0,
  coverage:{ready:false, checks:{}}, reason:'show_checkerboard', size:null});

function fixture(t, {permission, fetcher}={}) {
  const saved = new Map(['document','navigator','window','fetch','Option'].map(k=>[k,Object.getOwnPropertyDescriptor(globalThis,k)]));
  t.after(()=>{for(const [k,v] of saved) {if(v) Object.defineProperty(globalThis,k,v); else delete globalThis[k];}});
  const elements = new Map(), events={}, calls=[], busy=[], prepared=[], preparedOptions=[], constraints=[];
  const element = () => ({handlers:{}, value:'', checked:false, readyState:0, files:[], dataset:{},
    addEventListener(n,f){this.handlers[n]=f;}, replaceChildren(...children){this.children=children;},
    removeAttribute(){}, pause(){}, async play(){}, focus(){}});
  const get=id=>{if(!elements.has(id)) elements.set(id,element());return elements.get(id);};
  Object.entries({'calibration-columns':'9','calibration-rows':'6','calibration-square':'25',
    'calibration-source':'camera','calibration-size':'960x720'}).forEach(([id,value])=>{get(id).value=value;});
  const track={label:'Test camera',stops:0,addEventListener(){},stop(){this.stops++;}};
  const media={getTracks:()=>[track],getVideoTracks:()=>[track]};
  const values={
    document:{getElementById:get,createElement:tag=>tag==='canvas'?{getContext:()=>({drawImage(){}}),toBlob:f=>f(new Blob(['jpeg']))}:element()},
    navigator:{mediaDevices:{async enumerateDevices(){return [];},async getUserMedia(c){constraints.push(c);return permission?permission():media;}}},
    window:{addEventListener(n,f){events[n]=f;}},
    Option:class{constructor(label,value){this.label=label;this.value=value;}},
    fetch:async(url,options={})=>{
      const body=options.body instanceof Blob?options.body:options.body?JSON.parse(options.body):null;
      calls.push({url,body,options});
      const value=fetcher?await fetcher(url,body):url==='/api/calibration'?{enabled:true,busy:false}:state();
      return {ok:true,json:async()=>value};
    },
  };
  for(const [k,v]of Object.entries(values))Object.defineProperty(globalThis,k,{configurable:true,value:v});
  const handle=connectCalibration({profiles:{async prepareCamera(camera,options){prepared.push(camera);preparedOptions.push(options);}},onBusy:v=>busy.push(v)});
  t.after(()=>handle.stop());
  get('calibration-confirm').checked=true;
  return {get,calls,busy,prepared,preparedOptions,constraints,track,media,events,handle,
    start:()=>get('calibration-start').handlers.click(),stop:()=>get('calibration-stop').handlers.click()};
}

test('tutorial requires explicit board confirmation and keeps useful collection guidance',async t=>{
  const f=fixture(t);f.get('calibration-confirm').checked=false;
  await f.start();assert.equal(f.calls.length,0);assert.match(f.get('calibration-status').textContent,/확인란/);
  assert.equal(coverageGuide({checks:{views:true}}).filter(x=>x.done).length,1);
  assert.match(calibrationMessage('checkerboard_duplicate_pose'),/바꿔/);
});

test('permission denial never starts a server session and restores mapping controls',async t=>{
  const f=fixture(t,{permission:async()=>{throw new DOMException('denied','NotAllowedError');}});
  await f.start();assert.equal(f.calls.some(x=>x.url.endsWith('/start')),false);
  assert.equal(f.handle.active,false);assert.equal(f.busy.at(-1),false);
  assert.equal(f.constraints[0].audio,false);assert.match(f.get('calibration-status').textContent,/권한이 거부/);
});

test('cancel pending camera permission releases late media without creating calibration',async t=>{
  const pending=deferred(), f=fixture(t,{permission:()=>pending.promise});
  const starting=f.start();await Promise.resolve();await Promise.resolve();await Promise.resolve();
  await f.stop();pending.resolve(f.media);await starting;
  assert.equal(f.track.stops,1);assert.equal(f.calls.some(x=>x.url.endsWith('/start')),false);
});

test('cancel pending server start stops only its late ID and preserves the next run',async t=>{
  const pending=deferred();let starts=0;
  const f=fixture(t,{fetcher:async(url)=>{
    if(url==='/api/calibration')return {enabled:true,busy:false};
    if(url.endsWith('/start'))return ++starts===1?pending.promise:state('new');
    return state('new');
  }});
  const first=f.start();for(let i=0;i<12;i++)await Promise.resolve();
  assert.equal(starts,1);await f.stop();await f.start();
  pending.resolve(state('old'));await first;
  assert.equal(f.handle.active,true);
  assert.deepEqual(f.calls.filter(x=>x.url.endsWith('/stop')).map(x=>x.body.id),['old']);
  await f.stop();
});

test('successful result releases camera and prepares the existing profile registration form',async t=>{
  const camera={width:960,height:720,fx:850,fy:855,cx:480,cy:360,distortion:[0,0,0,0],
    calibration_summary:{views:20,training_views:16,validation_views:4,training_rms_px:.2,validation_rms_px:.3}};
  const f=fixture(t,{fetcher:async url=>url==='/api/calibration'?{enabled:true,busy:false}:
    {...state(),status:'complete',accepted:20,camera}});
  await f.start();assert.equal(f.track.stops,1);assert.equal(f.handle.active,false);
  assert.equal(f.get('calibration-result').hidden,false);assert.match(f.get('calibration-result-info').textContent,/검증 RMS 0.300/);
  await f.get('calibration-register').handlers.click();
  assert.equal(f.prepared[0].fx,850);assert.equal(f.preparedOptions[0].mode,'camera');
});

test('page exit releases media and sends a scoped keepalive stop',async t=>{
  const f=fixture(t);await f.start();f.events.pagehide();
  assert.equal(f.track.stops,1);assert.equal(f.handle.active,false);
  const stop=f.calls.find(x=>x.url.endsWith('/stop'));
  assert.equal(stop.body.id,'calibration_test');assert.equal(stop.options.keepalive,true);
});
