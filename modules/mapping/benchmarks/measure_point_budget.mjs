// Uses the production progressive-map module and its actual GPU buffers.
import {createRequire} from 'node:module';
import {writeFile} from 'node:fs/promises';
const [url,modulePath,output,budgetText]=process.argv.slice(2);
const budgets=budgetText.split(',').map(Number);
const {chromium}=createRequire(import.meta.url)(modulePath);
const browser=await chromium.launch({channel:'msedge',headless:true,
  args:['--disable-background-timer-throttling','--disable-renderer-backgrounding']});
const page=await browser.newPage({viewport:{width:1440,height:1000}});
const errors=[];page.on('pageerror',error=>{errors.push(error.message);console.error(error.message);});
page.on('console',message=>{if(message.type()==='error')console.error(message.text());});
try {
  await page.route('**/progressive-map.js',async route=>{
    const response=await route.fetch();let source=await response.text();
    const anchor='  const observer = new ResizeObserver(() => {';
    if(!source.includes(anchor))throw Error('renderer instrumentation anchor changed');
    source=source.replace(anchor,`
  const intervals=[], renders=[], gl=renderer.getContext();
  let previous=null, measuring=false;
  const ext=gl.getExtension('WEBGL_debug_renderer_info');
  window.pointBudgetBench={renderer:ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER),
    apply(value) {const start=performance.now();updatePoints(value,true);rescue.update(value.rescue);cloud.visible=true;
      route.geometry.dispose();route.geometry=new THREE.BufferGeometry().setFromPoints(value.cameras.map(p=>new THREE.Vector3(...p)));
      return performance.now()-start;},
    start() {intervals.length=0;renders.length=0;previous=null;measuring=true;},
    stop() {measuring=false;return {intervals:[...intervals],renders:[...renders],points:geometry.drawRange.count,capacity:pointCapacity};},
    fit};
  const observer = new ResizeObserver(() => {`);
    const render=source.match(/(?:controls\.update\(\); )?renderer\.render\(scene, camera\);/)?.[0];
    if(!render)throw Error('render instrumentation anchor changed');
    source=source.replace(render,`{
      const started=performance.now();
      ${render}
      if(measuring){if(previous!==null)intervals.push(started-previous);previous=started;renders.push(performance.now()-started);}
    }`);
    await route.fulfill({response,body:source});
  });
  await page.goto(url);
  await page.waitForFunction(()=>Boolean(window.pointBudgetBench));
  await page.evaluate(()=>{
    const host=document.getElementById('progressive-viewport');
    for(let p=host;p;p=p.parentElement){p.hidden=false;}
    host.style.cssText='position:fixed;left:0;top:0;width:1000px;height:700px;z-index:99999';
  });
  const rows=[];
  for(let repeat=0;repeat<3;repeat++)for(const budget of repeat===1?[...budgets].reverse():budgets){
    const row=await page.evaluate(async({url,budget})=>{
      const response=await fetch(url+'/benchmark/snapshot?budget='+budget);
      const text=await response.text();const value=JSON.parse(text), bench=window.pointBudgetBench;
      bench.apply(value);bench.fit();await new Promise(resolve=>setTimeout(resolve,400));
      const parses=[],applies=[],fetches=[],paints=[];bench.start();
      // Refresh the full point payload at 10 Hz, including local HTTP transfer.
      const zero=performance.now();
      for(let i=0;i<30;i++){
        await new Promise(resolve=>setTimeout(resolve,Math.max(0,zero+i*100-performance.now())));
        const begin=performance.now();const serialized=await(await fetch(url+'/benchmark/snapshot?budget='+budget)).text();
        const received=performance.now();const data=JSON.parse(serialized);const parsed=performance.now();
        data.rescue.point_revision=i+2;
        const apply=bench.apply(data);const done=performance.now();
        await new Promise(requestAnimationFrame);paints.push(performance.now()-begin);
        parses.push(parsed-received);applies.push(apply);fetches.push(received-begin);
      }
      return {budget,bytes:new TextEncoder().encode(text).length,parses,applies,fetches,paints,...bench.stop()};
    },{url,budget});
    row.repeat=repeat;rows.push(row);
    console.log(JSON.stringify({phase:'browser',budget,repeat,frames:row.intervals.length}));
  }
  const stats=values=>{const a=values.sort((a,b)=>a-b);return {n:a.length,mean:a.reduce((s,x)=>s+x,0)/a.length,p95:a[Math.ceil(a.length*.95)-1],maximum:a.at(-1)};};
  const results=budgets.map(budget=>{const a=rows.filter(r=>r.budget===budget),all=key=>a.flatMap(r=>r[key]);
    const interval=stats(all('intervals'));
    return {budget,displayed_points:a[0].points,bytes:a[0].bytes,fps:1000/interval.mean,frame_ms:interval,render_ms:stats(all('renders')),
      parse_ms:stats(all('parses')),apply_ms:stats(all('applies')),fetch_ms:stats(all('fetches')),paint_ms:stats(all('paints'))};});
  const renderer=await page.evaluate(()=>window.pointBudgetBench.renderer);
  await writeFile(output,JSON.stringify({renderer,viewport:{width:1440,height:1000},canvas:{width:1000,height:700},
    refresh_hz:10,repeats:3,results,errors},null,2));
} finally {await browser.close();}
