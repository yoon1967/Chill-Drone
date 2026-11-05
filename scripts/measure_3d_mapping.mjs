// Browser half of measure_3d_mapping.py. Uses the actual console and renderer.
import {createRequire} from 'node:module';
import {writeFile} from 'node:fs/promises';

const [url, playwrightModule, output] = process.argv.slice(2);
const displayOnly = process.argv[5] === 'display-only';
const {chromium} = createRequire(import.meta.url)(playwrightModule);
const browser = await chromium.launch({channel:'msedge',headless:true,
  args:['--disable-background-timer-throttling','--disable-renderer-backgrounding']});
const page = await browser.newPage({viewport:{width:1440,height:1000}});
const errors=[];
page.on('pageerror',error=>errors.push(error.message));
function inject(source, from, to) {
  if (!source.includes(from)) throw Error('Unsupported console instrumentation anchor: '+from.slice(0,70));
  return source.replace(from,to);
}
const distribution=values=>{
  const a=values.filter(Number.isFinite).sort((x,y)=>x-y);
  return {n:a.length,mean:a.length?a.reduce((s,x)=>s+x,0)/a.length:null,
    p50:a.length?a[Math.floor((a.length-1)/2)]:null,p95:a.length?a[Math.ceil(a.length*.95)-1]:null,maximum:a.at(-1)??null};
};
try {
  await page.addInitScript(()=>{window.__mappingBench={frames:[],renders:[],gpu:[],mode:'full',freeze:false,lastMesh:null,lastSnapshot:null};});
  await page.route('**/progressive-map.js',async route=>{
    const response=await route.fetch();
    let source=await response.text();
    const pointFusion = source.includes('function updatePoints(');
    source=inject(source,"  const observer = new ResizeObserver(() => {",`
  const bench = window.__mappingBench;
  const gl = renderer.getContext(), gpuExtension = gl.getExtension('EXT_disjoint_timer_query_webgl2');
  const gpuQueries = [];
  const debugRenderer = gl.getExtension('WEBGL_debug_renderer_info');
  if (bench && prefix === 'live-progressive') {
    bench.renderer = debugRenderer ? gl.getParameter(debugRenderer.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
    bench.gpuTimerSupported = Boolean(gpuExtension);
    bench.applyVariant = (variant, sample=0) => {
      const started = performance.now();
      const text = JSON.stringify(bench.lastSnapshot);
      const parsedAt = performance.now(), value=JSON.parse(text), parseMs=performance.now()-parsedAt;
      const size = item => new TextEncoder().encode(JSON.stringify(item)).length;
      const pointMode=value.rescue?.render_mode==='points';
      if (variant === 'no_surface' && value.rescue) {
        value.rescue.mesh = null;
        if(pointMode){value.points=[];value.rescue.point_labels=[];}
      }
      if (variant === 'no_people' && value.rescue) {
        value.rescue.people = [];
        if(pointMode){const labels=value.rescue.point_labels;value.points=value.points.filter((p,i)=>labels[i]!==1);value.rescue.point_labels=labels.filter(x=>x!==1);}
      }
      if (variant === 'no_compat_points' && !pointMode) value.points = [];
      if (variant === 'no_path') value.cameras = [];
      if (value.rescue) value.rescue.mesh_revision = 1000000 + sample;
      const payloadBytes=size(value), applyAt=performance.now();
      ${pointFusion ? 'updatePoints(value,true);' : `const position = new Float32Array(value.points.length*3), color = new Float32Array(position.length);
      value.points.forEach((p,i)=>{position.set(p.slice(0,3),i*3);color.set(p.slice(3,6).map(c=>c/255),i*3);});
      geometry.dispose();geometry.setAttribute('position',new THREE.BufferAttribute(position,3));
      geometry.setAttribute('color',new THREE.BufferAttribute(color,3));geometry.computeBoundingSphere();`}
      const pointsDone=performance.now();rescue.update(value.rescue);const rescueDone=performance.now();
      route.geometry.dispose();route.geometry=new THREE.BufferGeometry().setFromPoints(value.cameras.map(p=>new THREE.Vector3(...p)));
      bench.mode=variant;
      return {variant,parseMs,payloadBytes,pointsMs:pointsDone-applyAt,rescueMs:rescueDone-pointsDone,applyMs:performance.now()-applyAt,
        sourceBytes:size(bench.lastSnapshot),meshBytes:size(bench.lastSnapshot.rescue?.mesh||null),
        peopleBytes:size(bench.lastSnapshot.rescue?.people||[]),pointsBytes:size(bench.lastSnapshot.points),
        pathBytes:size(bench.lastSnapshot.cameras),cloneAndPreparationMs:performance.now()-started};
    };
  }
  const observer = new ResizeObserver(() => {`);
    source=inject(source,'      controls.update(); renderer.render(scene, camera);',`
      const b = prefix === 'live-progressive' ? window.__mappingBench : null;
      if (b?.freeze) {
        const mode=b.mode;
        rescue.group.visible=mode!=='points_only'&&mode!=='empty_3d';
        if (rescue.group.children[0]) rescue.group.children[0].visible=mode!=='no_surface';
        if (rescue.group.children[1]) rescue.group.children[1].visible=mode!=='no_people';
        cloud.visible=mode==='points_only'||(b.lastSnapshot?.rescue?.render_mode==='points'&&!['empty_3d','no_surface'].includes(mode));route.visible=mode!=='no_path'&&mode!=='empty_3d';
        cameraMarker.visible=mode!=='no_path'&&mode!=='empty_3d';
        $('progressive-frame').hidden=mode==='no_image';
      }
      if (b && gpuExtension) {
        while(gpuQueries.length && gl.getQueryParameter(gpuQueries[0].query,gl.QUERY_RESULT_AVAILABLE)) {
          const q=gpuQueries.shift();
          if(!gl.getParameter(gpuExtension.GPU_DISJOINT_EXT)) b.gpu.push({...q,query:undefined,ms:gl.getQueryParameter(q.query,gl.QUERY_RESULT)/1e6});
          gl.deleteQuery(q.query);
        }
      }
      const query = b&&gpuExtension ? gl.createQuery() : null;
      if(query) gl.beginQuery(gpuExtension.TIME_ELAPSED_EXT,query);
      const renderStart=performance.now();controls.update();renderer.render(scene,camera);const renderEnd=performance.now();
      if(query){gl.endQuery(gpuExtension.TIME_ELAPSED_EXT);gpuQueries.push({query,at:renderStart,mode:b.mode});}
      if(b){
        b.renders.push({at:renderStart,cpuMs:renderEnd-renderStart,mode:b.mode,calls:renderer.info.render.calls,triangles:renderer.info.render.triangles});
        if(b.pending){const row=b.pending;b.pending=null;requestAnimationFrame(now=>{row.paintAt=now;});}
      }`);
    source=inject(source,'    if (disposed || !active) return;','    if (disposed || !active || (prefix === \'live-progressive\' && window.__mappingBench?.freeze)) return;');
    source=inject(source,'      const response = await fetch(`/api/import/${endpoint}`,','      const benchFetchStart = performance.now();\n      const response = await fetch(`/api/import/${endpoint}`,');
    source=inject(source,'      const value = await response.json();',`
      const responseText=await response.text(), benchReceived=performance.now();
      const benchParseStart=performance.now(), value=JSON.parse(responseText), benchParsed=performance.now();`);
    source=inject(source,'      await image.decode();','      const benchImageStart=performance.now(); await image.decode(); const benchImageEnd=performance.now();');
    const pointAnchor=pointFusion?'      updatePoints(value);':'      const position = new Float32Array(value.points.length * 3), color = new Float32Array(position.length);';
    source=inject(source,pointAnchor,'      const benchApplyStart=performance.now();\n'+pointAnchor);
    const rescueAnchor=pointFusion?"      rescue.update(value.rescue); cloud.visible = !value.rescue || value.rescue.render_mode === 'points';":'      rescue.update(value.rescue); cloud.visible = !value.rescue;';
    source=inject(source,rescueAnchor,'      const benchPointEnd=performance.now(); rescue.update(value.rescue); const benchRescueEnd=performance.now();'+(pointFusion?" cloud.visible = !value.rescue || value.rescue.render_mode === 'points';":' cloud.visible = !value.rescue;'));
    source=inject(source,'      if (first && value.points.length) { first = false; fit(); }',`
      if (first && value.points.length) { first = false; fit(); }
      if(prefix === 'live-progressive' && value._benchmark){
        const b=window.__mappingBench;
        const row={revision:value.revision,frame:value.frame_number,tracking:value.tracking,stamp:value.timestamp_s,
          fetchWaitMs:benchReceived-benchFetchStart,parseMs:benchParsed-benchParseStart,
          imageDecodeMs:benchImageEnd-benchImageStart,pointsApplyMs:benchPointEnd-benchApplyStart,
          rescueApplyMs:benchRescueEnd-benchPointEnd,applyMs:performance.now()-benchApplyStart,
          receivedAt:benchReceived,applyAt:performance.now(),payloadBytes:new TextEncoder().encode(responseText).length,
          meshChanged:Boolean(value.rescue?.mesh),...value._benchmark};
        b.frames.push(row);b.pending=row;
        if(value.rescue?.mesh)b.lastMesh=value.rescue.mesh;
        if(value.rescue){b.lastSnapshot=structuredClone(value);b.lastSnapshot.rescue.mesh=b.lastMesh;}
      }`);
    await route.fulfill({response,body:source,headers:{...response.headers(),'cache-control':'no-store'}});
  });
  await page.goto(url+'/#live');
  await page.evaluate(()=>fetch('/benchmark/ready'));
  let finished=false;
  for(let i=0;i<900;i++){
    const state=await (await page.request.get(url+'/benchmark/state')).json();
    if(state.session_done){finished=true;break;}
    await page.waitForTimeout(400);
  }
  if(!finished)throw Error('Mapping completion timeout');
  await page.waitForFunction(()=>Boolean(window.__mappingBench?.lastSnapshot),null,{timeout:30000});
  await page.waitForTimeout(1000);
  const sync=await page.evaluate(async()=>{
    const samples=[];
    for(let i=0;i<12;i++){const a=performance.now();const v=await(await fetch('/benchmark/clock')).json();const z=performance.now();samples.push({offset:v.perf_ms-(a+z)/2,rtt:z-a});}
    return samples.sort((a,b)=>a.rtt-b.rtt)[0];
  });
  const baseline=await page.evaluate(()=>{
    const b=window.__mappingBench;b.freeze=true;
    return {frames:b.frames,renders:b.renders,gpu:b.gpu,renderer:b.renderer,gpuTimerSupported:b.gpuTimerSupported,
      lastSnapshot:b.lastSnapshot?{people:b.lastSnapshot.rescue?.people?.length,voxels:(b.lastSnapshot.rescue?.people||[]).reduce((n,p)=>n+p.voxels.length,0),
        vertices:b.lastSnapshot.rescue?.mesh?.vertices?.length,triangles:b.lastSnapshot.rescue?.mesh?.triangles?.length}:null};
  });
  if(!baseline.lastSnapshot)throw Error('No semantic snapshot reached the actual browser');
  const variants=['full','no_surface','no_people','no_compat_points','no_path','no_image','points_only','empty_3d'];
  const comparisons=[];
  let sample=0;
  for(let repeat=0;repeat<3;repeat++){
    const ordered=repeat===1?[...variants].reverse():repeat===2?[...variants.slice(3),...variants.slice(0,3)]:variants;
    for(const variant of ordered){
      const applies=[];
      for(let i=0;i<8;i++)applies.push(await page.evaluate(({variant,sample})=>window.__mappingBench.applyVariant(variant,sample),{variant,sample:++sample}));
      const startAt=await page.evaluate(()=>performance.now());
      await page.waitForTimeout(1300);
      const samples=await page.evaluate(start=>({renders:window.__mappingBench.renders.filter(r=>r.at>start+200),
        gpu:window.__mappingBench.gpu.filter(r=>r.at>start+200)}),startAt);
      comparisons.push({repeat,variant,applies:applies.slice(2),...samples});
    }
  }
  const frames=baseline.frames.filter(f=>f.paintAt&&f.tracking==='tracking');
  const frameMetric=key=>distribution(frames.map(f=>f[key]));
  const renderRows=baseline.renders.filter(r=>frames.length&&r.at>=frames[0].applyAt&&r.at<=frames.at(-1).paintAt);
  const intervals=renderRows.slice(1).map((r,i)=>r.at-renderRows[i].at);
  const summaries={};
  for(const variant of variants){
    const runs=comparisons.filter(r=>r.variant===variant), renders=runs.flatMap(r=>r.renders),gpu=runs.flatMap(r=>r.gpu),applies=runs.flatMap(r=>r.applies);
    const gaps=runs.flatMap(r=>r.renders.slice(1).map((x,i)=>x.at-r.renders[i].at));
    summaries[variant]={cpuRenderMs:distribution(renders.map(r=>r.cpuMs)),gpuRenderMs:distribution(gpu.map(r=>r.ms)),
      frameIntervalMs:distribution(gaps),calls:distribution(renders.map(r=>r.calls)),triangles:distribution(renders.map(r=>r.triangles)),
      applyMs:distribution(applies.map(r=>r.applyMs)),pointsApplyMs:distribution(applies.map(r=>r.pointsMs)),
      rescueApplyMs:distribution(applies.map(r=>r.rescueMs)),payloadBytes:distribution(applies.map(r=>r.payloadBytes))};
  }
  const design={repeatBlocks:3,variantWindowMs:1300,renderSettleExclusionMs:200,
    applyTrialsPerBlock:8,excludedApplyWarmupsPerBlock:2,comparisonBasis:'same observed snapshot; inference idle'};
  const report={design,viewport:{width:1440,height:1000},renderer:baseline.renderer,gpuTimerSupported:baseline.gpuTimerSupported,
    clockSync:{offsetMs:sync.offset,bestRoundTripMs:sync.rtt,estimatedUncertaintyMs:sync.rtt/2},errors,
    displayedMappedFrames:frames.length,displayedAllFrames:baseline.frames.length,
    updateIntervalMs:distribution(frames.slice(1).map((f,i)=>f.paintAt-frames[i].paintAt)),
    endToEndDecodedToPostRafMs:distribution(frames.map(f=>f.paintAt+sync.offset-f.decoded_ms)),
    parentReadyToPostRafMs:distribution(frames.map(f=>f.paintAt+sync.offset-f.parent_ready_ms)),
    parentReadyToResponseBodyMs:distribution(frames.map(f=>f.receivedAt+sync.offset-f.parent_ready_ms)),
    jsonParseMs:frameMetric('parseMs'),imageFetchDecodeMs:frameMetric('imageDecodeMs'),
    pointsApplyMs:frameMetric('pointsApplyMs'),rescueApplyMs:frameMetric('rescueApplyMs'),applyMs:frameMetric('applyMs'),
    payloadBytes:frameMetric('payloadBytes'),meshChangedPayloadBytes:distribution(frames.filter(f=>f.meshChanged).map(f=>f.payloadBytes)),
    meshReusedPayloadBytes:distribution(frames.filter(f=>!f.meshChanged).map(f=>f.payloadBytes)),
    liveCpuRenderMs:distribution(renderRows.map(r=>r.cpuMs)),liveGpuRenderMs:distribution(baseline.gpu.filter(r=>renderRows.length&&r.at>=renderRows[0].at&&r.at<=renderRows.at(-1).at).map(r=>r.ms)),
    liveFrameIntervalMs:distribution(intervals),lastSnapshot:baseline.lastSnapshot,
    payloadBreakdown:comparisons.find(r=>r.variant==='full').applies[0],comparisons:summaries};
  const selectedReport=displayOnly?{design,viewport:report.viewport,renderer:report.renderer,gpuTimerSupported:report.gpuTimerSupported,
    errors:report.errors,lastSnapshot:report.lastSnapshot,payloadBreakdown:report.payloadBreakdown,comparisons:report.comparisons}:report;
  await writeFile(output,JSON.stringify(selectedReport,null,2));
  await page.request.get(url+'/benchmark/done');
}finally{await browser.close();}
