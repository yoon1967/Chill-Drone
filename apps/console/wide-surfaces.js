// Large, image-supported planar hypotheses beside the existing bent local surfaces.
// No floor/green special case, gravity assumption, or mutation of the SLAM points.
import {lab, eigen} from './point-surfaces.js';

const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const sub=(a,b)=>a.map((v,i)=>v-b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>Math.hypot(...a);
const colorDistance=(a,b)=>Math.hypot((a[0]-b[0])*.2,a[1]-b[1],a[2]-b[2]);
const mean=values=>[0,1,2].map(a=>values.reduce((s,v)=>s+v[a],0)/values.length);
const residual=(p,plane)=>Math.abs(dot(p,plane.normal)+plane.offset);
const finite3=p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite);
const MAX_PLANES=4, MAX_SEEDS=8, TRIALS=144, MIN_SUPPORT=12;
const PATCH_POINTS=12, PATCH_DEPTH=9, PATCH_LIMIT=192, PATCH_VISITS=767;

function sameColor(pixel,color,tolerance) {
  if (colorDistance(pixel,color)>tolerance) return false;
  const a=Math.hypot(color[1],color[2]),b=Math.hypot(pixel[1],pixel[2]);
  // Bright neutral furniture must not become part of a chromatic carpet mask.
  return a<8 || (b>=Math.max(4,a*.3) && (pixel[1]*color[1]+pixel[2]*color[2])/(a*b)>.72);
}

export function planeHull(points) {
  const sorted=points.map(p=>[...p]).sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
  const unique=sorted.filter((p,i)=>!i||p[0]!==sorted[i-1][0]||p[1]!==sorted[i-1][1]);
  const turn=(a,b,c)=>(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  const half=items=>{const h=[];for(const p of items){while(h.length>1&&turn(h.at(-2),h.at(-1),p)<=0)h.pop();h.push(p);}return h;};
  if(unique.length<3)return [];
  return [...half(unique).slice(0,-1),...half([...unique].reverse()).slice(0,-1)];
}
function inside(p,hull) {
  return hull.every((a,i)=>{const b=hull[(i+1)%hull.length];return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])>=-1e-10;});
}

// Input scene xyz uses a display rotation; extrinsics are in the native world frame.
export function projectToView(xyz,camera,displayRotation) {
  if (!finite3(xyz)||!Array.isArray(displayRotation)||displayRotation.length!==3||
      !displayRotation.every(finite3)||!Array.isArray(camera.cam_from_world_rotation_raw)||
      !camera.cam_from_world_rotation_raw.every(finite3)||!finite3(camera.cam_from_world_translation_raw)||
      !camera.params?.slice(0,4).every(Number.isFinite)||camera.params.length<4) return null;
  const native=[0,1,2].map(i=>displayRotation.reduce((s,row,j)=>s+row[i]*xyz[j],0));
  const q=camera.cam_from_world_rotation_raw.map((row,i)=>dot(row,native)+camera.cam_from_world_translation_raw[i]);
  if(q[2]<=1e-8)return null;
  let x=q[0]/q[2],y=q[1]/q[2];
  const [fx,fy,cx,cy,k1=0,k2=0,p1=0,p2=0]=camera.params,r2=x*x+y*y,radial=1+k1*r2+k2*r2*r2;
  const u=fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+cx;
  const v=fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+cy;
  return Number.isFinite(u)&&Number.isFinite(v)&&u>=0&&v>=0&&u<camera.width&&v<camera.height ? [u/camera.width,v/camera.height] : null;
}

function sourceNodes(points,grid,observations,threshold,frameKeys) {
  const allowed=new Set(grid.cells.filter(c=>c.support>=threshold).flatMap(c=>c.sourceIds||[]));
  const frames=observations?.schema_version==='point-observations/1.0' ? observations.frame_index_by_point : null;
  if(!frames)return [];
  let nodes=[];
  for(const p of points) {
    const observed=frames[p[0]];
    if(!allowed.has(p[0])||!p.slice(1,7).every(Number.isFinite)||p.slice(4,7).some(v=>v<0||v>255)||
        !Array.isArray(observed)||observed.some(i=>!Number.isInteger(i)||i<0||i>=grid.cameraCount)||new Set(observed).size<2)continue;
    nodes.push({id:p[0],xyz:p.slice(1,4),color:lab(p.slice(4,7)),frames:new Set(observed.map(i=>frameKeys[i]??i))});
  }
  // Bound all plane fits independently of the number of raw source points in a merged cell.
  nodes.sort((a,b)=>a.id-b.id);
  if(nodes.length>6000)nodes=Array.from({length:6000},(_,i)=>nodes[Math.floor(i*nodes.length/6000)]);
  return nodes;
}

function fitPlane(pool,epsilon,previous=null) {
  if(pool.length<MIN_SUPPORT)return null;
  let best=[];
  if(previous) best=pool.filter(n=>residual(n.xyz,previous)<epsilon);
  // Reuse a persistent plane while it continues to explain most of its color pool.
  if(!previous || best.length<Math.max(MIN_SUPPORT,previous.support.length*.75)) {
    let state=42;
    const random=()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state/4294967296;};
    for(let t=0;t<TRIALS;t++) {
      const [a,b,c]=[0,1,2].map(()=>pool[Math.floor(random()*pool.length)].xyz);
      const n=cross(sub(b,a),sub(c,a)),d=norm(n);
      if(d<1e-12)continue;
      const plane={normal:n.map(v=>v/d),offset:-dot(n,a)/d};
      const inliers=pool.filter(p=>residual(p.xyz,plane)<epsilon);
      if(inliers.length>best.length)best=inliers;
    }
  }
  if(best.length<MIN_SUPPORT || best.length<pool.length*.18)return null;
  let fit;
  for(let step=0;step<2;step++) {
    fit=eigen(best.map(n=>n.xyz));
    if(fit.values[1]<fit.values[2]*.025)return null;
    const plane={normal:fit.axes[0],offset:-dot(fit.axes[0],fit.center)};
    best=pool.filter(p=>residual(p.xyz,plane)<epsilon);
    if(best.length<MIN_SUPPORT)return null;
  }
  const color=mean(best.map(n=>n.color));
  let plane={normal:fit.axes[0],offset:-dot(fit.axes[0],fit.center),center:fit.center,u:fit.axes[2],v:fit.axes[1],color,support:best};
  // Keep the previous coordinate basis until its accumulated geometric error is material.
  if(previous && best.every(n=>residual(n.xyz,previous)<epsilon) && Math.abs(dot(previous.normal,plane.normal))>.995)
    plane={...previous,color,support:best};
  return plane;
}

async function browserImage(url) {
  const response=await fetch(url,{signal:AbortSignal.timeout(5000)});
  if(!response.ok)throw new Error(`frame ${response.status}`);
  const bitmap=await createImageBitmap(await response.blob());
  const width=Math.min(240,bitmap.width),height=Math.max(1,Math.round(bitmap.height*width/bitmap.width));
  const canvas=new OffscreenCanvas(width,height),ctx=canvas.getContext('2d',{willReadFrequently:true});
  ctx.drawImage(bitmap,0,0,width,height);bitmap.close();
  return {width,height,data:ctx.getImageData(0,0,width,height).data};
}

function imageLabs(image) {
  const colors=new Float32Array(image.width*image.height*3);
  for(let i=0;i<image.width*image.height;i++)colors.set(lab(Array.from(image.data.slice(i*4,i*4+3))),i*3);
  return {...image,colors};
}

function connectedMask(image,plane,camera,rotation,tolerance) {
  const {width,height,colors}=image,labels=new Int32Array(width*height),queue=new Int32Array(width*height),sizes=[0];
  const allowed=new Uint8Array(width*height);
  for(let i=0;i<allowed.length;i++)allowed[i]=sameColor(colors.subarray(i*3,i*3+3),plane.color,tolerance);
  let label=0;
  for(let i=0;i<allowed.length;i++)if(allowed[i]&&!labels[i]) {
    label++;let head=0,tail=1;queue[0]=i;labels[i]=label;
    while(head<tail) {
      const p=queue[head++],x=p%width;
      for(const q of [x>0?p-1:-1,x+1<width?p+1:-1,p-width,p+width])
        if(q>=0&&q<allowed.length&&allowed[q]&&!labels[q]){labels[q]=label;queue[tail++]=q;}
    }
    sizes[label]=tail;
  }
  const seeds=new Map();
  for(const node of plane.support) {
    if(!node.frames.has(camera.frame_id))continue;
    const uv=projectToView(node.xyz,camera,rotation);if(!uv)continue;
    const x=Math.min(width-1,Math.floor(uv[0]*width)),y=Math.min(height-1,Math.floor(uv[1]*height));
    const id=labels[y*width+x];if(id)seeds.set(id,(seeds.get(id)||0)+1);
  }
  const accepted=new Set([...seeds].filter(([id,count])=>count>=3&&sizes[id]>=12).map(([id])=>id));
  const at=xyz=>{const uv=projectToView(xyz,camera,rotation);return !!uv&&accepted.has(labels[Math.floor(uv[1]*height)*width+Math.floor(uv[0]*width)]);};
  return {camera,at,covers(corners){
    if(!corners.every(at))return false;
    const polygon=corners.map(p=>projectToView(p,camera,rotation).map((v,a)=>v*(a?height:width)));
    const signed=polygon.reduce((s,p,i)=>{const q=polygon[(i+1)%4];return s+p[0]*q[1]-q[0]*p[1];},0),sign=Math.sign(signed);
    if(!sign||Math.abs(signed)<2)return false;
    const low=[0,1].map(a=>Math.max(0,Math.floor(Math.min(...polygon.map(p=>p[a])))));
    const high=[0,1].map(a=>Math.min(a?height:width,Math.ceil(Math.max(...polygon.map(p=>p[a])))));
    let tested=0;
    // Check every low-resolution interior pixel, so a large patch cannot jump over a color hole.
    for(let y=low[1];y<high[1];y++)for(let x=low[0];x<high[0];x++) {
      if(!polygon.every((p,i)=>{const q=polygon[(i+1)%4];return sign*((q[0]-p[0])*(y+.5-p[1])-(q[1]-p[1])*(x+.5-p[0]))>=0;}))continue;
      tested++;if(!accepted.has(labels[y*width+x]))return false;
    }
    return tested>=2;
  }};
}

function featureMesh(plane,masks,epsilon,colorThreshold,previous=null) {
  const projected=plane.support.map(n=>({...n,uv:[dot(sub(n.xyz,plane.center),plane.u),dot(sub(n.xyz,plane.center),plane.v)]}));
  const uv=projected.map(n=>n.uv),hull=planeHull(uv);
  if(hull.length<3||masks.length<2)return null;
  const span=Math.max(...[0,1].map(a=>Math.max(...uv.map(p=>p[a]))-Math.min(...uv.map(p=>p[a])))),tiny=span*1e-9;
  if(!(span>0))return null;
  // Every boundary is anchored to an observed feature's projected coordinate, never a uniform step.
  const axes=[0,1].map(a=>{
    const values=projected.map(n=>({value:n.uv[a],key:`${a}:${n.id}`,id:n.id})).sort((p,q)=>p.value-q.value||p.id-q.id);
    return values.filter((p,i)=>!i||p.value-values[i-1].value>tiny);
  });
  if(axes.some(a=>a.length<2))return null;
  const anchors=new Map(axes.flat().map(a=>[a.key,a])),layout=new Map();
  const world=(x,y)=>plane.center.map((v,a)=>v+x*plane.u[a]+y*plane.v[a]);
  const model={synthetic:true,algorithm:'feature-rectangles-v1',positions:[],nodeKeys:[],quads:[],quadAreas:[],quadBounds:[],
    triangles:[],strongTriangles:[],weakTriangles:[],edges:[],strongEdges:[],weakEdges:[],boundary:[],strongBoundary:[],weakBoundary:[],groups:1,lineGroups:0};
  const vertices=new Map(),vertexUV=[],outlines=[],rectangles=[];
  const vertex=(u,v)=>{
    const key=`plane:${u.key}|${v.key}`;
    if(!vertices.has(key)){
      vertices.set(key,model.nodeKeys.length);model.nodeKeys.push(key);model.positions.push(...world(u.value,v.value));
      vertexUV.push({u,v});
    }
    return vertices.get(key);
  };
  const root={lo:axes.map(a=>a[0]),hi:axes.map(a=>a.at(-1)),nodes:projected,depth:0,path:'r'};
  const stack=[root];let visits=0;
  while(stack.length&&visits++<PATCH_VISITS&&rectangles.length<PATCH_LIMIT) {
    const box=stack.pop(),widths=box.lo.map((a,i)=>box.hi[i].value-a.value);
    if(widths.some(w=>w<=tiny))continue;
    const corners=[[box.lo[0],box.lo[1]],[box.hi[0],box.lo[1]],[box.hi[0],box.hi[1]],[box.lo[0],box.hi[1]]];
    const inHull=corners.every(p=>inside(p.map(a=>a.value),hull));
    const confirmations=inHull?masks.filter(mask=>mask.covers(corners.map(p=>world(...p.map(a=>a.value))))).length:0;
    const color=box.nodes.length?mean(box.nodes.map(n=>n.color)):plane.color;
    const variation=box.nodes.length?Math.sqrt(box.nodes.reduce((s,n)=>s+colorDistance(n.color,color)**2,0)/box.nodes.length):0;
    const error=box.nodes.length?Math.sqrt(box.nodes.reduce((s,n)=>s+residual(n.xyz,plane)**2,0)/box.nodes.length):0;
    const refine=!confirmations||box.nodes.length>PATCH_POINTS||Math.max(...widths)>Math.min(...widths)*3||
      variation>colorThreshold*.55||error>epsilon*.65;
    let split=null;
    if(refine&&box.depth<PATCH_DEPTH&&rectangles.length+stack.length+2<=PATCH_LIMIT) {
      const order=[0,1].sort((a,b)=>widths[b]-widths[a]);
      for(const axis of order) {
        const candidates=axes[axis].filter(a=>a.value>box.lo[axis].value+tiny&&a.value<box.hi[axis].value-tiny);
        if(!candidates.length)continue;
        const old=previous?.layout?.get(box.path),retained=old?.axis===axis?anchors.get(old.key):null;
        if(retained&&retained.value>box.lo[axis].value+widths[axis]*.05&&retained.value<box.hi[axis].value-widths[axis]*.05) {
          split={axis,anchor:retained};break;
        }
        const local=[...box.nodes].sort((a,b)=>a.uv[axis]-b.uv[axis]);
        let j=0,best=null,score=Infinity;
        for(const anchor of candidates) {
          while(j<local.length&&local[j].uv[axis]<=anchor.value)j++;
          const spatial=Math.abs((anchor.value-box.lo[axis].value)/widths[axis]-.5);
          const balance=local.length?Math.abs(j/local.length-.5):spatial;
          const cost=spatial*.45+balance*.55;
          if(cost<score){score=cost;best=anchor;}
        }
        if(best){split={axis,anchor:best};break;}
      }
    }
    if(split) {
      const {axis,anchor}=split,leftHi=[...box.hi],rightLo=[...box.lo];leftHi[axis]=anchor;rightLo[axis]=anchor;
      const left=[],right=[];for(const n of box.nodes)(n.uv[axis]<=anchor.value?left:right).push(n);
      layout.set(box.path,{axis,key:anchor.key});
      stack.push({lo:rightLo,hi:box.hi,nodes:right,depth:box.depth+1,path:box.path+'1'});
      stack.push({lo:box.lo,hi:leftHi,nodes:left,depth:box.depth+1,path:box.path+'0'});
      continue;
    }
    if(!confirmations||Math.max(...widths)>Math.min(...widths)*6)continue;
    const ids=corners.map(p=>vertex(...p)),strong=confirmations>=2;
    const triangles=[ids[0],ids[1],ids[2],ids[0],ids[2],ids[3]];
    model.quads.push(...ids);model.quadAreas.push(widths[0]*widths[1]);
    model.quadBounds.push([box.lo[0].value,box.lo[1].value,box.hi[0].value,box.hi[1].value]);
    model.triangles.push(...triangles);model[strong?'strongTriangles':'weakTriangles'].push(...triangles);
    for(const [a,b] of [[ids[0],ids[1]],[ids[1],ids[2]],[ids[2],ids[3]],[ids[3],ids[0]]])outlines.push({a,b,strong});
    rectangles.push(box.path);
  }
  if(!rectangles.length)return null;
  // Split outlines at shared T vertices: a big rectangle beside small ones must not gain a false outer border.
  const lines=[new Map(),new Map()],edges=new Map();
  vertexUV.forEach((p,id)=>{for(const a of [0,1]){const key=a?p.u.key:p.v.key;if(!lines[a].has(key))lines[a].set(key,[]);lines[a].get(key).push({id,value:a?p.v.value:p.u.value});}});
  for(const axis of lines)for(const values of axis.values())values.sort((a,b)=>a.value-b.value);
  for(const edge of outlines) {
    const p=vertexUV[edge.a],q=vertexUV[edge.b],axis=p.v.key===q.v.key?0:1;
    const values=lines[axis].get(axis?p.u.key:p.v.key),lo=Math.min(axis?p.v.value:p.u.value,axis?q.v.value:q.u.value),hi=Math.max(axis?p.v.value:p.u.value,axis?q.v.value:q.u.value);
    let left=0,right=values.length;
    while(left<right){const mid=(left+right)>>1;if(values[mid].value<lo-tiny)left=mid+1;else right=mid;}
    for(let i=left;i+1<values.length&&values[i+1].value<=hi+tiny;i++) {
      const a=values[i].id,b=values[i+1].id,key=a<b?`${a},${b}`:`${b},${a}`;
      const e=edges.get(key)||{ids:[a,b],count:0,strong:false};e.count++;e.strong||=edge.strong;edges.set(key,e);
    }
  }
  for(const e of edges.values()) {
    model.edges.push(...e.ids);model[e.strong?'strongEdges':'weakEdges'].push(...e.ids);
    if(e.count===1){model.boundary.push(...e.ids);model[e.strong?'strongBoundary':'weakBoundary'].push(...e.ids);}
  }
  return {model,layout,visits};
}

export class WidePointSurfaces {
  constructor({loadImage=browserImage}={}) {this.loadImage=loadImage;this.images=new Map();this.patches=new Map();this.planes=[];this.scope=null;this.signature=null;this.nextId=0;}
  async image(camera,context) {
    const key=JSON.stringify([context.source,context.session,camera.frame_id]);
    if(!this.images.has(key)) {
      const prefix=['saved','live'].includes(context.source)?`/api/${context.source}/frame/`:'/api/frame/';
      const params=new URLSearchParams();
      if(context.source!=='saved'&&context.revision!=null)params.set('revision',String(context.revision).padStart(6,'0'));
      if(context.source==='live'&&context.session)params.set('session',context.session);
      const suffix=params.size?`?${params}`:'';
      const promise=this.loadImage(`${prefix}${encodeURIComponent(camera.frame_id)}${suffix}`).then(imageLabs).catch(()=>null);
      this.images.set(key,promise);
      if(this.images.size>12)this.images.delete(this.images.keys().next().value);
    }
    return this.images.get(key);
  }
  async update(points,grid,observations,settings={},context={}) {
    const start=performance.now(),scope=JSON.stringify([context.source,context.session,context.segment,context.poseSource]);
    const reset=scope!==this.scope||context.revision<this.revision;
    if(reset){this.planes=[];this.signature=null;this.images.clear();this.nextId=0;}
    this.scope=scope;this.revision=context.revision;
    const options={threshold:3,colorThreshold:30,spacing:0,wideSurfaces:true,...settings};
    const shapeSignature=JSON.stringify([grid.interval,options.threshold,options.colorThreshold]);
    if(shapeSignature!==this.shapeSignature)this.planes=[];
    this.shapeSignature=shapeSignature;
    const signature=JSON.stringify([points,grid.cells.map(c=>[c.sourceIds,c.support]),observations,context.cameras,context.displayRotation,grid.interval,options.threshold,options.colorThreshold,options.spacing,options.wideSurfaces]);
    const stats={widePlanes:0,wideFaces:0,wideQuads:0,wideWeakFaces:0,wideRetainedFaces:0,wideAddedFaces:0,wideRemovedFaces:0,wideReused:0,wideImageViews:0,wideAnalysisMs:0};
    if(!reset&&signature===this.signature) {
      stats.widePlanes=this.patches.size;stats.wideReused=this.patches.size;
      for(const m of this.patches.values()){stats.wideFaces+=m.triangles.length/3;stats.wideQuads+=m.quads.length/4;stats.wideWeakFaces+=m.weakTriangles.length/3;}
      stats.wideRetainedFaces=stats.wideFaces;stats.wideImageViews=this.imageViews||0;stats.wideAnalysisMs=performance.now()-start;
      return {patches:[],removed:[],stats};
    }
    const old=this.patches,next=new Map(),patches=[],planes=[],usedViews=new Set();
    if(options.wideSurfaces&&context.cameras?.length&&context.displayRotation) {
      const nodes=sourceNodes(points,grid,observations,options.threshold,context.frameKeys||[]),buckets=new Map();
      for(const n of nodes){const key=n.color.slice(1).map(v=>Math.floor(v/12)).join(',');if(!buckets.has(key))buckets.set(key,[]);buckets.get(key).push(n);}
      const seeds=[...buckets.values()].filter(p=>p.length>=MIN_SUPPORT).sort((a,b)=>b.length*(1+Math.hypot(...b[0].color.slice(1))/30)-a.length*(1+Math.hypot(...a[0].color.slice(1))/30));
      const chosen=[];
      for(const pool of seeds){const color=mean(pool.map(n=>n.color));if(chosen.every(c=>colorDistance(c,color)>options.colorThreshold*.45))chosen.push(color);if(chosen.length===MAX_SEEDS)break;}
      const epsilon=Math.max(options.spacing||.001,(grid.interval||0)*.1)*2.2;
      for(const color of chosen) {
        let pool=nodes.filter(n=>sameColor(n.color,color,options.colorThreshold));
        for(let extraction=0;extraction<2&&planes.length<MAX_PLANES;extraction++) {
          const previous=this.planes.find(p=>colorDistance(p.color,color)<options.colorThreshold);
          const plane=fitPlane(pool,epsilon,previous);
          if(!plane)break;
          const selected=new Set(plane.support.map(n=>n.id));pool=pool.filter(n=>!selected.has(n.id));
          if(planes.some(p=>Math.abs(dot(p.normal,plane.normal))>.98&&residual(plane.center,p)<epsilon*2))continue;
          const matched=this.planes.find(p=>Math.abs(dot(p.normal,plane.normal))>.98&&residual(plane.center,p)<epsilon*2&&colorDistance(p.color,plane.color)<options.colorThreshold);
          plane.key=matched?.key||`wide:${this.nextId++}`;
          const counts=new Map();for(const n of plane.support)for(const f of n.frames)counts.set(f,(counts.get(f)||0)+1);
          const cameras=context.cameras.filter(c=>counts.get(c.frame_id)>=4).sort((a,b)=>counts.get(b.frame_id)-counts.get(a.frame_id)).slice(0,3);
          const images=await Promise.all(cameras.map(c=>this.image(c,context)));
          const masks=cameras.flatMap((c,i)=>images[i]?[connectedMask(images[i],plane,c,context.displayRotation,Math.min(24,Math.max(10,options.colorThreshold*.8)))]:[]);
          const result=featureMesh(plane,masks,epsilon,options.colorThreshold,matched);
          if(!result)continue;
          for(const mask of masks)usedViews.add(mask.camera.frame_id);
          plane.layout=result.layout;planes.push(plane);next.set(plane.key,result.model);
          const before=old.get(plane.key),unchanged=before&&JSON.stringify(before)===JSON.stringify(result.model);
          if(unchanged){next.set(plane.key,before);stats.wideReused++;}
          else {
            const sameTopology=before&&['positions','nodeKeys','triangles','edges','boundary'].every(k=>JSON.stringify(before[k])===JSON.stringify(result.model[k]));
            patches.push({key:plane.key,kind:sameTopology?'evidence':'topology',model:result.model});
          }
        }
        if(planes.length===MAX_PLANES)break;
      }
    }
    const keys=m=>new Set(Array.from({length:m?.triangles.length/3||0},(_,i)=>m.triangles.slice(i*3,i*3+3).map(v=>m.nodeKeys[v]).sort().join('|')));
    for(const [key,m] of next) {
      stats.wideFaces+=m.triangles.length/3;stats.wideQuads+=m.quads.length/4;stats.wideWeakFaces+=m.weakTriangles.length/3;
      const before=keys(old.get(key)),after=keys(m);for(const f of after)if(before.has(f))stats.wideRetainedFaces++;else stats.wideAddedFaces++;
      for(const f of before)if(!after.has(f))stats.wideRemovedFaces++;
    }
    const removed=[...old.keys()].filter(k=>!next.has(k));for(const key of removed)stats.wideRemovedFaces+=old.get(key).triangles.length/3;
    this.patches=next;this.planes=planes;this.signature=signature;
    this.imageViews=usedViews.size;stats.wideImageViews=this.imageViews;
    stats.widePlanes=next.size;stats.wideAnalysisMs=performance.now()-start;
    return {patches,removed,stats};
  }
}
