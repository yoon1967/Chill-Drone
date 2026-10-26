// Display-only color regions and locally bent surface hypotheses. Never edits the source map.
const NEIGHBORS = 16, BUCKET_SAMPLES = 12;
const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const length = v => Math.hypot(...v);
const distance = (a, b) => length(sub(a, b));
const normalize = v => { const n = length(v); return n > 1e-15 ? v.map(x => x/n) : null; };
const median = a => [...a].sort((x, y) => x-y)[Math.floor(a.length/2)] || 0;
const pairKey = (a, b) => a < b ? `${a},${b}` : `${b},${a}`;
function cached(map,key,create) {
  if (!map) return create();
  if (map.has(key)) { map.hits=(map.hits||0)+1; return map.get(key); }
  const value=create(); map.set(key,value);
  if (map.size>24000) for (let i=0;i<8000;i++) map.delete(map.keys().next().value);
  return value;
}

// sRGB -> CIELAB (D65). Delta E 76 gives an inexpensive, perceptual color distance.
export function lab(rgb) {
  const [r, g, b] = rgb.map(v => { const c = v/255; return c <= .04045 ? c/12.92 : ((c+.055)/1.055)**2.4; });
  const f = t => t > 216/24389 ? Math.cbrt(t) : (24389/27*t+16)/116;
  const x = f((.4124564*r+.3575761*g+.1804375*b)/.95047);
  const y = f(.2126729*r+.7151522*g+.072175*b);
  const z = f((.0193339*r+.119192*g+.9503041*b)/1.08883);
  return [116*y-16, 500*(x-y), 200*(y-z)];
}

export function eigen(points) {
  const center = [0, 1, 2].map(a => points.reduce((s, p) => s+p[a], 0)/points.length);
  const matrix = Array.from({length:3}, () => [0, 0, 0]);
  for (const point of points) {
    const delta = sub(point, center);
    for (let a=0; a<3; a++) for (let b=0; b<3; b++) matrix[a][b] += delta[a]*delta[b];
  }
  const vectors = [[1,0,0], [0,1,0], [0,0,1]];
  for (let step=0; step<18; step++) {
    let a=0, b=1;
    for (const [i,j] of [[0,2], [1,2]]) if (Math.abs(matrix[i][j]) > Math.abs(matrix[a][b])) { a=i; b=j; }
    if (Math.abs(matrix[a][b]) < 1e-14*Math.max(matrix[0][0]+matrix[1][1]+matrix[2][2], 1e-20)) break;
    const angle=.5*Math.atan2(2*matrix[a][b], matrix[b][b]-matrix[a][a]);
    const c=Math.cos(angle), s=Math.sin(angle), aa=matrix[a][a], bb=matrix[b][b], ab=matrix[a][b];
    for (let i=0; i<3; i++) if (i!==a && i!==b) {
      const ia=matrix[i][a], ib=matrix[i][b];
      matrix[a][i]=matrix[i][a]=c*ia-s*ib;
      matrix[b][i]=matrix[i][b]=s*ia+c*ib;
    }
    matrix[a][a]=c*c*aa-2*s*c*ab+s*s*bb;
    matrix[b][b]=s*s*aa+2*s*c*ab+c*c*bb;
    matrix[a][b]=matrix[b][a]=0;
    for (let i=0; i<3; i++) {
      const va=vectors[i][a], vb=vectors[i][b];
      vectors[i][a]=c*va-s*vb; vectors[i][b]=s*va+c*vb;
    }
  }
  const order=[0,1,2].sort((a,b) => matrix[a][a]-matrix[b][b]);
  return {center, values:order.map(i => Math.max(0, matrix[i][i])), axes:order.map(i => vectors.map(row => row[i]))};
}

// Sorted voxel index, 27 bucket lookups per point, at most 12 representatives per bucket.
// Dense buckets must never turn into all-pairs scans. This is bounded approximate adjacency.
function neighbors(nodes, radius, colorThreshold, output) {
  const compare = (a,b) => a[0]-b[0] || a[1]-b[1] || a[2]-b[2];
  const entries=nodes.map((node,id) => ({key:node.position.map(v => Math.floor(v/radius)), id}))
    .filter(e => nodes[e.id].eligible).sort((a,b) => compare(a.key,b.key) || a.id-b.id);
  const buckets=[];
  for (let start=0; start<entries.length;) {
    let end=start+1;
    while (end<entries.length && compare(entries[start].key,entries[end].key)===0) end++;
    const count=Math.min(end-start,BUCKET_SAMPLES), ids=[];
    for (let j=0; j<count; j++) ids.push(entries[start+Math.floor(j*(end-start)/count)].id);
    buckets.push({key:entries[start].key,ids}); start=end;
  }
  const lookup = key => {
    let lo=0, hi=buckets.length;
    while (lo<hi) { const mid=(lo+hi)>>1; if (compare(buckets[mid].key,key)<0) lo=mid+1; else hi=mid; }
    return lo<buckets.length && compare(buckets[lo].key,key)===0 ? buckets[lo].ids : [];
  };
  return nodes.map((node,index) => {
    if (!node.eligible) return [];
    const key=node.position.map(v => Math.floor(v/radius)), candidates=[];
    for (let x=-1; x<=1; x++) for (let y=-1; y<=1; y++) for (let z=-1; z<=1; z++) {
      for (const id of lookup([key[0]+x,key[1]+y,key[2]+z])) {
        if (id===index) continue;
        output.neighborTests++;
        const d=distance(node.position,nodes[id].position);
        if (d>0 && d<=radius && distance(node.color,nodes[id].color)<=colorThreshold) candidates.push({id,d});
      }
    }
    return candidates.sort((a,b) => a.d-b.d || a.id-b.id).slice(0,NEIGHBORS);
  });
}

function wellObserved(nodes) {
  const smallest=nodes.reduce((a,b) => a.frames.size<=b.frames.size ? a : b);
  let common=0;
  for (const frame of smallest.frames) if (nodes.every(n => n.frames.has(frame)) && ++common===2) return true;
  return false;
}

// Bowyer-Watson only on a fixed-size neighborhood (at most NEIGHBORS+1 points).
function triangulate(points) {
  const low=[0,1].map(a => Math.min(...points.map(p => p[a])));
  const high=[0,1].map(a => Math.max(...points.map(p => p[a])));
  const span=Math.max(high[0]-low[0],high[1]-low[1]);
  if (!span) return [];
  const p=points.map(v => v.map((x,a) => (x-low[a])/span)), size=p.length;
  p.push([-20,-10], [0,20], [20,-10]);
  function triangle(a,b,c) {
    const [ax,ay]=p[a], [bx,by]=p[b], [cx,cy]=p[c];
    const d=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by));
    if (Math.abs(d)<1e-12) return null;
    const aa=ax*ax+ay*ay, bb=bx*bx+by*by, cc=cx*cx+cy*cy;
    const x=(aa*(by-cy)+bb*(cy-ay)+cc*(ay-by))/d, y=(aa*(cx-bx)+bb*(ax-cx)+cc*(bx-ax))/d;
    return {ids:[a,b,c], x,y,r:(ax-x)**2+(ay-y)**2};
  }
  let triangles=[triangle(size,size+1,size+2)];
  for (let i=0; i<size; i++) {
    const boundary=new Map(), kept=[];
    for (const t of triangles) {
      if ((p[i][0]-t.x)**2+(p[i][1]-t.y)**2>t.r+1e-10) { kept.push(t); continue; }
      for (const pair of [[t.ids[0],t.ids[1]], [t.ids[1],t.ids[2]], [t.ids[2],t.ids[0]]]) {
        const key=pairKey(...pair);
        if (boundary.has(key)) boundary.delete(key); else boundary.set(key,pair);
      }
    }
    for (const [a,b] of boundary.values()) { const t=triangle(a,b,i); if (t) kept.push(t); }
    triangles=kept;
  }
  return triangles.filter(t => t.ids.every(i => i<size) && t.ids.includes(0)).map(t => t.ids);
}

export function prepareSurfaceNodes(points, grid, observations, {threshold=3, maxPoints=6000, colorThreshold=30, frameKeys=[]} = {}) {
  const rows=new Map(points.map(p => [p[0],p])), visible=grid.cells.filter(c => c.support>=threshold);
  const metadata=observations?.schema_version==='point-observations/1.0' ? observations.frame_index_by_point : null;
  const nodes=[], count=Math.min(visible.length,maxPoints);
  for (let index=0; index<count; index++) {
    const cell=visible[Math.floor(index*visible.length/count)];
    const sources=(cell.sourceIds || []).map(id => rows.get(id)).filter(p => p && p.slice(1,4).every(Number.isFinite));
    if (!sources.length || !cell.position.every(Number.isFinite)) continue;
    const colors=sources.filter(p => p.slice(4,7).every(v => Number.isFinite(v) && v>=0 && v<=255)).map(p => lab(p.slice(4,7)));
    const color=colors.length ? [0,1,2].map(a => colors.reduce((s,c) => s+c[a],0)/colors.length) : null;
    const spread=color ? Math.sqrt(colors.reduce((s,c) => s+distance(c,color)**2,0)/colors.length) : Infinity;
    const raw=[0,1,2].map(a => sources.reduce((s,p) => s+p[a+1],0)/sources.length), frames=new Set();
    if (metadata) for (const p of sources) {
      const observed=metadata[p[0]];
      if (Array.isArray(observed) && observed.every(f => Number.isInteger(f) && f>=0 && f<grid.cameraCount))
        for (const f of observed) frames.add(frameKeys[f] ?? f);
    }
    const sourceIds=sources.map(p => p[0]).sort((a,b) => a-b);
    const key=grid.interval>0 ? `g:${cell.position.map(v => Math.round(v/grid.interval)).join(',')}` : `p:${sourceIds.join(',')}`;
    nodes.push({key,position:[...cell.position],raw,color,spread,frames,eligible:!!color && spread<=colorThreshold && frames.size>=2});
  }
  return nodes;
}

export function buildPointSurfaces(points, grid, observations,
  {threshold=3, connection=2.5, colorThreshold=30, normalAngle=55, maxPoints=6000, spacing=0, preparedNodes=null, geometryCache=null} = {}) {
  if (!Number.isFinite(connection) || connection<1 || connection>4 || !Number.isInteger(threshold) || threshold<1 ||
      !Number.isFinite(colorThreshold) || colorThreshold<1 || colorThreshold>100 ||
      !Number.isFinite(normalAngle) || normalAngle<5 || normalAngle>90 ||
      !Number.isInteger(maxPoints) || maxPoints<1 || !Number.isFinite(spacing) || spacing<0)
    throw new RangeError('Invalid surface preview settings');
  const visible=grid.cells.filter(c => c.support>=threshold);
  const metadata=observations?.schema_version==='point-observations/1.0' ? observations.frame_index_by_point : null;
  const nodes=preparedNodes ? preparedNodes.map(n => ({...n})) : prepareSurfaceNodes(points,grid,observations,{threshold,maxPoints,colorThreshold});
  const output={positions:nodes.flatMap(n => n.position), nodeKeys:nodes.map(n => n.key), triangles:[], strongTriangles:[], weakTriangles:[],
    edges:[], strongEdges:[], weakEdges:[], boundary:[], strongBoundary:[], weakBoundary:[],
    groups:0, lineGroups:0, regions:0, pointOnly:visible.length, sampled:nodes.length, visible:visible.length,
    algorithm:'color-local-graph-v1', neighborLimit:NEIGHBORS, bucketLimit:BUCKET_SAMPLES, neighborTests:0, candidateEdges:0};
  if (nodes.length<2 || !metadata) return output;
  let extent=0;
  for (let a=0; a<3; a++) {
    let lo=Infinity, hi=-Infinity;
    for (const node of nodes) { lo=Math.min(lo,node.position[a]); hi=Math.max(hi,node.position[a]); }
    extent+=(hi-lo)**2;
  }
  const base=spacing || grid.interval || Math.sqrt(extent/nodes.length);
  if (!base) return output;
  const interval=grid.interval || 0, radius=connection*Math.max(base*2,interval);
  const near=neighbors(nodes,radius,colorThreshold,output);
  nodes.forEach((node,i) => { node.spacing=Math.max(interval,Math.min(base*4,median(near[i].slice(0,4).map(n => n.d)) || base)); });
  const close=(a,b) => distance(nodes[a].position,nodes[b].position)<=connection*Math.min(nodes[a].spacing,nodes[b].spacing);
  nodes.forEach((node,i) => {
    const local=[node.raw,...near[i].filter(n => close(i,n.id)).slice(0,12).map(n => nodes[n.id].raw)];
    if (local.length<3) return;
    const fit=cached(geometryCache?.normals,JSON.stringify([node.key,local]),() => eigen(local));
    if (fit.values[1]>fit.values[2]*.02 && fit.values[0]<fit.values[1]*.5) node.fit=fit;
  });
  const cosine=Math.cos(normalAngle*Math.PI/180), graph=[];
  near.forEach((list,a) => {
    for (const {id:b,d} of list) {
      if (a>=b || !near[b].some(n => n.id===a) || !close(a,b)) continue;
      const na=nodes[a], nb=nodes[b], alignment=na.fit && nb.fit ? Math.abs(dot(na.fit.axes[0],nb.fit.axes[0])) : null;
      if (alignment!==null && alignment<cosine) continue;
      const color=distance(na.color,nb.color)/colorThreshold;
      const angle=alignment===null ? .2 : Math.acos(Math.min(1,alignment))/(normalAngle*Math.PI/180);
      const weight=.8*color+.15*angle+.05*d/(connection*Math.min(na.spacing,nb.spacing))+.1*(na.spread+nb.spread)/colorThreshold;
      graph.push({a,b,weight,strong:wellObserved([na,nb])});
    }
  });
  output.candidateEdges=graph.length;
  graph.sort((a,b) => a.weight-b.weight || a.a-b.a || a.b-b.b);
  // Felzenszwalb-Huttenlocher: boundary contrast versus each component's internal variation.
  // No minimum region size and no single plane/normal for a complete component.
  const parent=Int32Array.from(nodes,(_,i) => i), size=new Int32Array(nodes.length).fill(1), internal=new Float64Array(nodes.length);
  const find = i => { while (parent[i]!==i) { parent[i]=parent[parent[i]]; i=parent[i]; } return i; };
  const forest=[];
  for (const edge of graph) {
    let a=find(edge.a), b=find(edge.b);
    if (a===b || edge.weight>Math.min(internal[a]+1/size[a],internal[b]+1/size[b])) continue;
    if (size[a]<size[b]) [a,b]=[b,a];
    parent[b]=a; size[a]+=size[b]; internal[a]=edge.weight; forest.push(edge);
  }
  output.regions=nodes.reduce((s,_,i) => s+(find(i)===i && size[i]>1 ? 1 : 0),0);
  const adjacency=nodes.map(() => []), graphEdges=new Map();
  for (const e of graph) if (find(e.a)===find(e.b)) {
    adjacency[e.a].push(e.b); adjacency[e.b].push(e.a); graphEdges.set(pairKey(e.a,e.b),e);
  }
  const candidates=new Map();
  nodes.forEach((node,i) => {
    if (!node.fit || adjacency[i].length<2) return;
    const ids=[i,...adjacency[i].sort((a,b) => distance(node.position,nodes[a].position)-distance(node.position,nodes[b].position) || a-b)];
    const uv=ids.map(id => [dot(sub(nodes[id].position,node.position),node.fit.axes[2]),dot(sub(nodes[id].position,node.position),node.fit.axes[1])]);
    const localTriangles=cached(geometryCache?.triangles,JSON.stringify([ids.map(id => nodes[id].key),uv]),() => triangulate(uv));
    for (const local of localTriangles) {
      const triangle=local.map(j => ids[j]).sort((a,b) => a-b), key=triangle.join(',');
      if (candidates.has(key)) continue;
      const [a,b,c]=triangle, pairs=[[a,b],[b,c],[c,a]];
      if (pairs.some(pair => !graphEdges.has(pairKey(...pair)))) continue;
      const vertices=triangle.map(id => nodes[id]), [pa,pb,pc]=vertices.map(n => n.position);
      const ab=distance(pa,pb), bc=distance(pb,pc), ca=distance(pc,pa), vector=cross(sub(pb,pa),sub(pc,pa)), area2=length(vector);
      const sine=area2/Math.max(ab*bc,bc*ca,ca*ab), normal=normalize(vector);
      if (!normal || sine<.18 || ab*bc*ca/(2*area2)>connection*Math.min(...vertices.map(n => n.spacing))) continue;
      // Only local direction consistency. Bent regions never receive a global plane-depth test.
      if (vertices.some(n => n.fit && Math.abs(dot(normal,n.fit.axes[0]))<Math.cos(Math.max(65,normalAngle)*Math.PI/180))) continue;
      candidates.set(key,{triangle,quality:sine/(ab+bc+ca),strong:wellObserved(vertices)});
    }
  });
  const meshEdges=new Map(), used=new Set(), faceRegions=new Set(), lineRegions=new Set();
  const addEdge=(a,b,face=false,strong=false,other=null) => {
    const key=pairKey(a,b), edge=meshEdges.get(key) || {pair:[Math.min(a,b),Math.max(a,b)],thirds:[],line:false,strong:false};
    if (face) edge.thirds.push(other); else edge.line=true;
    edge.strong ||= strong; meshEdges.set(key,edge); used.add(a); used.add(b);
  };
  for (const candidate of [...candidates.values()].sort((a,b) => b.quality-a.quality || a.triangle[0]-b.triangle[0] || a.triangle[1]-b.triangle[1] || a.triangle[2]-b.triangle[2])) {
    const [a,b,c]=candidate.triangle, sides=[[a,b,c],[b,c,a],[c,a,b]];
    if (sides.some(([i,j,k]) => {
      const edge=meshEdges.get(pairKey(i,j));
      if (!edge) return false;
      if (edge.thirds.length>=2) return true;
      const axis=sub(nodes[j].position,nodes[i].position), next=cross(axis,sub(nodes[k].position,nodes[i].position));
      return edge.thirds.some(old => {
        const previous=cross(axis,sub(nodes[old].position,nodes[i].position));
        return dot(next,previous)>1e-10*length(next)*length(previous);
      });
    })) continue;
    output.triangles.push(a,b,c);
    (candidate.strong ? output.strongTriangles : output.weakTriangles).push(a,b,c);
    for (const [i,j,k] of sides) addEdge(i,j,true,candidate.strong,k);
    faceRegions.add(find(a));
  }
  // Keep a sparse region backbone where no triangle exists, rather than drawing every graph edge.
  for (const edge of forest) {
    if (!meshEdges.has(pairKey(edge.a,edge.b))) addEdge(edge.a,edge.b,false,edge.strong);
    lineRegions.add(find(edge.a));
  }
  for (const edge of meshEdges.values()) {
    output.edges.push(...edge.pair);
    (edge.strong ? output.strongEdges : output.weakEdges).push(...edge.pair);
    if (edge.thirds.length===1 || edge.line) {
      output.boundary.push(...edge.pair);
      (edge.strong ? output.strongBoundary : output.weakBoundary).push(...edge.pair);
    }
  }
  output.groups=faceRegions.size;
  output.lineGroups=[...lineRegions].filter(id => !faceRegions.has(id)).length;
  output.pointOnly=visible.length-used.size;
  return output;
}

const sameFrames = (a,b) => a.size===b.size && [...a].every(f => b.has(f));
const nodeHash = key => { let h=2166136261; for (let i=0;i<key.length;i++) h=Math.imul(h^key.charCodeAt(i),16777619); return h>>>0; };
const faceKey = (model,ids) => ids.map(i => model.nodeKeys[i]).sort().join('|');
const faceKeys = model => new Set(Array.from({length:model.triangles.length/3},(_,i) => faceKey(model,model.triangles.slice(i*3,i*3+3))));

function refreshEvidence(model,nodes) {
  for (const kind of ['Triangles','Edges','Boundary']) {
    const source=model[kind.toLowerCase()], width=kind==='Triangles' ? 3 : 2;
    model[`strong${kind}`]=[]; model[`weak${kind}`]=[];
    for (let i=0;i<source.length;i+=width) {
      const ids=source.slice(i,i+width), vertices=ids.map(id => nodes.get(model.nodeKeys[id]));
      const strong=vertices.every(Boolean) && wellObserved(vertices);
      model[`${strong ? 'strong' : 'weak'}${kind}`].push(...ids);
    }
  }
}

function compactSurface(model,triangles,edges,boundary) {
  const ids=[...new Set([...triangles,...edges,...boundary])].sort((a,b) => a-b), local=new Map(ids.map((id,i) => [id,i]));
  return {positions:ids.flatMap(i => model.positions.slice(i*3,i*3+3)), nodeKeys:ids.map(i => model.nodeKeys[i]),
    triangles:triangles.map(i => local.get(i)), edges:edges.map(i => local.get(i)), boundary:boundary.map(i => local.get(i)),
    groups:triangles.length ? 1 : 0,lineGroups:edges.length && !triangles.length ? 1 : 0};
}

// Fixed world blocks with a two-search-radius halo. Region grouping is deliberately local:
// this preserves untouched geometry, rather than promising equality to a global FH partition.
export class IncrementalPointSurfaces {
  constructor({blockScale=16, motionFraction=.2, colorTolerance=.5, reconcileEvery=30} = {}) {
    if (!Number.isFinite(blockScale) || blockScale<4 || !Number.isFinite(motionFraction) || motionFraction<0 ||
        !Number.isFinite(colorTolerance) || colorTolerance<0 || !Number.isInteger(reconcileEvery) || reconcileEvery<0)
      throw new RangeError('Invalid incremental surface settings');
    this.blockScale=blockScale; this.motionFraction=motionFraction; this.colorTolerance=colorTolerance;
    this.reconcileEvery=reconcileEvery; this.patches=new Map(); this.nodes=new Map(); this.references=new Map();
    this.scope=null; this.signature=null; this.revision=null; this.updates=0;
    this.geometryCache={normals:new Map(),triangles:new Map()};
  }

  update(points,grid,observations,settings={},context={}) {
    const start=performance.now(), options={threshold:3,connection:2.5,colorThreshold:30,normalAngle:55,maxPoints:6000,spacing:0,...settings};
    if (!Number.isFinite(options.connection) || options.connection<1 || options.connection>4 ||
        !Number.isInteger(options.threshold) || options.threshold<1 || !Number.isInteger(options.maxPoints) || options.maxPoints<1 ||
        !Number.isFinite(options.spacing) || options.spacing<0 || !Number.isFinite(options.colorThreshold) || options.colorThreshold<1 || options.colorThreshold>100 ||
        !Number.isFinite(options.normalAngle) || options.normalAngle<5 || options.normalAngle>90)
      throw new RangeError('Invalid incremental surface settings');
    const scope=JSON.stringify([context.source,context.session,context.segment,context.poseSource]);
    const signature=JSON.stringify([grid.interval||0,options.threshold,options.connection,options.colorThreshold,options.normalAngle,options.maxPoints]);
    let reason=!this.scope ? 'initial' : scope!==this.scope ? 'coordinate_system' : signature!==this.signature ? 'settings' :
      context.revision<this.revision ? 'rewind' : null;
    const proposed=options.spacing || grid.interval || this.spacing || .01;
    if (!reason && (proposed>this.spacing*1.3 || proposed<this.spacing/1.3)) reason='scale_change';
    if (!reason && this.reconcileEvery>0 && this.updates>=this.reconcileEvery) reason='periodic';
    const reset=!!reason;
    if (reset) {
      this.patches=new Map(); this.nodes=new Map(); this.references=new Map(); this.spacing=proposed;
      this.radius=options.connection*Math.max(this.spacing*2,grid.interval||0);
      this.width=this.radius*this.blockScale; this.halo=this.radius*2; this.updates=0;
      this.geometryCache={normals:new Map(),triangles:new Map()};
    }
    this.scope=scope; this.signature=signature; this.revision=context.revision; this.updates++;
    this.geometryCache.normals.hits=0; this.geometryCache.triangles.hits=0;
    const visible=grid.cells.filter(c => c.support>=options.threshold).length;
    let prepared=prepareSurfaceNodes(points,grid,observations,{...options,maxPoints:Number.MAX_SAFE_INTEGER,frameKeys:context.frameKeys||[]});
    if (prepared.length>options.maxPoints) prepared.sort((a,b) => nodeHash(a.key)-nodeHash(b.key) || a.key.localeCompare(b.key));
    prepared=prepared.slice(0,options.maxPoints);
    const current=new Map(prepared.map(n => [n.key,n])), blocks=new Map(), dirty=new Set(), evidence=new Set(), positions=new Set();
    const blockOf=p => p.map(v => Math.floor(v/this.width)).join(',');
    for (const node of prepared) {
      const key=blockOf(node.position);
      if (!blocks.has(key)) blocks.set(key,[]);
      blocks.get(key).push(node);
    }
    const mark = position => {
      const low=position.map(v => Math.floor((v-this.halo)/this.width-1e-10));
      const high=position.map(v => Math.floor((v+this.halo)/this.width+1e-10));
      for (let x=low[0];x<=high[0];x++) for (let y=low[1];y<=high[1];y++) for (let z=low[2];z<=high[2];z++) dirty.add(`${x},${y},${z}`);
    };
    let changedCells=0, observationCells=0;
    const tolerance=Math.max(grid.interval||0,this.spacing)*this.motionFraction;
    for (const [key,old] of this.nodes) if (!current.has(key)) { mark(old.position); this.references.delete(key); changedCells++; }
    for (const [key,node] of current) {
      const old=this.nodes.get(key), reference=this.references.get(key);
      const changed=!reference || node.eligible!==reference.eligible ||
        (node.color===null)!==(reference.color===null) || (node.color && distance(node.color,reference.color)>this.colorTolerance) ||
        Math.abs(node.spread-reference.spread)>this.colorTolerance || distance(node.raw,reference.raw)>tolerance ||
        (old && blockOf(old.position)!==blockOf(node.position));
      if (changed) {
        if (old) mark(old.position);
        mark(node.position); this.references.set(key,node); changedCells++;
      }
      if (old && !sameFrames(node.frames,old.frames)) { evidence.add(key); observationCells++; }
      if (old && distance(node.position,old.position)>0) positions.add(key);
    }
    for (const key of blocks.keys()) if (!this.patches.has(key) || reset) dirty.add(key);
    const removed=[], changed=[], oldFaces=new Set(), newFaces=new Set();
    let retainedFaces=0, remeshed=0, refreshed=0, moved=0, analyzedNodes=0;
    for (const [key,patch] of this.patches) {
      if (!blocks.has(key)) { removed.push(key); for (const f of faceKeys(patch)) oldFaces.add(f); this.patches.delete(key); }
      else if (patch.nodeKeys.some(id => !current.has(id))) dirty.add(key);
    }
    const poolFor = key => {
      const indices=key.split(',').map(Number), low=indices.map(v => v*this.width-this.halo), high=indices.map(v => (v+1)*this.width+this.halo), pool=[];
      for (let x=-1;x<=1;x++) for (let y=-1;y<=1;y++) for (let z=-1;z<=1;z++)
        for (const node of blocks.get([indices[0]+x,indices[1]+y,indices[2]+z].join(',')) || [])
          if (node.position.every((v,a) => v>=low[a] && v<=high[a])) pool.push(node);
      return pool.sort((a,b) => a.key.localeCompare(b.key));
    };
    const own = (model,flat,width,key) => {
      const result=[];
      for (let i=0;i<flat.length;i+=width) {
        const ids=flat.slice(i,i+width), owner=ids.reduce((a,b) => model.nodeKeys[a]<model.nodeKeys[b] ? a : b);
        if (blockOf(current.get(model.nodeKeys[owner]).position)===key) result.push(...ids);
      }
      return result;
    };
    const pending=[...dirty].filter(key => blocks.has(key)).sort(), pools=new Map(pending.map(key => [key,poolFor(key)]));
    const poolSize=[...pools.values()].reduce((sum,pool) => sum+pool.length,0);
    // If overlapping halos cost more than one map pass, analyze once with per-neighborhood caches.
    // Only affected owners are uploaded; unchanged GPU patches remain intact.
    const globalPass=reset || poolSize>prepared.length*1.1, partition=new Map();
    const reconstruct = pool => {
      const anchored=pool.map(node => ({...this.references.get(node.key),position:node.position,frames:node.frames}));
      const localGrid={interval:grid.interval,cameraCount:grid.cameraCount,cells:anchored.map(() => ({support:options.threshold}))};
      analyzedNodes+=anchored.length;
      return buildPointSurfaces([],localGrid,observations,{...options,spacing:this.spacing,preparedNodes:anchored,geometryCache:this.geometryCache});
    };
    let globalModel=null;
    if (pending.length && globalPass) {
      globalModel=reconstruct([...prepared].sort((a,b) => a.key.localeCompare(b.key)));
      for (const [name,width] of [['triangles',3],['edges',2],['boundary',2]]) {
        for (let i=0;i<globalModel[name].length;i+=width) {
          const ids=globalModel[name].slice(i,i+width), owner=ids.reduce((a,b) => globalModel.nodeKeys[a]<globalModel.nodeKeys[b] ? a : b);
          const key=blockOf(current.get(globalModel.nodeKeys[owner]).position);
          if (!partition.has(key)) partition.set(key,{triangles:[],edges:[],boundary:[]});
          partition.get(key)[name].push(...ids);
        }
      }
    }
    for (const key of pending) {
      const previous=this.patches.get(key);
      if (previous) for (const f of faceKeys(previous)) oldFaces.add(f);
      let model;
      if (globalModel) {
        const primitives=partition.get(key)||{triangles:[],edges:[],boundary:[]};
        model=compactSurface(globalModel,primitives.triangles,primitives.edges,primitives.boundary);
      } else {
        const local=reconstruct(pools.get(key));
        model=compactSurface(local,own(local,local.triangles,3,key),own(local,local.edges,2,key),own(local,local.boundary,2,key));
      }
      refreshEvidence(model,current);
      for (const f of faceKeys(model)) newFaces.add(f);
      this.patches.set(key,model); changed.push({key,kind:'topology',model}); remeshed++;
    }
    for (const [key,model] of this.patches) {
      if (dirty.has(key)) continue;
      retainedFaces+=model.triangles.length/3;
      const refresh=model.nodeKeys.some(id => evidence.has(id)), move=model.nodeKeys.some(id => positions.has(id));
      if (refresh) { refreshEvidence(model,current); refreshed++; }
      if (move) { model.positions=model.nodeKeys.flatMap(id => current.get(id).position); moved++; }
      if (refresh || move) changed.push({key,kind:refresh ? 'evidence' : 'positions',model});
    }
    for (const key of oldFaces) if (newFaces.has(key)) retainedFaces++;
    let addedFaces=0, removedFaces=0;
    for (const key of newFaces) if (!oldFaces.has(key)) addedFaces++;
    for (const key of oldFaces) if (!newFaces.has(key)) removedFaces++;
    const stats={algorithm:'incremental-color-blocks-v1',visible,sampled:prepared.length,groups:0,lineGroups:0,
      faces:0,weakFaces:0,edges:0,pointOnly:visible,blocks:this.patches.size,remeshedBlocks:remeshed,
      reusedBlocks:Math.max(0,this.patches.size-remeshed),evidenceBlocks:refreshed,positionBlocks:moved,
      changedCells,observationCells,retainedFaces,addedFaces,removedFaces,reason:reason||'local',reset,analyzedNodes,globalPass:!!globalModel,
      cachedFits:this.geometryCache.normals.hits,cachedTriangulations:this.geometryCache.triangles.hits,
      blockWidth:this.width,spacing:this.spacing,halo:this.halo};
    const used=new Set();
    for (const model of this.patches.values()) {
      stats.groups+=model.groups; stats.lineGroups+=model.lineGroups; stats.faces+=model.triangles.length/3;
      stats.weakFaces+=model.weakTriangles.length/3; stats.edges+=model.edges.length/2;
      for (const i of model.edges) used.add(model.nodeKeys[i]);
      for (const i of model.triangles) used.add(model.nodeKeys[i]);
    }
    stats.pointOnly=visible-used.size; stats.analysisMs=performance.now()-start;
    this.nodes=current;
    return {reset,patches:changed,removed,stats};
  }
}
