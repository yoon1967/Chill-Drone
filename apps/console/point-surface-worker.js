import {IncrementalPointSurfaces} from './point-surfaces.js';
import {WidePointSurfaces} from './wide-surfaces.js';
import {pointGridCells} from './point-grid.js';
import {WorkerPointState} from './point-worker-state.js';
const surfaces=new IncrementalPointSurfaces();
const wide=new WidePointSurfaces();
const state=new WorkerPointState();
let version=0;
self.onmessage=async ({data}) => {
  try {
    const start=performance.now();
    const {points,observations}=state.apply(data.update);
    const grid=pointGridCells(points,data.interval,observations,data.context.cameras.length);
    const resync=data.ack!==version;
    const result=surfaces.update(points,grid,observations,data.settings,data.context);
    const large=await wide.update(points,grid,observations,data.settings,data.context);
    result.patches.push(...large.patches);result.removed.push(...large.removed);
    Object.assign(result.stats,large.stats);
    result.stats.faces+=large.stats.wideFaces;result.stats.weakFaces+=large.stats.wideWeakFaces;
    result.stats.retainedFaces+=large.stats.wideRetainedFaces;result.stats.addedFaces+=large.stats.wideAddedFaces;
    result.stats.removedFaces+=large.stats.wideRemovedFaces;
    for(const model of wide.patches.values())result.stats.edges+=model.edges.length/2;
    result.stats.analysisMs=performance.now()-start;
    if (resync || result.reset) {
      result.reset=true;
      result.patches=[...surfaces.patches,...wide.patches].map(([key,model]) => ({key,kind:'topology',model}));
      result.removed=[];
    }
    self.postMessage({id:data.id,version:++version,result});
  } catch (error) {
    surfaces.scope=null;
    wide.scope=null;
    self.postMessage({id:data.id,error:error.message});
  }
};
