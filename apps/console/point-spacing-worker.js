import {pointSpacingProfile} from './point-grid.js';
import {WorkerPointState} from './point-worker-state.js';
const state = new WorkerPointState();
self.onmessage = ({data}) => {
  try {
    const start = performance.now();
    const {points} = state.apply(data.update);
    self.postMessage({profile: pointSpacingProfile(points), analysisMs: performance.now() - start});
  } catch (error) { self.postMessage({error: error.message}); }
};
