// Worker modules cannot use the document's Three.js import map. Keep their
// incremental input store independent of rendering and DOM modules.
export class WorkerPointState {
  constructor() { this.rows = new Map(); this.frames = {}; this.version = -1; }
  apply(update) {
    if (!update.reset && this.version !== update.base) throw new Error('Point worker base mismatch');
    if (update.reset) { this.rows.clear(); this.frames = {}; }
    for (const id of update.removed) { this.rows.delete(id); delete this.frames[id]; }
    for (const row of update.points) this.rows.set(row[0], row);
    for (const [id, frames] of Object.entries(update.observations)) {
      if (frames === null) delete this.frames[id]; else this.frames[id] = frames;
    }
    this.version = update.version;
    return {points: [...this.rows.values()], observations: update.metadata ? {...update.metadata, frame_index_by_point: this.frames} : null};
  }
}
