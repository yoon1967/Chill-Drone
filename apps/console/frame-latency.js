// One browser monotonic clock covers capture, transport, mapping and display.
// A sample ends at the rAF AFTER submitting the corresponding map to WebGL.
// This is a browser presentation estimate, not a physical monitor measurement.
export class FrameLatency {
  constructor(capacity = 1800) { this.capacity = capacity; this.start(null); }
  start(session) {
    this.session = session;
    this.frames = new Map(); this.shown = new Set(); this.samples = [];
    this.submitted = this.expired = this.measured = 0;
  }
  key(pts) { return Math.round(pts * 1e6); }
  record({session, pts, ...marks}) {
    if (!this.session || session !== this.session || !Number.isFinite(pts)) return;
    const key = this.key(pts);
    let frame = this.frames.get(key);
    if (!frame) {
      if (!Number.isFinite(marks.capturedAtMs)) return;
      frame = {pts}; this.frames.set(key, frame); this.submitted++;
    }
    Object.assign(frame, marks);
    while (this.frames.size > this.capacity) { this.frames.delete(this.frames.keys().next().value); this.expired++; }
  }
  present(scene, now, stages = {}, visiblePointIds = null) {
    if (!this.session || scene.source !== 'live' || scene.input_run_id !== this.session ||
        scene.pose_source !== 'orb_slam3' || scene.point_observations?.schema_version !== 'point-observations/1.0') return 0;
    const evidence = scene.point_observations.frame_index_by_point;
    const supported = new Set((visiblePointIds === null ? Object.values(evidence) :
      visiblePointIds.map(id => evidence[id] || [])).flat());
    let count = 0;
    for (const index of supported) {
      const camera = scene.cameras[index];
      if (!camera || this.shown.has(camera.frame_id)) continue;
      const frame = this.frames.get(this.key(camera.timestamp_s));
      if (!frame || !Number.isFinite(now) || now < frame.capturedAtMs) continue;
      this.shown.add(camera.frame_id);
      this.samples.push({frame: camera.frame_id, pts: frame.pts, revision: scene.revision, mapId: scene.segment_id,
        endToEndMs: now - frame.capturedAtMs,
        encodeMs: Number.isFinite(frame.encodedAtMs) ? frame.encodedAtMs - frame.capturedAtMs : null,
        uploadMs: Number.isFinite(frame.ackAtMs) ? frame.ackAtMs - frame.encodedAtMs : null,
        confirmationMs: scene.quality?.landmark_confirmation?.processing_ms ?? null,
        ...stages});
      count++; this.measured++;
    }
    if (this.samples.length > 600) this.samples.splice(0, this.samples.length - 600);
    while (this.shown.size > this.capacity) this.shown.delete(this.shown.values().next().value);
    return count;
  }
  summary() {
    const values = this.samples.map(s => s.endToEndMs), sorted = [...values].sort((a, b) => a - b);
    const mean = a => a.length ? a.reduce((sum, n) => sum + n, 0) / a.length : null;
    const stage = key => mean(this.samples.map(s => s[key]).filter(Number.isFinite));
    return {basis: 'browser_frame_available_to_post_render_raf', session: this.session,
      samples: values.length, measuredKeyframes: this.measured, submittedFrames: this.submitted,
      expiredFrameMarks: this.expired, meanMs: mean(values),
      p95Ms: sorted.length ? sorted[Math.ceil(sorted.length * .95) - 1] : null,
      maxMs: sorted.length ? sorted.at(-1) : null, lastMs: values.at(-1) ?? null,
      firstWindowMeanMs: mean(values.slice(0, 10)), lastWindowMeanMs: mean(values.slice(-10)),
      encodeMeanMs: stage('encodeMs'), uploadMeanMs: stage('uploadMs'),
      fetchMeanMs: stage('fetchMs'), applyMeanMs: stage('applyMs'),
      confirmationMeanMs: stage('confirmationMs'), renderMeanMs: stage('renderMs')};
  }
  report() { return {...this.summary(), measurements: this.samples}; }
}
