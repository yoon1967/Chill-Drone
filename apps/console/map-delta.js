// Reconstruct the exact snapshot, including native point order and current camera-frame indices.
export function applySceneDelta(previous,payload) {
  if (payload.schema_version!=='phone-map-delta/1.0') return payload;
  if (!previous || previous.pose_source!=='orb_slam3' || previous.revision!==payload.base_revision ||
      previous.input_run_id!==payload.session_id || previous.segment_id!==payload.segment_id ||
      payload.scene?.input_run_id!==payload.session_id || payload.scene?.segment_id!==payload.segment_id ||
      payload.scene?.revision!==payload.revision || payload.revision<=payload.base_revision)
    throw new Error('Map delta base mismatch');
  const rows=new Map(previous.points.map(p => [p[0],p]));
  for (const id of payload.points_removed) rows.delete(id);
  for (const point of payload.points_upsert) rows.set(point[0],point);
  if (payload.point_order.length!==rows.size || new Set(payload.point_order).size!==rows.size ||
      payload.point_order.some(id => !rows.has(id))) throw new Error('Invalid map delta point order');
  const frames={...previous.point_observations?.frame_index_by_point};
  for (const id of payload.observation_removed) delete frames[id];
  Object.assign(frames,payload.observation_upserts);
  return {...payload.scene,points:payload.point_order.map(id => rows.get(id)),
    point_observations:{...payload.observation_metadata,frame_index_by_point:frames}};
}
