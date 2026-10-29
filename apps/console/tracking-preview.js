// This reader is independent of scene conversion, map downloads and rendering.
export function trackingDescription(state) {
  const tracking = state.frame_tracking || {};
  const age = state.tracker_age_s;
  const stale = state.input_state === 'reconnecting' || (Number.isFinite(age) && age > 1);
  const names = {2: '추적 중', 3: '재위치 탐색', 4: '추적 상실'};
  const status = state.session_status || state.status;
  const label = status === 'failed' ? '추적 중단 · 마지막 지도 보존' : status === 'complete' ? '추적 종료' :
    stale ? '영상 대기 · 마지막 처리 프레임' : names[state.tracking_state] || '초기화 중';
  const delay = Number.isFinite(state.receive_to_track_ms) ? `${state.receive_to_track_ms.toFixed(0)}ms` : '측정 대기';
  const timestamp = Number.isFinite(state.last_map_timestamp_s) ? `${state.last_map_timestamp_s.toFixed(2)}초` : '대기';
  return {label, stale, detail: `처리 프레임 ${timestamp} · 지도 점 ${tracking.tracked_features || 0}개 · 직전과 동일 ${tracking.reused_features || 0}개 · 추적 ${Number(state.tracking_ms || 0).toFixed(1)}ms · 수신→추적 ${delay}${Number.isFinite(age) ? ` · 마지막 처리 ${age.toFixed(1)}초 전` : ''}`};
}

export function connectTrackingPreview({source, following, element = (id) => document.getElementById(id), schedule = setTimeout} = {}) {
  let frame = null, session = null;
  async function poll() {
    const context = source.context();
    let delay = 1000;
    try {
      if (context.source !== 'live' || !following()) return;
      const response = await source.read('tracking', context);
      if (!response?.ok || !source.isCurrent(context) || !following()) return;
      const state = response.value;
      if (state.transport_active) delay = 100;
      const text = trackingDescription(state);
      element('slam-tracking-state').textContent = text.label;
      element('slam-timing').textContent = text.detail;
      const image = element('slam-preview');
      image.dataset.stale = String(text.stale);
      if (state.preview_ready && (session !== state.session_id || frame !== state.frame_id)) {
        session = state.session_id; frame = state.frame_id;
        image.src = `/api/live/slam-preview?frame=${encodeURIComponent(frame)}&session=${encodeURIComponent(session)}`;
        image.alt = `SLAM 처리 프레임 ${frame}`;
      }
    } catch {
      if (source.isCurrent(context) && following()) element('slam-timing').textContent = '추적 상태 재연결 중 · 마지막 영상과 지도 보존';
    } finally { schedule(poll, delay); }
  }
  poll();
}
