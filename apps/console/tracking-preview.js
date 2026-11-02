// This reader is independent of scene conversion, map downloads and rendering.
export function trackingDescription(state) {
  const tracking = state.frame_tracking || {};
  const age = state.tracker_age_s;
  const stale = state.input_state === 'reconnecting' || (Number.isFinite(age) && age > 1);
  const names = {2: '추적 중', 3: '재위치 탐색 · 지도 갱신 대기', 4: '추적 상실 · 지도 갱신 대기'};
  const status = state.session_status || state.status;
  const label = status === 'failed' ? '추적 중단 · 마지막 지도 보존' : status === 'complete' ? '추적 종료' :
    stale ? '영상 대기 · 마지막 처리 프레임' : names[state.tracking_state] || '초기화 중';
  const delay = Number.isFinite(state.receive_to_track_ms) ? `${state.receive_to_track_ms.toFixed(0)}ms` : '측정 대기';
  const timestamp = Number.isFinite(state.last_map_timestamp_s) ? `${state.last_map_timestamp_s.toFixed(2)}초` : '대기';
  const detected = Number.isFinite(tracking.detected_features) ? ` · 검출 ${tracking.detected_features}개 · 지도점 후보 ${tracking.map_candidates || 0}개` : '';
  return {label, stale, detail: `처리 프레임 ${timestamp} · 지도 점 ${tracking.tracked_features || 0}개 · 직전과 동일 ${tracking.reused_features || 0}개${detected} · 추적 ${Number(state.tracking_ms || 0).toFixed(1)}ms · 수신→추적 ${delay}${Number.isFinite(age) ? ` · 마지막 처리 ${age.toFixed(1)}초 전` : ''}`};
}

export function liveTrackingNotices(state) {
  const lifecycle = state.session_status || state.status;
  const active = lifecycle === 'starting' || lifecycle === 'receiving';
  const receiving = state.input_state === 'receiving' &&
    (!Number.isFinite(state.tracker_age_s) || state.tracker_age_s <= 1);
  const lost = active && (state.tracking_state === 3 || state.tracking_state === 4);
  const pendingNewMap = active && state.map_available && Number.isInteger(state.map_id) &&
    Number.isInteger(state.segment_id) && state.map_id !== state.segment_id;
  let tracking = null;
  if (lost) {
    tracking = {
      title: receiving ? '영상 수신 중 · 지도 추적 중단' : '지도 추적 중단 · 영상 상태 확인',
      detail: `${receiving ? '새 영상은 들어오고 있습니다. ' : ''}현재 프레임이 기존 지도와 연결되지 않아 점과 경로 갱신이 멈췄습니다. 마지막 지도는 유지하며 재위치를 찾습니다.`,
    };
  } else if (pendingNewMap) {
    tracking = {
      title: '새 지도 초기화 중 · 이전 지도 유지',
      detail: '기존 지도에서 위치를 찾지 못해 독립 좌표계로 초기화하고 있습니다. 새 점이 나타나도 이전 지도에 이어 붙은 것은 아닙니다.',
    };
  }
  const count = Array.isArray(state.segments) ? state.segments.length : 0;
  const segment = count > 1 ? {
    title: `독립 지도 ${count}개 구간`,
    detail: '새 구간은 이전 지도와 위치·방향이 정렬되지 않았습니다. 지도 시간 슬라이더로 이전 구간을 확인할 수 있습니다.',
  } : null;
  return {tracking, segment, pendingNewMap};
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
