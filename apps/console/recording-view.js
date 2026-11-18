export function recordingDescription(state) {
  const recording = state.recording;
  if (!state.session_id || !recording) return null;
  const terminal = ['complete', 'failed'].includes(state.session_status || state.status);
  const finalized = recording.finalized && terminal;
  let label = recording.bytes_received ? recording.original_h264 ? '드론 원본 녹화 중' : '수신 영상 저장 중' : '녹화 준비 중';
  if (recording.status === 'finalizing') label = '영상·세션 저장 마무리 중';
  if (finalized) label = recording.bytes_written ? '영상·세션 저장 완료' : '영상 없이 세션 정보 저장 완료';
  if (recording.error) label = finalized ? '영상 저장 일부 실패 · 세션 기록 보존' :
    terminal || state.session_status === 'stopping' ? '영상 저장 오류 · 세션 기록 정리 중' : '영상 저장 오류 · 지도 처리는 계속됩니다';
  const kind = recording.original_h264 ? '수신한 H.264 원본과 재생용 MP4' : '서버에 전달된 JPEG 영상';
  const size = ((recording.bytes_written || 0) / 1048576).toFixed(1);
  let detail = `${kind} · ${size}MB 저장 · 연결 구간 ${recording.parts || 0}개`;
  if (recording.playback_errors) detail += ' · 원본은 보존했으나 일부 재생 파일을 만들지 못했습니다.';
  if (recording.error) detail += ' · 저장 장치 상태를 확인하세요. 자세한 내용은 세션 기록에 남습니다.';
  return {label, detail, path: recording.session_path || '', downloadable: finalized,
    url: `/api/live/session-download?session=${encodeURIComponent(state.session_id)}`};
}

export function renderRecording(state, element = id => document.getElementById(id)) {
  const view = recordingDescription(state);
  element('session-recording').hidden = !view;
  if (!view) return;
  element('recording-state').textContent = view.label;
  element('recording-detail').textContent = view.detail;
  element('recording-path').textContent = view.path ? `저장 폴더: ${view.path}` : '세션 폴더에 저장합니다.';
  const download = element('session-download');
  download.hidden = !view.downloadable;
  if (view.downloadable) download.href = view.url;
  else download.removeAttribute('href');
}
