import {connectProgressiveMap} from '/progressive-map.js';

export function connectLiveDepth() {
  let panel = null, controller = null;
  const recording = document.getElementById('session-recording');
  const recordingParent = recording.parentNode, recordingNext = recording.nextSibling;
  function prepare() {
    if (panel) return;
    panel = document.getElementById('progressive-workspace').cloneNode(true);
    panel.querySelectorAll('canvas').forEach(canvas => canvas.remove());
    for (const node of [panel, ...panel.querySelectorAll('[id]')]) node.id = node.id.replace(/^progressive/, 'live-progressive');
    panel.querySelectorAll('[for]').forEach(node => node.htmlFor = node.htmlFor.replace(/^progressive/, 'live-progressive'));
    panel.querySelectorAll('[aria-describedby]').forEach(node => node.setAttribute('aria-describedby', node.getAttribute('aria-describedby').replace(/progressive/g, 'live-progressive')));
    panel.setAttribute('aria-label', '실시간 영상의 순차 깊이 복원');
    panel.querySelector('h2').textContent = '실시간 영상에서 공간을 복원하고 있습니다';
    panel.querySelector('#live-progressive-pose').textContent = '카메라 위치·방향 확정 대기 · 확정된 프레임만 깊이를 복원합니다.';
    panel.querySelector('#live-progressive-count').textContent = '처리 0프레임';
    panel.querySelector('#live-progressive-latency').textContent = '모델 준비 중 · 영상 복원 대기';
    panel.querySelector('#live-progressive-map-empty').hidden = false;
    const open = document.createElement('a'); open.id = 'live-depth-open'; open.hidden = true;
    open.textContent = '전체 경로 지도 열기 ↗'; open.className = 'saved-primary';
    panel.append(open);
    const error = document.createElement('p'); error.id = 'live-depth-error'; error.hidden = true;
    error.setAttribute('role', 'alert'); panel.append(error);
    document.getElementById('reconstruction-stage').before(panel);
    controller = connectProgressiveMap({prefix: 'live-progressive'});
  }
  return {update(state, source) {
    const enabled = source === 'live' && Boolean(state.depth_job_id);
    document.body.classList.toggle('live-depth-processing', enabled);
    if (!enabled) {
      if (panel) panel.hidden = true;
      if (recording.parentNode !== recordingParent) recordingParent.insertBefore(recording, recordingNext);
      return;
    }
    prepare(); panel.hidden = false;
    if (recording.parentNode !== panel) panel.append(recording);
    if (state.depth_job) controller.update(state.depth_job);
    const error = document.getElementById('live-depth-error');
    error.hidden = !state.last_error;
    error.textContent = state.last_error ? `복원을 마치지 못했습니다. ${state.last_error}` : '';
    const open = document.getElementById('live-depth-open');
    open.hidden = state.depth_job?.status !== 'complete';
    open.href = `/?asset=${encodeURIComponent(state.depth_job_id)}#saved`;
  }};
}
