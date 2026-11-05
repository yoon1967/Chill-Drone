import {connectProgressiveMap} from '/progressive-map.js';
import {mappingPreparation} from '/map-source.js';
import {processingSteps} from '/import-view.js';

export function connectLiveDepth({onStop = async () => {}, onRestart = async () => {}} = {}) {
  let panel = null, controller = null, stopPending = false, restartPending = false, stopError = null, session = null;
  let latestState = null;
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
    const actions = document.createElement('div'); actions.className = 'live-depth-actions';
    actions.setAttribute('aria-label', '실시간 촬영 제어');
    const stop = document.createElement('button'); stop.id = 'live-depth-stop'; stop.type = 'button';
    stop.textContent = '촬영 종료';
    stop.addEventListener('click', async () => {
      if (stopPending || stop.disabled) return;
      stopPending = true; stopError = null; stop.disabled = true; stop.textContent = '촬영 종료 중…';
      try { await onStop(); }
      catch (error) { stopError = `촬영 종료 요청을 완료하지 못했습니다. ${error.message}`; }
      finally { stopPending = false; }
    });
    const restart = document.createElement('button'); restart.id = 'live-depth-restart'; restart.type = 'button';
    restart.textContent = '새로 촬영';
    restart.addEventListener('click', async () => {
      if (restartPending || restart.disabled) return;
      restartPending = true; stopError = null; restart.disabled = true; restart.textContent = '새 촬영 시작 중…';
      try { await onRestart(latestState); }
      catch (error) { stopError = `새 촬영을 시작하지 못했습니다. ${error.message}`; }
      finally { restartPending = false; }
    });
    actions.append(stop, restart); panel.querySelector('.progressive-map .section-title').append(actions);
    const steps = document.createElement('ol'); steps.id = 'live-depth-steps';
    steps.className = 'saved-processing-steps'; steps.setAttribute('aria-label', '실시간 3D 지도 복원 단계');
    panel.querySelector('.progressive-heading').after(steps);
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
    latestState = state;
    if (session !== state.session_id) { session = state.session_id; stopError = null; restartPending = false; }
    if (recording.parentNode !== panel) panel.append(recording);
    if (state.depth_job) controller.update(state.depth_job);
    const stop = document.getElementById('live-depth-stop');
    const stopping = stopPending || state.session_status === 'stopping';
    const ended = ['complete', 'failed'].includes(state.session_status);
    const preparation = !stopping && !ended && mappingPreparation(state.depth_job);
    const restart = document.getElementById('live-depth-restart');
    const finalized = ended && ['complete', 'failed', 'cancelled'].includes(state.depth_job?.status);
    restart.disabled = restartPending || !finalized;
    restart.textContent = restartPending ? '새 촬영 시작 중…' : '새로 촬영';
    restart.title = finalized ? '현재 결과를 보존하고 같은 카메라로 새 지도를 만듭니다.' : '촬영 종료 후 지도 저장이 끝나면 새로 촬영할 수 있습니다.';
    document.getElementById('live-depth-steps').replaceChildren(...processingSteps({
      ...state.depth_job, options: {...state.depth_job?.options, mode: 'depth_rgbd'}
    }).map(step => {
      const li = document.createElement('li'); li.dataset.state = step.state;
      li.textContent = `${step.state === 'done' ? '✓ ' : ''}${step.label}`;
      if (step.state === 'active') li.setAttribute('aria-current', 'step');
      return li;
    }));
    stop.disabled = stopping || ended;
    stop.textContent = ended ? '촬영 종료됨' : stopping ? '촬영 종료 중…' : '촬영 종료';
    panel.querySelector('h2').textContent = ended ? '실시간 영상의 공간 복원 결과' : stopping ? '촬영을 종료하고 누적 지도를 저장하고 있습니다' : preparation ? preparation.title : '실시간 영상에서 공간을 복원하고 있습니다';
    const error = document.getElementById('live-depth-error');
    const renderWindow = state.depth_job?.options?.render_window_s;
    const legacyWarning = renderWindow != null && !ended
      ? `현재 실행 서버는 최근 ${renderWindow}초 지도만 표시합니다. 촬영을 종료한 뒤 기존 서버 창을 닫고 실행 CMD를 다시 켜 새 주소를 여세요.` : '';
    error.hidden = !state.last_error && !stopError && !legacyWarning;
    error.textContent = stopError || (state.last_error ? `복원을 마치지 못했습니다. ${state.last_error}` : legacyWarning);
    const open = document.getElementById('live-depth-open');
    open.hidden = state.depth_job?.status !== 'complete';
    open.href = `/?asset=${encodeURIComponent(state.depth_job_id)}#saved`;
  }};
}
