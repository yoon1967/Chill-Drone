export const workflowSteps = {
  live: ['드론 연결', '카메라 보정·설정', '실시간 지도 생성', '결과 확인·저장'],
  saved: ['영상 선택', '복원 설정', '공간 복원', '결과 확인'],
};
export function liveStep(state) {
  if (state.finalizing || state.can_save || (state.mapping_session_id && state.map_available && !state.mapping_active)) return 3;
  if (state.mapping_active) return 2;
  return state.connected ? 1 : 0;
}
export const canVisitStep = (current, target) => Number.isInteger(target) && target >= 0 && target <= current;

export function connectWorkflowGuide({onNavigate, element = id => document.getElementById(id)} = {}) {
  const states = {live: {current: 0, observed: 0}, saved: {current: 0, observed: 0}};
  let source = location.hash === '#live' ? 'live' : 'saved';
  let pinned = false;
  const panel = element('workflow-guide'), trigger = element('workflow-toggle');
  function expand(value) { panel.dataset.open = String(value); trigger.setAttribute('aria-expanded', String(value)); }
  function render() {
    const {current} = states[source];
    element('workflow-title').textContent = source === 'live' ? '실시간 지도 순서' : '저장 지도 순서';
    element('workflow-current').textContent = `${current + 1}. ${workflowSteps[source][current]}`;
    element('workflow-list').replaceChildren(...workflowSteps[source].map((label, index) => {
      const li = document.createElement('li'), button = document.createElement('button');
      button.type = 'button'; button.textContent = `${index + 1}. ${label}`;
      button.disabled = !canVisitStep(current, index);
      if (index === current) button.setAttribute('aria-current', 'step');
      button.dataset.state = index < current ? 'done' : index === current ? 'active' : 'waiting';
      button.addEventListener('click', async () => {
        if (!canVisitStep(states[source].current, index)) return;
        const selectedSource = source;
        if (await onNavigate(selectedSource, index) === false) return;
        states[selectedSource].current = index; render();
      });
      li.append(button); return li;
    }));
  }
  trigger.addEventListener('click', () => { pinned = !pinned; expand(pinned); });
  panel.addEventListener('mouseenter', () => expand(true));
  panel.addEventListener('mouseleave', () => { if (!pinned && !panel.contains(document.activeElement)) expand(false); });
  panel.addEventListener('focusin', () => expand(true));
  panel.addEventListener('focusout', event => { if (!panel.contains(event.relatedTarget)) { pinned = false; expand(false); } });
  panel.addEventListener('keydown', event => { if (event.key === 'Escape') { pinned = false; trigger.focus(); expand(false); } });
  render();
  return {
    current(value) { return states[value].current; },
    select(value) { source = value; render(); },
    update(value, current) {
      // Repeated state polling must not undo a user's return to an earlier step.
      if (states[value].observed === current) return;
      states[value] = {current, observed: current};
      if (source === value) render();
    },
  };
}
