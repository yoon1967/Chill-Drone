import {requestImport} from './import-api.js';

const messages = {
  invalid_video_folder: '이 PC에 이미 있는 폴더의 전체 경로를 입력하세요.',
  input_folder_changed: '폴더 위치가 바뀌었습니다. 화면을 새로고침한 뒤 다시 열어 주세요.',
  video_folder_open_unavailable: '이 환경에서는 탐색기를 열 수 없습니다.',
  video_folder_open_failed: '탐색기를 열지 못했습니다. 폴더 위치를 확인해 주세요.',
};

export function connectInputFolder(kind, onChange) {
  const get = id => document.getElementById(`${kind}-folder-${id}`);
  const form = get('form'), input = get('input'), status = get('status');
  const open = get('open'), change = get('change');
  const buttons = [open, change, ...form.querySelectorAll('button')];
  let folder, canOpen = false;

  function showForm(show) {
    form.hidden = !show;
    change.setAttribute('aria-expanded', String(show));
    if (show) { input.value = folder || ''; input.focus(); input.select(); }
  }

  async function request(body) {
    buttons.forEach(button => { button.disabled = true; });
    status.textContent = '';
    try {
      const value = await requestImport(`${kind}-folder`, body);
      folder = value.folder;
      canOpen = value.can_open;
      get('path').textContent = folder;
      if (['set', 'reset'].includes(body?.action)) {
        showForm(false);
        status.textContent = '폴더 위치를 저장했습니다.';
        onChange();
      } else if (body?.action === 'open') status.textContent = '파일 탐색기에서 폴더를 열었습니다.';
    } catch (error) {
      if (!folder) get('path').textContent = '폴더 위치를 확인할 수 없습니다.';
      status.textContent = messages[error.cause] || '폴더 요청을 처리하지 못했습니다. 잠시 후 다시 시도해 주세요.';
    } finally {
      buttons.forEach(button => { button.disabled = false; });
      open.disabled = !canOpen;
      if (['set', 'reset'].includes(body?.action)) (form.hidden ? change : input).focus();
      if (body?.action === 'open' && canOpen) open.focus();
    }
  }

  open.addEventListener('click', () => request({action: 'open', folder}));
  change.addEventListener('click', () => showForm(form.hidden));
  form.addEventListener('submit', event => { event.preventDefault(); request({action: 'set', folder: input.value.trim()}); });
  get('reset').addEventListener('click', () => request({action: 'reset'}));
  request();
}
