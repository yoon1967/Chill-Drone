const messages = {
  invalid_video_folder: '이 PC에 이미 있는 폴더의 전체 경로를 입력하세요.',
  video_folder_open_unavailable: '이 환경에서는 탐색기를 열 수 없습니다.',
  video_folder_open_failed: '탐색기를 열지 못했습니다. 폴더 위치를 확인해 주세요.',
};

export function connectVideoFolder() {
  const get = id => document.getElementById(`video-folder-${id}`);
  const form = get('form'), input = get('input'), status = get('status');
  const open = get('open'), change = get('change');
  let folder;

  function showForm(show) {
    form.hidden = !show;
    change.setAttribute('aria-expanded', String(show));
    if (show) { input.value = folder || ''; input.focus(); input.select(); }
  }

  async function request(body) {
    const buttons = [open, change, ...form.querySelectorAll('button')];
    buttons.forEach(button => { button.disabled = true; });
    status.hidden = true;
    let canOpen = open.dataset.supported === 'true';
    try {
      const response = await fetch('/api/import/video-folder', body ? {
        method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'},
        body: JSON.stringify(body),
      } : {cache: 'no-store'});
      const value = await response.json();
      if (!response.ok) throw new Error(value.error);
      folder = value.folder;
      canOpen = value.can_open;
      open.dataset.supported = String(canOpen);
      get('path').textContent = folder;
      if (['set', 'reset'].includes(body?.action)) {
        showForm(false);
        status.textContent = '영상 폴더 위치를 저장했습니다.';
        status.hidden = false;
      }
    } catch (error) {
      if (!folder) get('path').textContent = '폴더 위치를 확인할 수 없습니다.';
      status.textContent = messages[error.message] || '영상 폴더 요청을 처리하지 못했습니다. 잠시 후 다시 시도해 주세요.';
      status.hidden = false;
    } finally {
      buttons.forEach(button => { button.disabled = false; });
      open.disabled = !canOpen;
      if (body?.action !== 'open' && form.hidden && folder && !status.hidden) change.focus();
    }
  }

  open.addEventListener('click', () => request({action: 'open'}));
  change.addEventListener('click', () => showForm(form.hidden));
  form.addEventListener('submit', event => { event.preventDefault(); request({action: 'set', folder: input.value.trim()}); });
  get('reset').addEventListener('click', () => request({action: 'reset'}));
  request();
}
