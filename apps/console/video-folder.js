import {requestImport} from './import-api.js';

const messages = {
  invalid_video_folder: '폴더 위치를 사용할 수 없습니다. 이 PC에 있는 로컬 폴더를 선택하세요.',
  input_folder_changed: '폴더 위치가 바뀌었습니다. 화면을 새로고침한 뒤 다시 열어 주세요.',
  video_folder_open_unavailable: '이 환경에서는 탐색기를 열 수 없습니다.',
  video_folder_open_failed: 'Windows 파일 탐색기를 시작하지 못했습니다.',
  video_folder_pick_unavailable: '이 환경에서는 Windows 폴더 선택 창을 열 수 없습니다.',
  video_folder_pick_failed: 'Windows 폴더 선택 창을 열지 못했습니다. 다시 시도해 주세요.',
};

export function connectInputFolder(kind, onChange) {
  const get = id => document.getElementById(`${kind}-folder-${id}`);
  const status = get('status');
  const open = get('open'), change = get('change'), reset = get('reset');
  const buttons = [open, change, reset];
  let folder, canOpen = false, canPick = false;

  async function request(body) {
    buttons.forEach(button => { button.disabled = true; });
    status.textContent = '';
    try {
      const value = await requestImport(`${kind}-folder`, body);
      folder = value.folder;
      canOpen = value.can_open;
      canPick = value.can_pick;
      reset.hidden = !value.configured;
      get('path').textContent = folder;
      if (value.cancelled) {
        status.textContent = '폴더 선택을 취소했습니다.';
      } else if (['reset', 'pick'].includes(body?.action)) {
        status.textContent = '폴더 위치를 저장했습니다.';
        onChange();
      } else if (body?.action === 'open') {
        status.textContent = '파일 탐색기에서 폴더를 열었습니다.';
      }
    } catch (error) {
      if (!folder) get('path').textContent = '폴더 위치를 확인할 수 없습니다.';
      status.textContent = messages[error.cause] || '폴더 요청을 처리하지 못했습니다. 잠시 후 다시 시도해 주세요.';
    } finally {
      buttons.forEach(button => { button.disabled = false; });
      open.disabled = !canOpen;
      change.disabled = !canPick;
      if (['pick', 'reset'].includes(body?.action) && canPick && document.hasFocus()) change.focus();
    }
  }

  open.addEventListener('click', () => request({action: 'open', folder}));
  change.addEventListener('click', () => request({action: 'pick'}));
  reset.addEventListener('click', () => request({action: 'reset'}));
  request();
}
