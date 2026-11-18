export function saveShortcut(event) {
  return (event.ctrlKey || event.metaKey) && !event.altKey && event.key.toLowerCase() === 's' &&
    !event.target?.isContentEditable && !/INPUT|SELECT|TEXTAREA/.test(event.target?.tagName || '');
}

export function mapDownload(source, run) {
  if (!run || (!run.points?.length && !run.stream?.session_id) || run.source !== source.source) throw new Error('저장할 지도가 없습니다.');
  const query = new URLSearchParams();
  if (run.stream?.session_id) query.set('session', run.stream.session_id);
  return source.url(`map.zip?${query}`);
}

export function connectMapSave({source, scene, element = id => document.getElementById(id),
  allowed = () => true,
  fetcher = fetch, download = (blob, name) => {
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = name; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }, target = window}) {
  let busy = false;
  function update() {
    const ready = allowed();
    const bar = element('map-save-bar');
    if (bar) bar.hidden = !ready;
    element('map-save').disabled = busy || !ready;
  }
  async function save() {
    if (busy) return;
    if (!allowed()) {
      element('map-save-status').textContent = '드론 연결 해제 후 마지막 지도 처리가 끝나면 저장할 수 있습니다.';
      return;
    }
    busy = true;
    element('map-save').disabled = true;
    try {
      const run = scene(), url = mapDownload(source, run);
      element('map-save-status').textContent = '3D 지도 전체 이력을 저장하고 있습니다…';
      const response = await fetcher(url);
      if (!response.ok) throw new Error('해당 지도 회차를 저장할 수 없습니다. 지도를 확인한 뒤 다시 시도하세요.');
      download(await response.blob(), `${String(run.input_run_id || '3d-map').replace(/[^a-zA-Z0-9_-]/g, '_')}-history.zip`);
      element('map-save-status').textContent = '지도 전체 이력 ZIP을 다운로드했습니다. 자료 불러오기에서 타임라인을 다시 열 수 있습니다.';
    } catch (error) { element('map-save-status').textContent = error.message; }
    finally { busy = false; update(); }
  }
  element('map-save').addEventListener('click', save);
  target.addEventListener('keydown', event => {
    if (!saveShortcut(event)) return;
    event.preventDefault();
    save();
  });
  update();
  return {save, update};
}
