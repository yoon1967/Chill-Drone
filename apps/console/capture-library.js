// Selection contract only. A reconstruction consumer can read this bundle later.
let selection = null;
export function selectedCaptureBundle() { return selection ? structuredClone(selection) : null; }

export function connectCaptureLibrary({request, onSelect, onClear}) {
  const $ = id => document.getElementById(id);
  let timer, enabled = false, loading = false, version = 0, rendered = '';
  const date = value => value ? new Date(value).toLocaleString('ko-KR') : '촬영 시각 없음';

  function clear() {
    const hadSelection = Boolean(selection);
    selection = null;
    $('capture-selection').hidden = true;
    for (const button of $('captures-list').querySelectorAll('button')) button.setAttribute('aria-pressed', 'false');
    if (hadSelection) document.dispatchEvent(new CustomEvent('mapping-capture-selected', {detail: null}));
  }

  async function select(record) {
    if (loading || !enabled || !record.selectable) return;
    const current = version;
    loading = true;
    $('captures-status').textContent = '촬영 자료 확인 중';
    try {
      const bundle = await request('capture-bundle', undefined, '?session_id=' + encodeURIComponent(record.session_id));
      if (!enabled || current !== version) return;
      selection = bundle;
      $('capture-name').textContent = bundle.name;
      const count = bundle.files.filter(file => file.role === 'video').length;
      $('capture-summary').textContent = `영상 ${count}개 · 센서 로그 · 시각 대응 정보 · 촬영 메타데이터`;
      $('capture-warning').hidden = !['partial', 'approximate'].includes(bundle.sync_status);
      $('capture-warning').textContent = bundle.sync_status === 'partial' ?
        '일부 구간의 영상·센서 시각 대응 정보가 불완전합니다.' : '영상·센서 시각 대응에 수신 시각 기준 근삿값이 포함되어 있습니다.';
      $('capture-files').replaceChildren(...bundle.files.map(file => {
        const item = document.createElement('li');
        item.textContent = file.path;
        return item;
      }));
      $('capture-selection').hidden = false;
      $('captures-status').textContent = '영상과 센서 자료를 함께 선택했습니다.';
      for (const button of $('captures-list').querySelectorAll('button')) {
        button.setAttribute('aria-pressed', String(button.dataset.sessionId === bundle.session_id));
      }
      onSelect(bundle);
      document.dispatchEvent(new CustomEvent('mapping-capture-selected', {detail: selectedCaptureBundle()}));
    } catch (error) {
      if (current === version && enabled) $('captures-status').textContent = error.cause === 'capture_recording' ?
        '아직 촬영 중입니다. 촬영이 끝난 뒤 선택하세요.' : '촬영 자료를 연결할 수 없습니다. 파일이 모두 저장되었는지 확인하세요.';
    } finally { loading = false; }
  }

  async function refresh() {
    clearTimeout(timer);
    if (!enabled || document.hidden) return;
    const current = version;
    try {
      const value = await request('capture-library');
      if (!enabled || current !== version) return;
      $('captures-root').textContent = value.root_path;
      const signature = JSON.stringify(value.items);
      if (signature !== rendered) {
        rendered = signature;
        $('captures-status').textContent = value.items.length ? '' : '아직 저장된 실시간 감독 촬영이 없습니다.';
        $('captures-list').replaceChildren(...value.items.map(record => {
          const item = document.createElement('li'), button = document.createElement('button');
          button.type = 'button'; button.className = 'recent-record capture-record';
          button.dataset.sessionId = record.session_id;
          button.setAttribute('aria-pressed', String(selection?.session_id === record.session_id));
          button.disabled = !record.selectable;
          const title = document.createElement('strong'), detail = document.createElement('span');
          title.textContent = record.name;
          detail.textContent = `${date(record.saved_at || record.started_at)} · 영상 ${record.parts.length}개${record.saved_at ? ' · 누적 저장본' : ''}`;
          const label = document.createElement('span');
          if (record.status === 'recording') label.textContent = '촬영 중 · 저장이 끝나면 선택할 수 있습니다.';
          else if (!record.selectable) label.textContent = '자료 미완료 · ' + record.missing.join(', ');
          else if (record.sync_status === 'partial') label.textContent = '센서 포함 · 시각 대응 일부 불완전';
          else if (record.sync_status === 'approximate') label.textContent = '센서 포함 · 시각 대응 근삿값';
          else label.textContent = '센서 포함 · 시각 대응 연결';
          if (!record.selectable || record.sync_status !== 'complete') label.className = 'capture-issue';
          button.append(title, detail, label);
          button.addEventListener('click', () => select(record));
          item.append(button);
          return item;
        }));
      }
    } catch {
      if (enabled && current === version) {
        rendered = '';
        $('captures-status').textContent = '촬영 목록을 불러올 수 없습니다. 새로고침으로 다시 확인하세요.';
      }
    } finally { if (enabled && current === version) timer = setTimeout(refresh, 5000); }
  }

  $('captures-refresh').addEventListener('click', refresh);
  $('captures-open-folder').addEventListener('click', async () => {
    const button = $('captures-open-folder');
    button.disabled = true;
    try {
      await request('capture-folder', {});
      $('captures-status').textContent = '파일 탐색기에서 촬영 저장 폴더를 열었습니다.';
    } catch { $('captures-status').textContent = '촬영 저장 폴더를 열 수 없습니다. 표시된 경로를 확인하세요.'; }
    finally { button.disabled = false; }
  });
  $('capture-clear').addEventListener('click', () => { clear(); onClear(); });
  document.addEventListener('visibilitychange', () => { if (!document.hidden && enabled) refresh(); });
  return {
    clear,
    setEnabled(value) {
      if (enabled === value) return;
      enabled = value; version++;
      clearTimeout(timer);
      if (enabled) refresh();
    },
  };
}
