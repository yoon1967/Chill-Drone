// The console owns the recording. This client never sends flight/video commands.
window.createLiveMapSave = function ({fetcher = fetch, onChange = () => {}} = {}) {
  let base = null, timer, loading = false, posting = false, generation = 0, signature = '', readError = false;
  let value = {available: false, latest: null, session_id: null, error: ''};
  const messages = {
    checkpoint_busy: '이전 저장을 마무리하고 있습니다.',
    checkpoint_map_not_ready: '지도가 준비된 뒤 다시 저장하세요.',
    checkpoint_video_not_ready: '촬영 영상이 들어온 뒤 다시 저장하세요.',
    checkpoint_session_finished: '촬영 종료 후 저장 마무리 중입니다.',
    checkpoint_session_unavailable: '현재 촬영 회차를 확인한 뒤 다시 저장하세요.',
  };
  function state() {
    const latest = value.latest;
    const busy = posting || latest?.status === 'saving';
    let message = '촬영 시작부터 누적 저장합니다. 영상과 조종은 계속됩니다.';
    if (value.error) message = value.error;
    else if (busy) message = '요청 시점까지의 지도·영상·센서 자료 저장 중…';
    else if (latest?.status === 'complete') message = latest.capture_status === 'partial' ?
      '저장 완료 · 일부 영상·센서 기록 확인 필요' : '지도·영상·센서 자료 저장 완료';
    else if (latest?.status === 'failed') message = '저장하지 못했습니다. 저장 공간을 확인한 뒤 다시 시도하세요.';
    else if (!value.available) message = '실시간 지도가 준비되면 저장할 수 있습니다.';
    return {busy, available: Boolean(base && value.available), message,
      path: latest?.status === 'complete' ? latest.path || '' : ''};
  }
  function notify() {
    const next = JSON.stringify(state());
    if (next !== signature) { signature = next; onChange(state()); }
  }
  async function refresh() {
    clearTimeout(timer);
    if (!base || loading) return;
    const current = generation, target = base;
    loading = true;
    try {
      const response = await fetcher(target + '/api/live/checkpoint-status', {cache: 'no-store', signal: AbortSignal.timeout(10000)});
      if (!response.ok) throw Error('status_unavailable');
      const next = await response.json();
      if (current === generation) {
        const changed = next.latest?.id !== value.latest?.id || next.latest?.status !== value.latest?.status;
        value = {...next, error: changed || readError ? '' : value.error};
        readError = false;
      }
    } catch {
      if (current === generation) {
        readError = true;
        value = {...value, available: false, error: '지도 저장 서버에 연결할 수 없습니다.'};
      }
    } finally {
      loading = false;
      if (current === generation) { notify(); timer = setTimeout(refresh, 2000); }
      else if (base) refresh();
    }
  }
  async function save() {
    if (state().busy || !state().available) return;
    const current = generation, target = base;
    posting = true; value.error = ''; notify();
    try {
      const response = await fetcher(target + '/api/live/checkpoint', {method: 'POST',
        headers: {'Content-Type': 'application/json', 'X-Mapping-Client': '1'},
        body: JSON.stringify({session_id: value.session_id}), signal: AbortSignal.timeout(15000)});
      const next = await response.json();
      if (!response.ok) throw Error(messages[next.error] || '지도를 저장할 수 없습니다. 잠시 뒤 다시 시도하세요.');
      if (current === generation) value.latest = next;
    } catch (error) {
      if (current === generation) value.error = error.message;
    } finally {
      posting = false;
      if (current === generation) { notify(); refresh(); }
    }
  }
  return {state, save, refresh,
    setTarget(origin) {
      if (base === origin) return;
      if (origin) {
        const parsed = new URL(origin);
        if (!['http:', 'https:'].includes(parsed.protocol) || parsed.origin !== origin) return;
      }
      base = origin; generation++;
      value = {available: false, latest: null, session_id: null, error: ''};
      clearTimeout(timer);
      if (base) refresh();
    },
    dispose() { base = null; generation++; clearTimeout(timer); },
  };
};
