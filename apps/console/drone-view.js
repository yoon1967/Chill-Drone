const statusText = {observed: '응답 확인', unsupported: '미지원 응답', unconfirmed: '미확인 · 응답 없음', stale: '오래된 값'};
export function renderDroneStages(source, state, element = id => document.getElementById(id)) {
  const started = state.mapping_active === true || state.finalizing === true ||
    Boolean(state.mapping_session_id && state.map_available);
  element('reconstruction-stage').hidden = source === 'live' && !started;
  element('live-map-placeholder').hidden = source !== 'live' || started;
}
export function wifiSignal(wifi) {
  if (wifi?.status === 'pending') return '신호 조회 중…';
  if (!wifi || wifi.status !== 'ok') return wifi?.status === 'stale' ? '신호 확인 지연 · 오래된 값' : '신호 확인 불가';
  if (!wifi.connected) return 'Wi-Fi 연결 안 됨';
  const signal = wifi.signal_pct;
  if (!Number.isFinite(signal) || signal < 0 || signal > 100) return '신호 강도 미확인';
  return `${signal}% · ${signal >= 75 ? '강함' : signal >= 40 ? '보통' : '약함'}`;
}
const errors = {
  drone_input_is_active: '영상·보정 입력이 사용 중입니다. 입력을 종료한 뒤 연결을 시도하세요.',
  scan_and_select_a_drone_first: '목록을 검색하고 드론을 선택하세요.',
  connect_a_drone_first: '먼저 드론에 연결하세요.',
  drone_ports_busy_or_unavailable: '드론 통신 포트를 열지 못했습니다. 다른 드론 수신 프로그램을 종료하고 다시 시도하세요.',
  tello_sdk_unavailable_check_wifi: '드론 SDK 응답이 없습니다. 전원과 PC Wi-Fi 연결을 확인하세요.',
  wifi_os_request_failed_check_permissions_and_saved_profile: 'Wi-Fi 연결을 요청하지 못했습니다. OS에서 해당 Wi-Fi에 한 번 연결해 프로필을 만든 뒤 다시 시도하세요.',
  camera_session_mismatch: '지도 실행 회차가 바뀌었습니다. 상태를 확인한 뒤 다시 연결 해제하세요.',
  land_before_disconnect: '비행 중입니다. 착륙을 완료한 뒤 드론 연결을 해제하세요.',
};

export function connectDrones({element = id => document.getElementById(id), fetcher = fetch,
  schedule = setTimeout, onState = () => {}, onDisconnected = async () => {}, onMapStart = async () => {}} = {}) {
  let pending = false, connected = false, open = false, preview = false;
  let lastState = {}, epoch = 0;
  let wifiVisible = false;
  let selectedWifi = null;
  const wifiRows = new Map();
  function chooseWifi(id) {
    selectedWifi = id;
    for (const [key, row] of wifiRows) row.setAttribute('aria-selected', String(key === id));
    element('drone-connect').disabled = pending || !id;
  }
  function renderWifi(items) {
    const list = element('drone-list'); list.replaceChildren(); wifiRows.clear();
    chooseWifi(null);
    for (const item of items) {
      const row = document.createElement('button');
      row.type = 'button'; row.className = 'wifi-row'; row.setAttribute('role', 'option');
      row.setAttribute('aria-selected', 'false');
      const label = document.createElement('span'), signal = document.createElement('span');
      label.textContent = item.ssid;
      signal.textContent = item.kind === 'current' ? 'OS에서 직접 연결한 드론 확인' :
        `${item.signal_pct == null ? '신호 미확인' : '검색 시 ' + item.signal_pct + '%'} · ${item.drone_candidate ? '드론 후보' : 'SDK 미확인 Wi-Fi'}`;
      row.baseSignal = signal.textContent;
      row.append(label, signal);
      row.signalLabel = signal;
      row.addEventListener('click', () => { if (!pending) chooseWifi(item.id); });
      row.addEventListener('dblclick', () => {
        if (pending) return;
        chooseWifi(item.id); void action('connect', {id: item.id});
      });
      row.addEventListener('keydown', event => {
        if (event.key !== 'Enter' || pending) return;
        event.preventDefault(); chooseWifi(item.id); void action('connect', {id: item.id});
      });
      wifiRows.set(item.id, row); list.append(row);
    }
    element('drone-wifi-results').hidden = false;
  }
  const message = value => { element('drone-status').textContent = value; element('drone-status').hidden = !value; };
  function render(state) {
    lastState = state;
    connected = state.connected;
    const wifi = state.wifi;
    element('drone-wifi-status').hidden = element('drone-wifi-signal').hidden = !wifiVisible && !connected;
    element('drone-wifi-status').textContent = wifi?.status === 'pending' ? 'PC Wi-Fi 상태 조회 중…' : wifi?.status === 'ok' ?
      wifi.connected ? `PC Wi-Fi 연결됨 · ${wifi.ssid}` : 'PC Wi-Fi 연결 안 됨' : 'PC Wi-Fi 상태 확인 불가 · OS 권한/어댑터를 확인하세요.';
    element('drone-wifi-signal').textContent = `연결 신호 · ${wifiSignal(wifi)}${wifi?.age_s != null ? ` · ${wifi.age_s}초 전 측정` : ''}`;
    for (const [id, row] of wifiRows) {
      row.setAttribute('data-connected', String(wifi?.status === 'ok' && wifi?.ssid === id && wifi?.connected));
      if (wifi?.status === 'ok' && wifi.connected && wifi.ssid === id)
        row.signalLabel.textContent = `Wi-Fi 연결됨 · ${wifiSignal(wifi)}`;
      else row.signalLabel.textContent = row.baseSignal;
    }
    onState(state);
    element('drone-camera-panel').hidden = !connected;
    element('drone-map-start').disabled = pending || !connected || state.mapping_active;
    element('drone-flow-status').textContent = state.finalizing ? '연결을 해제했습니다. 마지막 지도 처리가 끝나면 저장 버튼이 표시됩니다…' :
      state.can_save ? '전체 지도 이력 저장 준비가 끝났습니다.' : state.manual_disconnected ? '연결을 해제했습니다. 생성된 지도가 없어 저장할 수 없습니다.' :
      connected ? state.mapping_active ? '3D 지도 생성 중 · 완료하려면 드론 연결 해제를 누르세요.' : '카메라를 확인하고 3D 지도 생성을 시작하세요.' :
      state.disconnect_available ? '연결이 끊겼거나 불안정합니다. 드론 연결 해제를 눌러 지도를 마무리하세요.' : '먼저 드론을 연결하세요.';
    element('drone-details').disabled = !connected;
    element('drone-disconnect').disabled = pending || !(state.disconnect_available ?? connected);
    element('drone-link-state').textContent = connected ? `연결됨 · ${state.selected?.ssid || '사용 중인 영상 입력'}` :
      state.telemetry_age_s != null ? '연결 확인 필요 · 상태 패킷이 오래됐습니다' : '연결 전 · SDK 응답을 확인하세요';
    if (!connected) {
      preview = false;
      element('drone-resource-image').hidden = true;
      open = false;
      element('drone-resource-panel').hidden = true;
      element('drone-details').setAttribute('aria-expanded', 'false');
    }
    if (!open) return;
    const list = element('drone-resources');
    list.replaceChildren();
    for (const item of state.resources || []) {
      const row = document.createElement('div'), name = document.createElement('dt'), value = document.createElement('dd');
      name.textContent = item.name;
      value.textContent = `${statusText[item.status] || item.status} · ${item.value == null ? '—' : typeof item.value === 'object' ? JSON.stringify(item.value) : item.value} ${item.unit || ''}`;
      row.append(name, value); list.append(row);
    }
    element('drone-video-resource').textContent = state.video?.error || state.video?.note || '영상 자원은 확인되지 않았습니다.';
  }
  async function action(path, body = {}) {
    if (pending) return;
    if (path === 'scan' || path === 'connect') wifiVisible = true;
    pending = true;
    if (path === 'connect') onState({can_save: false, connected: false});
    epoch++;
    for (const id of ['drone-scan', 'drone-connect', 'drone-details', 'drone-disconnect', 'drone-resource-refresh', 'drone-video-start', 'drone-map-start']) element(id).disabled = true;
    message(path === 'scan' ? 'PC Wi-Fi에서 드론 후보를 검색하고 있습니다…' : path === 'connect' ? 'Wi-Fi 연결과 드론 SDK 응답을 확인하고 있습니다…' : '드론 응답을 확인하고 있습니다…');
    try {
      const response = await fetcher(`/api/drones/${path}`, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Mapping-Client': '1'}, body: JSON.stringify(body)});
      const state = await response.json();
      if (!response.ok) throw new Error(errors[state.error] || state.error || '드론 요청에 실패했습니다.');
      if (path === 'scan') {
        renderWifi(state.items);
        message(`${state.items.filter(item => item.kind === 'wifi').length}개 Wi-Fi. ${state.warning ? 'OS 검색 제한: Wi-Fi 권한·어댑터를 확인하세요. ' : ''}연결할 항목을 더블클릭하세요. 드론은 SDK 응답까지 확인합니다.`);
      } else {
        if (path === 'preview/start') preview = !state.video?.error;
        if (path === 'preview/stop') preview = false;
        if (path === 'explore') open = true;
        element('drone-resource-panel').hidden = !open;
        element('drone-details').setAttribute('aria-expanded', String(open));
        render(state);
        if (path === 'disconnect') await onDisconnected(state);
        message(path === 'explore' ? '실제 응답한 자원과 미확인 자원을 구분해 표시합니다.' : state.connected ? 'SDK 연결을 확인했습니다.' : '연결을 해제했습니다.');
      }
    } catch (error) { message(error.message); }
    finally {
      pending = false;
      for (const id of ['drone-scan', 'drone-resource-refresh', 'drone-video-start']) element(id).disabled = false;
      element('drone-connect').disabled = !selectedWifi;
      element('drone-details').disabled = !connected;
      await poll(false);
      if (path === 'connect' && connected) await action('preview/start');
    }
  }
  async function poll(repeat = true) {
    const version = epoch;
    if (!pending) {
      try {
        const response = await fetcher('/api/drones/state');
        if (!response.ok) throw new Error();
        const state = await response.json();
        if (version === epoch && !pending) render(state);
        if (connected && (preview || lastState.owner === 'video-input')) element('drone-resource-image').src = `/api/drones/preview.jpg?t=${Date.now()}`;
      } catch {
        if (version === epoch && !pending) {
          connected = false;
          onState({connected: false, can_save: false});
          element('drone-camera-panel').hidden = true;
          preview = false;
          element('drone-resource-image').hidden = true;
          element('drone-link-state').textContent = '확인 불가 · 서버 연결 실패';
          element('drone-wifi-status').textContent = '서버 연결 실패 · Wi-Fi 상태 확인 불가';
          element('drone-wifi-signal').textContent = '신호 확인 불가';
          element('drone-details').disabled = element('drone-disconnect').disabled = true;
          message('서버에 연결할 수 없어 드론 상태를 확인하지 못했습니다.');
        }
      }
    }
    if (repeat) schedule(poll, 3000);
  }
  element('drone-scan').addEventListener('click', () => action('scan'));
  element('drone-connect').addEventListener('click', () => { if (selectedWifi) void action('connect', {id: selectedWifi}); });
  element('drone-disconnect').addEventListener('click', () => action('disconnect', {session_id: lastState.mapping_session_id}));
  element('drone-map-start').addEventListener('click', async () => {
    if (pending || !connected || lastState.mapping_active) return;
    element('drone-map-start').disabled = true;
    try { await onMapStart(); }
    catch (error) { message(error.message); }
    finally { await poll(false); }
  });
  element('drone-resource-refresh').addEventListener('click', () => action('explore'));
  element('drone-video-start').addEventListener('click', () => action('preview/start'));
  element('drone-resource-image').addEventListener('load', () => {
    element('drone-resource-image').hidden = !connected || !(preview || lastState.owner === 'video-input');
    element('drone-image-status').textContent = '현재 카메라 이미지 수신 확인';
  });
  element('drone-resource-image').addEventListener('error', () => {
    element('drone-resource-image').hidden = true;
    element('drone-image-status').textContent = '이미지 수신 대기 · 영상이 없거나 최신 프레임이 없습니다.';
  });
  element('drone-details').addEventListener('click', () => {
    if (!open) { action('explore'); return; }
    open = false; element('drone-resource-panel').hidden = true;
    element('drone-details').setAttribute('aria-expanded', 'false');
  });
  poll();
  return {action};
}
