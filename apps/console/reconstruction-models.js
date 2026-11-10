const $ = id => document.getElementById(id);
const endpoint = '/api/reconstruction-models';
const rows = Object.fromEntries(['pose', 'depth'].map(kind => [kind, {
  select: $(`${kind}-model`), label: $(`${kind}-model-status`), button: $(`${kind}-model-action`),
  progress: $(`${kind}-model-progress`), version: 0, ready: false, action: null, stream: null,
}]));

const start = $('start-reconstruction');
let catalogRequest, busy = false;
const sizes = new Intl.NumberFormat('ko-KR', {minimumFractionDigits: 1, maximumFractionDigits: 1});
const errors = {
  slam_docker_unavailable: 'Docker Desktop을 실행한 후 다시 확인하세요.',
  slam_docker_timeout: 'Docker Desktop이 준비되면 다시 확인하세요.',
  slam_build_failed: 'ORB-SLAM3 설치에 실패했습니다. 인터넷과 Docker 상태를 확인하세요.',
  model_download_failed: '다운로드에 실패했습니다. 인터넷과 저장 공간을 확인하세요.',
};

async function request(path = '', body) {
  const response = await fetch(endpoint + path, {
    cache: 'no-store', signal: AbortSignal.timeout(30000),
    ...(body && {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Mapping-Client': '1'},
      body: JSON.stringify(body)}),
  });
  if (!response.ok) throw new Error('model_request_failed');
  return response.json();
}

function render(row, status = {}) {
  row.ready = status.status === 'ready';
  row.progress.hidden = status.status !== 'downloading';
  row.action = null;
  row.button.hidden = true;
  if (row.ready) row.label.textContent = '모델 준비 완료';
  else if (status.status === 'downloading') {
    row.progress.value = status.percent ?? 0;
    const activity = status.method === 'build' ? '설치 단계' : '다운로드 중';
    row.label.textContent = `${activity} ${row.progress.value}%`;
  } else if (status.status === 'missing' || status.status === 'failed') {
    row.label.textContent = errors[status.error] || '로컬에 모델이 없습니다.';
    row.action = 'download';
    row.button.textContent = '모델 다운받기';
  } else if (status.status === 'blocked') {
    row.label.textContent = errors[status.error] || '모델 상태를 확인할 수 없습니다.';
    row.action = 'check';
    row.button.textContent = '다시 확인';
  } else {
    row.label.textContent = status.status === 'checking' ? '로컬 모델 확인 중' : '모델을 선택하세요.';
  }
  row.button.hidden = !row.action;
  updateButton();
}

function updateButton() {
  start.disabled = busy || Object.values(rows).some(row => !row.ready || !row.select.value);
  for (const row of Object.values(rows)) {
    row.select.disabled = busy || !row.select.options.length;
    row.button.disabled = busy;
  }
}

async function check(row, action = 'check') {
  row.stream?.close();
  row.stream = null;
  const key = row.select.value;
  const version = ++row.version;
  if (!key) return render(row);
  const query = '?id=' + encodeURIComponent(key);
  const current = () => version === row.version;
  render(row, {status: 'checking'});
  try {
    const status = await (action === 'download' ? request('/download', {id: key}) : request('/status' + query));
    if (!current()) return;
    render(row, status);
    if (status.status !== 'downloading') return;
    const stream = new EventSource(endpoint + '/events' + query);
    row.stream = stream;
    stream.onmessage = event => {
      if (!current()) return;
      const next = JSON.parse(event.data);
      render(row, next);
      if (next.status !== 'downloading') {
        stream.close();
        row.stream = null;
      }
    };
    stream.onerror = () => {
      stream.close();
      if (!current()) return;
      row.stream = null;
      render(row, {status: 'blocked'});
    };
  } catch {
    if (current()) render(row, {status: 'blocked'});
  }
}

export function showModelPicker() {
  if (catalogRequest) return catalogRequest;
  const retry = $('model-picker-retry');
  retry.hidden = true;
  modelPickerMessage('모델 목록 불러오는 중');
  catalogRequest = request().then(({models}) => {
    for (const [kind, row] of Object.entries(rows)) {
      const options = models.filter(model => model.kind === kind).map(model => {
        const size = sizes.format(model.size_bytes / 1e6) + ' MB';
        const memory = model.vram_gb ? `권장 VRAM ${model.vram_gb} GB` : 'VRAM 0 GB · CPU';
        return new Option(`${model.name} (${size} · ${memory})`, model.id);
      });
      row.select.replaceChildren(new Option('모델을 선택하세요', ''), ...options);
      render(row);
    }
    modelPickerMessage('');
  }).catch(() => {
    catalogRequest = null;
    modelPickerMessage('모델 목록을 불러오지 못했습니다.');
    retry.hidden = false;
  });
  return catalogRequest;
}

export function selectedModels() {
  if (start.disabled) return null;
  return {pose_model: rows.pose.select.value, depth_model: rows.depth.select.value};
}

export function setModelPickerBusy(next) {
  busy = next;
  updateButton();
}

export function modelPickerMessage(text) { $('model-picker-error').textContent = text; }
export function refreshModelChecks() { for (const row of Object.values(rows)) check(row); }
export function closeModelPicker({clear = false} = {}) {
  for (const row of Object.values(rows)) {
    row.version++;
    row.stream?.close();
    row.stream = null;
    if (clear) {
      row.select.value = '';
      render(row);
    }
  }
}

for (const row of Object.values(rows)) {
  row.select.addEventListener('change', () => {
    modelPickerMessage('');
    check(row);
  });
  row.button.addEventListener('click', () => check(row, row.action));
}
$('model-picker-retry').addEventListener('click', showModelPicker);
