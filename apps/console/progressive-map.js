import {ProgressiveReplay} from '/progressive-replay.js';
import * as THREE from '/vendor/three.module.js';
import {OrbitControls} from '/vendor/OrbitControls.js';

export function connectProgressiveMap() {
  const $ = id => document.getElementById(id);
  const listeners = new AbortController();
  const on = (id, event, callback) => $(id).addEventListener(event, callback, {signal: listeners.signal});
  const host = $('progressive-viewport');
  const renderer = new THREE.WebGLRenderer({antialias: false, powerPreference: 'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  host.append(renderer.domElement);
  const scene = new THREE.Scene(); scene.background = new THREE.Color('#142326');
  const camera = new THREE.PerspectiveCamera(55, 1, .02, 100);
  camera.position.set(2, -2, -3); camera.up.set(0, -1, 0);
  const controls = new OrbitControls(camera, renderer.domElement);
  const geometry = new THREE.BufferGeometry();
  const cloud = new THREE.Points(geometry, new THREE.PointsMaterial({size: 2, sizeAttenuation: false, vertexColors: true}));
  scene.add(cloud);
  const route = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: '#f1c36c'})); scene.add(route);
  const cameraMarker = new THREE.Group();
  const corners = [[-.05, -.035, .08], [.05, -.035, .08], [.05, .035, .08], [-.05, .035, .08]];
  const edges = corners.flatMap((point, i) => [[0, 0, 0], point, point, corners[(i + 1) % 4]]).map(p => new THREE.Vector3(...p));
  const markerLines = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(edges), new THREE.LineBasicMaterial({color: '#ffb95e'}));
  cameraMarker.add(markerLines); cameraMarker.visible = false; scene.add(cameraMarker);
  let active = null, revision = 0, generation = 0, timer = null, polling = false, first = true, running = false, disposed = false, again = false;
  let replay = new ProgressiveReplay(), historyAt = 0;
  let resultReady = false;
  function replayControls() {
    const available = replay.entries.length > 0;
    $('progressive-timeline').disabled = $('progressive-play').disabled = $('progressive-seek-go').disabled = !available;
    $('progressive-timeline').min = replay.first; $('progressive-timeline').max = replay.last;
    $('progressive-timeline').value = replay.time;
    if (document.activeElement !== $('progressive-seek')) $('progressive-seek').value = replay.time.toFixed(2);
    $('progressive-seek').max = replay.last;
    $('progressive-replay-time').textContent = `${replay.time.toFixed(2)} / ${replay.last.toFixed(2)}s`;
    $('progressive-play').textContent = replay.playing ? '\uC77C\uC2DC\uC815\uC9C0' : '\uC7AC\uC0DD';
    $('progressive-follow').setAttribute('aria-pressed', String(replay.following));
    $('progressive-replay-state').textContent = replay.following ?
      (resultReady ? '마지막 복원 기록 표시' : '최신 복원 표시') :
      (running ? '기록 재생 · 복원은 계속 진행' : '저장된 복원 기록 재생');
    $('progressive-download').hidden = !available || !resultReady;
    $('progressive-download').href = `/api/import/history.zip?id=${encodeURIComponent(active)}`;
  }
  function changed() { clearTimeout(timer); replayControls(); poll(); }
  on('progressive-timeline', 'input', event => { replay.seek(Number(event.target.value)); changed(); });
  on('progressive-seek-go', 'click', () => { replay.seek(Number($('progressive-seek').value)); changed(); });
  on('progressive-play', 'click', () => { if (replay.playing) replay.pause(); else replay.play(performance.now()); changed(); });
  on('progressive-follow', 'click', () => { replay.follow(); changed(); });
  on('progressive-speed', 'change', () => { replay.tick(performance.now()); replay.rate = Number($('progressive-speed').value); });
  function fit() {
    if (!geometry.getAttribute('position')?.count) return;
    geometry.computeBoundingSphere();
    const {center, radius} = geometry.boundingSphere;
    controls.target.copy(center); camera.position.copy(center).add(new THREE.Vector3(1, -.6, -1.3).multiplyScalar(Math.max(radius, .5) * 1.7));
    controls.update();
  }
  const observer = new ResizeObserver(() => {
    const {width, height} = host.getBoundingClientRect();
    if (width && height) { renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); }
  }); observer.observe(host);
  renderer.setAnimationLoop(() => { if (!disposed && !document.hidden && !host.closest('[hidden]')) { controls.update(); renderer.render(scene, camera); } });
  async function poll() {
    if (disposed || !active) return;
    if (polling) { again = true; return; }
    again = false;
    polling = true; const id = active, token = generation; let cursor = replay.version;
    try {
      if (performance.now() - historyAt > 1000 || !historyAt) {
        historyAt = performance.now();
        const history = await fetch(`/api/import/history?id=${encodeURIComponent(id)}`, {cache: 'no-store', signal: AbortSignal.timeout(10000)});
        if (token !== generation || disposed) return;
        if (history.ok) replay.setEntries((await history.json()).entries);
      }
      replay.tick(performance.now()); replayControls(); cursor = replay.version;
      const selected = replay.selected();
      if (!replay.following && (!selected || selected.revision === revision)) return;
      const endpoint = replay.following ? `preview?id=${encodeURIComponent(id)}&revision=${revision}` : `history-frame?id=${encodeURIComponent(id)}&revision=${selected.revision}`;
      const response = await fetch(`/api/import/${endpoint}`, {cache: 'no-store', signal: AbortSignal.timeout(10000)});
      const value = await response.json();
      if (token !== generation || cursor !== replay.version || disposed || !response.ok || value.waiting || value.unchanged) return;
      // Decode the corresponding image first; stale responses never replace another job.
      const image = new Image();
      image.src = `/api/import/preview-frame?id=${encodeURIComponent(id)}&name=${encodeURIComponent(value.frame_id)}`;
      await image.decode();
      if (token !== generation || cursor !== replay.version || disposed) return;
      revision = value.revision;
      $('progressive-frame').src = image.src; $('progressive-frame').hidden = false; $('progressive-video-empty').hidden = true;
      $('progressive-frame-time').textContent = `${value.timestamp_s.toFixed(2)}초 · 원본 ${value.frame_number}번째 프레임`;
      $('progressive-frame-detail').textContent = {lost: '위치·방향 추적 실패 · 깊이 추정과 지도 반영을 건너뛰고 마지막 지도를 유지합니다.',
        initializing: '영상으로 카메라 위치·방향을 초기화 중입니다. 아직 깊이 추정을 실행하지 않습니다.',
        scale_wait: '위치·방향은 확인했습니다. 같은 좌표 크기로 깊이를 연결할 근거를 기다립니다.',
        tracking: '먼저 확정한 이 프레임의 카메라 자세를 유지한 채 깊이와 지도를 복원했습니다.'}[value.tracking] || '프레임 처리 중';
      const confirmed = Boolean(value.pose_confirmed && value.pose_position && value.pose_rotation);
      cameraMarker.visible = confirmed && value.map_id === value.pose_map_id;
      if (confirmed) {
        const r = value.pose_rotation;
        cameraMarker.position.fromArray(value.pose_position);
        cameraMarker.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(r[0][0], r[0][1], r[0][2], 0, r[1][0], r[1][1], r[1][2], 0, r[2][0], r[2][1], r[2][2], 0, 0, 0, 0, 1));
        $('progressive-pose').textContent = `위치·방향 확정 · 원본 ${value.frame_number}번째 · 위치 (${value.pose_position.map(n => n.toFixed(3)).join(', ')}) · 전방 (${r.map(row => row[2].toFixed(3)).join(', ')}) · 단안 상대 좌표${value.submaps > 1 ? ' · 다른 지도 구간은 분리 보존' : ''}`;
      } else $('progressive-pose').textContent = '위치·방향 확정 대기 · 이 프레임은 깊이 복원에 사용하지 않습니다.';
      const position = new Float32Array(value.points.length * 3), color = new Float32Array(position.length);
      value.points.forEach((p, i) => { position.set(p.slice(0, 3), i * 3); color.set(p.slice(3, 6).map(c => c / 255), i * 3); });
      geometry.dispose();
      geometry.setAttribute('position', new THREE.BufferAttribute(position, 3));
      geometry.setAttribute('color', new THREE.BufferAttribute(color, 3)); geometry.computeBoundingSphere();
      route.geometry.dispose(); route.geometry = new THREE.BufferGeometry().setFromPoints(value.cameras.map(p => new THREE.Vector3(...p)));
      $('progressive-map-empty').hidden = value.points.length > 0;
      $('progressive-count').textContent = `처리 ${value.processed} · 지도 반영 ${value.registered} · 대기 교체 ${value.dropped}프레임 · ${(value.map_points ?? value.points.length).toLocaleString()}점`;
      $('progressive-latency').textContent = `영상 시각 → 지도 ${value.latency_ms}ms · 계산 ${value.processing_ms}ms`;
      $('progressive-map-detail').textContent = `\uCD5C\uADFC ${value.render_window_s ?? 10}\uCD08 \uD45C\uC2DC \u00B7 \uD654\uBA74\uC5D0\uC11C \uC81C\uC678\uD55C \uBCF5\uC6D0 \uC790\uB8CC\uB3C4 \uAE30\uB85D \uBCF4\uC874`;
      if (first && value.points.length) { first = false; fit(); }
    } catch (error) {
      if (token === generation && !disposed) $('progressive-map-detail').textContent = `지도 갱신 재시도 중 · ${error.message}`;
    } finally {
      polling = false;
      if ((running || replay.playing || again) && !disposed) timer = setTimeout(poll, 200);
    }
  }
  return {update(job) {
    if (active !== job.id) { active = job.id; revision = 0; replay = new ProgressiveReplay(); historyAt = 0; generation++; first = true;
      geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(), 3));
      $('progressive-frame').hidden = true; $('progressive-video-empty').hidden = false; }
    running = ['running', 'cancelling'].includes(job.status);
    resultReady = ['ready', 'complete'].includes(job.status);
    $('progressive-state').textContent = job.status === 'ready' && job.history_available ? '저장된 복원 과정 다시보기 · 저장 지도 열기로 결과를 확인하세요' : job.status === 'complete' ? '복원 완료 · 저장 지도 열기로 계속 살펴보세요' : job.status === 'failed' ? '처리 중단 · 마지막 지도 유지' : job.status === 'cancelled' ? '취소됨 · 마지막 지도 유지' :
      {model: '추적·깊이 모델 준비 중', depth: '자세 확정 완료 → 깊이 추정 중', tracking: '카메라 위치·방향 추적 중', pose_wait: '자세 확정 대기 · 깊이 복원 보류', mapping: '확정한 자세로 지도 복원 중', saving: '전체 통합 지도 저장 중'}[job.stage] || '영상 처리 준비 중';
    if (job.stage === 'integration') $('progressive-state').textContent = `전체 경로 통합 중 · ${job.progress?.integrated_frames || 0} / ${job.progress?.integration_total || 0}`;
    clearTimeout(timer); poll();
  }, dispose() { listeners.abort(); disposed = true; running = false; generation++; clearTimeout(timer); observer.disconnect(); renderer.setAnimationLoop(null); controls.dispose(); geometry.dispose(); cloud.material.dispose(); route.geometry.dispose(); route.material.dispose(); markerLines.geometry.dispose(); markerLines.material.dispose(); renderer.dispose(); renderer.domElement.remove(); }};
}
