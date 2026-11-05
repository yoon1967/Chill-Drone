import {ProgressiveReplay} from '/progressive-replay.js';
import {createRescueLayer} from '/rescue-view.js';
import {mappingPreparation} from '/map-source.js';
import {mappingSettings, onMappingSettings, rescuePointColor} from '/mapping-settings.js';
import * as THREE from '/vendor/three.module.js';
import {OrbitControls} from '/vendor/OrbitControls.js';

export function connectProgressiveMap({prefix = 'progressive'} = {}) {
  const $ = id => document.getElementById(id.replace(/^progressive/, prefix));
  const listeners = new AbortController();
  const on = (id, event, callback) => $(id).addEventListener(event, callback, {signal: listeners.signal});
  const host = $('progressive-viewport');
  const renderer = new THREE.WebGLRenderer({antialias: false, powerPreference: 'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  host.append(renderer.domElement);
  const scene = new THREE.Scene(); scene.background = new THREE.Color('#142326');
  scene.add(new THREE.HemisphereLight(0xe6f6ff, 0x33484a, 2));
  const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(-2, -3, -1); scene.add(light);
  const rescue = createRescueLayer(THREE); scene.add(rescue.group);
  const camera = new THREE.PerspectiveCamera(55, 1, .02, 100);
  camera.position.set(2, -2, -3); camera.up.set(0, -1, 0);
  const controls = new OrbitControls(camera, renderer.domElement);
  const geometry = new THREE.BufferGeometry();
  const cloud = new THREE.Points(geometry, new THREE.PointsMaterial({size: 2, sizeAttenuation: false, vertexColors: true}));
  let pointCapacity = 0, pointPositions = new Float32Array(), pointColors = new Float32Array(), pointKey = null, pointValue = null;
  function updatePoints(value, force = false) {
    const key = value.rescue?.render_mode === 'points' ? `${value.rescue.map_id}:${value.rescue.point_revision}:${value.points.length}` : null;
    pointValue = value;
    if (key && key === pointKey && !force) return;
    pointKey = key;
    if (value.points.length > pointCapacity) {
      pointCapacity = Math.max(1024, 2**Math.ceil(Math.log2(value.points.length)));
      geometry.dispose(); pointPositions = new Float32Array(pointCapacity*3); pointColors = new Float32Array(pointCapacity*3);
      geometry.setAttribute('position', new THREE.BufferAttribute(pointPositions,3).setUsage(THREE.DynamicDrawUsage));
      geometry.setAttribute('color', new THREE.BufferAttribute(pointColors,3).setUsage(THREE.DynamicDrawUsage));
    }
    const prefs = mappingSettings(), low = [Infinity,Infinity,Infinity], high = [-Infinity,-Infinity,-Infinity], linear = new THREE.Color();
    value.points.forEach((p,i) => {
      pointPositions.set(p.slice(0,3),i*3);
      const color = rescuePointColor(p.slice(3,6),value.rescue?.point_labels?.[i],value.rescue || {},prefs);
      linear.setRGB(color[0]/255,color[1]/255,color[2]/255,THREE.SRGBColorSpace);
      pointColors.set(linear.toArray(),i*3);
      for(let axis=0;axis<3;axis++){low[axis]=Math.min(low[axis],p[axis]);high[axis]=Math.max(high[axis],p[axis]);}
    });
    for(const name of ['position','color']) {
      const attribute = geometry.getAttribute(name); if(!attribute) continue;
      attribute.clearUpdateRanges(); attribute.addUpdateRange(0,value.points.length*3); attribute.needsUpdate=true;
    }
    geometry.setDrawRange(0,value.points.length);
    const center = value.points.length ? new THREE.Vector3(...low.map((n,i)=>(n+high[i])/2)) : new THREE.Vector3();
    geometry.boundingSphere = new THREE.Sphere(center,value.points.length ? Math.hypot(...high.map((n,i)=>n-low[i]))/2 : 0);
    Object.assign(host.dataset,{pointCount:value.points.length,pointBufferCapacity:pointCapacity,
      pointRevision:value.rescue?.point_revision ?? '', pointGeometryId:geometry.id,
      pointAttributeId:geometry.getAttribute('position')?.id ?? ''});
  }
  const unsubscribeSettings = onMappingSettings(()=>{if(pointValue) updatePoints(pointValue,true);});
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
  let preparing = false;
  let hasPresentedMap = false;
  let meshRevision = -1;
  let pendingPaint = null, paintedRevision = 0;
  let displayedMap = null;
  const segments = new Map();
  let segmentEntries = 0;
  async function acknowledge(id, value) {
    if (!value.needs_presentation || paintedRevision < value.revision) return;
    const response = await fetch(`/api/import/presented?id=${encodeURIComponent(id)}`, {
      method: 'POST', headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'},
      body: JSON.stringify({revision: value.revision}), signal: AbortSignal.timeout(10000)});
    if (!response.ok) throw new Error('프레임 표시 확인을 다시 시도합니다.');
    await response.json();
  }
  function replayControls() {
    const available = replay.entries.length > 0 && (!running || hasPresentedMap);
    $('progressive-timeline').disabled = $('progressive-play').disabled = $('progressive-seek-go').disabled = !available;
    $('progressive-timeline').min = replay.first; $('progressive-timeline').max = replay.last;
    $('progressive-timeline').value = replay.time;
    if (document.activeElement !== $('progressive-seek')) $('progressive-seek').value = replay.time.toFixed(2);
    $('progressive-seek').max = replay.last;
    $('progressive-replay-time').textContent = `${replay.time.toFixed(2)} / ${replay.last.toFixed(2)}초`;
    $('progressive-play').textContent = replay.playing ? '\uC77C\uC2DC\uC815\uC9C0' : '\uC7AC\uC0DD';
    $('progressive-follow').setAttribute('aria-pressed', String(replay.following));
    $('progressive-replay-state').textContent = replay.following ?
      (resultReady ? '마지막 복원 기록 표시' : '최신 복원 표시') :
      (running ? '기록 재생 · 최신 복원은 표시 대기' : '저장된 복원 기록 재생');
    $('progressive-download').hidden = !available || !resultReady;
    $('progressive-download').href = `/api/import/history.zip?id=${encodeURIComponent(active)}`;
    if (replay.entries.length < segmentEntries) { segments.clear(); segmentEntries = 0; }
    for (let i = segmentEntries; i < replay.entries.length; i++) {
      const entry = replay.entries[i];
      if (entry.map_id != null && entry.map_points > 0) segments.set(entry.map_id, entry);
    }
    segmentEntries = replay.entries.length;
    const select = $('progressive-segment');
    const selectedMap = replay.following ? null : replay.selected()?.map_id;
    const options = [new Option('현재 구간 따라가기', 'latest')];
    for (const mapId of segments.keys()) options.push(new Option(`구간 ${mapId + 1} · 분리 보존`, String(mapId)));
    // Avoid replacing a focused select at every frame.
    const signature = options.map(option => `${option.value}:${option.text}`).join('|');
    if (select.dataset.options !== signature) { select.replaceChildren(...options); select.dataset.options = signature; }
    select.value = selectedMap == null ? 'latest' : String(selectedMap);
    $('progressive-segments').hidden = segments.size < 2;
  }
  function changed() { clearTimeout(timer); replayControls(); poll(); }
  on('progressive-timeline', 'input', event => { replay.seek(Number(event.target.value)); changed(); });
  on('progressive-seek-go', 'click', () => { replay.seek(Number($('progressive-seek').value)); changed(); });
  on('progressive-play', 'click', () => { if (replay.playing) replay.pause(); else replay.play(performance.now()); changed(); });
  on('progressive-follow', 'click', () => { replay.follow(); changed(); });
  on('progressive-segment', 'change', event => {
    if (event.target.value === 'latest') replay.follow();
    else {
      const entry = segments.get(Number(event.target.value));
      if (entry) replay.seek(entry.timestamp_s);
    }
    changed();
  });
  on('progressive-speed', 'change', () => { replay.tick(performance.now()); replay.rate = Number($('progressive-speed').value); });
  function fit() {
    if (!geometry.drawRange.count || !geometry.boundingSphere) return;
    const {center, radius} = geometry.boundingSphere;
    controls.target.copy(center); camera.position.copy(center).add(new THREE.Vector3(1, -.6, -1.3).multiplyScalar(Math.max(radius, .5) * 1.7));
    controls.update();
  }
  const observer = new ResizeObserver(() => {
    const {width, height} = host.getBoundingClientRect();
    if (width && height) { renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); }
  }); observer.observe(host);
  renderer.setAnimationLoop(() => {
    if (!disposed && !document.hidden && !host.closest('[hidden]') && host.getBoundingClientRect().width > 0) {
      controls.update(); renderer.render(scene, camera);
      if (pendingPaint) {
        const resolve = pendingPaint; pendingPaint = null;
        requestAnimationFrame(() => resolve(!disposed));
      }
    }
  });
  async function poll() {
    if (disposed || !active) return;
    if (polling) { again = true; return; }
    again = false;
    polling = true; const id = active, token = generation; let cursor = replay.version, retry = false;
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
      const endpoint = replay.following ? `preview?id=${encodeURIComponent(id)}&revision=${revision}&mesh_revision=${meshRevision}${running ? '&wait=1' : ''}` : `history-frame?id=${encodeURIComponent(id)}&revision=${selected.revision}`;
      const response = await fetch(`/api/import/${endpoint}`, {cache: 'no-store', signal: AbortSignal.timeout(10000)});
      if (!response.ok) throw new Error('지도 응답을 받지 못해 다시 시도합니다.');
      const value = await response.json();
      if (token !== generation || cursor !== replay.version || disposed) return;
      if (value.done) running = false;
      if (token === generation && cursor === replay.version && !disposed && replay.following && value.unchanged) await acknowledge(id, value);
      if (token !== generation || cursor !== replay.version || disposed || !response.ok || value.waiting || value.unchanged) return;
      const hasGeometry = Boolean(value.points.length || value.rescue?.mesh?.triangles?.length ||
        value.rescue?.people?.length);
      // Before the first map, consume tracking status and keep the video hidden.
      // Sequential jobs still receive their acknowledgement after the status is painted.
      let image = null;
      if (hasPresentedMap || hasGeometry) {
        image = new Image();
        image.src = `/api/import/preview-frame?id=${encodeURIComponent(id)}&name=${encodeURIComponent(value.frame_id)}`;
        const imageTimeout = setTimeout(() => { image.src = ''; }, 3000);
        try {
          await image.decode();
        } finally { clearTimeout(imageTimeout); }
      }
      if (token !== generation || cursor !== replay.version || disposed) return;
      revision = value.revision;
      $('progressive-frame-detail').textContent = {lost: '위치·방향 추적 실패 · 깊이 추정과 지도 반영을 건너뛰고 마지막 지도를 유지합니다.',
        initializing: '영상으로 카메라 위치·방향을 초기화 중입니다. 아직 깊이 추정을 실행하지 않습니다.',
        scale_wait: '위치·방향은 확인했습니다. 깊이 배율의 일관성이 부족해 이번 프레임은 합치지 않고 누적 지도를 유지합니다.',
        tracking: '먼저 확정한 이 프레임의 카메라 자세를 유지한 채 깊이와 지도를 복원했습니다.'}[value.tracking] || '프레임 처리 중';
      const confirmed = Boolean(value.pose_confirmed && value.pose_position && value.pose_rotation);
      cameraMarker.visible = confirmed && value.map_id === value.pose_map_id;
      if (confirmed) {
        const r = value.pose_rotation;
        cameraMarker.position.fromArray(value.pose_position);
        cameraMarker.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(r[0][0], r[0][1], r[0][2], 0, r[1][0], r[1][1], r[1][2], 0, r[2][0], r[2][1], r[2][2], 0, 0, 0, 0, 1));
        $('progressive-pose').textContent = `위치·방향 확정 · 원본 ${value.frame_number}번째 · 위치 (${value.pose_position.map(n => n.toFixed(3)).join(', ')}) · 전방 (${r.map(row => row[2].toFixed(3)).join(', ')}) · 단안 상대 좌표${value.submaps > 1 ? ' · 다른 지도 구간은 분리 보존' : ''}`;
      } else $('progressive-pose').textContent = '위치·방향 확정 대기 · 이 프레임은 깊이 복원에 사용하지 않습니다.';
      // A transient empty live result is not an instruction to erase the map.
      // Historical seeks still show the exact recorded (possibly empty) frame.
      const retaining = replay.following && !value.points.length && Boolean(pointValue?.points.length);
      if (!retaining) {
        if (value.points.length && value.map_id !== displayedMap) { displayedMap = value.map_id; first = true; }
        updatePoints(value);
        rescue.update(value.rescue); cloud.visible = !value.rescue || value.rescue.render_mode === 'points';
        meshRevision = value.rescue?.mesh_revision ?? -1;
        Object.assign(host.dataset, {rescuePeople: value.rescue?.people?.length || 0,
          rescueVoxels: (value.rescue?.people || []).reduce((n, p) => n + p.voxels.length, 0),
          rescueTriangles: value.rescue?.mesh?.triangles?.length || 0});
        route.geometry.dispose(); route.geometry = new THREE.BufferGeometry().setFromPoints(value.cameras.map(p => new THREE.Vector3(...p)));
      }
      Object.assign(host.dataset, {displayedMap: displayedMap ?? '', retainingMap: String(retaining)});
      if (retaining) cameraMarker.visible = false;
      $('progressive-map-empty').hidden = retaining || hasGeometry;
      $('progressive-map-empty').textContent = value.registered > 0
        ? '이 시점에 표시할 지도 점이 없습니다. 복원 기록이나 전체 경로 지도를 확인하세요.'
        : value.tracking === 'lost' ? '카메라 위치·방향 추적을 잃었습니다. 같은 공간을 천천히 비춰 주세요.'
        : value.tracking === 'scale_wait' ? '깊이 배율을 연결할 근거를 기다리고 있습니다.'
        : '첫 깊이 지도가 준비되면 영상과 함께 표시합니다.';
      $('progressive-count').textContent = `처리 ${value.processed} · 지도 반영 ${value.registered}프레임 · ${(retaining ? pointValue.points.length : (value.map_points ?? value.points.length)).toLocaleString()}점`;
      $('progressive-latency').textContent = `현재 프레임 복원 ${value.processing_ms}ms · 추적 포함 ${value.latency_ms}ms`;
      const fusion = value.rescue?.point_fusion;
      const scope = fusion ? `전체 공간 통합 · 구조물 최대 ${fusion.point_budget.toLocaleString()}점 · 밀도 단계 ${fusion.coarsenings}` : value.render_window_s == null ? '현재 지도 구간의 전체 누적 지도' : `최근 ${value.render_window_s}초 지도`;
      $('progressive-map-detail').textContent = !image ? $('progressive-map-empty').textContent : retaining ? '새 점군 대기 · 마지막 유효 지도를 유지합니다.' :
        `${scope} · 드래그 회전 · 휠 확대${value.submaps > 1 ? ` · 현재 구간 ${value.map_id + 1} · 이전 지도는 위 지도 구간에서 다시 열 수 있습니다. 구간 사이 좌표는 아직 연결되지 않았습니다.` : ''}`;
      if (first && value.points.length) { first = false; fit(); }
      if (image) {
        const frame = $('progressive-frame');
        frame.src = image.src;
        frame.hidden = false;
        $('progressive-video-empty').hidden = true;
        $('progressive-frame-time').textContent = `${value.timestamp_s.toFixed(2)}초 · 원본 ${value.frame_number}번째 프레임`;
        if (!hasPresentedMap) {
          hasPresentedMap = true;
          replayControls();
        }
      }
      if (replay.following && value.needs_presentation) {
        const painted = await new Promise(resolve => { pendingPaint = resolve; });
        if (!painted || token !== generation || disposed) return;
        paintedRevision = value.revision;
        await acknowledge(id, value);
      }
    } catch (error) {
      retry = true;
      if (token === generation && !disposed) $('progressive-map-detail').textContent = `지도 갱신 재시도 중 · ${error.message}`;
    } finally {
      polling = false;
      if ((running || replay.playing || again || retry) && !disposed) timer = setTimeout(poll, retry ? 500 : running && replay.following ? 0 : 200);
    }
  }
  return {update(job) {
    if (active !== job.id) { active = job.id; revision = 0; paintedRevision = 0; replay = new ProgressiveReplay(); historyAt = 0; generation++; first = true;
      hasPresentedMap = false;
      geometry.setDrawRange(0,0); pointKey = pointValue = null; displayedMap = null; segments.clear(); segmentEntries = 0; rescue.update(null); meshRevision = -1;
      route.geometry.dispose(); route.geometry = new THREE.BufferGeometry(); cameraMarker.visible = false;
      Object.assign(host.dataset, {pointCount: 0, displayedMap: '', retainingMap: 'false'});
      $('progressive-frame').hidden = true;
      $('progressive-frame-time').textContent = '첫 지도 대기';
      $('progressive-video-empty').hidden = false;
      $('progressive-video-empty').textContent = '첫 깊이 지도가 준비되면 영상과 함께 표시합니다.'; }
    running = ['running', 'cancelling'].includes(job.status);
    resultReady = ['ready', 'complete'].includes(job.status);
    replayControls();
    const preparation = mappingPreparation(job);
    $('progressive-state').textContent = job.status === 'ready' && job.history_available ? '저장된 복원 과정 다시보기 · 저장 지도 열기로 결과를 확인하세요' : job.status === 'complete' ? '복원 완료 · 저장 지도 열기로 계속 살펴보세요' : job.status === 'failed' ? '처리 중단 · 마지막 지도 유지' : job.status === 'cancelling' ? '현재 프레임 처리를 중단하고 있습니다' : job.status === 'cancelled' ? '취소됨 · 마지막 지도 유지' :
      {model: '모델 준비 중 · 영상은 대기합니다', semantic_model: '사람·공간 인식 모델 준비 중', semantics: '사람과 구조물 구분 중', depth: '자세 확정 완료 → 깊이 추정 중', tracking: '카메라 위치·방향 추적 중', pose_wait: '자세 확정 대기 · 깊이 복원 보류', mapping: '확정한 자세로 지도 복원 중', presentation: '현재 프레임 표시 후 다음 프레임 진행', saving: '전체 통합 지도 저장 중'}[job.stage] || '영상 처리 준비 중';
    if (job.stage === 'integration') $('progressive-state').textContent = `전체 경로 통합 중 · ${job.progress?.integrated_frames || 0} / ${job.progress?.integration_total || 0}`;
    if (preparation) {
      $('progressive-state').textContent = preparation.title;
      $('progressive-video-empty').textContent = preparation.title;
      $('progressive-map-empty').textContent = preparation.detail;
      $('progressive-pose').textContent = '모델 준비가 끝나면 카메라 위치 추적을 시작합니다.';
      $('progressive-latency').textContent = preparation.title;
      $('progressive-map-detail').textContent = preparation.detail;
    } else if (!hasPresentedMap && ['failed', 'cancelled'].includes(job.status)) {
      $('progressive-video-empty').textContent = '첫 지도가 준비되기 전에 처리를 종료했습니다.';
    } else if (preparing) {
      $('progressive-video-empty').textContent = running ? '첫 깊이 지도가 준비되면 영상과 함께 표시합니다.' : '영상 처리 종료';
      if (!revision) {
        $('progressive-map-empty').textContent = running ? '카메라 위치가 확인되면 3D 지도를 생성합니다.' : '생성된 지도가 없습니다.';
        $('progressive-pose').textContent = running ? '카메라 위치·방향 확정 대기' : '위치 추적 종료';
        $('progressive-latency').textContent = running ? '첫 프레임 처리 대기' : '영상 처리 종료';
        $('progressive-map-detail').textContent = running ? '모델 준비 완료 · 지도 생성 대기' : '영상 처리가 종료되었습니다.';
      }
    }
    preparing = Boolean(preparation);
    clearTimeout(timer); poll();
  }, dispose() { unsubscribeSettings(); rescue.dispose(); listeners.abort(); disposed = true; running = false; generation++; clearTimeout(timer); pendingPaint?.(false); pendingPaint = null; observer.disconnect(); renderer.setAnimationLoop(null); controls.dispose(); geometry.dispose(); cloud.material.dispose(); route.geometry.dispose(); route.material.dispose(); markerLines.geometry.dispose(); markerLines.material.dispose(); renderer.dispose(); renderer.domElement.remove(); }};
}
