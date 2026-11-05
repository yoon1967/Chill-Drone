// 실내 탐색 관제 화면.
// - 게이트웨이(gateway_server.py)가 이 화면을 내주면 /ws/drone으로 실제 상태를 받고 명령을 보낸다(실시간).
// - 정적 서버로 열거나 주소에 ?sample을 붙이면 계약 형식의 샘플을 그리기만 한다(명령 안 보냄).
// 화면 3개: #/live(PC 실시간 감독), #/review(임무 후 탐색 결과), #/control(폰 현장 조작).
"use strict";

const S = window.SAMPLE;
const $ = (sel, root = document) => root.querySelector(sel);
const ui = {
  scenario: "person", reviews: {},      // reviews: event_id → confirmed / false_positive
  live: false, ws: null, wsOpen: false, everOpen: false, state: null, timeline: [], seq: 0, cache: {}, view: null,
  findings: new Map(), lastDet: null, newFindingAt: 0,   // 실시간 사람 탐지: finding_id → 발견, 영상 위 윤곽선용 최근 탐지
  feed: loadPref("controller.feed", "raw") === "ai" ? "ai" : "raw",   // 영상 칸: raw(원본) / ai(사람 탐지 합성 영상). 합성이 끊기면 원본을 보여 준다
  map3d: { mode: loadPref("map-mode", "3d"), host: null, view: null, revision: undefined, loading: false, has: false },  // 3D 점군(지도 연결기)
  mapBig: loadPref("map-big", "0") === "1",   // 지도 칸을 영상 자리(왼쪽 큰 칸)로 바꿔 크게 본다
  console: { frame: null, url: null, slot: null, ro: null, loadedUp: null },   // 지도 콘솔 iframe(placeConsole). loadedUp: 열 때 콘솔 상태
};
if (!["2d", "3d"].includes(ui.map3d.mode)) ui.map3d.mode = "3d";   // 지도 콘솔은 따로 화면(#/map)
function loadPref(key, fallback) { try { return localStorage.getItem(key) || fallback; } catch (e) { return fallback; } }
function savePref(key, value) { try { localStorage.setItem(key, value); } catch (e) { /* 저장 안 돼도 화면은 동작 */ } }
const DET_OVERLAY_MS = 1000;   // 탐지 결과는 실시간 영상보다 조금 늦게 온다. 윤곽선은 1초만 보이고 지연을 함께 적는다
const PERSON_ALERT_MS = 10000; // 새 발견 알림을 띄워 두는 시간

const MODE = {
  disconnected: ["연결 끊김", "danger"], grounded: ["지상", "neutral"], manual: ["수동 비행", "success"],
  auto: ["자율 탐색", "accent"], paused: ["일시정지", "warning"], landing: ["착륙 중", "warning"], emergency: ["비상 정지", "danger"],
};
const AUTHORITY = { operator: "사람이 조종 중", autonomy: "자율이 조종 중", none: "제자리 대기" };
const LINK = { good: ["통신 좋음", "success"], fair: ["통신 주의", "warning"], weak: ["통신 약함", "warning"], critical: ["통신 위험", "danger"] };
const PHASE = { idle: "대기", running: "진행 중", paused: "멈춤", arrived: "도착", failed: "실패", stopped: "종료" };
const SOURCES = { drone: "드론 상태", autonomy: "자율 비행", detect: "사람 탐지", map: "지도", video: "드론 영상" };
const REASON = {
  not_allowed_in_mode: "지금 상태에서는 할 수 없습니다", preflight_failed: "이륙 전 점검을 통과하지 못했습니다",
  client_not_allowed: "자율 시작은 PC 감독 화면에서만 할 수 있습니다", autonomy_not_connected: "자율 비행이 연결되지 않았습니다",
  autonomy_joined_in_air: "이륙 뒤에 연결된 자율이라 시작할 수 없습니다", replay_source: "기록 재생 중에는 명령을 보내지 않습니다",
  busy: "앞 요청을 처리하는 중입니다", unknown_message: "알 수 없는 요청입니다",
  video_disabled: "게이트웨이가 영상 수신 없이(--no-video) 실행됐습니다", video_receiver_failed: "영상 수신을 시작하지 못했습니다",
  no_response: "기체가 응답하지 않았습니다", error_response: "기체가 거절했습니다",
};
const VIDEO_REASON = { operator: "화면", takeoff: "이륙", ground_timeout: "지상 오래 켜짐" };
const CHECK = { sdk: "SDK 모드", state: "상태 수신", battery: "배터리", temp: "온도", operator: "주변 확인" };  // 거절 사유 이름

const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const pct = (v) => (v == null ? "없음" : `${Math.round(v * 100)}%`);
const clock = (s) => (s == null ? "없음" : `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(Math.round(s % 60)).padStart(2, "0")}`);
const meters = (v) => (v == null ? "없음" : `${v.toFixed(2)}m`);
const chip = (text, tone = "neutral", cls = "") => `<span class="chip ${tone} ${cls}">${esc(text)}</span>`;
const src = (key) => `<span class="src" title="이 칸에 들어올 팀원 작업물">${esc(SOURCES[key])}</span>`;
const hhmmss = (iso) => (iso ? new Date(iso).toLocaleTimeString("ko-KR", { hour12: false }) : "");

// 게이트웨이 상태를 화면이 쓰는 모양으로. 아직 게이트웨이가 주지 않는 값은 "없음"으로 둔다(지어내지 않는다).
function adapt(st) {
  return {
    ...st,
    link: st.link || { level: null, state_hz: null, rtt_ms: null },
    flight_time_s: st.flight_time_s ?? st.motor_time_s ?? null,
    front_tof: st.front_tof || { m: null, valid: false, reason: "missing" },
    down_tof: st.down_tof || { m: null, valid: false, reason: "missing" },
    video: st.video || { available: false },
    overlay: st.overlay || { alive: false },
    allowed: st.allowed || ["emergency"],
    warnings: st.warnings || [],
  };
}

function current() {
  if (ui.live) {
    const st = adapt(ui.state || { mode: "disconnected", authority: "none", connected: false });
    if (!ui.wsOpen) { st.mode = ui.everOpen ? "disconnected" : st.mode; st.allowed = []; }
    const w = st.warnings;
    const alert = !ui.wsOpen ? (ui.everOpen ? "gateway" : "connecting") : w.includes("link_weak") ? "link" : w.includes("autonomy_stale") ? "stale" : null;
    const events = liveFindings();
    const person = !alert && events[0] && Date.now() - ui.newFindingAt < PERSON_ALERT_MS && events[0].review === "unreviewed";
    return { sc: { alert: alert || (person ? "person" : null) }, st, events, track: [] };
  }
  const sc = S.scenarios[ui.scenario];
  const events = S.events.slice(0, sc.detections).map((e) => ({ ...e, review: ui.reviews[e.event_id] || "unreviewed" }));
  const tNow = sc.state.mode === "grounded" ? 0 : S.track[S.track.length - 1].t_s;
  return { sc, st: sc.state, events: events.reverse(), track: S.track.filter((p) => p.t_s <= tNow) };
}

// 게이트웨이 발견(finding)을 카드 모양으로. 실시간은 발견 하나가 카드 하나다(같은 사람이 보이는 동안 묶임).
function liveFindings() {
  return [...ui.findings.values()].reverse().map((f, i, all) => {
    const b = f.best, det = b.detections.reduce((a, d) => (d.confidence > a.confidence ? d : a));
    return {
      event_id: f.finding_id, n: all.length - i, confidence: b.confidence, people: f.max_people, events: f.events,
      bbox_xyxy_px: det.bbox_xyxy_px, image_width: b.image.width_px, image_height: b.image.height_px,
      seen_at: f.first_seen, last_seen: f.last_seen, frame_id: b.frame_id, photo_url: b.photo_url, drone: b.drone,
      review: ui.reviews[f.finding_id] || "unreviewed",
    };
  });
}

const findingNo = (e) => e.n ?? S.events.findIndex((x) => x.event_id === e.event_id) + 1;
const findingTime = (e) => (e.seen_at ? hhmmss(e.seen_at) : clock(e.pts_us / 1e6));

function poseAt(t_s) {  // 탐지 시각에 가장 가까운 드론 위치. 사람의 위치가 아니다.
  return S.track.reduce((a, b) => (Math.abs(b.t_s - t_s) < Math.abs(a.t_s - t_s) ? b : a));
}

function toast(msg) {
  const el = $("#toast");
  el.textContent = msg;
  el.classList.add("show");
  clearTimeout(toast.t);
  toast.t = setTimeout(() => el.classList.remove("show"), 2600);
}

// ---------- 공통 조각 (data-part 단위로 바뀐 것만 다시 그린다) ----------
function statusStrip(st, events, compact = false) {
  const [mode, tone] = MODE[st.mode] || [st.mode, "neutral"];
  const [link, ltone] = LINK[st.link.level] || ["통신 측정 전", "neutral"];
  const found = events.length;
  return `<section class="strip ${compact ? "compact" : ""}" data-part="strip" aria-label="비행 상태">
    <span class="mode ${tone}">${esc(mode)}</span>
    <span class="who">${AUTHORITY[st.authority] || ""}</span>
    ${found ? chip(`사람 ${found}건 발견`, "danger") : chip("발견 없음")}
    ${st.mission ? chip(`탐색 ${pct(st.mission.progress)}`) : ""}
    <span class="spacer"></span>
    ${chip(link, ltone)}
    ${chip(`배터리 ${st.battery_pct ?? "없음"}${st.battery_pct == null ? "" : "%"}`, st.battery_pct != null && st.battery_pct < 30 ? "warning" : "neutral")}
    ${compact ? "" : chip(`${ui.live ? "모터" : "비행"} ${clock(st.flight_time_s)}`)}
  </section>`;
}

function alertBox(sc, events) {
  let body = "";
  if (sc.alert === "connecting") {
    body = `<div class="alert warning slim" role="status"><b>게이트웨이 연결 중</b><span>연결되면 버튼이 열립니다.</span></div>`;
  } else if (sc.alert === "gateway") {
    body = `<div class="alert danger" role="alert"><b>게이트웨이 연결 끊김</b><span>다시 연결하는 중입니다. 이 화면에서 보낸 조종 입력은 0으로 처리됩니다.</span></div>`;
  } else if (sc.alert === "person" && events[0]) {
    body = `<div class="alert danger" role="alert"><b>사람 발견</b><span>신뢰도 ${events[0].confidence.toFixed(2)} · ${ui.live ? "발견" : "지도"} ${findingNo(events[0])}번. 자동으로 멈추지 않습니다.</span>
      <button data-cmd="pause">여기서 멈추기</button></div>`;
  } else if (sc.alert === "link") {
    body = `<div class="alert warning" role="alert"><b>통신 약함</b><span>더 진행하지 않고 멈췄습니다. 다시 시작하면 연결이 끊길 수 있습니다.</span>
      <button data-cmd="return_home">출발점으로 복귀</button></div>`;
  } else if (sc.alert === "stale") {
    body = `<div class="alert warning" role="alert"><b>자율 응답 지연</b><span>0.5초 넘게 명령이 없어 제자리에서 기다립니다. 2초가 지나면 일시정지합니다.</span></div>`;
  }
  return `<div class="alert-slot" data-part="alert">${body}</div>`;
}

// 영상은 지상에서 켜 두면 기체가 뜨거워져 게이트웨이가 자동으로 켜지 않는다. 이륙하면 켜지고, 지상에서 오래 켜져 있으면 꺼진다.
function videoControl(st) {
  const v = st.video;
  const on = v.on ?? !!v.available;  // 샘플에는 on이 없다
  const [text, tone] = v.enabled === false ? ["영상 수신 안 함", "neutral"]
    : !on ? ["영상 꺼짐", "neutral"]
    : v.ground_s != null ? [`지상 ${Math.round(v.ground_s)}초 · ${Math.round(v.ground_off_s)}초에 꺼짐`, st.warnings.includes("video_on_ground") ? "warning" : "neutral"]
    : ["영상 켜짐", "success"];
  const name = on ? "video_off" : "video_on";
  const off = v.enabled === false;
  // 칩(초 단위로 바뀜)과 버튼을 다른 칸으로 둔다. 버튼이 매초 바뀌면 누르는 중에 사라져 클릭이 빠진다.
  return `<span class="video-ctl"><span data-part="videostate">${chip(text, tone)}</span>
    <span data-part="videobtn"><button data-cmd="${name}" class="${off ? "off" : ""}" ${off ? 'aria-disabled="true"' : ""}>${CMD_LABEL[name]}</button></span></span>`;
}

// 원본 / AI 합성 고르기. 합성 영상은 사람 탐지 프로그램이 게이트웨이 영상에 윤곽선·장면 분할을 그려 보낸 것이다.
function feedSelect(st) {
  const ai = st.overlay.alive;
  const btn = (key, label, off) => `<button data-feed="${key}" class="${off ? "off" : ""}" aria-pressed="${ui.feed === key}" ${off ? 'aria-disabled="true"' : ""}>${label}</button>`;
  return `<span class="feed-sel" data-part="feedsel" role="group" aria-label="영상 고르기">${btn("raw", "원본", false)}${btn("ai", "AI 합성", !ai)}</span>`;
}

function videoPanel(st, events, big = true) {
  const latest = events[0];
  const aiOn = ui.live && st.video.available && ui.feed === "ai" && st.overlay.alive;
  const box = aiOn ? "" : ui.live ? liveOutline() : latest && st.mode !== "grounded" ? (() => {
    const [x1, y1, x2, y2] = latest.bbox_xyxy_px;
    const w = latest.image_width, h = latest.image_height;
    return `<div class="bbox" style="left:${(x1 / w) * 100}%;top:${(y1 / h) * 100}%;width:${((x2 - x1) / w) * 100}%;height:${((y2 - y1) / h) * 100}%">
      <span>사람 ${latest.confidence.toFixed(2)}</span></div>`;
  })() : "";
  const media = aiOn ? `<img class="feed" src="/overlay.mjpg" alt="사람 탐지 합성 영상">`
    : ui.live && st.video.available
    ? `<img class="feed" src="/video.mjpg" alt="드론 카메라 실시간 영상">`
    : `<div class="placeholder">${!ui.live ? "영상 연결 전 · 960×720"
      : st.video.enabled === false ? "영상 수신 안 함 · 게이트웨이가 --no-video로 실행됐습니다"
      : st.video.on === false ? "영상 꺼짐 · 지상에서 켜 두면 기체가 뜨거워집니다. 이륙하면 자동으로 켜집니다"
      : "영상 없음 · 게이트웨이가 아직 프레임을 받지 못했습니다"}</div>`;
  const tof = st.front_tof.valid ? `앞 ${meters(st.front_tof.m)}` : "앞 거리 없음";
  return `<section class="panel video ${big ? "" : "small"}" aria-label="드론 영상">
    <header><h2>영상</h2>${src("video")}${chip(aiOn ? "합성: 사람 탐지" : "탐지 상자: 사람 탐지", "neutral", "src-inline")}${feedSelect(st)}${videoControl(st)}</header>
    <div class="frame-wrap"><div class="frame">
      <div class="media" data-part="media">${media}</div>
      <div class="overlays" data-part="overlay">${box}
        <span class="overlay bl">${tof} · 높이 ${meters(st.height_m)}</span>
        <span class="overlay br">${!st.video.available ? "영상 없음"
          : aiOn ? `AI 합성 ${st.overlay.fps ?? "?"}fps · 원본 ${st.video.fps ?? "?"}fps`
          : ui.live && ui.feed === "ai" ? `원본 ${st.video.fps ?? "?"}fps · AI 합성 끊김`
          : `${st.video.fps ?? "?"}fps`}</span></div>
    </div></div>
    ${big ? detailLine(st) : ""}
  </section>`;
}

// 최근 탐지의 사람 윤곽선. 영상과 같은 4:3 칸에 원본 픽셀 좌표 그대로 그린다.
function liveOutline() {
  const d = ui.lastDet;
  if (!d || performance.now() - d.at > DET_OVERLAY_MS) return "";
  const { width_px: w, height_px: h } = d.event.image;
  const shapes = d.event.detections.map((p) => {
    const [x1, y1] = p.bbox_xyxy_px;
    const polys = p.polygons_px.map((poly) => `<polygon points="${poly.map(([x, y]) => `${x},${y}`).join(" ")}"/>`).join("");
    return `${polys}<text x="${x1}" y="${Math.max(26, y1 - 8)}">사람 ${p.confidence.toFixed(2)}</text>`;
  }).join("");
  const lag = d.latency_ms == null ? "" : ` · 탐지 지연 ${d.latency_ms}ms`;
  return `<svg class="outline" viewBox="0 0 ${w} ${h}" preserveAspectRatio="xMidYMid meet" aria-label="사람 윤곽선">${shapes}</svg>
    <span class="overlay tl">사람 ${d.event.detections.length}명${lag}</span>`;
}

function detectionCard(e, big = false) {
  const state = { confirmed: ["확인됨", "success"], false_positive: ["오탐", "neutral"], unreviewed: ["미확인", "warning"] }[e.review];
  const n = findingNo(e);
  const thumb = e.photo_url
    ? `<a class="thumb photo" href="${esc(e.photo_url)}" target="_blank" rel="noopener" title="사진 크게 보기"><img src="${esc(e.photo_url)}" alt="발견 ${n} 사진" loading="lazy"><span>${n}</span></a>`
    : `<div class="thumb" aria-hidden="true"><span>${n}</span></div>`;
  const where = !big ? ""
    : e.seen_at ? `<div class="muted">그때 드론: ${(MODE[e.drone?.mode] || ["없음"])[0]} · 높이 ${meters(e.drone?.height_m)} · 사람 위치는 아직 없음</div>`
    : `<div class="muted">발견 위치: 그때 드론 위치 (${poseAt(e.pts_us / 1e6).x_m.toFixed(1)}, ${poseAt(e.pts_us / 1e6).y_m.toFixed(1)}) m · 사람 위치는 아직 없음</div>`;
  const more = e.seen_at ? ` · ${e.people > 1 ? `최대 ${e.people}명 · ` : ""}${e.events}장` : ` · ${esc(e.frame_id)}`;
  return `<article class="det ${big ? "big" : ""} ${e.review}" data-event="${esc(e.event_id)}" tabindex="0">
    ${thumb}
    <div class="det-body">
      <div class="det-title">사람 · ${e.confidence.toFixed(2)} ${chip(state[0], state[1])}</div>
      <div class="muted">${findingTime(e)}${more}</div>
      ${where}
      <div class="det-actions">
        <button data-review="confirmed" ${e.review === "confirmed" ? 'aria-pressed="true"' : ""}>확인</button>
        <button data-review="false_positive" ${e.review === "false_positive" ? 'aria-pressed="true"' : ""}>오탐</button>
      </div>
    </div>
  </article>`;
}

// 탐지 프로그램이 게이트웨이에 결과를 보내고 있는지. 사람이 없어도 결과는 계속 온다.
function detectStatus() {
  const d = ui.state?.detection;
  if (d?.alive) return { text: `탐지 동작 중 · ${d.events_per_s}장/s`, tone: "success", alive: true, ever: true };
  if (d?.events) return { text: "탐지 끊김", tone: "warning", alive: false, ever: true };
  return { text: "탐지 연결 전", tone: "neutral", alive: false, ever: false };
}

function detectionList(events, big = false) {
  const ds = ui.live ? detectStatus() : null;
  const empty = !ui.live ? "아직 발견한 사람이 없습니다. 탐지 결과가 오면 최신순으로 쌓입니다."
    : ds.alive ? "탐지 동작 중 · 아직 발견한 사람이 없습니다."
    : ds.ever ? "탐지 결과가 끊겼습니다. 탐지 프로그램을 확인하세요."
    : "사람 탐지가 아직 연결되지 않았습니다. 탐지 프로그램이 게이트웨이 영상을 받아 결과를 보내면 발견이 최신순으로 쌓입니다.";
  const body = events.length ? events.map((e) => detectionCard(e, big)).join("") : `<p class="empty">${esc(empty)}</p>`;
  return `<section class="panel dets ${big ? "big" : ""}" data-part="dets" aria-label="발견 목록">
    <header><h2>발견</h2>${src("detect")}${ds ? chip(ds.text, ds.tone, "src-inline") : ""}</header>
    <div class="det-list">${body}</div>
  </section>`;
}

const MAP_MODES = { "2d": "2D 경로", "3d": "3D 점군" };

function mapPanel(st, events, track, tall = false) {
  const mode = ui.live && !tall ? ui.map3d.mode : "2d";   // 탐색 결과(tall)는 3D 칸이 따로 있다
  const seg = ui.live && !tall ? `<div class="seg map-mode" role="group" aria-label="지도 보기">
      ${Object.entries(MAP_MODES).map(([m, label]) => `<button data-map-mode="${m}" class="${mode === m ? "on" : ""}" aria-pressed="${mode === m}">${label}</button>`).join("")}</div>
      <a class="map-open" href="#/map" title="지도 파트 콘솔의 진행 상황 화면을 크게 봅니다">지도 콘솔 ↗</a>
      <button class="map-big" data-map-big aria-pressed="${ui.mapBig}" title="${ui.mapBig ? "지도를 오른쪽 칸으로 되돌립니다" : "지도와 영상 자리를 바꿔 지도를 크게 봅니다"}">${ui.mapBig ? "⤡ 원래대로" : "⤢ 크게"}</button>` : "";
  const head = `<header><h2>지도</h2>${src("map")}${mode === "2d" ? chip("경로: 자율 비행", "neutral", "src-inline") : ""}
      <span class="spacer"></span>${seg || `<a class="link" href="${CONSOLE_URL}" target="_blank" rel="noopener">3D 지도 열기 (apps/console)</a>`}</header>`;
  if (mode === "3d") return map3dPanel(st, head);
  if (ui.live) {
    return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
      <div class="map-empty">자율 비행의 위치 추정(pose)이 연결되면 지나온 경로와 현재 위치가 그려집니다.</div></section>`;
  }
  const pts = [...S.track, ...S.plan, ...S.rooms.flatMap((r) => [{ x_m: r.x, y_m: r.y }, { x_m: r.x + r.w, y_m: r.y + r.h }])];
  const minX = Math.min(...pts.map((p) => p.x_m)) - 0.6, maxX = Math.max(...pts.map((p) => p.x_m)) + 0.6;
  const minY = Math.min(...pts.map((p) => p.y_m)) - 0.6, maxY = Math.max(...pts.map((p) => p.y_m)) + 0.6;
  const W = 800, H = tall ? 360 : 230, k = Math.min(W / (maxX - minX), H / (maxY - minY));
  const X = (x) => ((x - minX) * k + (W - (maxX - minX) * k) / 2).toFixed(1);
  const Y = (y) => (H - (y - minY) * k - (H - (maxY - minY) * k) / 2).toFixed(1);  // y 왼쪽(위)이 +
  const rooms = S.rooms.map((r) => `<rect class="room" x="${X(r.x)}" y="${Y(r.y + r.h)}" width="${(r.w * k).toFixed(1)}" height="${(r.h * k).toFixed(1)}"/>
    <text class="room-label" x="${+X(r.x) + 8}" y="${+Y(r.y + r.h) + 18}">${esc(r.name)}</text>`).join("");
  const path = track.length > 1 ? `<polyline class="flown" points="${track.map((p) => `${X(p.x_m)},${Y(p.y_m)}`).join(" ")}"/>` : "";
  const planned = st.mission && st.mode !== "grounded"
    ? `<polyline class="planned" points="${S.plan.map((p) => `${X(p.x_m)},${Y(p.y_m)}`).join(" ")}"/>` : "";
  const marks = events.map((e) => {
    const p = poseAt(e.pts_us / 1e6), n = S.events.findIndex((x) => x.event_id === e.event_id) + 1;
    return `<g class="mark ${e.review}" data-event="${esc(e.event_id)}"><circle cx="${X(p.x_m)}" cy="${Y(p.y_m)}" r="10"/><text x="${X(p.x_m)}" y="${+Y(p.y_m) + 4}">${n}</text></g>`;
  }).join("");
  const now = track[track.length - 1] || S.track[0];
  const drone = `<g class="drone" transform="translate(${X(now.x_m)},${Y(now.y_m)})"><path d="M0,-11 L8,7 L0,3 L-8,7 Z"/></g>`;
  const start = `<circle class="start" cx="${X(0)}" cy="${Y(0)}" r="7"/><text class="start-label" x="${+X(0) + 12}" y="${+Y(0) + 4}">출발</text>`;
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="지나온 경로, 계획 경로, 발견 지점, 현재 드론 위치">
      ${rooms}${path}${planned}${start}${marks}${drone}
    </svg>
    <p class="legend"><i class="lg flown"></i>지나온 경로 <i class="lg planned"></i>계획 경로 <i class="lg mark"></i>발견 지점 = 그때 드론 위치(사람 위치 아님)</p>
  </section>`;
}

// 3D 점군: 캔버스(ui.map3d.host)는 한 번만 만들고 다시 그릴 때마다 이 자리(.map3d-slot)로 옮긴다(attachMap3d).
// 화면 전체를 0.1초마다 다시 그리므로, 캔버스를 HTML로 다시 만들면 점군을 매번 처음부터 올리게 된다.
function map3dPanel(st, head, cls = "map") {   // cls: 실시간 감독은 지도 칸(map), 탐색 결과는 3D 칸
  const m = st.map, has = ui.map3d.has;
  const unit = m?.unit === "m" ? "m" : "임의 단위(m 아님)";
  const info = !m || !m.alive ? ["지도 연결기 꺼짐", "warning", "지도 서버를 켜고 gateway_map_bridge.py를 실행하면 점군이 들어옵니다."]
    : m.upstream === "unreachable" ? ["지도 서버 꺼짐", "danger", `${m.source_url || "지도 서버"}에 연결할 수 없습니다.`]
    : m.upstream === "no_map" && !has ? ["지도 아직 없음", "neutral", `지도 서버 상태: ${m.mapping_state || "알 수 없음"}`]
    : m.upstream === "error" ? ["지도 읽기 오류", "danger", ""]
    : ["지도 수신 중", "success", ""];
  const detail = has && m ? `버전 ${m.revision ?? "?"} · 점 ${m.points}${m.points_total > m.points ? `/${m.points_total}` : ""} · ${unit}` : info[2];
  return `<section class="panel ${cls}" data-part="map3d" aria-label="3D 지도">${head}
    <div class="map3d-slot">${has ? "" : `<div class="map-empty">${esc(info[2] || "점군을 기다리는 중입니다.")}</div>`}</div>
    <div class="map3d-info">${chip(info[0], info[1])}<span class="muted">${esc(detail)}</span>
      ${has ? `<span class="muted keys-hint">끌어서 돌리기 · 휠/두 손가락 확대 · 두 번 눌러 처음 시점</span>` : ""}</div></section>`;
}

function attachMap3d() {
  const slot = $("#app .map3d-slot");
  if (!slot || !ui.live) return;
  const m = ui.map3d;
  if (!m.host) {
    m.host = document.createElement("div");
    m.host.className = "map3d-host";
    m.host.innerHTML = `<canvas aria-label="3D 점군과 카메라 경로"></canvas>`;
    m.view = window.PointView ? new window.PointView(m.host.firstChild) : null;   // map3d.js를 못 읽었으면(옛 캐시 등) 안내만
    if (!m.view?.ok) m.host.innerHTML = `<div class="map-empty">이 브라우저는 WebGL을 쓸 수 없어 3D 점군을 그리지 못합니다.</div>`;
  }
  if (m.has && m.host.parentNode !== slot) { slot.replaceChildren(m.host); m.view?.draw(); }
  const rev = ui.state?.map?.revision;
  if (rev != null && rev !== m.revision && !m.loading) loadMap();
}

async function loadMap() {
  const m = ui.map3d;
  m.loading = true;
  try {
    const r = await fetch("/api/map", { cache: "no-store" });
    if (r.ok) {
      const map = await r.json();
      m.revision = map.source?.revision;
      if (m.view?.ok) m.view.setData(map);
      const first = !m.has;
      m.has = true;
      if (first) { ui.cache = {}; render(); }   // 안내 문구 → 캔버스
    }
  } catch (e) { /* 다음 상태 갱신 때 다시 */ }
  m.loading = false;
}

// 지도 콘솔 화면(#/map): 지도 파트 콘솔(apps/console)의 실시간 지도 화면을 크게 띄운다.
// 콘솔에서 시작한 드론 지도는 콘솔 화면이 살아 있어야 이어진다(콘솔이 2초마다 하트비트, 45초 끊기면 멈춤).
// 그래서 iframe은 #app 밖(body)에 한 번만 만들고 이 화면의 자리(.console-slot) 위에 겹쳐 둔다(placeConsole).
// 화면을 다시 그리거나 실시간 감독 등 다른 화면으로 가도 콘솔이 새로 열리지 않는다(다른 화면에서는 숨기기만).
// 별도 브라우저 창은 가려지거나 최소화되면 타이머가 느려져 하트비트가 끊길 수 있어 기본으로 쓰지 않는다.
const CONSOLE_URL = "http://127.0.0.1:8790/";   // 지도 연결기가 콘솔 주소를 알려 주기 전 기본값(apps/console 기본 포트)
const LOOPBACK = ["127.0.0.1", "localhost", "[::1]"];

function consoleTarget(st) {
  let u;
  try { u = new URL(st?.map?.source_url || CONSOLE_URL); } catch (e) { return { url: null, note: "지도 콘솔 주소를 읽을 수 없습니다." }; }
  if (LOOPBACK.includes(u.hostname) && !LOOPBACK.includes(location.hostname)) {
    return { url: null, note: "지도 콘솔이 드론 PC 안(127.0.0.1)에서만 열려 있어 이 기기에서는 볼 수 없습니다. 드론 PC의 화면에서 보세요." };
  }
  u.hash = "live";
  return { url: u.href, origin: u.origin };
}

function consolePanel(st) {
  const head = `<header><h2>지도 콘솔</h2>${src("map")}<span class="muted src-inline">지도 파트 콘솔(apps/console)의 실시간 지도 화면</span></header>`;
  const t = consoleTarget(st), m = st.map;
  const info = !t.url ? ["볼 수 없음", "warning"]
    : m?.alive && m.upstream === "unreachable" ? ["지도 콘솔 꺼짐", "danger"]
    : m?.alive && m.upstream && m.upstream !== "unreachable" ? ["지도 콘솔 연결", "success"]
    : [`지도 연결기 꺼짐 · ${m?.source_url ? "마지막 주소" : "기본 주소"}`, "neutral"];   // 연결기가 꺼져도 게이트웨이는 마지막 콘솔 주소를 기억한다
  const state = m?.alive && m.mapping_state ? ` · 지도 상태 ${m.mapping_state}` : "";
  return `<section class="panel console-view" aria-label="지도 콘솔">${head}
    <div class="console-slot">${t.url ? "" : `<div class="map-empty">${esc(t.note)}</div>`}</div>
    <div class="map3d-info" data-part="console-info">${chip(info[0], info[1])}
      <span class="muted">${t.url ? esc(`${t.origin}${state}`) : ""}</span>
      ${t.url ? `<span class="console-tools"><button class="mini" data-console-reload>다시 불러오기</button>
      <a class="link" href="${esc(t.url)}" target="_blank" rel="noopener">새 창</a></span>
      <span class="muted keys-hint console-hint">콘솔에서 시작한 지도는 이 관제 화면(또는 콘솔 창)이 열려 있어야 이어집니다 · 다른 화면으로 가도 콘솔은 닫히지 않습니다</span>` : ""}</div></section>`;
}

function placeConsole() {
  const c = ui.console;
  const slot = ui.live && view() === "map" ? $("#app .console-slot") : null;
  const t = slot ? consoleTarget(ui.state) : null;
  const up = ui.state?.map?.alive ? ui.state.map.upstream : null;
  // 콘솔이 꺼진 채로 열었으면(오류 화면) 켜졌을 때 새로 연다. 열린 뒤 잠깐 끊긴 건 콘솔 화면이 알아서 다시 붙고,
  // 새로 열면 그 화면에서 만들던 지도가 끊길 수 있으므로 그대로 둔다(필요하면 "다시 불러오기").
  if (c.frame && c.loadedUp === "unreachable" && up && up !== "unreachable") return reloadConsole();
  if (!c.frame && t?.url) {   // 콘솔 보기를 처음 고를 때 연다
    c.frame = document.createElement("iframe");
    c.frame.className = "console-frame away";
    c.frame.title = "지도 콘솔(apps/console) 실시간 지도";
    c.frame.allow = "fullscreen";
    document.body.append(c.frame);
  }
  if (!c.frame) return;
  if (t?.url && c.url !== t.url) { c.url = t.url; c.frame.src = t.url; c.loadedUp = up; }
  if (c.slot !== slot) {   // 칸이 다시 그려지면 새 자리의 크기 변화를 따라간다
    c.ro?.disconnect();
    c.slot = slot;
    c.ro = slot && window.ResizeObserver ? new ResizeObserver(() => placeConsole()) : null;
    c.ro?.observe(slot);
  }
  if (!slot || !t?.url) return c.frame.classList.add("away");   // 숨기기만 한다(콘솔은 계속 살아 있음)
  const r = slot.getBoundingClientRect();
  Object.assign(c.frame.style, { left: `${r.left + scrollX}px`, top: `${r.top + scrollY}px`, width: `${r.width}px`, height: `${r.height}px` });
  c.frame.classList.remove("away");
}

function reloadConsole() {   // 같은 주소(#live)로 src를 다시 넣으면 새로 읽지 않으므로 iframe을 새로 만든다
  const c = ui.console;
  c.frame?.remove();
  c.frame = null; c.url = null;
  placeConsole();
}

const CMD_LABEL = {
  takeoff: "이륙", land: "착륙", pause: "일시정지", resume: "다시 시작", takeover: "넘겨받기",
  auto_start: "자율 시작", auto_stop: "자율 종료", return_home: "복귀", confirm_area: "주변 확인", emergency: "비상 정지",
  video_on: "영상 켜기", video_off: "영상 끄기",
};

function areaConfirmed(st) {
  return !!(st.preflight || []).find((c) => c.key === "operator" && c.ok);
}

function preflightLine(st) {
  if (st.mode !== "grounded" || !st.preflight) return "";
  // 막는 항목은 ✓/✗, 알리기만 하는 항목(온도)은 ✓/!로. 기준값은 게이트웨이가 준 제목을 그대로 쓴다.
  return `<span class="checks">${st.preflight.map((c) => {
    const label = c.key === "operator" ? "주변 확인" : c.title;
    if (c.ok) return chip(`✓ ${label}`, "success");
    return c.blocking === false ? chip(`! ${c.detail || label}`, "warning") : chip(`✗ ${label}`, "danger");
  }).join("")}</span>`;
}

function controlBar(st) {
  const can = (n) => st.allowed.includes(n);
  const btn = (n, extra = "") => `<button data-cmd="${n}" class="${can(n) ? "" : "off"} ${extra}" ${can(n) ? "" : 'aria-disabled="true"'}>${CMD_LABEL[n]}</button>`;
  const manualOn = st.mode === "manual", autoOn = st.mode === "auto";
  const autoCmd = st.mode === "paused" && can("resume") ? "resume" : "auto_start";
  const grounded = st.mode === "grounded";
  const confirmed = areaConfirmed(st);
  const confirm = grounded
    ? `<button data-cmd="confirm_area" data-confirmed="${!confirmed}" class="${can("confirm_area") ? "" : "off"}" aria-pressed="${confirmed}">${confirmed ? "✓ 주변 확인됨" : "주변 비움·가드·안전 담당 확인"}</button>` : "";
  return `<section class="controls" data-part="controls" aria-label="조종">
    <span class="muted">조종</span>
    <div class="seg" role="group" aria-label="수동 자율 전환">
      <button data-cmd="takeover" class="${manualOn ? "on" : ""} ${can("takeover") || manualOn ? "" : "off"}" aria-pressed="${manualOn}">수동</button>
      <button data-cmd="${autoCmd}" class="${autoOn ? "on" : ""} ${can(autoCmd) || autoOn ? "" : "off"}" aria-pressed="${autoOn}">자율</button>
    </div>
    ${st.mode === "paused" ? btn("resume") : btn("pause")}
    ${btn("return_home", "soon")}
    ${confirm}
    ${grounded ? btn("takeoff") : btn("land")}
    ${preflightLine(st)}
    <span class="spacer"></span>
    ${ui.live && st.mode === "manual" ? '<span class="muted keys-hint">WASD 이동 · ↑↓ 높이 · ←→ 회전 · L 착륙 · Esc 길게 비상 정지</span>' : ""}
    <button class="estop" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>길게</small></button>
  </section>`;
}

function detailItems(st) {
  const recv = st.link.state_hz != null ? `${st.link.state_hz}Hz · ${st.link.rtt_ms}ms`
    : st.state_age_s != null ? `${Math.round(st.state_age_s * 1000)}ms 전` : "없음";
  return [
    ["온도", st.temp_high_c == null ? "없음" : `${st.temp_high_c}°C`],
    ["하방", st.down_tof.valid ? meters(st.down_tof.m) : "없음"],
    ["전방", st.front_tof.valid ? meters(st.front_tof.m) : "없음"],
    ["상태 수신", recv],
    ["개입", `${st.operator_interventions ?? 0}회`],
    ["미션", st.mission ? `${st.mission.state} · ${PHASE[st.mission.phase] || st.mission.phase}` : "없음"],
  ];
}

function detailLine(st) {
  return `<p class="detail-line" data-part="detail" title="${esc(SOURCES.drone)}">${detailItems(st).map(([k, v]) => `<span>${k} <b>${esc(v)}</b></span>`).join("")}</p>`;
}

// ---------- 화면 ----------
function renderLive() {  // 모니터 한 화면(1366×768 이상)에 스크롤 없이 들어가게 배치한다.
  const { sc, st, events, track } = current(), big = ui.live && ui.mapBig;   // 크게 보기 버튼은 게이트웨이 연결 화면에만 있다
  return `${statusStrip(st, events)}${alertBox(sc, events)}
    <div class="live-main ${big ? "map-big" : ""}">${big   // 크게: 지도가 왼쪽 큰 칸, 영상이 오른쪽 아래 칸
      ? `${mapPanel(st, events, track)}<div class="side">${detectionList(events)}${videoPanel(st, events)}</div>`
      : `${videoPanel(st, events)}<div class="side">${detectionList(events)}${mapPanel(st, events, track)}</div>`}</div>
    ${controlBar(st)}`;
}

// 지도 콘솔 화면: 콘솔을 크게 보되, 상태 줄과 조작 줄(착륙·비상 정지)은 실시간 감독과 같이 둔다.
// 콘솔 안을 누르면 키보드 입력이 콘솔로 가서 WASD·L·Esc가 먹지 않으므로 버튼 줄이 꼭 있어야 한다.
function renderMap() {
  const { sc, st, events } = current();
  if (!ui.live) {
    return `${statusStrip(st, events)}<section class="panel console-view" aria-label="지도 콘솔"><header><h2>지도 콘솔</h2>${src("map")}</header>
      <div class="map-empty">게이트웨이에 연결된 화면에서 지도 파트 콘솔(apps/console)의 실시간 지도 화면이 여기에 크게 보입니다.</div></section>`;
  }
  return `${statusStrip(st, events)}${alertBox(sc, events)}${consolePanel(st)}${controlBar(st)}`;
}

function renderReview() {
  const { st, events, track } = current();
  const all = events;
  const count = (r) => all.filter((e) => e.review === r).length;
  const autonomyOk = st.autonomy_used && st.operator_interventions === 0;
  const cards = [
    [ui.live ? "모터 동작 시간" : "비행 시간", clock(st.flight_time_s)],
    ["탐색 진행", st.mission ? pct(st.mission.progress) : "없음"],
    ["발견", `${all.length}건`, `확인 ${count("confirmed")} · 오탐 ${count("false_positive")} · 미확인 ${count("unreviewed")}`],
    ["자율 판정", !st.autonomy_used ? "자율 안 씀" : autonomyOk ? "개입 없음" : `개입 ${st.operator_interventions}회`,
      st.autonomy_used && !autonomyOk ? "자율 성공으로 세지 않음" : ""],
  ];
  const tl = ui.live
    ? ui.timeline.map((x) => `<li class="${x.kind}"><time>${esc(x.time)}</time>${esc(x.text)}</li>`).join("")
    : S.timeline.filter((x) => x.t_s <= (track[track.length - 1]?.t_s ?? 0))
      .map((x) => `<li class="${x.kind}"><time>${clock(x.t_s)}</time>${esc(x.text)}</li>`).join("");
  return `<section class="summary" data-part="summary" aria-label="임무 요약">${cards.map(([k, v, sub]) => `<div class="metric"><span>${k}</span><b>${v}</b>${sub ? `<small>${sub}</small>` : ""}</div>`).join("")}</section>
    <p class="note">임무 후 보고용 화면입니다. 발견마다 확인·오탐을 정하면 요약에 바로 반영됩니다.</p>
    <div class="review-grid">${detectionList(all, true)}${mapPanel(st, all, track, true)}</div>
    <div class="review-grid two">
      ${ui.live ? map3dPanel(st, `<header><h2>3D 지도</h2>${src("map")}</header>`, "map3d-panel") : `<section class="panel" aria-label="3D 지도"><header><h2>3D 지도</h2>${src("map")}</header>
        <div class="map3d"><p>팀 지도 화면(<code>apps/console</code>)의 점군·카메라 경로를 여기에 띄울 자리입니다.</p>
        <p class="muted">게이트웨이에 연결하고 지도 연결기를 켜면 여기에 점군이 그려집니다. 단위·좌표계는 지도 쪽 표시를 따릅니다.</p>
        <a class="link" href="http://127.0.0.1:8790/" target="_blank" rel="noopener">3D 지도 열기</a></div></section>`}
      <section class="panel" data-part="timeline" aria-label="비행 기록"><header><h2>비행 기록</h2>${src("drone")}</header>
        <ol class="timeline">${tl || '<li class="empty">기록 없음</li>'}</ol></section>
    </div>`;
}

function renderControl() {
  const { sc, st, events } = current();
  const latest = events[0];
  const can = (n) => st.allowed.includes(n);
  const big = (n, label) => `<button class="big ${can(n) ? "" : "off"}" data-cmd="${n}">${label}</button>`;
  const grounded = st.mode === "grounded";
  const confirmed = areaConfirmed(st);
  return `<div class="phone">
    ${statusStrip(st, events, true)}
    ${alertBox(sc, events)}
    <div data-part="latest">${latest ? `<div class="alert danger slim"><b>최근 발견</b><span>사람 ${latest.confidence.toFixed(2)} · ${findingTime(latest)}</span></div>` : ""}</div>
    <p class="note">현장에서 드론 옆에 두는 화면입니다. 자율 시작은 PC 감독 화면에서만 할 수 있습니다.</p>
    <div class="big-grid" data-part="big">
      ${st.mode === "paused" ? big("resume", "다시 시작") : big("pause", "일시정지")}
      ${big("return_home", "복귀")}
      ${grounded ? big("takeoff", "이륙") : big("land", "착륙")}
      ${grounded
        ? `<button class="big ${can("confirm_area") ? "" : "off"}" data-cmd="confirm_area" data-confirmed="${!confirmed}" aria-pressed="${confirmed}">${confirmed ? "✓ 주변 확인됨" : "주변 확인"}</button>`
        : `<button class="big ${can("takeover") ? "" : "off"}" data-cmd="takeover">넘겨받기</button>`}
    </div>
    <div class="sticks" aria-label="조이스틱">
      <div class="stick" data-axes="ud,yaw"><span class="knob"></span><small>상하 · 회전</small></div>
      <div class="stick" data-axes="lr,fb"><span class="knob"></span><small>좌우 · 앞뒤</small></div>
    </div>
    <p class="muted center" id="axes">입력 없음 · 자율 중에 움직이면 넘겨받기</p>
    <button class="estop wide" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>0.6초 길게</small></button>
  </div>`;
}

// ---------- 그리기: 바뀐 칸만 바꾼다 (영상·조이스틱·길게 누르기가 10Hz 갱신에 끊기지 않게) ----------
function view() {
  const v = (location.hash.replace("#/", "") || "live");
  return ["live", "map", "review", "control"].includes(v) ? v : "live";
}

function render() {
  const v = view();
  const html = { live: renderLive, map: renderMap, review: renderReview, control: renderControl }[v]();
  const app = $("#app");
  const tpl = document.createElement("template");
  tpl.innerHTML = html;
  const parts = [...tpl.content.querySelectorAll("[data-part]")];
  const keys = parts.map((p) => p.dataset.part).join(",");
  if (ui.view !== v || ui.cache.__keys !== keys) {
    ui.view = v;
    document.body.dataset.view = v;
    document.querySelectorAll(".views a").forEach((a) => a.setAttribute("aria-current", a.dataset.view === v ? "page" : "false"));
    app.innerHTML = html;
    ui.cache = { __keys: keys };
    parts.forEach((p) => { ui.cache[p.dataset.part] = p.outerHTML; });
    bindSticks();
    attachMap3d();
    return placeConsole();
  }
  for (const p of parts) {
    const k = p.dataset.part, s = p.outerHTML;
    if (ui.cache[k] === s) continue;
    ui.cache[k] = s;
    const el = app.querySelector(`[data-part="${k}"]`);
    if (el) el.replaceWith(document.importNode(p, true));
  }
  attachMap3d();
  placeConsole();
}

// ---------- 게이트웨이 연결 ----------
function send(msg) {
  if (!ui.live) return false;
  if (!ui.wsOpen) { toast("게이트웨이에 연결되지 않았습니다"); return false; }
  ui.ws.send(JSON.stringify({ schema_version: "0.1.0", client: view() === "control" ? "control" : "monitor", ...msg }));
  return true;
}

function sendCmd(name, args = {}) {
  if (!ui.live) {
    if (name === "emergency") return toast("샘플 화면: 비상 정지는 게이트웨이에 연결된 화면에서만 보냅니다");
    return toast(`샘플 화면: ${CMD_LABEL[name]} 명령은 보내지 않습니다`);
  }
  send({ type: "cmd", id: `c-${++ui.seq}`, name, args });
}

function onMessage(msg) {
  if (msg.type === "drone_state") {
    ui.state = msg;
  } else if (msg.type === "result" && !msg.ok) {
    const failed = (msg.failed || []).map((k) => CHECK[k] || k).join(", ");
    toast(`${REASON[msg.reason] || msg.reason}${failed ? `: ${failed}` : ""}`);
  } else if (msg.type === "detection") {
    onDetection(msg);
  } else if (msg.type === "event") {
    const d = msg.detail || {};
    const text = msg.kind === "mode_changed" ? `${(MODE[d.from] || [d.from])[0]} → ${(MODE[d.to] || [d.to])[0]} (${d.reason})`
      : msg.kind === "safety_landing" ? `안전 착륙: ${d.reason}`
      : msg.kind === "video_changed" ? `영상 ${d.on ? "켜짐" : "꺼짐"} (${VIDEO_REASON[d.reason] || d.reason})` : msg.kind;
    const kind = msg.kind === "mode_changed" ? "mode" : msg.kind === "video_changed" ? "video" : "intervention";
    ui.timeline.unshift({ time: hhmmss(msg.at), kind, text });
    ui.timeline.length = Math.min(ui.timeline.length, 200);
    if (msg.kind === "safety_landing") toast(`안전 착륙: ${d.reason}`);
    if (msg.kind === "video_changed" && d.reason === "ground_timeout") toast("지상에서 영상이 오래 켜져 있어 껐습니다 (기체 발열)");
  }
  render();
}

function onDetection(msg) {
  ui.lastDet = { event: msg.event, latency_ms: msg.latency_ms, at: performance.now() };
  const f = msg.finding, isNew = !ui.findings.has(f.finding_id);
  ui.findings.set(f.finding_id, f);
  if (isNew) {
    ui.newFindingAt = Date.now();
    ui.timeline.unshift({ time: hhmmss(f.first_seen), kind: "detection", text: `사람 발견 ${f.best.confidence.toFixed(2)} (${ui.findings.size}번)` });
    ui.timeline.length = Math.min(ui.timeline.length, 200);
  }
}

async function loadFindings() {  // 화면을 새로 열거나 다시 연결했을 때 그동안의 발견
  try {
    const r = await fetch("/api/detections", { cache: "no-store" });
    if (!r.ok) return;
    const d = await r.json();
    for (const f of [...d.findings].reverse()) ui.findings.set(f.finding_id, f);
  } catch (e) { /* 탐지 수신이 없는 게이트웨이 */ }
}

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/drone`);
  ui.ws = ws;
  ws.onopen = async () => { ui.wsOpen = ui.everOpen = true; await loadFindings(); render(); };
  ws.onmessage = (ev) => { try { onMessage(JSON.parse(ev.data)); } catch (e) { /* 잘못된 메시지는 무시 */ } };
  ws.onclose = () => { ui.wsOpen = false; held.clear(); render(); setTimeout(connect, 1000); };
}

async function detectGateway() {
  if (new URLSearchParams(location.search).has("sample") || location.protocol === "file:") return false;
  try {
    const r = await fetch("/api/drone/state", { cache: "no-store" });
    if (!r.ok) return false;
    ui.state = await r.json();
    return !!ui.state && ui.state.type === "drone_state";
  } catch (e) {
    return false;
  }
}

// ---------- 입력 ----------
document.addEventListener("click", (ev) => {
  const mm = ev.target.closest("[data-map-mode]");
  if (mm) {
    ui.map3d.mode = mm.dataset.mapMode;
    savePref("map-mode", ui.map3d.mode);
    return render();
  }
  if (ev.target.closest("[data-map-big]")) {
    ui.mapBig = !ui.mapBig;
    savePref("map-big", ui.mapBig ? "1" : "0");
    return render();
  }
  if (ev.target.closest("[data-console-reload]")) return reloadConsole();
  const rv = ev.target.closest("[data-review]");
  if (rv) {
    const id = rv.closest("[data-event]").dataset.event;
    ui.reviews[id] = ui.reviews[id] === rv.dataset.review ? "unreviewed" : rv.dataset.review;
    return render();
  }
  const pick = ev.target.closest(".det[data-event], .mark[data-event]");
  if (pick) {
    document.querySelectorAll("[data-event]").forEach((el) => el.classList.toggle("sel", el.dataset.event === pick.dataset.event));
    return;
  }
  const fb = ev.target.closest("[data-feed]");
  if (fb) {
    if (fb.classList.contains("off")) return toast("AI 합성: 사람 탐지 프로그램이 합성 영상을 보내고 있지 않습니다");
    ui.feed = fb.dataset.feed;
    savePref("controller.feed", ui.feed);
    return render();
  }
  const b = ev.target.closest("[data-cmd]");
  if (!b) return;
  const name = b.dataset.cmd;
  if (name === "return_home") return toast("복귀는 자율 비행 쪽과 맞춘 뒤 연결합니다 (화면 자리만)");
  if (b.classList.contains("off")) {
    const why = name === "auto_start" && ui.live ? "자율 비행이 연결되지 않았습니다"
      : name.startsWith("video_") ? REASON.video_disabled : "지금 상태에서는 할 수 없습니다";
    return toast(`${CMD_LABEL[name]}: ${why}`);
  }
  if (name === "takeoff" && ui.live && !confirm("이륙할까요? 주변을 다시 확인하세요.")) return;
  sendCmd(name, name === "confirm_area" ? { confirmed: b.dataset.confirmed === "true" } : {});
});

// 비상 정지는 길게 눌러야 한다(0.6초). 실수로 눌러 공중에서 모터가 멈추지 않게.
// 마우스·터치, 버튼에서 Space·Enter, 화면 어디서나 Esc.
const HOLD_MS = 600;
function startHold(b, endEvents, target = b) {
  if (b.classList.contains("holding")) return;
  b.classList.add("holding");
  const t = setTimeout(() => { b.classList.remove("holding"); sendCmd("emergency"); }, HOLD_MS);
  const cancel = () => { clearTimeout(t); b.classList.remove("holding"); };
  endEvents.forEach((name) => target.addEventListener(name, cancel, { once: true }));
}
document.addEventListener("pointerdown", (ev) => {
  const b = ev.target.closest("[data-hold]");
  if (b) startHold(b, ["pointerup", "pointerleave", "pointercancel"]);
});

// 키보드 조종(PC): 누르고 있는 동안만 20Hz로 보낸다. 떼거나 창을 벗어나면 0.
const held = new Set();
const AXES = { a: [0, -1], d: [0, 1], s: [1, -1], w: [1, 1], arrowdown: [2, -1], arrowup: [2, 1], arrowleft: [3, -1], arrowright: [3, 1] };
function keyAxes() {
  const v = [0, 0, 0, 0];
  for (const k of held) { const a = AXES[k]; if (a) v[a[0]] += a[1]; }
  return v.map((x) => Math.max(-1, Math.min(1, x)));
}
const typing = (t) => t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA");
document.addEventListener("keydown", (ev) => {
  const k = ev.key.toLowerCase();
  const hold = ev.target.closest && ev.target.closest("[data-hold]");
  if (hold && (k === " " || k === "enter") && !ev.repeat) { ev.preventDefault(); return startHold(hold, ["keyup", "blur"]); }
  if (typing(ev.target)) return;
  if (k === "escape" && !ev.repeat) {
    const b = $("[data-hold]");
    if (b) { ev.preventDefault(); startHold(b, ["keyup"], document); }
    return;
  }
  if (!ui.live || view() === "review") return;
  if (AXES[k]) { ev.preventDefault(); held.add(k); send({ type: "rc", axes: keyAxes() }); return; }
  if (k === "l" && !ev.repeat) sendCmd("land");
});
document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (AXES[k] && held.delete(k)) send({ type: "rc", axes: keyAxes() });
});
window.addEventListener("blur", () => { if (held.size) { held.clear(); send({ type: "rc", axes: [0, 0, 0, 0] }); } });

// 폰 조이스틱: 왼쪽 상하·회전, 오른쪽 좌우·앞뒤. 오른쪽·앞·위·시계 방향이 +.
const stickAxes = { "ud,yaw": [0, 0], "lr,fb": [0, 0] };   // 각 스틱의 [가로, 세로]
const STICK_NAMES = { "ud,yaw": ["회전", "상하"], "lr,fb": ["좌우", "앞뒤"] };   // [가로, 세로]
function joyAxes() {
  const [yaw, ud] = stickAxes["ud,yaw"], [lr, fb] = stickAxes["lr,fb"];
  return [lr, fb, ud, yaw].map((x) => Math.round(x * 100) / 100);
}
setInterval(() => {
  if (!ui.live || !ui.wsOpen) return;
  if (held.size) send({ type: "rc", axes: keyAxes() });
  else if (Object.values(stickAxes).some(([x, y]) => x || y)) send({ type: "rc", axes: joyAxes() });
}, 50);

function bindSticks() {
  document.querySelectorAll(".stick").forEach((stick) => {
    const knob = $(".knob", stick);
    const key = stick.dataset.axes;
    const move = (ev) => {
      const r = stick.getBoundingClientRect(), rad = r.width / 2;
      let dx = ev.clientX - (r.left + rad), dy = ev.clientY - (r.top + rad);
      const d = Math.hypot(dx, dy), lim = rad - 22;
      if (d > lim) { dx *= lim / d; dy *= lim / d; }
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
      stickAxes[key] = [dx / lim, -dy / lim];
      const [h, v] = STICK_NAMES[key];
      $("#axes").textContent = `${h} ${(dx / lim).toFixed(2)} · ${v} ${(-dy / lim).toFixed(2)}${ui.live ? "" : " (샘플: 보내지 않음)"}`;
    };
    stick.onpointerdown = (ev) => { stick.setPointerCapture(ev.pointerId); move(ev); stick.onpointermove = move; };
    stick.onpointerup = stick.onpointercancel = () => {
      stick.onpointermove = null; knob.style.transform = ""; stickAxes[key] = [0, 0];
      $("#axes").textContent = "입력 없음 · 손 떼면 제자리";
      send({ type: "rc", axes: joyAxes() });
    };
  });
}

// ---------- 시작 ----------
(async () => {
  ui.live = await detectGateway();
  const sel = $("#scenario");
  const badge = $(".badge.sample");
  if (ui.live) {
    $("#scenario-wrap").hidden = true;
    badge.textContent = ui.state.source === "fake" ? "가짜 Tello" : ui.state.source === "replay" ? "기록 재생" : "실기체";
    badge.title = "게이트웨이에 연결된 화면입니다. 버튼이 실제로 명령을 보냅니다.";
    badge.classList.toggle("real", ui.state.source === "tello");
    connect();
  } else {
    sel.innerHTML = Object.entries(S.scenarios).map(([k, v]) => `<option value="${k}">${v.label}</option>`).join("");
    sel.value = ui.scenario;
    sel.onchange = () => { ui.scenario = sel.value; render(); };
  }
  window.addEventListener("hashchange", render);
  window.addEventListener("resize", placeConsole);
  render();
})();
