// 실내 탐색 관제 화면.
// - 게이트웨이(gateway_server.py)가 이 화면을 내주면 /ws/drone으로 실제 상태를 받고 명령을 보낸다(실시간).
// - 정적 서버로 열거나 주소에 ?sample을 붙이면 계약 형식의 샘플을 그리기만 한다(명령 안 보냄).
// 화면 3개: #/live(PC 실시간 감독), #/review(임무 후 탐색 결과), #/control(폰 현장 조작).
// 실시간 감독의 칸 배치는 프리셋(presets.js)을 따른다(PANELS).
"use strict";

const S = window.SAMPLE;
const $ = (sel, root = document) => root.querySelector(sel);
const ui = {
  scenario: "person", reviews: {},      // reviews: event_id → confirmed / false_positive
  live: false, ws: null, wsOpen: false, everOpen: false, state: null, timeline: [], seq: 0, cache: {}, view: null,
  findings: new Map(), lastDet: null, newFindingAt: 0,   // 실시간 사람 탐지: finding_id → 발견, 영상 위 윤곽선용 최근 탐지
  feed: loadPref("controller.feed", "raw") === "ai" ? "ai" : "raw",   // 영상 칸: raw(원본) / ai(사람 탐지 합성 영상). 합성이 끊기면 원본을 보여 준다
  map3d: { mode: loadPref("map-mode", "3d"), host: null, view: null, revision: undefined, loading: false, has: false, editable: false },  // 3D 점군(지도 연결기)
  mapBig: loadPref("map-big", "0") === "1",   // 지도 칸을 가장 큰 칸(기본 프리셋은 영상)과 바꿔 크게 본다
  preset: new URLSearchParams(location.search).get("preset") || loadPref("controller.preset", ""),   // 실시간 감독 칸 배치(presets.js)
  edit: null,   // 화면 편집 중이면 { draft: 고치는 배치, from: 시작한 프리셋 id }
  console: { frame: null, url: null, slot: null, ro: null, loadedUp: null },
  autonomy: { connected: false, path: null, people: null, pathRev: 0, peopleRev: 0, loading: false, track: [] },  // 자율 지도(/api/autonomy)
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
  invalid_command_args: "잘못된 조종 요청입니다", confirm_area_required: "이 화면에서 주변을 다시 확인하세요",
  takeoff_video_required: "이륙 전에 카메라 영상을 켜고 확인하세요",
  video_required_in_flight: "비행 중에는 조종 영상을 유지합니다",
  drone_io_error: "기체와 통신할 수 없습니다",
  client_not_allowed: "자율 시작은 PC 감독 화면에서만 할 수 있습니다", autonomy_not_connected: "자율 비행이 연결되지 않았습니다",
  autonomy_joined_in_air: "이륙 뒤에 연결된 자율이라 시작할 수 없습니다", replay_source: "기록 재생 중에는 명령을 보내지 않습니다",
  autonomy_rejected: "자율이 시작을 거절했습니다",
  busy: "앞 요청을 처리하는 중입니다", unknown_message: "알 수 없는 요청입니다",
  video_disabled: "게이트웨이가 영상 수신 없이(--no-video) 실행됐습니다", video_receiver_failed: "영상 수신을 시작하지 못했습니다",
  no_response: "기체가 응답하지 않았습니다", error_response: "기체가 거절했습니다",
};
const VIDEO_REASON = { operator: "화면", takeoff: "이륙", ground_timeout: "지상 오래 켜짐", link_lost: "드론 연결 끊김" };
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
    const done = st.mode === "paused" && ["arrived", "failed"].includes(st.mission?.phase);
    const alert = !ui.wsOpen ? (ui.everOpen ? "gateway" : "connecting") : w.includes("link_weak") ? "link"
      : w.includes("autonomy_disconnected") && st.mode === "paused" ? "autonomy_lost" : w.includes("autonomy_timeout") ? "autonomy_timeout"
      : w.includes("autonomy_stale") ? "stale" : done ? "mission_done" : null;
    const events = liveFindings();
    const person = !alert && events[0] && Date.now() - ui.newFindingAt < PERSON_ALERT_MS && events[0].review === "unreviewed";
    return { sc: { alert: alert || (person ? "person" : null) }, st, events, track: [] };
  }
  const sc = S.scenarios[ui.scenario];
  const events = S.events.slice(0, sc.detections).map((e) => ({ ...e, review: ui.reviews[e.event_id] || "unreviewed" }));
  const tNow = sc.state.mode === "grounded" ? 0 : S.track[S.track.length - 1].t_s;
  return { sc, st: adapt(sc.state), events: events.reverse(), track: S.track.filter((p) => p.t_s <= tNow) };   // 샘플에 없는 값(overlay 등)도 "없음"으로
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
    ${ui.live && st.autonomy?.connected ? chip(st.autonomy.ground_origin ? "자율 연결" : "자율 연결 · 공중에서 붙음", st.autonomy.ground_origin ? "accent" : "warning") : ""}
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
  } else if (sc.alert === "autonomy_lost") {
    body = `<div class="alert danger" role="alert"><b>자율 연결 끊김</b><span>자율 비행 연결이 끊겨 제자리에서 멈췄습니다. 착륙하거나 넘겨받아 조종하세요.</span>
      <button data-cmd="takeover">넘겨받기</button></div>`;
  } else if (sc.alert === "autonomy_timeout") {
    body = `<div class="alert warning" role="alert"><b>자율 명령 끊김</b><span>2초 넘게 이동 명령이 없어 일시정지했습니다. 자율 상태를 확인하고 다시 시작하거나 넘겨받으세요.</span>
      <button data-cmd="takeover">넘겨받기</button></div>`;
  } else if (sc.alert === "mission_done") {
    const failed = ui.state?.mission?.phase === "failed";
    const why = failed ? esc(ui.state.mission.reason || "자율이 임무를 끝내지 못했습니다") : "자율 탐색을 마쳤습니다";
    body = `<div class="alert ${failed ? "danger" : "warning"} slim" role="status"><b>${failed ? "임무 실패" : "임무 끝"}</b><span>${why}. 제자리에서 기다립니다. 착륙하거나 넘겨받으세요.</span>
      <button data-cmd="land">착륙</button></div>`;
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
    : v.ground_s != null ? [v.ground_off_s != null ? `지상 ${Math.round(v.ground_s)}초 · ${Math.round(v.ground_off_s)}초에 꺼짐`
        : `지상 ${Math.round(v.ground_s)}초 켜짐${st.warnings.includes("video_on_ground") ? " · 발열 주의" : ""}`,
      st.warnings.includes("video_on_ground") ? "warning" : "neutral"]
    : ["영상 켜짐", "success"];
  const name = on ? "video_off" : "video_on";
  const off = v.enabled === false || (on && st.mapping_console_url &&
    (st.flight_busy === "takeoff" || ["manual", "auto", "paused", "landing"].includes(st.mode)));
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
  const live = ui.live && !tall ? consoleLiveUrl(st) : null;
  const seg = ui.live && !tall ? `<div class="seg map-mode" role="group" aria-label="지도 보기">
      ${Object.entries(MAP_MODES).map(([m, label]) => `<button data-map-mode="${m}" class="${mode === m ? "on" : ""}" aria-pressed="${mode === m}">${label}</button>`).join("")}</div>
      ${live ? `<a class="map-open" href="${esc(live)}" target="_blank" rel="noopener" title="지도 파트 콘솔의 실시간 지도를 새 탭에서 엽니다">실시간 지도 ↗</a>` : ""}
      ${!ui.mapCanBig ? "" : `<button class="map-big" data-map-big aria-pressed="${ui.mapBig}" title="${ui.mapBig ? "지도를 원래 칸으로 되돌립니다" : "지도와 가장 큰 칸의 자리를 바꿔 지도를 크게 봅니다"}">${ui.mapBig ? "⤡ 원래대로" : "⤢ 크게"}</button>`}` : "";
  const head = `<header><h2>지도</h2>${src("map")}${mode === "2d" ? chip("경로: 자율 비행", "neutral", "src-inline") : ""}
      <span class="spacer"></span>${seg || `<a class="link" href="${CONSOLE_URL}" target="_blank" rel="noopener">3D 지도 열기 (apps/console)</a>`}</header>`;
  if (mode === "3d") return map3dPanel(st, head);
  if (ui.live) return autonomyMapPanel(st, head, tall);
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

// 자율 지도(2D): 자율 비행이 보낸 위치·계획 경로·찾은 사람(frame map: 켠 자리 원점, x 앞·y 왼쪽, m).
// 화면에는 x를 오른쪽, y를 위로 그린다. 지나온 길은 화면이 위치를 모아 그린다(자율 연결이 새로 붙으면 비운다).
// 경로·찾은 사람은 길어서 상태에는 버전만 오고, 버전이 바뀌면 GET /api/autonomy로 읽는다.
function syncAutonomy(st) {
  const a = ui.autonomy, s = st.autonomy;
  if (!s) return;
  if (s.connected && !a.connected) a.track = [];
  a.connected = s.connected;
  const p = st.pose;
  if (p && p.valid) {
    const last = a.track[a.track.length - 1];
    if (!last || Math.hypot(p.x_m - last[0], p.y_m - last[1]) > 0.03) a.track.push([p.x_m, p.y_m]);
    if (a.track.length > 3000) a.track.splice(0, a.track.length - 3000);
  }
  if ((s.path_revision !== a.pathRev || s.people_revision !== a.peopleRev) && !a.loading) loadAutonomy();
}

async function loadAutonomy() {
  const a = ui.autonomy;
  a.loading = true;
  try {
    const r = await fetch("/api/autonomy", { cache: "no-store" });
    if (r.ok) {
      const d = await r.json();
      a.path = d.path; a.people = d.people;
      a.pathRev = d.status.path_revision; a.peopleRev = d.status.people_revision;
    }
  } catch (e) { /* 다음 상태 때 다시 */ }
  a.loading = false;
}

function autonomyMapPanel(st, head, tall) {
  const a = ui.autonomy, pose = st.pose, s = st.autonomy || {};
  const path = a.path?.points || [], found = a.people?.people || [], unsure = a.people?.unconfirmed || [];
  if (!pose && !path.length && !found.length && !unsure.length) {
    return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
      <div class="map-empty">${s.connected ? "자율 비행이 연결됐습니다. 위치 추정이 시작되면 위치·경로·찾은 사람이 m 단위로 그려집니다."
        : "자율 비행이 연결되면 드론 위치·지나온 길·계획 경로·찾은 사람이 m 단위로 그려집니다."}</div></section>`;
  }
  const pts = [[0, 0], ...path, ...a.track, ...found.map((q) => [q.x_m, q.y_m]), ...unsure.map((q) => [q.x_m, q.y_m]),
    ...(pose ? [[pose.x_m, pose.y_m]] : [])];
  const span = (vals) => {
    const lo = Math.min(...vals) - 0.6, hi = Math.max(...vals) + 0.6, d = Math.max(3, hi - lo), c = (lo + hi) / 2;
    return [c - d / 2, c + d / 2];
  };
  const [minX, maxX] = span(pts.map((p) => p[0])), [minY, maxY] = span(pts.map((p) => p[1]));
  const W = 800, H = tall ? 360 : 260, k = Math.min(W / (maxX - minX), H / (maxY - minY));
  const X = (x) => ((x - minX) * k + (W - (maxX - minX) * k) / 2).toFixed(1);
  const Y = (y) => (H - (y - minY) * k - (H - (maxY - minY) * k) / 2).toFixed(1);
  const line = (list, cls) => list.length > 1 ? `<polyline class="${cls}" points="${list.map((p) => `${X(p[0])},${Y(p[1])}`).join(" ")}"/>` : "";
  const grid = [];
  for (let gx = Math.ceil(minX); gx <= maxX; gx++) grid.push(`<line x1="${X(gx)}" y1="0" x2="${X(gx)}" y2="${H}"/>`);
  for (let gy = Math.ceil(minY); gy <= maxY; gy++) grid.push(`<line x1="0" y1="${Y(gy)}" x2="${W}" y2="${Y(gy)}"/>`);
  const label = { found: "확정", gone: "다시 봤는데 없음", unsure: "미확정" };
  const person = (q, n, cls) => {
    const tip = `사람 ${n} · ${label[cls]} · 신뢰도 ${q.conf != null ? q.conf.toFixed(2) : "?"} · ${q.hits ?? "?"}번 봄${q.top_m != null ? ` · 키 약 ${q.top_m.toFixed(1)} m` : ""}`;
    const num = cls === "unsure" ? "" : `<text x="${X(q.x_m)}" y="${+Y(q.y_m) + 4}">${n}</text>`;
    return `<g class="mark ${cls}"><title>${esc(tip)}</title><circle cx="${X(q.x_m)}" cy="${Y(q.y_m)}" r="${cls === "unsure" ? 7 : 10}"/>${num}</g>`;
  };
  const marks = found.map((q, i) => person(q, i + 1, q.gone ? "gone" : "found")).join("")
    + unsure.map((q, i) => person(q, found.length + i + 1, "unsure")).join("");
  const drone = pose ? `<g class="drone ${pose.valid ? "" : "stale"}" transform="translate(${X(pose.x_m)},${Y(pose.y_m)}) rotate(${(90 - pose.yaw_rad * 180 / Math.PI).toFixed(1)})"><path d="M0,-11 L8,7 L0,3 L-8,7 Z"/></g>` : "";
  const start = `<circle class="start" cx="${X(0)}" cy="${Y(0)}" r="7"/><text class="start-label" x="${+X(0) + 12}" y="${+Y(0) + 4}">출발</text>`;
  const scale = `<g class="scale"><line x1="12" y1="${H - 12}" x2="${(12 + k).toFixed(1)}" y2="${H - 12}"/><text x="${(16 + k).toFixed(1)}" y="${H - 8}">1 m</text></g>`;
  const ms = st.mission, people = s.people;
  const info = [s.connected ? "자율 연결" : "자율 끊김",
    ms ? `${ms.state || "?"} · ${PHASE[ms.phase] || ms.phase || "?"}${ms.progress != null ? ` · 진행 ${pct(ms.progress)}` : ""}` : null,
    people ? `찾은 사람 ${people.confirmed}명${people.gone ? ` · 사라짐 ${people.gone}` : ""}${people.unconfirmed ? ` · 미확정 ${people.unconfirmed}` : ""}` : null,
    pose && !pose.valid ? `위치 ${pose.age_s ?? "?"}초 전` : null].filter(Boolean).join(" · ");
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="자율 지도: 드론 위치, 지나온 길, 계획 경로, 찾은 사람">
      <g class="grid">${grid.join("")}</g>${line(path, "planned")}${line(a.track, "flown")}${start}${marks}${drone}${scale}
    </svg>
    <p class="legend"><i class="lg flown"></i>지나온 길 <i class="lg planned"></i>계획 경로 <i class="lg mark"></i>찾은 사람 <i class="lg mark gone"></i>다시 봤는데 없음 · ${esc(info)} · 자율 지도(m, 켠 자리 기준)</p>
  </section>`;
}

// 3D 점군: 캔버스(ui.map3d.host)는 한 번만 만들고 다시 그릴 때마다 이 자리(.map3d-slot)로 옮긴다(attachMap3d).
// 화면 전체를 0.1초마다 다시 그리므로, 캔버스를 HTML로 다시 만들면 점군을 매번 처음부터 올리게 된다.
// 지도 칸 상태 [이름, 색, 설명]: 연결기 → 지도 서버 → 지도 세션 → 영상 입력 순으로 본다.
// 세션이 끝나거나 영상이 끊겨도 받은 점군은 그대로 보여 주고(has) 상태만 바꾼다.
// 지도 콘솔은 영상이 오래 끊겨 세션을 끝낸 뒤에도 입력 상태를 reconnecting으로 두므로, 세션이 끝났는지를 먼저 본다.
function mapInfo(m, has) {
  const ms = m?.mapping_state, lost = ["reconnecting", "failed"].includes(m?.input_state);
  const videoLost = ["영상 끊겨 지도 끝남", "warning", "영상이 오래 끊겨 지도 콘솔이 지도를 끝냄"];
  if (!m || !m.alive) return ["지도 연결기 꺼짐", "warning", "지도 서버를 켜고 gateway_map_bridge.py를 실행하면 점군이 들어옵니다."];
  if (m.upstream === "unreachable") return ["지도 서버 꺼짐", "danger", `${m.source_url || "지도 서버"}에 연결할 수 없습니다.`];
  if (m.upstream === "error") return ["지도 읽기 오류", "danger", ""];
  if (ms === "failed") return lost ? videoLost : ["지도 오류로 끝남", "danger", "이유는 지도 콘솔에서 확인"];
  if (ms === "stopping") return ["지도 마무리 중", "neutral", "지도 콘솔이 마지막 지도를 정리하는 중"];
  if (["complete", "completed", "stopped", "cancelled"].includes(ms)) return lost ? videoLost : ["지도 끝남", "neutral", "끝난 지도"];
  if (m.input_state === "reconnecting") {   // 지도 콘솔이 드론 영상을 다시 기다리는 중(곧 돌아오면 같은 세션)
    return ["영상 기다리는 중", "warning", `영상${m.input_age_s != null ? ` ${Math.round(m.input_age_s)}초째` : ""} 없음 · 곧 돌아오면 같은 지도로 이어짐`];
  }
  if (m.upstream === "no_map") {
    if (ms && !["idle", "offline"].includes(ms)) return ["지도 시작 중", "neutral", "첫 점군을 기다리는 중"];
    return has ? ["지도 멈춤", "neutral", "진행 중인 지도 없음"] : ["지도 아직 없음", "neutral", `지도 서버 상태: ${ms || "알 수 없음"}`];
  }
  return ["지도 수신 중", "success", ""];
}

function map3dPanel(st, head, cls = "map") {   // cls: 실시간 감독은 지도 칸(map), 탐색 결과는 3D 칸
  const m = st.map, has = ui.map3d.has;
  const unit = m?.unit === "m" ? "m" : "임의 단위(m 아님)";
  const info = mapInfo(m, has);
  const detail = has && m ? `버전 ${m.revision ?? "?"} · 점 ${m.points}${m.points_total > m.points ? `/${m.points_total}` : ""} · ${unit}${info[2] ? ` · ${info[2]}` : ""}` : info[2];
  const target = cls === "map" && ui.map3d.editable && m?.points > 0 ? consoleTarget(st) : null;
  const editor = target?.url && new URL("/editor.html?source=supervisor", target.url).href;
  return `<section class="panel ${cls}" data-part="map3d" aria-label="3D 지도">${head}
    <div class="map3d-slot">${has ? "" : `<div class="map-empty">${esc(info[2] || "점군을 기다리는 중입니다.")}</div>`}</div>
    <div class="map3d-info">${chip(info[0], info[1])}<span class="muted">${esc(detail)}</span>
      ${editor ? `<a class="map-open" href="${esc(editor)}" target="_blank" rel="noopener">3D 지도 편집 ↗</a>` : ""}
      ${has ? `<span class="muted keys-hint">왼쪽 드래그 회전 · 우클릭 드래그 이동 · 휠/두 손가락 확대 · 두 번 눌러 처음 시점</span>` : ""}</div></section>`;
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
      const editable = map.source?.pose_source !== "fake";   // 체험의 가짜 지도 서버는 편집기를 제공하지 않는다.
      const changed = !m.has || m.editable !== editable;
      m.has = true;
      m.editable = editable;
      if (changed) { ui.cache = {}; render(); }
    }
  } catch (e) { /* 다음 상태 갱신 때 다시 */ }
  m.loading = false;
}

// 지도 콘솔 화면(#/map): 촬영 영상 복원 전용 화면을 연다.
// 그래서 iframe은 #app 밖(body)에 한 번만 만들고 이 화면의 자리(.console-slot) 위에 겹쳐 둔다(placeConsole).
// 화면을 다시 그리거나 실시간 감독 등 다른 화면으로 가도 콘솔이 새로 열리지 않는다(다른 화면에서는 숨기기만).
const CONSOLE_URL = "http://127.0.0.1:8790/";   // 지도 연결기가 콘솔 주소를 알려 주기 전 기본값(apps/console 기본 포트)
const DEMO_CONSOLE_URL = "http://127.0.0.1:8796/";
const LOOPBACK = ["127.0.0.1", "localhost", "[::1]"];

function consoleTarget(st) {
  const fallback = st?.source === "fake" && location.port === "8030" ? DEMO_CONSOLE_URL : st?.map?.source_url || CONSOLE_URL;
  let u;
  try { u = new URL(st?.mapping_console_url || fallback); } catch (e) { return { url: null, note: "지도 콘솔 주소를 읽을 수 없습니다." }; }
  if (!['http:', 'https:'].includes(u.protocol)) return { url: null, note: "지도 콘솔 주소를 읽을 수 없습니다." };
  if (LOOPBACK.includes(u.hostname) && !LOOPBACK.includes(location.hostname)) {
    return { url: null, note: "지도 콘솔이 드론 PC 안(127.0.0.1)에서만 열려 있어 이 기기에서는 볼 수 없습니다. 드론 PC의 화면에서 보세요." };
  }
  return { url: new URL("/reconstruct.html", u).href, origin: u.origin };
}

// 지도 칸 "실시간 지도 ↗": 지도 콘솔 첫 화면의 실시간 지도 탭(#live). #/map은 영상 복원 전용이라 따로 연다.
function consoleLiveUrl(st) {
  const target = consoleTarget(st);
  return target.url ? new URL("/#live", target.origin).href : null;   // 이 기기에서 못 여는 주소면 링크를 안 보인다
}

function placeConsole() {
  const c = ui.console;
  const slot = view() === "map" ? $("#app .console-slot") : null;
  const t = slot ? consoleTarget(ui.state) : null;
  const up = ui.state?.map?.alive ? ui.state.map.upstream : null;
  if (c.frame && t?.url && c.loadedUp === "unreachable" && up && up !== "unreachable") c.url = null;
  if (!c.frame && t?.url) {   // 콘솔 보기를 처음 고를 때 연다
    c.frame = document.createElement("iframe");
    c.frame.className = "console-frame away";
    c.frame.title = "촬영 영상 복원";
    c.frame.allow = "fullscreen";
    document.body.append(c.frame);
  }
  if (!c.frame) return;
  if (t?.url && c.url !== t.url) {
    c.url = t.url; c.frame.src = t.url; c.loadedUp = up;
  }
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
  const btn = (n, extra = "") => `<button data-cmd="${n}" class="${can(n) ? "" : "off"} ${extra}" ${can(n) ? "" : 'aria-disabled="true"'}>${n === "takeoff" && takeoffUntil ? "주변 확인 후 한 번 더" : CMD_LABEL[n]}</button>`;
  const manualOn = st.mode === "manual", autoOn = st.mode === "auto";
  const autoCmd = st.mode === "paused" && can("resume") ? "resume" : "auto_start";
  const grounded = st.mode === "grounded" && st.flight_busy !== "takeoff";
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
    ${ui.live && st.mapping_console_url ? `<button data-keyboard-focus aria-pressed="${keyboardEnabled}">${keyboardEnabled ? "키보드 조종 활성" : "키보드 조종 활성화"}</button>` : ""}
    ${ui.live ? '<span class="muted keys-hint">WASD 이동 · ↑↓ 높이 · ←→ 회전 · T 이륙 확인 · L 착륙 · Esc 비상 정지</span>' : ""}
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

// ---------- 실시간 감독 칸(패널)과 배치(프리셋) ----------
// 칸마다 그리는 함수 하나. 어느 칸을 격자 어디에 둘지는 프리셋(presets.js)이 정한다.
// 상태 줄·알림·조종 줄(착륙·비상 정지)은 칸이 아니다. 어떤 프리셋에서도 격자 밖에 그대로 있다.
// min: 편집할 때 줄일 수 있는 가장 작은 크기 [가로 칸, 세로 칸].
const PANELS = {
  video: { name: "영상", min: [3, 2], render: (c) => videoPanel(c.st, c.events) },
  findings: { name: "발견 목록", min: [3, 2], render: (c) => detectionList(c.events) },
  map: { name: "지도", min: [3, 2], render: (c) => mapPanel(c.st, c.events, c.track) },
  timeline: { name: "비행 기록", min: [3, 1], render: (c) => timelinePanel(c.track) },
};
const PRESETS = window.PRESETS || [];   // 기본 프리셋(presets.js, 고칠 수 없음)
const overlaps = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
const MINE_KEY = "controller.presets.mine";   // 내 프리셋: 화면 편집에서 저장한 것(이 브라우저에만)
let MINE = loadMine();
const allPresets = () => [...PRESETS, ...MINE];

function livePreset() {
  if (ui.edit) return ui.edit.draft;   // 편집 중에는 고치고 있는 배치
  return allPresets().find((p) => p.id === ui.preset) || PRESETS[0]
    || { id: "all", name: "모든 칸", grid: [12, 6], panels: Object.keys(PANELS).map((id, i) => ({ id, x: 0, y: i * 2, w: 12, h: 2 })) };
}

// 저장해 둔 프리셋을 믿고 쓸 수 있게: 격자 크기를 제한하고, 모르는 칸·같은 칸 두 번·겹치는 칸은 뺀다.
function normalizePreset(p) {
  if (!p || typeof p !== "object" || !Array.isArray(p.panels)) return null;
  const int = (v, lo, hi, d) => (Number.isFinite(Number(v)) ? Math.max(lo, Math.min(hi, Math.round(Number(v)))) : d);
  const grid = [int(p.grid?.[0], 4, 24, 12), int(p.grid?.[1], 3, 12, 6)];
  const panels = [];
  for (const c of presetCells({ grid, panels: p.panels.filter((x) => x && typeof x === "object") })) {
    if (panels.some((o) => o.id === c.id || overlaps(o, c))) continue;
    panels.push({ id: c.id, x: c.x, y: c.y, w: c.w, h: c.h, ...(c.id === "map" && MAP_MODES[c.mode] ? { mode: c.mode } : {}) });
  }
  return panels.length ? { id: String(p.id || ""), name: String(p.name || "내 프리셋").slice(0, 40), grid, panels } : null;
}

function loadMine() {
  try {
    const list = JSON.parse(localStorage.getItem(MINE_KEY) || "[]");
    return Array.isArray(list) ? list.map(normalizePreset).filter((p) => p && p.id.startsWith("mine-")) : [];
  } catch (e) { return []; }
}

function storeMine() {
  try { localStorage.setItem(MINE_KEY, JSON.stringify(MINE)); return true; } catch (e) { return false; }
}

// 프리셋 바꾸기: 크게 보기는 풀고, 프리셋이 지도 보기(2D/3D)를 정해 두었으면 그것으로 시작한다(그 뒤 바꾸는 건 자유).
function choosePreset(id) {
  ui.preset = id;
  savePref("controller.preset", id);
  ui.mapBig = false;
  savePref("map-big", "0");
  const mode = livePreset().panels.find((c) => c.id === "map")?.mode;
  if (MAP_MODES[mode]) { ui.map3d.mode = mode; savePref("map-mode", mode); }
  render();
}

// 프리셋을 그릴 칸 목록으로: 모르는 칸은 빼고, 격자 밖으로 나간 칸은 안으로 넣는다. o는 좁은 화면에서 쌓는 순서.
function presetCells(p) {
  const [cols, rows] = p.grid;
  const fit = (v, lo, hi) => Math.max(lo, Math.min(hi, Math.round(Number(v) || 0)));
  return p.panels.filter((c) => PANELS[c.id]).map((c, o) => {
    const w = fit(c.w, 1, cols), h = fit(c.h, 1, rows);
    return { id: c.id, x: fit(c.x, 0, cols - w), y: fit(c.y, 0, rows - h), w, h, o, mode: c.mode };
  });
}

// 지도 크게 보기: 지도 칸과 가장 큰 칸의 자리(위치·크기·쌓는 순서)를 바꾼다.
function swapBiggest(cells, id) {
  const a = cells.find((c) => c.id === id);
  const b = cells.filter((c) => c !== a).sort((x, y) => y.w * y.h - x.w * x.h)[0];
  if (!a || !b || b.w * b.h <= a.w * a.h) return cells;
  return cells.map((c) => (c === a ? { ...b, id: a.id } : c === b ? { ...a, id: b.id } : c));
}

function board(ctx, p) {
  let cells = presetCells(p);
  const m = cells.find((c) => c.id === "map");
  ui.mapCanBig = !!m && cells.some((c) => c.w * c.h > m.w * m.h);   // 지도가 이미 가장 큰 칸이면 크게 보기 버튼을 숨긴다
  if (!ui.edit && ui.live && ui.mapBig && ui.mapCanBig) cells = swapBiggest(cells, "map");   // 크게 보기 버튼은 게이트웨이 연결 화면에만 있다
  const [cols, rows] = p.grid;
  const layout = `${ui.edit ? "edit|" : ""}${cells.map((c) => `${c.id}:${c.x},${c.y},${c.w},${c.h},${c.o}`).join(";")}`;   // 배치가 바뀌면 다시 그린다(render)
  const slots = ui.edit ? Array.from({ length: cols * rows }, (_, i) => `<i class="slot" style="--x:${(i % cols) + 1};--y:${Math.floor(i / cols) + 1}"></i>`).join("") : "";
  return `<div class="live-main board ${ui.edit ? "editing" : ""}" data-layout="${esc(layout)}" style="--cols:${cols};--rows:${rows}">${slots}${cells.map((c) =>
    `<div class="cell" data-panel="${c.id}" style="--x:${c.x + 1};--y:${c.y + 1};--w:${c.w};--h:${c.h};--o:${c.o}">${PANELS[c.id].render(ctx)}${ui.edit ? editCover(c, cells.length) : ""}</div>`).join("")}
    ${ui.edit && !cells.length ? `<p class="board-empty">위 편집 막대에서 넣을 칸을 고르세요.</p>` : ""}</div>`;
}

// ---------- 화면 ----------
function renderLive() {  // 모니터 한 화면(1366×768 이상)에 스크롤 없이 들어가게 배치한다.
  const ctx = current();
  const locked = ui.edit && editLocked();   // 편집 중에 이륙했거나 창이 좁아지면 편집을 닫는다(저장 안 함)
  if (locked) { ui.edit = null; syncEditbar(); toast(EDIT_CLOSED[locked]); }
  return `${statusStrip(ctx.st, ctx.events)}${alertBox(ctx.sc, ctx.events)}${board(ctx, livePreset())}${controlBar(ctx.st)}`;
}

function renderMap() {
  const target = consoleTarget(ui.state);
  return `<div class="console-slot" data-part="video-reconstruction">${target.url ? "" : `<p class="map-empty">${esc(target.note)}</p>`}</div>`;
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
  return `<section class="summary" data-part="summary" aria-label="임무 요약">${cards.map(([k, v, sub]) => `<div class="metric"><span>${k}</span><b>${v}</b>${sub ? `<small>${sub}</small>` : ""}</div>`).join("")}</section>
    <p class="note">임무 후 보고용 화면입니다. 발견마다 확인·오탐을 정하면 요약에 바로 반영됩니다.</p>
    <div class="review-grid">${detectionList(all, true)}${mapPanel(st, all, track, true)}</div>
    <div class="review-grid two">
      ${ui.live ? map3dPanel(st, `<header><h2>3D 지도</h2>${src("map")}</header>`, "map3d-panel") : `<section class="panel" aria-label="3D 지도"><header><h2>3D 지도</h2>${src("map")}</header>
        <div class="map3d"><p>팀 지도 화면(<code>apps/console</code>)의 점군·카메라 경로를 여기에 띄울 자리입니다.</p>
        <p class="muted">게이트웨이에 연결하고 지도 연결기를 켜면 여기에 점군이 그려집니다. 단위·좌표계는 지도 쪽 표시를 따릅니다.</p>
        <a class="link" href="http://127.0.0.1:8790/" target="_blank" rel="noopener">3D 지도 열기</a></div></section>`}
      ${timelinePanel(track)}
    </div>`;
}

// 비행 기록: 모드 바뀜·자율 연결·발견·거절 등. 실시간은 게이트웨이 이벤트를 모은 것(최근 200개), 샘플은 지금 시각까지.
function timelinePanel(track) {
  const tl = ui.live
    ? ui.timeline.map((x) => `<li class="${x.kind}"><time>${esc(x.time)}</time>${esc(x.text)}</li>`).join("")
    : S.timeline.filter((x) => x.t_s <= (track[track.length - 1]?.t_s ?? 0))
      .map((x) => `<li class="${x.kind}"><time>${clock(x.t_s)}</time>${esc(x.text)}</li>`).join("");
  return `<section class="panel timeline-panel" data-part="timeline" aria-label="비행 기록"><header><h2>비행 기록</h2>${src("drone")}</header>
    <ol class="timeline">${tl || '<li class="empty">기록 없음</li>'}</ol></section>`;
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
  const layout = tpl.content.querySelector("[data-layout]")?.dataset.layout ?? "";
  const keys = `${parts.map((p) => p.dataset.part).join(",")}|${layout}`;
  if (ui.view !== v || ui.cache.__keys !== keys) {
    ui.view = v;
    document.body.dataset.view = v;
    document.querySelectorAll(".views a").forEach((a) => a.setAttribute("aria-current", a.dataset.view === v ? "page" : "false"));
    $("#preset-wrap").hidden = v !== "live";   // 프리셋·화면 편집은 실시간 감독 화면만
    syncEditbar();
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
  if (!ui.wsOpen || ui.ws?.readyState !== WebSocket.OPEN) return false;
  if (msg.type === "rc" && msg.axes.some(Boolean) && ui.ws.bufferedAmount > 0) {
    ui.ws.close();   // 밀린 조종 입력에 새 이동을 쌓지 않는다.
    return false;
  }
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
    if (!msg.connected || (ui.state?.mode === "manual" && msg.mode !== "manual") ||
        (msg.mapping_console_url && ui.state?.video?.available && !msg.video?.available)) resetOperator();
    if (takeoffUntil && !takeoffReady(msg)) cancelTakeoff();
    ui.state = msg;
    syncAutonomy(msg);
  } else if (msg.type === "result" && !msg.ok) {
    const failed = (msg.failed || []).map((k) => CHECK[k] || k).join(", ");
    toast(`${REASON[msg.reason] || msg.reason}${failed ? `: ${failed}` : ""}`);
  } else if (msg.type === "detection") {
    onDetection(msg);
  } else if (msg.type === "event") {
    const d = msg.detail || {};
    if (msg.kind === "cmd_ignored") return render();   // 자율이 auto가 아닐 때 보낸 이동 명령(1초에 한 번까지). 기록에는 남기지 않는다
    const text = msg.kind === "mode_changed" ? `${(MODE[d.from] || [d.from])[0]} → ${(MODE[d.to] || [d.to])[0]} (${d.reason})`
      : msg.kind === "safety_landing" ? `안전 착륙: ${d.reason}`
      : msg.kind === "video_changed" ? `영상 ${d.on ? "켜짐" : "꺼짐"} (${VIDEO_REASON[d.reason] || d.reason})`
      : msg.kind === "autonomy_connected" ? `자율 연결 (${d.ground_origin ? "지상에서" : "공중에서 붙어 자율 시작 불가"})`
      : msg.kind === "autonomy_disconnected" ? "자율 연결 끊김"
      : msg.kind === "mission_finished" ? `임무 끝: ${d.phase === "arrived" ? "도착" : "실패"}${d.reason ? ` (${d.reason})` : ""}`
      : msg.kind === "command_rejected" ? `${CMD_LABEL[d.name] || d.name} 거절: ${REASON[d.reason] || d.reason}${d.detail ? ` (${d.detail})` : ""}` : msg.kind;
    const kind = msg.kind === "mode_changed" || msg.kind.startsWith("autonomy") || msg.kind === "mission_finished" ? "mode"
      : msg.kind === "video_changed" ? "video" : "intervention";
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
  ws.onclose = () => { ui.wsOpen = false; resetOperator(); render(); setTimeout(connect, 1000); };
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
  const keyboard = ev.target.closest("[data-keyboard-focus]");
  if (keyboard) {
    keyboardEnabled = !keyboardEnabled && ui.wsOpen && ui.state?.connected === true;
    if (!keyboardEnabled) resetOperator();
    render();
    $("[data-keyboard-focus]")?.focus();
    return;
  }
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
  if (name === "takeoff" && ui.live) {
    if (ev.detail > 0) requestTakeoff();
    return;
  }
  sendCmd(name, name === "confirm_area" ? { confirmed: b.dataset.confirmed === "true" } : {});
});

let takeoffUntil = 0, takeoffTimer = null, keyboardEnabled = false;
function takeoffReady(st) {
  return ui.wsOpen && st?.connected && st.allowed?.includes("takeoff") &&
    st.preflight?.every(check => check.ok || check.blocking === false) &&
    (!st.mapping_console_url || st.video?.available);
}
function cancelTakeoff() {
  clearTimeout(takeoffTimer);
  takeoffUntil = 0;
}
function requestTakeoff() {
  if (!takeoffReady(ui.state)) {
    cancelTakeoff();
    return toast("이륙 전 영상과 주변 확인, 기체 상태를 확인하세요.");
  }
  if (performance.now() < takeoffUntil) {
    cancelTakeoff();
    sendCmd("takeoff");
  } else {
    takeoffUntil = performance.now() + 4000;
    takeoffTimer = setTimeout(() => { cancelTakeoff(); render(); }, 4000);
  }
  render();
}

// 길게 누르는 비상 버튼은 포커스·포인터를 놓으면 취소한다.
const HOLD_MS = 600;
let emergencyHold = null;
function cancelEmergencyHold() {
  if (!emergencyHold) return;
  clearTimeout(emergencyHold.timer);
  emergencyHold.button.classList.remove("holding");
  emergencyHold = null;
}
function startHold(b, release) {
  if (emergencyHold || !ui.wsOpen || !document.hasFocus() || document.hidden) return;
  b.classList.add("holding");
  emergencyHold = {button: b, release, timer: setTimeout(() => {
    cancelEmergencyHold();
    if (b.isConnected && document.hasFocus() && !document.hidden) {
      resetOperator();
      sendCmd("emergency");
      render();
    }
  }, HOLD_MS)};
}
document.addEventListener("pointerdown", (ev) => {
  const b = ev.target.closest("[data-hold]");
  if (b && ev.isPrimary && ev.button === 0) {
    startHold(b, ev.pointerId);
    b.addEventListener("pointerleave", cancelEmergencyHold, {once: true});
  }
});
for (const event of ["pointerup", "pointercancel"]) document.addEventListener(event, (ev) => {
  if (emergencyHold?.release === ev.pointerId) cancelEmergencyHold();
});

// 허병우 팀원의 키 배치를 사용하고, 새로 누른 키만 조종 입력으로 삼는다.
const held = new Set();
const AXES = { KeyA: [0, -1], KeyD: [0, 1], KeyS: [1, -1], KeyW: [1, 1], ArrowDown: [2, -1], ArrowUp: [2, 1], ArrowLeft: [3, -1], ArrowRight: [3, 1] };
function keyAxes() {
  const v = [0, 0, 0, 0];
  for (const k of held) { const a = AXES[k]; if (a) v[a[0]] += a[1]; }
  return v.map((x) => Math.max(-1, Math.min(1, x)));
}
const typing = (t) => t?.isContentEditable || ["INPUT", "SELECT", "TEXTAREA", "IFRAME"].includes(t?.tagName);
function controlPageFocused() {
  return ui.wsOpen && !document.hidden && document.hasFocus() && !typing(document.activeElement) &&
    view() === "live";
}
function keysActive() {
  return controlPageFocused() && (!ui.state?.mapping_console_url || (keyboardEnabled && ui.state.video?.available));
}
function resetOperator() {
  const wasMoving = held.size || Object.values(stickAxes).some(([x, y]) => x || y);
  held.clear();
  keyboardEnabled = false;
  cancelEmergencyHold();
  cancelTakeoff();
  for (const key of Object.keys(stickAxes)) stickAxes[key] = [0, 0];
  for (const stick of document.querySelectorAll(".stick")) {
    stick.onpointermove = null;
    $(".knob", stick).style.transform = "";
  }
  if (wasMoving) send({ type: "rc", axes: [0, 0, 0, 0] });
}
document.addEventListener("keydown", (ev) => {
  const k = ev.code;
  if (ev.repeat || ev.isComposing || ev.ctrlKey || ev.altKey || ev.metaKey || typing(ev.target)) return;
  const hold = ev.target.closest?.("[data-hold]");
  if (hold && (k === "Space" || k === "Enter")) {
    ev.preventDefault();
    return startHold(hold, k);
  }
  if (!controlPageFocused()) return;
  if (k === "Escape") {
    ev.preventDefault();
    resetOperator();
    if (ui.state?.flight_busy === "takeoff" || ["manual", "auto", "paused", "landing"].includes(ui.state?.mode)) {
      sendCmd("emergency");
    }
    render();
  } else if (k === "KeyT" && keysActive()) {
    ev.preventDefault();
    requestTakeoff();
  } else if (k === "KeyL" && ui.state?.allowed?.includes("land")) {
    ev.preventDefault();
    resetOperator();
    sendCmd("land");
    render();
  } else if (AXES[k] && keysActive() && ui.state?.connected && ui.state.mode === "manual") {
    ev.preventDefault();
    held.add(k);
    send({ type: "rc", axes: keyAxes() });
  }
});
document.addEventListener("keyup", (ev) => {
  const k = ev.code;
  if (emergencyHold?.release === k) cancelEmergencyHold();
  if (AXES[k] && held.delete(k)) send({ type: "rc", axes: keyAxes() });
});
const leaveControls = () => { resetOperator(); render(); };
window.addEventListener("blur", leaveControls);
window.addEventListener("pagehide", resetOperator);
document.addEventListener("visibilitychange", () => { if (document.hidden) leaveControls(); });
document.addEventListener("focusin", (ev) => {
  if (typing(ev.target)) leaveControls();
  else if (emergencyHold && ev.target !== emergencyHold.button) cancelEmergencyHold();
});

// 폰 조이스틱: 왼쪽 상하·회전, 오른쪽 좌우·앞뒤. 오른쪽·앞·위·시계 방향이 +.
const stickAxes = { "ud,yaw": [0, 0], "lr,fb": [0, 0] };   // 각 스틱의 [가로, 세로]
const STICK_NAMES = { "ud,yaw": ["회전", "상하"], "lr,fb": ["좌우", "앞뒤"] };   // [가로, 세로]
function joyAxes() {
  const [yaw, ud] = stickAxes["ud,yaw"], [lr, fb] = stickAxes["lr,fb"];
  return [lr, fb, ud, yaw].map((x) => Math.round(x * 100) / 100);
}
setInterval(() => {
  if (held.size) {
    if (!keysActive() || ui.state?.mode !== "manual" || !ui.state.connected) return leaveControls();
    send({ type: "rc", axes: keyAxes() });
  } else if (Object.values(stickAxes).some(([x, y]) => x || y) && ui.live) {
    if (document.hidden || !ui.wsOpen || !ui.state?.connected ||
        view() !== "control" || !["manual", "auto", "paused"].includes(ui.state.mode)) return leaveControls();
    send({ type: "rc", axes: joyAxes() });
  }
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
    stick.onpointerdown = (ev) => {
      if (ui.live && (!ui.wsOpen || !ui.state?.connected || !["manual", "auto", "paused"].includes(ui.state.mode))) return;
      stick.setPointerCapture(ev.pointerId); move(ev); stick.onpointermove = move;
    };
    stick.onpointerup = stick.onpointercancel = stick.onlostpointercapture = () => {
      stick.onpointermove = null; knob.style.transform = ""; stickAxes[key] = [0, 0];
      $("#axes").textContent = "입력 없음 · 손 떼면 제자리";
      send({ type: "rc", axes: joyAxes() });
    };
  });
}

// ---------- 화면 편집(내 프리셋) ----------
// 땅에 있을 때, 넓은 창(격자 배치)에서만 연다. 칸을 끌어 옮기고(다른 칸 위에 놓으면 자리 바꿈) 오른쪽 아래 모서리로 크기를 바꾼다.
// 편집하는 동안 칸마다 덮개를 씌워 칸 안 버튼은 눌리지 않는다. 기본 프리셋은 고치지 않고 "내 프리셋"으로 새로 저장한다(이 브라우저에만).
const WIDE = "(min-width:1100px) and (min-height:700px)";   // style.css 격자 배치 기준과 같다
const EDIT_CLOSED = {
  flight: "비행 중이라 화면 편집을 닫았습니다(저장 안 됨). 프리셋 바꾸기는 됩니다",
  narrow: "창이 좁아 화면 편집을 닫았습니다(저장 안 됨). 가로 1100·세로 700 이상에서 편집하세요",
};
const isMine = (id) => MINE.some((p) => p.id === id);

function editLocked() {   // 편집할 수 없는 까닭(없으면 null)
  const st = ui.state;
  if (ui.live && st && (st.flight_busy === "takeoff" || !["grounded", "disconnected"].includes(st.mode))) return "flight";
  if (!matchMedia(WIDE).matches) return "narrow";
  return null;
}

function fillPresetPicker() {
  const opt = (p) => `<option value="${esc(p.id)}" title="${esc(p.note || "")}">${esc(p.name)}</option>`;
  $("#preset").innerHTML = PRESETS.map(opt).join("") + (MINE.length ? `<optgroup label="내 프리셋">${MINE.map(opt).join("")}</optgroup>` : "");
  $("#preset").value = livePreset().id;
}

// 칸 덮개: 중요한 순서(좁은 화면에서 쌓는 순서)·이름·크기, 더 중요하게·빼기 버튼, 크기 조절 손잡이.
function editCover(c, n) {
  return `<div class="cell-edit" data-edit-cell="${c.id}">
    <span class="cell-head"><span class="cell-name"><b>${c.o + 1}</b> ${esc(PANELS[c.id].name)} · <span class="cell-size">${c.w}×${c.h}</span></span>
      <span class="cell-tools"><button data-edit-up="${c.id}" ${c.o === 0 ? "disabled" : ""} title="더 중요하게(좁은 화면에서 더 위에 쌓임)">중요 ↑</button><button data-edit-remove="${c.id}" title="이 칸 빼기" aria-label="${esc(PANELS[c.id].name)} 칸 빼기">✕</button></span></span>
    <span class="cell-hint">${n > 1 ? "끌어서 옮기기 · 다른 칸 위에 놓으면 자리 바꿈" : "끌어서 옮기기"}</span>
    <span class="cell-resize" data-edit-resize="${c.id}" title="끌어서 크기 조절"></span></div>`;
}

function openEdit() {
  const why = editLocked();
  if (why) return toast(why === "flight" ? "비행 중에는 화면을 편집할 수 없습니다. 프리셋 바꾸기는 됩니다" : "화면 편집은 넓은 창(가로 1100·세로 700 이상)에서 합니다");
  const p = livePreset();
  ui.edit = { draft: JSON.parse(JSON.stringify(p)), from: p.id };
  ui.mapBig = false;
  $("#edit-name").value = isMine(p.id) ? p.name : `${p.name} (내 것)`;
  syncEditbar();
  render();
}

// 편집 막대(index.html #editbar)는 #app 밖이라 10Hz 다시 그리기에 걸리지 않는다(이름 입력이 끊기지 않게). 바뀔 때만 여기서 맞춘다.
function syncEditbar() {
  const e = ui.edit;
  $("#editbar").hidden = !e || view() !== "live";
  $("#preset").disabled = !!e;
  $("#edit-layout").disabled = !!e;
  if (!e) return;
  const on = new Set(e.draft.panels.map((c) => c.id));
  $("#edit-panels").innerHTML = Object.entries(PANELS).map(([id, p]) =>
    `<button data-edit-toggle="${id}" aria-pressed="${on.has(id)}" title="${on.has(id) ? "빼기" : "빈 자리에 넣기"}">${on.has(id) ? "✓" : "+"} ${esc(p.name)}</button>`).join("");
  const mine = isMine(e.from);
  $('[data-edit="save"]').textContent = mine ? "저장" : "내 프리셋으로 저장";
  $('[data-edit="saveas"]').hidden = !mine;
  $('[data-edit="delete"]').hidden = !mine;
}

function editChanged() {
  syncEditbar();
  ui.cache = {};   // 끌다가 제자리로 돌아와도 칸 위치를 다시 맞추게 통째로 다시 그린다
  render();
}

function togglePanel(id) {
  const d = ui.edit.draft, i = d.panels.findIndex((c) => c.id === id);
  if (i >= 0) { d.panels.splice(i, 1); return editChanged(); }
  const [w, h] = PANELS[id].min, [cols, rows] = d.grid;
  for (let y = 0; y + h <= rows; y++) {
    for (let x = 0; x + w <= cols; x++) {
      if (!d.panels.some((o) => overlaps(o, { x, y, w, h }))) { d.panels.push({ id, x, y, w, h }); return editChanged(); }
    }
  }
  toast(`${PANELS[id].name} 칸을 넣을 빈 자리가 없습니다. 다른 칸을 줄이거나 빼세요`);
}

function raisePanel(id) {   // 중요한 순서를 하나 앞으로
  const l = ui.edit.draft.panels, i = l.findIndex((c) => c.id === id);
  if (i > 0) { [l[i - 1], l[i]] = [l[i], l[i - 1]]; editChanged(); }
}

// 끌기: 놓는 자리는 칸 단위로 맞춘다. 비어 있으면 옮기고, 다른 칸 위면 두 칸의 자리(위치·크기)를 바꾸고, 겹치면 되돌린다.
function startEditDrag(ev, id, kind) {
  const boardEl = $("#app .board"), cellEl = boardEl?.querySelector(`.cell[data-panel="${id}"]`);
  const d = ui.edit.draft, c0 = d.panels.find((c) => c.id === id);
  if (!cellEl || !c0) return;
  ev.preventDefault();
  const [cols, rows] = d.grid, r = boardEl.getBoundingClientRect();
  const gap = parseFloat(getComputedStyle(boardEl).columnGap) || 0;
  const cw = (r.width + gap) / cols, rh = (r.height + gap) / rows;
  const fit = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const fits = (c, rect) => rect.w >= PANELS[c.id].min[0] && rect.h >= PANELS[c.id].min[1];
  let next = c0, swap = null, bad = false;
  const move = (e) => {
    const dx = Math.round((e.clientX - ev.clientX) / cw), dy = Math.round((e.clientY - ev.clientY) / rh);
    swap = null;
    if (kind === "move") {
      const gx = Math.floor((e.clientX - r.left) / cw), gy = Math.floor((e.clientY - r.top) / rh);
      swap = d.panels.find((o) => o !== c0 && gx >= o.x && gx < o.x + o.w && gy >= o.y && gy < o.y + o.h) || null;
      next = swap ? c0 : { ...c0, x: fit(c0.x + dx, 0, cols - c0.w), y: fit(c0.y + dy, 0, rows - c0.h) };
    } else {
      const [mw, mh] = PANELS[id].min;
      next = { ...c0, w: fit(c0.w + dx, mw, cols - c0.x), h: fit(c0.h + dy, mh, rows - c0.y) };
    }
    bad = swap ? !fits(c0, swap) || !fits(swap, c0) : d.panels.some((o) => o !== c0 && overlaps(o, next));
    for (const [k, v] of [["--x", next.x + 1], ["--y", next.y + 1], ["--w", next.w], ["--h", next.h]]) cellEl.style.setProperty(k, v);
    const size = cellEl.querySelector(".cell-size");
    if (size) size.textContent = `${next.w}×${next.h}`;
    cellEl.classList.toggle("bad", bad);
    boardEl.querySelectorAll(".cell").forEach((el) => el.classList.toggle("swap", el.dataset.panel === swap?.id));
  };
  const end = (e) => {
    document.removeEventListener("pointermove", move);
    document.removeEventListener("pointerup", end);
    document.removeEventListener("pointercancel", end);
    if (!ui.edit) return;
    if (e.type === "pointerup" && bad) toast(swap ? "칸이 너무 작아져서 자리를 바꿀 수 없습니다" : "다른 칸과 겹치는 자리에는 놓을 수 없습니다");
    else if (e.type === "pointerup" && swap) {
      const a = { x: c0.x, y: c0.y, w: c0.w, h: c0.h };
      Object.assign(c0, { x: swap.x, y: swap.y, w: swap.w, h: swap.h });
      Object.assign(swap, a);
    } else if (e.type === "pointerup") Object.assign(c0, { x: next.x, y: next.y, w: next.w, h: next.h });
    editChanged();
  };
  document.addEventListener("pointermove", move);
  document.addEventListener("pointerup", end);
  document.addEventListener("pointercancel", end);
}

function saveEdit(asNew) {
  const d = ui.edit.draft;
  if (!d.panels.length) return toast("칸이 하나도 없습니다. 칸을 하나 이상 넣으세요");
  const keep = !asNew && isMine(ui.edit.from);
  const p = normalizePreset({ ...d, id: keep ? ui.edit.from : `mine-${Date.now().toString(36)}`, name: $("#edit-name").value.trim() || d.name });
  MINE = keep ? MINE.map((x) => (x.id === p.id ? p : x)) : [...MINE, p];
  const stored = storeMine();
  ui.edit = null;
  ui.preset = p.id;
  savePref("controller.preset", p.id);
  fillPresetPicker();
  editChanged();
  toast(stored ? `"${p.name}" 저장 · 이 브라우저에만 남습니다` : "브라우저 저장소에 쓰지 못했습니다. 이 창을 닫으면 사라집니다");
}

function deleteMine(btn) {   // 한 번 더 눌러야 지운다(되돌릴 수 없음)
  if (btn.dataset.armed !== "1") {
    btn.dataset.armed = "1";
    btn.textContent = "한 번 더 누르면 삭제";
    clearTimeout(deleteMine.t);
    deleteMine.t = setTimeout(() => { btn.dataset.armed = ""; btn.textContent = "삭제"; }, 3000);
    return;
  }
  clearTimeout(deleteMine.t);
  btn.dataset.armed = "";
  btn.textContent = "삭제";
  const gone = MINE.find((p) => p.id === ui.edit.from);
  MINE = MINE.filter((p) => p !== gone);
  storeMine();
  ui.edit = null;
  ui.preset = PRESETS[0]?.id || "";
  savePref("controller.preset", ui.preset);
  fillPresetPicker();
  editChanged();
  toast(`"${gone?.name}" 삭제`);
}

document.addEventListener("pointerdown", (ev) => {
  if (!ui.edit || ev.button !== 0 || ev.target.closest("button")) return;
  const h = ev.target.closest("[data-edit-resize], [data-edit-cell]");
  if (h) startEditDrag(ev, h.dataset.editResize || h.dataset.editCell, h.dataset.editResize ? "resize" : "move");
});

document.addEventListener("click", (ev) => {
  const t = ev.target.closest("#edit-layout, [data-edit], [data-edit-toggle], [data-edit-up], [data-edit-remove]");
  if (!t) return;
  if (t.id === "edit-layout") return openEdit();
  if (!ui.edit) return;
  if (t.dataset.editToggle) return togglePanel(t.dataset.editToggle);
  if (t.dataset.editRemove) return togglePanel(t.dataset.editRemove);
  if (t.dataset.editUp) return raisePanel(t.dataset.editUp);
  const a = t.dataset.edit;
  if (a === "save" || a === "saveas") return saveEdit(a === "saveas");
  if (a === "delete") return deleteMine(t);
  if (a === "cancel") { ui.edit = null; editChanged(); }
});

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
  // 프리셋 고르기. 고른 뒤 선택 상자에서 포커스를 빼야 WASD 조종 키가 다시 먹는다(선택 상자에 포커스가 가면 조종 입력은 멈춘다).
  const pick = $("#preset");
  fillPresetPicker();
  pick.onchange = () => { choosePreset(pick.value); pick.blur(); };
  window.addEventListener("resize", () => { if (ui.edit && editLocked()) render(); });
  window.addEventListener("hashchange", leaveControls);
  window.addEventListener("resize", placeConsole);
  render();
})();
