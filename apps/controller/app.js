// 실내 탐색 관제 화면.
// - 게이트웨이(gateway_server.py)가 이 화면을 내주면 /ws/drone으로 실제 상태를 받고 명령을 보낸다(실시간).
// - 정적 서버로 열거나 주소에 ?sample을 붙이면 계약 형식의 샘플을 그리기만 한다(명령 안 보냄).
// 화면 3개: #/live(PC 실시간 감독), #/review(임무 후 탐색 결과), #/control(폰 현장 조작).
// 실시간 감독의 칸 배치는 프리셋(presets.js)을 따른다(PANELS).
"use strict";

const S = window.SAMPLE;
const $ = (sel, root = document) => root.querySelector(sel);
const ui = {
  scenario: "person", reviews: {},      // 샘플의 확인·오탐: event_id → confirmed / false_positive(실시간은 게이트웨이가 발견마다 둔다)
  reviewPending: {},                    // 실시간: 게이트웨이에 보내는 중인 확인·오탐(finding_id → 값). 응답이 오면 지운다
  live: false, ws: null, wsOpen: false, everOpen: false, state: null, timeline: [], seq: 0, cache: {}, view: null,
  findings: new Map(), lastDet: null, newFindingAt: 0,   // 실시간 사람 탐지: finding_id → 발견, 영상 위 윤곽선용 최근 탐지
  feed: loadPref("controller.feed", "raw") === "ai" ? "ai" : "raw",   // 영상 칸: raw(원본) / ai(사람 탐지 합성 영상). 합성이 끊기면 원본을 보여 준다
  outline: loadPref("controller.outline", "on") !== "off",   // 영상 칸: 원본 위에 사람 윤곽선을 그릴지(보기 고르기, 이 브라우저에 기억)
  map3d: { mode: loadPref("map-mode", "3d"), host: null, view: null, revision: undefined, loading: false, has: false,  // 3D 점군(지도 연결기)
    data: null, plan: null, planOf: null },   // 받은 지도 한 버전과 그것으로 만든 평면도(2D 지도 칸, floorplan.js)
  mapBig: loadPref("map-big", "0") === "1",   // 지도 칸을 가장 큰 칸(기본 프리셋은 영상)과 바꿔 크게 본다
  mapHidden: new Set(loadPref("controller.map.hidden", "").split(",").filter(Boolean)),   // 지도 칸에서 끈 표시(flown·planned·people)
  preset: new URLSearchParams(location.search).get("preset") || loadPref("controller.preset", ""),   // 실시간 감독 칸 배치(presets.js)
  edit: null,   // 화면 편집 중이면 { draft: 고치는 배치, from: 시작한 프리셋 id }
  console: { frame: null, url: null, slot: null, ro: null, loadedUp: null, height: 0, viewportHeight: 0 },
  autonomy: { connected: false, path: null, people: null, pathRev: 0, peopleRev: 0, loading: false, track: [] },  // 자율 지도(/api/autonomy)
  takeoff: { open: false, confirmedAt: null, sampleChecked: false, focus: null },   // 이륙 전 확인 칸(열림, 주변 확인 보낸 시각, 샘플 체크, 다시 그린 뒤 초점)
};
if (!["2d", "3d"].includes(ui.map3d.mode)) ui.map3d.mode = "3d";   // 지도 콘솔은 따로 화면(#/map)
const liveSave = window.createLiveMapSave?.({onChange: () => render()});
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
// 비행 기록에 적는 까닭: 게이트웨이 조종권 관리자(mode_arbiter)가 모드를 바꾼 까닭과 자율 비행이 보낸 임무 까닭을 쉬운 말로.
// 모르는 코드는 그대로 적는다(지어내지 않는다).
const WHY = {
  takeoff: "이륙", land: "착륙 명령", landed: "착륙 끝", emergency: "비상 정지 명령", emergency_stopped: "비상 정지 뒤 멈춤",
  pause: "일시정지 명령", resume: "다시 시작 명령", auto_start: "자율 시작 명령", auto_stop: "자율 종료 명령",
  takeover: "넘겨받기", joystick_takeover: "조이스틱으로 넘겨받기",
  state_lost: "드론 상태 끊김", state_ok: "드론 상태 다시 받음", battery_low: "배터리 부족", overheat: "과열",
  autonomy_disconnected: "자율 연결 끊김", autonomy_timeout: "자율 응답 없음", autonomy_rejected: "자율이 시작을 거절",
  autonomy_land: "자율이 착륙 요청", emergency_request: "자율이 비상 요청(제자리 멈춤)",
  mission_arrived: "임무 끝(도착)", mission_failed: "임무 실패",
  route_done: "경로 끝", goal_reached: "목표 도착", operator_stop: "사람이 멈춤", pose_not_ready: "위치가 아직 준비되지 않음",
};
const reasonText = (code) => (code == null ? "" : WHY[code] || code);

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

// 실시간 발견 번호는 게이트웨이가 매긴 발견 번호(finding-0003 → 3번)다. 화면마다 받은 순서가 달라도(목록을 받기 전에
// 새 발견이 먼저 오는 등) 모든 화면에서 같은 번호·같은 순서가 되게 한다. 번호를 못 읽으면 받은 순서.
const findingSeq = (id) => +(/(\d+)$/.exec(id || "")?.[1] || 0);

// 게이트웨이 발견(finding)을 카드 모양으로(최신순). 실시간은 발견 하나가 카드 하나다(같은 사람이 보이는 동안 묶임).
function liveFindings() {
  const list = [...ui.findings.values()].map((f, i) => [f, findingSeq(f.finding_id) || i + 1]).sort((a, b) => b[1] - a[1]);
  return list.map(([f, n]) => {
    const b = f.best, det = b.detections.reduce((a, d) => (d.confidence > a.confidence ? d : a));
    return {
      event_id: f.finding_id, n, confidence: b.confidence, people: f.max_people, events: f.events,
      bbox_xyxy_px: det.bbox_xyxy_px, image_width: b.image.width_px, image_height: b.image.height_px,
      seen_at: f.first_seen, last_seen: f.last_seen, frame_id: b.frame_id, photo_url: b.photo_url, drone: b.drone,
      review: ui.reviewPending[f.finding_id] || f.review || "unreviewed",   // 보내는 중이면 누른 값을 먼저 보인다
    };
  });
}

const findingNumberOf = (id) => findingSeq(id) || [...ui.findings.keys()].indexOf(id) + 1;   // 카드와 같은 번호

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
// 핵심 상태 카드: 모드·배터리·높이·발견·연결. 비행 중 짧은 순간에 보는 값이라 크게, 늘 같은 자리에 둔다.
// 연결 카드는 팀 파트(드론·영상·사람 탐지·지도·자율 비행)가 지금 붙어 있는지를 한 줄로 보여 준다.
const statCard = (name, value, sub, tone = "", cls = "") =>
  `<div class="stat ${tone} ${cls}"><span>${esc(name)}</span><b>${value}</b>${sub ? `<small>${sub}</small>` : ""}</div>`;

// 파트마다 [색, 짧은 말, 설명]. 색만으로 뜻을 나누지 않게 정상이 아니면 짧은 말을 붙인다. 모르면 모른다고 쓴다.
function partStates(st, events) {
  if (ui.live && !ui.wsOpen) {
    const unknown = ["neutral", "모름", "게이트웨이에 연결되지 않아 알 수 없습니다"];
    return [["드론", unknown], ["영상", unknown], ["사람 탐지", unknown], ["지도", unknown], ["자율 비행", unknown]];
  }
  const [linkText, linkTone] = LINK[st.link.level] || [];
  const drone = st.connected === false ? ["danger", "끊김", "드론 상태가 오지 않습니다"]
    : linkTone && linkTone !== "success" ? [linkTone, linkText.replace("통신 ", ""), linkText]
    : ["success", "", linkText || "드론 상태 수신 중"];
  const v = st.video;
  const video = v.enabled === false ? ["neutral", "안 받음", "게이트웨이가 영상 수신 없이(--no-video) 켜졌습니다"]
    : v.available ? ["success", "", `영상 ${v.fps ?? "?"}fps`]
    : !ui.live || v.on === false ? ["neutral", "꺼짐", "이륙하면 영상이 켜집니다"]
    : ["warning", "기다림", "영상을 켰지만 아직 프레임이 없습니다"];
  const ds = ui.live ? detectStatus() : null;
  const detect = !ui.live ? (events.length ? ["success", "", "샘플 발견"] : ["neutral", "연결 전", "샘플"])
    : ds.alive ? ["success", "", ds.text] : ds.idle ? ["neutral", ds.ever ? "쉼" : "연결 전", ds.text]
    : ds.ever ? ["warning", "끊김", ds.text] : ["neutral", "연결 전", ds.text];
  const mi = ui.live ? mapInfo(st.map, ui.map3d.has) : null;
  const map = !ui.live ? ["neutral", "샘플", "샘플 화면은 지도에 연결하지 않습니다"]
    : [mi[1], mi[1] === "success" ? "" : mi[0].replace(/^지도 /, ""), mi[2] || mi[0]];   // 설명이 있으면 설명만(짧은 말과 겹치지 않게)
  const a = st.autonomy || {};
  const auto = !ui.live ? (st.autonomy_used ? ["success", "", "샘플 자율 비행"] : ["neutral", "연결 전", "샘플"])
    : a.connected ? (a.ground_origin ? ["success", "", "자율 연결(지상에서)"] : ["warning", "공중 연결", "이륙 뒤에 연결된 자율이라 자율 시작을 할 수 없습니다"])
    : st.warnings.includes("autonomy_disconnected") ? ["danger", "끊김", "자율 비행 연결이 끊겼습니다"]
    : ["neutral", "연결 전", "자율 비행이 아직 연결되지 않았습니다"];
  return [["드론", drone], ["영상", video], ["사람 탐지", detect], ["지도", map], ["자율 비행", auto]];
}

function statusStrip(st, events, compact = false) {
  const [mode, tone] = MODE[st.mode] || [st.mode, "neutral"];
  const who = [AUTHORITY[st.authority], st.mission?.progress != null ? `진행 ${pct(st.mission.progress)}` : ""].filter(Boolean).join(" · ");
  const b = st.battery_pct;
  const btone = b == null ? "" : b <= 15 ? "danger" : b < 30 ? "warning" : "";
  const d = st.down_tof;
  const hsub = d.valid ? "하방 센서" : d.reason === "below_min_or_on_ground" ? "땅이거나 너무 가까움" : "하방 센서 값 없음";
  const n = events.length, unreviewed = events.filter((e) => e.review === "unreviewed").length;
  // 연결 전·꺼짐(빈 점)은 말을 줄이고 마우스를 올리면 설명을 보인다. 주의·끊김·모름은 말을 붙인다.
  const parts = partStates(st, events).map(([name, [ptone, word, tip]]) => {
    const say = word && (ptone !== "neutral" || word === "모름");
    return `<span class="part ${ptone}" title="${esc(`${name}: ${word ? `${word} · ` : ""}${tip}`)}">${esc(name)}${say ? ` <em>${esc(word)}</em>` : ""}</span>`;
  }).join("");
  const [linkText] = LINK[st.link.level] || ["통신 측정 전"];
  const recv = st.link.state_hz != null ? ` · ${st.link.state_hz}Hz · ${st.link.rtt_ms}ms` : "";
  return `<section class="status ${compact ? "compact" : ""}" data-part="strip" aria-label="비행 상태">
    ${statCard("모드", esc(mode), esc(who), tone === "neutral" ? "" : tone)}
    ${statCard("배터리", b == null ? "없음" : `${b}%`, `${ui.live ? "모터" : "비행"} ${clock(st.flight_time_s)}`, btone)}
    ${statCard("높이", d.valid ? meters(d.m) : "없음", hsub)}
    ${statCard("발견", n ? `${n}건` : "없음", n ? `미확인 ${unreviewed}${n > unreviewed ? ` · 정리 ${n - unreviewed}` : ""}` : "아직 없음", n ? "person" : "")}
    <div class="stat wide"><span>연결</span><p class="parts">${parts}</p><small>${esc(linkText + recv)}</small></div>
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
  const lost = ui.live && !ui.wsOpen;   // 게이트웨이와 끊기면 마지막 값 대신 끊겼다고 적고 버튼을 잠근다
  const [text, tone] = lost ? ["게이트웨이 끊김", "neutral"] : v.enabled === false ? ["영상 수신 안 함", "neutral"]
    : !on ? ["영상 꺼짐", "neutral"]
    : v.ground_s != null ? [v.ground_off_s != null ? `지상 ${Math.round(v.ground_s)}초 · ${Math.round(v.ground_off_s)}초에 꺼짐`
        : `지상 ${Math.round(v.ground_s)}초 켜짐${st.warnings.includes("video_on_ground") ? " · 발열 주의" : ""}`,
      st.warnings.includes("video_on_ground") ? "warning" : "neutral"]
    : ["영상 켜짐", "success"];
  const name = on ? "video_off" : "video_on";
  const off = lost || v.enabled === false || (on && st.mapping_console_url &&
    (st.flight_busy === "takeoff" || ["manual", "auto", "paused", "landing"].includes(st.mode)));
  // 칩(초 단위로 바뀜)과 버튼을 다른 칸으로 둔다. 버튼이 매초 바뀌면 누르는 중에 사라져 클릭이 빠진다.
  return `<span class="video-ctl"><span data-part="videostate">${chip(text, tone)}</span>
    <span data-part="videobtn"><button data-cmd="${name}" class="${off ? "off" : ""}" ${off ? 'aria-disabled="true"' : ""}>${CMD_LABEL[name]}</button></span></span>`;
}

// 보기 고르기: 원본 / AI 합성, 그리고 원본 위 사람 윤곽선 켜고 끄기(AI 합성은 윤곽이 영상에 들어 있어 잠근다).
// 합성 영상은 사람 탐지 프로그램이 게이트웨이 영상에 윤곽선·장면 분할을 그려 보낸 것이다.
function feedSelect(st, aiOn) {
  const ai = st.overlay.alive;
  const btn = (key, label, off) => `<button data-feed="${key}" class="${off ? "off" : ""}" aria-pressed="${ui.feed === key}" ${off ? 'aria-disabled="true"' : ""}>${label}</button>`;
  const outline = `<button class="mini outline-btn ${aiOn ? "off" : ""}" data-outline aria-pressed="${ui.outline}" ${aiOn ? 'aria-disabled="true"' : ""}
    title="${aiOn ? "AI 합성 영상에는 윤곽선이 들어 있습니다" : "원본 영상 위에 사람 탐지 윤곽선을 그립니다"}">사람 윤곽</button>`;
  return `<span class="view-sel" data-part="feedsel"><span class="feed-sel" role="group" aria-label="영상 고르기">${btn("raw", "원본", false)}${btn("ai", "AI 합성", !ai)}</span>${outline}</span>`;
}

// 영상이 없을 때 까닭. 게이트웨이와 끊기면 마지막 화면이 실시간처럼 보이지 않게 영상을 내린다.
function videoWhy(st) {
  if (!ui.live) return "샘플 화면 · 영상 없음";
  if (!ui.wsOpen) return "게이트웨이와 끊겨 영상이 멈췄습니다 · 다시 연결되면 이어집니다";
  if (st.video.enabled === false) return "영상 수신 안 함 · 게이트웨이를 영상 없이(--no-video) 켰습니다";
  if (st.video.on === false) return "영상 꺼짐 · 지상에서 켜 두면 기체가 뜨거워집니다. 이륙하면 자동으로 켜집니다";
  return "영상을 기다리는 중 · 아직 프레임이 오지 않았습니다";
}

function videoPanel(st, events, big = true) {
  const latest = events[0];
  const showing = ui.live && ui.wsOpen && st.video.available;
  const aiOn = showing && ui.feed === "ai" && st.overlay.alive;
  const box = aiOn || !ui.outline || (ui.live && !ui.wsOpen) ? "" : ui.live ? liveOutline() : latest && st.mode !== "grounded" ? (() => {
    const [x1, y1, x2, y2] = latest.bbox_xyxy_px;
    const w = latest.image_width, h = latest.image_height;
    return `<div class="bbox" style="left:${(x1 / w) * 100}%;top:${(y1 / h) * 100}%;width:${((x2 - x1) / w) * 100}%;height:${((y2 - y1) / h) * 100}%">
      <span>사람 ${latest.confidence.toFixed(2)}</span></div>`;
  })() : "";
  const media = aiOn ? `<img class="feed" src="/overlay.mjpg" alt="사람 탐지 합성 영상">`
    : showing ? `<img class="feed" src="/video.mjpg" alt="드론 카메라 실시간 영상">`
    : `<div class="placeholder">${esc(videoWhy(st))}</div>`;
  // 높이는 상태 카드에 있어 영상 위에는 앞 거리(확장 키트가 있을 때)만 적는다.
  const front = st.front_tof.valid ? `<span class="overlay bl">앞 ${meters(st.front_tof.m)}</span>` : "";
  const rate = !showing ? "" : aiOn ? `AI 합성 ${st.overlay.fps ?? "?"}fps · 원본 ${st.video.fps ?? "?"}fps`
    : ui.feed === "ai" ? `원본 ${st.video.fps ?? "?"}fps · AI 합성 끊김` : `원본 ${st.video.fps ?? "?"}fps`;
  return `<section class="panel video ${big ? "" : "small"}" aria-label="드론 영상">
    <header><h2>영상</h2><span class="src" title="이 칸을 채우는 파트">${aiOn ? "드론 영상 · 사람 탐지 합성" : "드론 영상 · 사람 탐지"}</span>${feedSelect(st, aiOn)}${videoControl(st)}</header>
    <div class="frame-wrap"><div class="frame">
      <div class="media" data-part="media">${media}</div>
      <div class="overlays" data-part="overlay">${box}${front}${rate ? `<span class="overlay br">${rate}</span>` : ""}</div>
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
  const more = e.seen_at ? ` · ${e.people > 1 ? `최대 ${e.people}명 · ` : ""}${e.events}번 봄` : ` · ${esc(e.frame_id)}`;   // 이 발견으로 묶인 탐지 장면 수
  return `<article class="det ${big ? "big" : ""} ${e.review}" data-event="${esc(e.event_id)}" tabindex="0">
    ${thumb}
    <div class="det-body">
      <div class="det-title">사람 · 신뢰도 ${e.confidence.toFixed(2)} ${chip(state[0], state[1])}</div>
      <div class="muted">${findingTime(e)}${more}</div>
      ${where}
      <div class="det-actions">${reviewButtons(e)}</div>
    </div>
  </article>`;
}

// 확인·오탐 버튼. 실시간에서 게이트웨이와 끊겼거나 기록 재생이면 흐리게 두고, 누르면 까닭을 알린다(sendReview).
function reviewButtons(e) {
  const locked = !ui.live ? "" : !ui.wsOpen ? "게이트웨이에 연결되지 않았습니다" : ui.state?.replay ? "기록 재생 중에는 바꿀 수 없습니다" : "";
  const b = (value, label) => `<button data-review="${value}"${locked ? ` class="off" title="${locked}"` : ""} aria-pressed="${e.review === value}">${label}</button>`;
  return b("confirmed", "확인") + b("false_positive", "오탐");
}

// 탐지 프로그램이 게이트웨이에 결과를 보내고 있는지. 사람이 없어도 결과는 계속 온다.
// 영상이 꺼져 있으면 탐지는 영상을 기다리므로 끊김이 아니라 쉼이다. 게이트웨이와 끊기면 마지막 상태 대신 끊겼다고 적는다.
function detectStatus() {
  const d = ui.state?.detection;
  if (ui.live && !ui.wsOpen) return { text: "게이트웨이 끊김", tone: "neutral", alive: false, ever: !!d?.events, lost: true };
  if (d?.alive) return { text: `탐지 동작 중 · ${d.events_per_s}장/s`, tone: "success", alive: true, ever: true };
  if (!ui.state?.video?.available) {   // 영상이 없으면 탐지도 볼 것이 없다(탐지가 켜져 있어도 결과를 안 보낸다)
    const off = ui.state?.video?.on === false;
    const word = off ? "영상 꺼짐" : "영상 기다림";
    return { text: d?.events ? `${word} · 탐지 쉼` : word, tone: "neutral", alive: false, ever: !!d?.events, idle: true, off };
  }
  if (d?.events) return { text: "탐지 끊김", tone: "warning", alive: false, ever: true };
  return { text: "탐지 연결 전", tone: "neutral", alive: false, ever: false };
}

function detectionList(events, big = false) {
  const ds = ui.live ? detectStatus() : null;
  // 빈 칸: 무엇을 기다리는지와 할 일을 쉬운 말로(프로그램 이름 대신). 받은 발견은 끊겨도 그대로 둔다.
  const empty = !ui.live ? "아직 발견한 사람이 없습니다. 탐지 결과가 오면 최신순으로 쌓입니다."
    : ds.lost ? "게이트웨이에 연결되지 않았습니다 · 연결되면 발견이 최신순으로 쌓입니다"
    : ds.alive ? "탐지 동작 중 · 아직 발견한 사람이 없습니다"
    : ds.idle ? (ds.ever ? "영상이 없어 탐지가 쉬고 있습니다 · 영상이 다시 들어오면 이어서 봅니다"
      : `${ds.off ? "영상이 꺼져 있습니다 · 영상을 켜면" : "영상을 기다리는 중입니다 · 영상이 들어오면"} 사람 탐지가 영상을 보고 발견을 보냅니다`)
    : ds.ever ? "사람 탐지 결과가 끊겼습니다 · 실행기로 켜면 함께 켜집니다"
    : "사람 탐지가 아직 연결되지 않았습니다 · 실행기로 켜면 함께 켜지고, 발견은 최신순으로 쌓입니다";
  const body = events.length ? events.map((e) => detectionCard(e, big)).join("") : `<p class="empty">${esc(empty)}</p>`;
  return `<section class="panel dets ${big ? "big" : ""}" data-part="dets" aria-label="발견 목록">
    <header><h2>발견</h2>${src("detect")}${ds ? chip(ds.text, ds.tone, "src-inline") : ""}</header>
    <div class="det-list">${body}</div>
  </section>`;
}

const MAP_MODES = { "2d": "2D 지도", "3d": "3D 점군" };
const MAP_MODE_TIP = { "2d": "자율 비행이 보낸 드론 위치·경로·찾은 사람(1칸 1 m). 자율 비행이 없으면 3D 점군을 위에서 본 평면도", "3d": "지도 파트가 드론 영상으로 만든 3D 점군" };

// 지도 칸 표시 고르기: 범례가 곧 켜고 끄는 버튼이다. 끈 표시는 이 브라우저에 기억한다(controller.map.hidden).
// 드론 위치와 출발은 늘 그린다. 3D 점군은 지나온 길(카메라 경로)만 고른다(사람 점은 지도 파트가 점군에 칠한다).
const LAYER_SWATCH = { flown: "flown", planned: "planned", people: "mark" };
const shown = (key) => !ui.mapHidden.has(key);

function layerButton(key, label, tip) {
  const on = shown(key);
  return `<button class="layer" data-map-layer="${key}" aria-pressed="${on}" title="${esc(`${tip} · 누르면 ${on ? "숨깁니다" : "다시 보입니다"}`)}"><i class="lg ${LAYER_SWATCH[key]}"></i>${esc(label)}</button>`;
}

// 지도 칸 아래 줄: 상태 칩과 설명(왼쪽), 범례 겸 표시 고르기(오른쪽).
function mapFoot(status, detail, layers, extra = "") {
  return `<div class="map-foot">${status ? chip(status[0], status[1]) : ""}${detail ? `<span class="muted">${esc(detail)}</span>` : ""}${extra}
    ${layers ? `<span class="layers" role="group" aria-label="지도에 보일 것">${layers}</span>` : ""}</div>`;
}

function mapPanel(st, events, track, tall = false) {
  const mode = ui.live && !tall ? ui.map3d.mode : "2d";   // 탐색 결과(tall)는 3D 칸이 따로 있다
  // 2D인데 자율 비행이 보낸 것이 하나도 없으면, 받은 3D 점군을 위에서 본 평면도를 그린다(점군도 없으면 빈 칸 안내).
  const plan = ui.live && mode === "2d" && !autonomyHas(st) ? floorPlan() : null;
  const seg = ui.live && !tall ? `<div class="seg map-mode" role="group" aria-label="지도 보기">
      ${Object.entries(MAP_MODES).map(([m, label]) => `<button data-map-mode="${m}" class="${mode === m ? "on" : ""}" aria-pressed="${mode === m}" title="${MAP_MODE_TIP[m]}">${label}</button>`).join("")}</div>
      ${!ui.mapCanBig ? "" : `<button class="map-big" data-map-big aria-pressed="${ui.mapBig}" title="${ui.mapBig ? "지도를 원래 칸으로 되돌립니다" : "지도와 가장 큰 칸의 자리를 바꿔 지도를 크게 봅니다"}">${ui.mapBig ? "⤡ 원래대로" : "⤢ 크게"}</button>`}` : "";
  // 출처는 칩 하나: 2D는 자율 비행이 보낸 위치·경로·찾은 사람, 3D와 평면도는 지도 파트의 점군. 샘플의 발견 지점은 사람 탐지 결과다.
  const source = mode === "3d" || plan ? "지도 파트" : ui.live ? "자율 비행" : "자율 비행 · 사람 탐지";   // 제목 "지도"와 겹치지 않게 "지도 파트"
  const head = `<header><h2>지도</h2><span class="src" title="이 칸을 채우는 파트">${source}</span><span class="spacer"></span>${seg || '<a class="link" href="#/map">지도 콘솔 열기</a>'}</header>`;
  if (mode === "3d") return map3dPanel(st, head);
  if (plan) return floorPlanPanel(st, head, plan, tall);
  if (ui.live) return autonomyMapPanel(st, head, tall);
  return sampleMapPanel(st, events, track, head, tall);
}

// 샘플 2D 지도(게이트웨이 없이): 방 구획·지나온 길·계획 경로·발견 지점. 발견 지점은 그때 드론 위치다(사람 위치 아님).
function sampleMapPanel(st, events, track, head, tall) {
  const pts = [...S.track, ...S.plan, ...S.rooms.flatMap((r) => [{ x_m: r.x, y_m: r.y }, { x_m: r.x + r.w, y_m: r.y + r.h }])];
  const minX = Math.min(...pts.map((p) => p.x_m)) - 0.6, maxX = Math.max(...pts.map((p) => p.x_m)) + 0.6;
  const minY = Math.min(...pts.map((p) => p.y_m)) - 0.6, maxY = Math.max(...pts.map((p) => p.y_m)) + 0.6;
  const W = 800, H = tall ? 360 : 230, k = Math.min(W / (maxX - minX), H / (maxY - minY));
  const X = (x) => ((x - minX) * k + (W - (maxX - minX) * k) / 2).toFixed(1);
  const Y = (y) => (H - (y - minY) * k - (H - (maxY - minY) * k) / 2).toFixed(1);  // y 왼쪽(위)이 +
  const rooms = S.rooms.map((r) => `<rect class="room" x="${X(r.x)}" y="${Y(r.y + r.h)}" width="${(r.w * k).toFixed(1)}" height="${(r.h * k).toFixed(1)}"/>
    <text class="room-label" x="${+X(r.x) + 8}" y="${+Y(r.y + r.h) + 18}">${esc(r.name)}</text>`).join("");
  const path = shown("flown") && track.length > 1 ? `<polyline class="flown" points="${track.map((p) => `${X(p.x_m)},${Y(p.y_m)}`).join(" ")}"/>` : "";
  const planned = shown("planned") && st.mission && st.mode !== "grounded"
    ? `<polyline class="planned" points="${S.plan.map((p) => `${X(p.x_m)},${Y(p.y_m)}`).join(" ")}"/>` : "";
  const marks = !shown("people") ? "" : events.map((e) => {
    const p = poseAt(e.pts_us / 1e6), n = S.events.findIndex((x) => x.event_id === e.event_id) + 1;
    return `<g class="mark ${e.review}" data-event="${esc(e.event_id)}"><circle cx="${X(p.x_m)}" cy="${Y(p.y_m)}" r="10"/><text x="${X(p.x_m)}" y="${+Y(p.y_m) + 4}">${n}</text></g>`;
  }).join("");
  const now = track[track.length - 1] || S.track[0];
  const drone = `<g class="drone" transform="translate(${X(now.x_m)},${Y(now.y_m)})"><path d="M0,-11 L8,7 L0,3 L-8,7 Z"/></g>`;
  const start = `<circle class="start" cx="${X(0)}" cy="${Y(0)}" r="7"/><text class="start-label" x="${+X(0) + 12}" y="${+Y(0) + 4}">출발</text>`;
  const layers = layerButton("flown", "지나온 길", "드론이 지나온 길") + layerButton("planned", "계획 경로", "자율 비행이 가려는 길")
    + layerButton("people", "발견 지점", "사람을 발견했을 때 드론이 있던 자리(사람 위치 아님)");
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="샘플 지도: 지나온 길, 계획 경로, 발견 지점, 지금 드론 위치">
      ${rooms}${path}${planned}${start}${marks}${drone}
    </svg>
    ${mapFoot(null, shown("people") && events.length ? "발견 지점은 그때 드론 위치입니다(사람 위치 아님)" : "", layers)}
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

// 자율 비행이 보낸 것(위치·경로·찾은 사람)이 하나라도 있는지. 없으면 2D 지도 칸은 평면도나 빈 칸 안내를 그린다.
function autonomyHas(st) {
  const a = ui.autonomy;
  return !!(st.pose || a.path?.points?.length || a.people?.people?.length || a.people?.unconfirmed?.length);
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

// 2D 지도 상태 [칩, 색, 설명]. 게이트웨이나 자율 비행이 끊겨도 받은 지도는 그대로 두고, 마지막으로 받은 것이라고 적는다.
function autonomyStatus(st) {
  const s = st.autonomy || {}, pose = st.pose;
  const last = "마지막으로 받은 지도입니다 · 다시 연결되면 이어집니다";
  if (!ui.wsOpen) return ["게이트웨이 끊김", "neutral", last];
  if (!s.connected) return ["자율 끊김", st.warnings.includes("autonomy_disconnected") ? "danger" : "neutral", last];
  if (pose && !pose.valid) return ["위치 멈춤", "warning", `마지막 위치 ${pose.age_s ?? "?"}초 전`];
  return ["자율 연결됨", "success", ""];
}

function autonomyMapPanel(st, head, tall) {
  const a = ui.autonomy, pose = st.pose, s = st.autonomy || {};
  const path = a.path?.points || [], found = a.people?.people || [], unsure = a.people?.unconfirmed || [];
  if (!autonomyHas(st)) {
    // 빈 칸: 무엇을 기다리는지 쉬운 말로. 지도 파트의 3D 점군이 들어오고 있으면 바로 바꿔 볼 수 있게 한다.
    const why = !ui.wsOpen ? "게이트웨이에 연결되지 않았습니다 · 연결되면 자율 비행이 보낸 지도를 그립니다"
      : s.connected ? "자율 비행이 연결됐습니다 · 위치가 오면 바로 그립니다"
      : "자율 비행이 아직 연결되지 않았습니다 · 연결되면 드론 위치·지나온 길·계획 경로·찾은 사람을 1 m 눈금 위에 그립니다";
    const to3d = !tall && ui.wsOpen && st.map?.alive && st.map.points > 0 ? `<button data-map-mode="3d">3D 점군 보기</button>` : "";
    return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
      <div class="map-empty"><span>${esc(why)}</span>${to3d}</div>${mapSaveBar()}</section>`;
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
  const marks = !shown("people") ? "" : found.map((q, i) => person(q, i + 1, q.gone ? "gone" : "found")).join("")
    + unsure.map((q, i) => person(q, found.length + i + 1, "unsure")).join("");
  // 게이트웨이와 끊기거나 위치가 늦으면 드론을 흐리게 그린다(마지막 위치).
  const drone = pose ? `<g class="drone ${pose.valid && ui.wsOpen ? "" : "stale"}" transform="translate(${X(pose.x_m)},${Y(pose.y_m)}) rotate(${(90 - pose.yaw_rad * 180 / Math.PI).toFixed(1)})"><path d="M0,-11 L8,7 L0,3 L-8,7 Z"/></g>` : "";
  const start = `<g><title>출발: 자율 비행 지도의 원점(켠 자리)</title><circle class="start" cx="${X(0)}" cy="${Y(0)}" r="7"/><text class="start-label" x="${+X(0) + 12}" y="${+Y(0) + 4}">출발</text></g>`;
  const scale = `<g class="scale"><line x1="12" y1="${H - 12}" x2="${(12 + k).toFixed(1)}" y2="${H - 12}"/><text x="${(16 + k).toFixed(1)}" y="${H - 8}">1 m</text></g>`;
  // 범례: 고르기 버튼 셋에, 사람 표시가 켜져 있으면 지도에 있는 것만(다시 봤는데 없음·미확정) 덧붙인다.
  // 임무 진행은 모드 카드에, 자율 연결은 아래 칩에 있어 범례에 다시 적지 않는다.
  const n = found.filter((q) => !q.gone).length, gone = found.length - n;
  const keys = !shown("people") ? "" : `${gone ? `<span class="key"><i class="lg mark gone"></i>다시 봤는데 없음 ${gone}</span>` : ""}${unsure.length ? `<span class="key"><i class="lg mark unsure"></i>미확정 ${unsure.length}</span>` : ""}`;
  const layers = layerButton("flown", "지나온 길", "드론이 지나온 길(화면이 받은 위치를 이어 그림)")
    + layerButton("planned", "계획 경로", "자율 비행이 가려는 길")
    + layerButton("people", n ? `사람 ${n}명` : "사람", "자율 비행이 지도에 찍은 사람 위치. 번호는 찾은 사람, 점선은 다시 봤는데 없음, 흐린 점은 미확정");
  const status = autonomyStatus(st);
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="자율 지도: 드론 위치, 지나온 길, 계획 경로, 찾은 사람(1칸 1 m, 출발이 원점)">
      <g class="grid">${grid.join("")}</g>${shown("planned") ? line(path, "planned") : ""}${shown("flown") ? line(a.track, "flown") : ""}${start}${marks}${drone}${scale}
    </svg>
    ${mapFoot(status, status[2], layers + keys)}${mapSaveBar()}
  </section>`;
}

// 평면도(2D): 자율 비행 지도가 없으면 2D 지도 칸에 받은 3D 점군을 위에서 본 그림을 그린다(floorplan.js).
// 좌표는 지도 파트 것이다: 지도를 시작한 카메라 자리가 원점, 그때 본 쪽(앞)이 오른쪽·왼쪽이 위(자율 지도와 같은 방향).
// 자율 지도와는 원점·방향을 맞춘 적이 없어 한 그림에 겹치지 않는다(자율 비행이 보내기 시작하면 자율 지도를 그린다).
// 벽 그림은 흰 투명도 그림을 mask로 써서 CSS 토큰(--map-plan)으로 칠한다. 테마가 바뀌어도 그림을 다시 만들지 않는다.
function floorPlan() {
  const m = ui.map3d;
  if (!m.data || !window.FloorPlan) return null;   // floorplan.js를 못 읽었으면(옛 캐시 등) 예전처럼 빈 칸 안내
  if (m.planOf !== m.data) {   // 지도 한 버전에 한 번만 만든다
    m.planOf = m.data;
    try { m.plan = window.FloorPlan.build(m.data); } catch (e) { m.plan = null; }
  }
  return m.plan?.png ? m.plan : null;
}

function floorPlanPanel(st, head, plan, tall) {
  // 상태는 3D 점군과 같다. 게이트웨이와 끊기거나 지도가 멈춰도 받은 그림은 두고 칩과 마지막 자리(흐리게)로 알린다.
  const info = !ui.wsOpen ? ["게이트웨이 끊김", "neutral", "마지막으로 받은 지도입니다 · 다시 연결되면 이어집니다"] : mapInfo(st.map, true);
  const fit = (lo, hi) => { const d = Math.max(plan.metric ? 3 : 0, hi - lo), c = (lo + hi) / 2; return [c - d / 2, c + d / 2]; };
  const [minX, maxX] = fit(plan.a0, plan.a1), [minY, maxY] = fit(plan.b0, plan.b1);
  const W = 800, H = tall ? 360 : 260, k = Math.min(W / (maxX - minX), H / (maxY - minY));
  const X = (x) => ((x - minX) * k + (W - (maxX - minX) * k) / 2).toFixed(1);
  const Y = (y) => (H - (y - minY) * k - (H - (maxY - minY) * k) / 2).toFixed(1);
  const grid = [];
  if (plan.metric) {   // m 단위가 아니면 눈금을 그리지 않는다(1칸이 1 m라고 말할 수 없다)
    for (let gx = Math.ceil(minX); gx <= maxX; gx++) grid.push(`<line x1="${X(gx)}" y1="0" x2="${X(gx)}" y2="${H}"/>`);
    for (let gy = Math.ceil(minY); gy <= maxY; gy++) grid.push(`<line x1="0" y1="${Y(gy)}" x2="${W}" y2="${Y(gy)}"/>`);
  }
  const box = `x="${X(plan.a0)}" y="${Y(plan.b1)}" width="${((plan.a1 - plan.a0) * k).toFixed(1)}" height="${((plan.b1 - plan.b0) * k).toFixed(1)}"`;
  const walls = `<mask id="floor-plan-mask" maskUnits="userSpaceOnUse" x="0" y="0" width="${W}" height="${H}" style="mask-type:alpha">
      <image href="${plan.png}" ${box} preserveAspectRatio="none"/></mask><rect class="floor" ${box} mask="url(#floor-plan-mask)"/>`;
  // 카메라 경로는 화면에서 1.5px보다 가까운 점은 건너뛴다(긴 비행이면 카메라 자리가 수천 개).
  const route = [];
  for (const p of plan.path) {
    const q = [X(p[0]), Y(p[1])], last = route[route.length - 1];
    if (!last || Math.hypot(q[0] - last[0], q[1] - last[1]) > 1.5) route.push(q);
  }
  const flown = shown("flown") && route.length > 1 ? `<polyline class="flown" points="${route.map((q) => q.join(",")).join(" ")}"/>` : "";
  const end = plan.path[plan.path.length - 1], fresh = ui.wsOpen && info[1] === "success";
  const now = end ? `<g class="cam-now ${fresh ? "" : "stale"}"><title>${fresh ? "지도 파트가 마지막으로 추정한 카메라(드론) 자리" : "마지막으로 받은 카메라(드론) 자리 · 지금은 새 지도가 오지 않습니다"}</title><circle cx="${X(end[0])}" cy="${Y(end[1])}" r="7"/></g>` : "";
  // 출발 글자는 맨 위에 그린다(드론이 출발 근처에 있으면 점이 글자를 가리므로).
  const startTip = "<title>지도 시작: 지도 파트 좌표의 원점(지도를 시작한 카메라 자리)</title>";
  const start = `<g>${startTip}<circle class="start" cx="${X(0)}" cy="${Y(0)}" r="7"/></g>`;
  const startLabel = `<g>${startTip}<text class="start-label" x="${+X(0) + 12}" y="${+Y(0) + 4}">지도 시작</text></g>`;
  const scale = plan.metric ? `<g class="scale"><line x1="12" y1="${H - 12}" x2="${(12 + k).toFixed(1)}" y2="${H - 12}"/><text x="${(16 + k).toFixed(1)}" y="${H - 8}">1 m</text></g>` : "";
  const what = !plan.band ? "모든 점" : plan.metric ? `드론 높이 ±${plan.half} m의 점` : "드론 높이 근처의 점";
  const detail = ["위에서 본 3D 점군", what, plan.metric ? "1칸 1 m" : "실제 크기 모름", info[2]].filter(Boolean).join(" · ");
  const keys = `<span class="key" title="드론 높이 근처에 점이 많이 쌓인 곳일수록 진합니다"><i class="lg floor"></i>벽·물체</span>`;
  const layers = layerButton("flown", "지나온 길", "지도 파트가 추정한 카메라 경로. 마지막 자리(큰 점)는 늘 보입니다");
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도(위에서 본 3D 점군)">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="위에서 본 3D 점군: 드론 높이 근처의 점(벽·물체), 지나온 길, 지도 시작 자리${plan.metric ? "(1칸 1 m)" : "(실제 크기 모름)"}">
      <g class="grid">${grid.join("")}</g>${walls}${flown}${start}${now}${startLabel}${scale}
    </svg>
    ${mapFoot(info, detail, keys + layers)}${mapSaveBar()}
  </section>`;
}

// 3D 점군: 캔버스(ui.map3d.host)는 한 번만 만들고 다시 그릴 때마다 이 자리(.map3d-slot)로 옮긴다(attachMap3d).
// 화면 전체를 0.1초마다 다시 그리므로, 캔버스를 HTML로 다시 만들면 점군을 매번 처음부터 올리게 된다.
// 지도 칸 상태 [이름, 색, 설명]: 연결기 → 지도 서버 → 지도 세션 → 영상 입력 순으로 본다.
// 세션이 끝나거나 영상이 끊겨도 받은 점군은 그대로 보여 주고(has) 상태만 바꾼다.
// 지도 콘솔은 영상이 오래 끊겨 세션을 끝낸 뒤에도 입력 상태를 reconnecting으로 두므로, 세션이 끝났는지를 먼저 본다.
// 설명은 무엇이 꺼졌는지·무엇을 기다리는지와 할 일을 쉬운 말로 적는다(프로그램 파일 이름 대신).
function mapInfo(m, has) {
  const ms = m?.mapping_state, lost = ["reconnecting", "failed"].includes(m?.input_state);
  const videoLost = ["영상 끊겨 지도 끝남", "warning", "드론 영상이 오래 끊겨 지도 콘솔이 지도를 끝냈습니다"];
  if (!m || !m.alive) return ["지도 연결기 꺼짐", "warning", "지도 연결기가 꺼져 점군이 들어오지 않습니다 · 실행기로 켜면 함께 켜집니다"];
  if (m.upstream === "unreachable") {
    let at = "";
    try { at = m.source_url ? `(${new URL(m.source_url).host})` : ""; } catch (e) { /* 주소를 못 읽으면 빼고 적는다 */ }
    return ["지도 서버 꺼짐", "danger", `지도 서버${at}에 연결할 수 없습니다 · 지도 서버가 켜져 있는지 확인하세요`];
  }
  if (m.upstream === "error") return ["지도 읽기 오류", "danger", "지도 서버가 오류를 돌려줬습니다 · 지도 콘솔에서 확인하세요"];
  if (ms === "failed") return lost ? videoLost : ["지도 오류로 끝남", "danger", "까닭은 지도 콘솔에서 볼 수 있습니다"];
  if (ms === "stopping") return ["지도 마무리 중", "neutral", "지도 콘솔이 마지막 지도를 정리하는 중입니다"];
  if (["complete", "completed", "stopped", "cancelled"].includes(ms)) return lost ? videoLost : ["지도 끝남", "neutral", "지도를 끝냈습니다"];
  if (m.input_state === "reconnecting") {   // 지도 콘솔이 드론 영상을 다시 기다리는 중(곧 돌아오면 같은 세션)
    return ["영상 기다리는 중", "warning", `드론 영상이${m.input_age_s != null ? ` ${Math.round(m.input_age_s)}초째` : ""} 오지 않습니다 · 곧 돌아오면 같은 지도로 이어집니다`];
  }
  if (m.upstream === "no_map") {
    if (ms && !["idle", "offline"].includes(ms)) return ["지도 시작 중", "neutral", "첫 점군을 기다리는 중입니다"];
    return has ? ["지도 멈춤", "neutral", "진행 중인 지도가 없습니다"] : ["지도 아직 없음", "neutral", "아직 시작한 지도가 없습니다 · 영상을 켜면 지도가 시작됩니다"];
  }
  // 연결기는 지도 서버와 잘 붙어 있다고 하지만 게이트웨이에 점군이 한 번도 오지 않았을 때(예: 게이트웨이만 다시 켬)
  if (!m.points) return ["점군 기다리는 중", "neutral", "지도 서버와는 연결됐지만 아직 점군이 오지 않았습니다"];
  return ["지도 수신 중", "success", ""];
}

function mapSaveBar() {
  if (!ui.live || !liveSave || view() !== 'live') return '';
  const state = liveSave.state();
  return `<footer class="live-map-save"><button type="button" data-live-map-save ${state.busy || !state.available ? 'disabled' : ''}>${state.busy ? '저장 중…' : '지도 저장'}</button>
    <span role="status">${esc(state.message)}${state.path ? `<small title="${esc(state.path)}">${esc(state.path)}</small>` : ''}</span></footer>`;
}

function map3dPanel(st, head, cls = "map") {   // cls: 실시간 감독은 지도 칸(map), 탐색 결과는 3D 칸
  const m = st.map, has = ui.map3d.has;
  // 게이트웨이와 끊기면 마지막 상태("지도 수신 중" 등) 대신 끊겼다고 적는다. 받은 점군은 그대로 둔다.
  const info = !ui.wsOpen ? ["게이트웨이 끊김", "neutral", has ? "마지막으로 받은 점군입니다 · 다시 연결되면 이어집니다"
    : "게이트웨이에 연결되지 않았습니다 · 연결되면 점군이 들어옵니다"] : mapInfo(m, has);
  const count = (v) => Number(v).toLocaleString("ko-KR");
  const size = has && m ? `점 ${count(m.points)}개${m.points_total > m.points ? `(전체 ${count(m.points_total)}개 중)` : ""} · ${m.unit === "m" ? "m 단위" : "실제 크기 모름(m 아님)"}` : "";
  const detail = has ? [size, info[2]].filter(Boolean).join(" · ") : "";   // 점군이 없으면 설명은 빈 칸 안에 적는다
  // 조작 안내는 지도에 마우스를 올리면 늘 보이고, 아래 줄에는 칸이 넉넉할 때만 적는다(낮은 칸은 그림을 지키려고 CSS가 숨김).
  const keys = "왼쪽 드래그 회전 · 우클릭 드래그 이동 · 휠/두 손가락 확대 · 두 번 눌러 처음 시점";
  const extra = has ? `<span class="muted keys-hint">${keys}</span>` : "";
  const layers = has ? layerButton("flown", "지나온 길", "지도 파트가 추정한 카메라 경로. 지금 위치(큰 점)는 늘 보입니다") : "";
  return `<section class="panel ${cls}" data-part="map3d" aria-label="3D 지도">${head}
    <div class="map3d-slot"${has ? ` title="${keys}"` : ""}>${has ? "" : `<div class="map-empty">${esc(info[2] || "점군을 기다리는 중입니다.")}</div>`}</div>
    ${mapFoot(info, detail, layers, extra)}${mapSaveBar()}</section>`;
}

function attachMap3d() {
  if (!ui.live) return;
  const m = ui.map3d, slot = $("#app .map3d-slot");
  // 점군은 3D 칸이나 2D 지도 칸(자율 지도가 없으면 평면도를 그린다)이 화면에 있을 때 새 버전을 읽는다.
  const rev = ui.state?.map?.revision;
  if ((slot || $('#app [data-part="map"]')) && rev != null && rev !== m.revision && !m.loading) loadMap();
  if (!slot) return;
  if (!m.host) {
    m.host = document.createElement("div");
    m.host.className = "map3d-host";
    m.host.innerHTML = `<canvas aria-label="3D 점군과 카메라 경로"></canvas>`;
    m.view = window.PointView ? new window.PointView(m.host.firstChild) : null;   // map3d.js를 못 읽었으면(옛 캐시 등) 안내만
    if (!m.view?.ok) m.host.innerHTML = `<div class="map-empty">이 브라우저는 WebGL을 쓸 수 없어 3D 점군을 그리지 못합니다.</div>`;
    else if (m.data) m.view.setData(m.data);   // 2D 지도 칸(평면도)에서 먼저 받아 둔 점군
  }
  m.view?.setPathVisible?.(shown("flown"));   // 지도 칸 "지나온 길" 고르기(옛 map3d.js면 건너뜀)
  if (m.has && m.host.parentNode !== slot) { slot.replaceChildren(m.host); m.view?.draw(); }
}

async function loadMap() {
  const m = ui.map3d;
  m.loading = true;
  try {
    const r = await fetch("/api/map", { cache: "no-store" });
    if (r.ok) {
      const map = await r.json();
      m.revision = map.source?.revision;
      m.data = map;   // 평면도(2D 지도 칸)와 나중에 만드는 3D 보기도 쓴다
      if (m.view?.ok) m.view.setData(map);
      const changed = !m.has;
      m.has = true;
      if (changed) ui.cache = {};
      render();   // 새 버전이면 평면도도 바로 다시 그린다
    }
  } catch (e) { /* 다음 상태 갱신 때 다시 */ }
  m.loading = false;
}

// 지도 콘솔 화면(#/map): 지도 복원·편집 탭을 연다.
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

function placeConsole() {
  const c = ui.console;
  const slot = view() === "map" ? $("#app .console-slot") : null;
  const t = slot ? consoleTarget(ui.state) : null;
  const up = ui.state?.map?.alive ? ui.state.map.upstream : null;
  if (c.frame && t?.url && c.loadedUp === "unreachable" && up && up !== "unreachable") c.url = null;
  if (!c.frame && t?.url) {   // 콘솔 보기를 처음 고를 때 연다
    c.frame = document.createElement("iframe");
    c.frame.className = "console-frame away";
    c.frame.title = "지도 콘솔";
    c.frame.allow = "fullscreen";
    c.frame.addEventListener("load", () => { c.viewportHeight = 0; placeConsole(); });
    document.body.append(c.frame);
  }
  if (!c.frame) return;
  if (t?.url && c.url !== t.url) {
    c.height = c.viewportHeight = 0;
    c.url = t.url; c.frame.src = t.url; c.loadedUp = up;
  }
  if (c.slot !== slot) {   // 칸이 다시 그려지면 새 자리의 크기 변화를 따라간다
    c.ro?.disconnect();
    c.slot = slot;
    c.ro = slot && window.ResizeObserver ? new ResizeObserver(() => placeConsole()) : null;
    c.ro?.observe(slot);
  }
  if (!slot || !t?.url) return c.frame.classList.add("away");   // 숨기기만 한다(콘솔은 계속 살아 있음)
  const viewportHeight = Math.max(1, Math.floor(innerHeight - (slot.getBoundingClientRect().top + scrollY)));
  slot.style.minHeight = `${Math.max(viewportHeight, c.height)}px`;
  const r = slot.getBoundingClientRect();
  Object.assign(c.frame.style, { left: `${r.left + scrollX}px`, top: `${r.top + scrollY}px`, width: `${r.width}px`, height: `${r.height}px` });
  c.frame.classList.remove("away");
  if (viewportHeight !== c.viewportHeight) {
    c.viewportHeight = viewportHeight;
    c.frame.contentWindow.postMessage({ type: "mapping-console-viewport", height: viewportHeight }, t.origin);
  }
}

// The parent page scrolls the header and iframe together.
window.addEventListener("message", event => {
  const c = ui.console;
  if (!c.url || event.source !== c.frame?.contentWindow || event.origin !== new URL(c.url).origin) return;
  const { type, height } = event.data || {};
  if (type !== "mapping-console-height" || !Number.isSafeInteger(height) || height <= 0 || height === c.height) return;
  c.height = height;
  placeConsole();
});

const CMD_LABEL = {
  takeoff: "이륙", land: "착륙", pause: "일시정지", resume: "다시 시작", takeover: "넘겨받기",
  auto_start: "자율 시작", auto_stop: "자율 종료", return_home: "복귀", confirm_area: "주변 확인", emergency: "비상 정지",
  video_on: "영상 켜기", video_off: "영상 끄기",
};

function areaConfirmed(st) {
  return !!(st.preflight || []).find((c) => c.key === "operator" && c.ok);
}

// 기체 점검(게이트웨이 preflight). 막는 항목은 ✓/✗, 알리기만 하는 항목(온도)은 ✓/!. 기준값은 게이트웨이가 준 제목을 그대로 쓴다.
// 주변 확인(operator)은 여기서 빼고 이륙 전 확인 칸의 체크로 따로 한다.
function preflightChips(st) {
  const checks = (st.preflight || []).filter((c) => c.key !== "operator");
  if (!checks.length) return chip(ui.live ? "점검 결과 없음" : "샘플: 점검 결과 없음");
  return checks.map((c) => (c.ok ? chip(`✓ ${c.title}`, "success")
    : c.blocking === false ? chip(`! ${c.detail || c.title}`, "warning") : chip(`✗ ${c.title}`, "danger"))).join("");
}

// ---------- 이륙: 버튼 하나 → 이륙 전 확인 칸(기체 점검 → 사람이 주변 확인 체크 → 이륙) ----------
// 칸을 열 때마다 주변 확인을 새로 한다(전에 해 둔 확인은 푼다). 취소하거나 창을 떠나거나 조종 입력을 초기화하면 칸을 닫고 확인을 푼다.
// 게이트웨이 규칙은 그대로다: 이륙은 점검(주변 확인 포함)을 통과해야 받고, 지도 콘솔 수동 조종 모드는 확인 뒤 30초 안·영상 켜짐이어야 한다.
const AREA_CONFIRM_MS = 30000;   // 게이트웨이 AREA_CONFIRM_S(지도 콘솔 수동 조종 모드)와 같게
const onGround = (st) => st.mode === "grounded" && st.flight_busy !== "takeoff";
const areaChecked = (st) => (ui.live ? areaConfirmed(st) : ui.takeoff.sampleChecked);

function takeoffBlocker(st) {   // 지금 이륙할 수 없는 까닭. 이륙할 수 있으면 null
  if (ui.live && !ui.wsOpen) return "게이트웨이에 연결되지 않았습니다";
  if (ui.live && !st.connected) return "드론 상태가 오지 않습니다";
  if (!onGround(st) || !st.allowed.includes("takeoff")) return "지금 상태에서는 이륙할 수 없습니다";
  const failed = (st.preflight || []).filter((c) => c.key !== "operator" && !c.ok && c.blocking !== false);
  if (failed.length) return `기체 점검을 통과하지 못했습니다: ${failed.map((c) => c.title).join(", ")}`;
  if (st.mapping_console_url && !st.video?.available) return "이륙 전에 영상을 켜세요";
  if (!areaChecked(st)) return "주변을 확인하고 체크하세요";
  return null;
}

// inline: 폰(현장 조작)은 큰 버튼 위에 그대로, PC는 조종 줄 위 오른쪽에 뜬다.
function takeoffPanel(st, inline = false) {
  const wrap = (html) => (inline ? `<div data-part="takeoff">${html}</div>` : html);
  if (!ui.takeoff.open || !onGround(st)) return wrap("");
  const checked = areaChecked(st), why = takeoffBlocker(st);
  const video = !st.mapping_console_url ? "" : `<div class="to-row"><span class="to-name">영상</span>${st.video?.available
    ? chip("✓ 영상 켜짐", "success") : `${chip("✗ 영상 꺼짐", "danger")}<button data-cmd="video_on" class="mini">영상 켜기</button>`}</div>`;
  const deadline = st.mapping_console_url && checked && ui.takeoff.confirmedAt ? ui.takeoff.confirmedAt + AREA_CONFIRM_MS : null;
  const hint = why || (ui.live ? "이륙 버튼이나 T를 누르면 이륙합니다" : "샘플 화면: 이륙 명령은 보내지 않습니다");
  return wrap(`<section class="takeoff-pop ${inline ? "inline" : ""}" role="dialog" aria-label="이륙 전 확인">
    <header><h2>이륙 전 확인</h2><span class="spacer"></span><button data-takeoff="cancel">취소</button></header>
    <div class="to-row"><span class="to-name">기체 점검</span><span class="checks">${preflightChips(st)}</span></div>
    ${video}
    <button type="button" class="area-check" role="checkbox" aria-checked="${checked}" data-takeoff="area">주변에 사람·장애물이 없고, 프로펠러 가드와 안전 담당을 확인했습니다</button>
    <div class="to-go"><span class="muted">${esc(hint)}${deadline ? `<span data-countdown="${deadline}"></span>` : ""}</span>
      <button class="primary ${why ? "off" : ""}" data-takeoff="go" ${why ? 'aria-disabled="true"' : ""}>이륙</button></div>
  </section>`);
}

function controlBar(st) {
  const can = (n) => st.allowed.includes(n);
  const btn = (n, extra = "", attrs = "") => `<button data-cmd="${n}" class="${can(n) ? "" : "off"} ${extra}" ${can(n) ? "" : 'aria-disabled="true"'} ${attrs}>${CMD_LABEL[n]}</button>`;
  const manualOn = st.mode === "manual", autoOn = st.mode === "auto";
  const autoCmd = st.mode === "paused" && can("resume") ? "resume" : "auto_start";
  const grounded = onGround(st);
  // 왼쪽: 모드 바꾸기, 오른쪽 끝: 이륙/착륙과 비상 정지(늘 같은 자리). 이륙을 누르면 그 위에 이륙 전 확인 칸이 뜬다.
  return `<section class="controls" data-part="controls" aria-label="조종">
    ${takeoffPanel(st)}
    <div class="seg" role="group" aria-label="수동 자율 전환">
      <button data-cmd="takeover" class="${manualOn ? "on" : ""} ${can("takeover") || manualOn ? "" : "off"}" aria-pressed="${manualOn}">수동</button>
      <button data-cmd="${autoCmd}" class="${autoOn ? "on" : ""} ${can(autoCmd) || autoOn ? "" : "off"}" aria-pressed="${autoOn}">자율</button>
    </div>
    ${st.mode === "paused" ? btn("resume") : btn("pause")}
    ${btn("return_home", "soon")}
    <span class="spacer"></span>
    ${ui.live && st.mapping_console_url ? `<button data-keyboard-focus aria-pressed="${keyboardEnabled}">${keyboardEnabled ? "키보드 조종 활성" : "키보드 조종 활성화"}</button>` : ""}
    ${ui.live ? '<span class="muted keys-hint">키보드: WASD 이동 · ↑↓ 높이 · ←→ 회전 · T 이륙 · L 착륙 · Esc 비상 정지</span>' : ""}
    <span class="safety">${grounded ? btn("takeoff", "", `aria-haspopup="dialog" aria-expanded="${ui.takeoff.open}"`) : btn("land")}
      <button class="estop" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>길게</small></button></span>
  </section>`;
}

function detailItems(st) {
  const recv = ui.live && !ui.wsOpen ? "게이트웨이 끊김"
    : st.link.state_hz != null ? `${st.link.state_hz}Hz · ${st.link.rtt_ms}ms`
    : st.state_age_s != null ? `${Math.round(st.state_age_s * 1000)}ms 전` : "없음";
  return [   // 높이(하방)는 상태 카드에 있어 뺀다
    ["온도", st.temp_high_c == null ? "없음" : `${st.temp_high_c}°C`],
    ["전방", st.front_tof.valid ? meters(st.front_tof.m) : "없음"],
    ["상태 수신", recv],
    ["개입", `${st.operator_interventions ?? 0}회`],
    ["임무", st.mission ? PHASE[st.mission.phase] || st.mission.phase || "없음" : "없음"],   // 자율 비행 내부 단계(state)는 코드라 적지 않는다
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
      ${ui.live ? map3dPanel(st, `<header><h2>3D 지도</h2>${src("map")}</header>`, "map3d-panel") : `<section class="panel map3d-panel" aria-label="3D 지도"><header><h2>3D 지도</h2>${src("map")}</header>
        <div class="map3d-slot"><div class="map-empty"><span>샘플 화면에는 3D 점군이 없습니다 · 게이트웨이에 연결하고 지도를 켜면 지도 파트의 점군과 카메라 경로가 여기에 그려집니다</span><a class="link" href="#/map">지도 콘솔 열기</a></div></div></section>`}
      ${timelinePanel(track)}
    </div>`;
}

// 비행 기록: 모드 바뀜·자율 연결·발견·거절·게이트웨이 끊김. 실시간은 게이트웨이 이벤트를 모은 것(최근 200개), 샘플은 지금 시각까지.
// 까닭은 코드 대신 쉬운 말(WHY)로 적고, 이어진 사람 발견은 한 줄로 묶는다(발견 하나하나는 발견 칸에 있다).
function logLine(entry) {
  ui.timeline.unshift(entry);
  ui.timeline.length = Math.min(ui.timeline.length, 200);
}

const findingsText = (g) => (g.count === 1 ? `사람 발견 ${g.first}번 · 신뢰도 ${g.best.toFixed(2)}`
  : `사람 발견 ${g.count}건 · ${g.first}~${g.last}번 · 가장 높은 신뢰도 ${g.best.toFixed(2)}`);

function timelinePanel(track) {
  const li = (kind, time, text) => `<li class="${kind}"><time>${esc(time)}</time><span>${esc(text)}</span></li>`;
  const tl = ui.live
    ? ui.timeline.map((x) => li(x.kind, x.time, x.group ? findingsText(x.group) : x.text)).join("")
    : S.timeline.filter((x) => x.t_s <= (track[track.length - 1]?.t_s ?? 0)).map((x) => li(x.kind, clock(x.t_s), x.text)).join("");
  const empty = !ui.live ? "기록 없음"
    : !ui.wsOpen ? "게이트웨이에 연결되지 않았습니다 · 연결되면 이륙·모드 바뀜·발견·거절이 시각과 함께 쌓입니다"
    : "아직 기록이 없습니다 · 이륙·모드 바뀜·발견·거절이 시각과 함께 쌓입니다";
  return `<section class="panel timeline-panel" data-part="timeline" aria-label="비행 기록"><header><h2>비행 기록</h2>
    <span class="src" title="이 칸을 채우는 파트">드론 · 자율 비행 · 사람 탐지</span></header>
    <ol class="timeline">${tl || `<li class="empty">${esc(empty)}</li>`}</ol></section>`;
}

function renderControl() {
  const { sc, st, events } = current();
  const latest = events[0];
  const can = (n) => st.allowed.includes(n);
  const big = (n, label) => `<button class="big ${can(n) ? "" : "off"}" data-cmd="${n}">${label}</button>`;
  const grounded = onGround(st);
  return `<div class="phone">
    ${statusStrip(st, events, true)}
    ${alertBox(sc, events)}
    <div data-part="latest">${latest ? `<div class="alert danger slim"><b>최근 발견</b><span>사람 ${latest.confidence.toFixed(2)} · ${findingTime(latest)}</span></div>` : ""}</div>
    <p class="note">현장에서 드론 옆에 두는 화면입니다. 자율 시작은 PC 감독 화면에서만 할 수 있습니다.</p>
    ${takeoffPanel(st, true)}
    <div class="big-grid" data-part="big">
      ${st.mode === "paused" ? big("resume", "다시 시작") : big("pause", "일시정지")}
      ${big("return_home", "복귀")}
      ${grounded ? big("takeoff", "이륙") : big("land", "착륙")}
      ${big("takeover", "넘겨받기")}
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

// 머리줄 연결 표시: 어디에 붙었는지(실기체·가짜 Tello·기록 재생)와 게이트웨이 연결(연결됨·연결 중·끊김).
const SOURCE_NAME = { tello: "실기체", fake: "가짜 Tello", replay: "기록 재생" };
function syncHeader() {
  const badge = $("#conn");
  if (!ui.live || !badge) return;
  const [word, tone] = ui.wsOpen ? ["연결됨", "success"] : ui.everOpen ? ["끊김", "danger"] : ["연결 중", "warning"];
  const text = `${SOURCE_NAME[ui.state?.source] || "실기체"} · ${word}`;
  if (badge.dataset.text === text) return;
  badge.dataset.text = text;
  badge.innerHTML = `<i class="dot ${tone}" aria-hidden="true"></i>${esc(text)}`;
}

// 다시 그려도 초점을 지킬 버튼: 이륙 전 확인 칸(체크한 뒤 바로 이륙 버튼·T로), 영상·지도 보기 고르기.
// 칸을 다시 그리면 버튼이 새로 만들어지므로, 같은 data 값을 가진 새 버튼(같은 값이 여럿이면 같은 순서, 없으면 마지막)으로 초점을 옮긴다.
const KEEP_FOCUS = ["takeoff", "feed", "outline", "mapMode", "mapLayer", "mapBig"];

function focusToKeep() {
  if (ui.takeoff.focus) return { sel: `[data-takeoff="${ui.takeoff.focus}"]`, i: 0 };
  const el = document.activeElement, key = el?.dataset && KEEP_FOCUS.find((k) => k in el.dataset);
  if (!key) return null;
  const sel = `[data-${key.replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`)}="${el.dataset[key]}"]`;
  return { sel, i: Math.max(0, [...document.querySelectorAll(sel)].indexOf(el)) };
}

function render() {
  liveSave?.setTarget(ui.live ? consoleTarget(ui.state).origin || null : null);
  syncHeader();
  const v = view();
  const html = { live: renderLive, map: renderMap, review: renderReview, control: renderControl }[v]();
  const app = $("#app");
  const tpl = document.createElement("template");
  tpl.innerHTML = html;
  const parts = [...tpl.content.querySelectorAll("[data-part]")];
  const layout = tpl.content.querySelector("[data-layout]")?.dataset.layout ?? "";
  const keys = `${parts.map((p) => p.dataset.part).join(",")}|${layout}`;
  const keep = focusToKeep();
  ui.takeoff.focus = null;
  const refocus = () => {
    const all = keep ? document.querySelectorAll(keep.sel) : [];
    const el = all[Math.min(keep?.i ?? 0, all.length - 1)];   // 누른 버튼이 사라졌으면(빈 칸의 "3D 점군 보기") 같은 값의 남은 버튼으로
    if (el && document.activeElement !== el) el.focus({ preventScroll: true });
  };
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
    refocus();
    return placeConsole();
  }
  for (const p of parts) {
    const k = p.dataset.part, s = p.outerHTML;
    if (ui.cache[k] === s) continue;
    ui.cache[k] = s;
    const el = app.querySelector(`[data-part="${k}"]`);
    if (el) el.replaceWith(document.importNode(p, true));
  }
  refocus();
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
    if (ui.takeoff.open && !onGround(msg)) closeTakeoff(true);   // 이륙했거나(확인은 게이트웨이가 이미 씀) 땅이 아니다
    ui.state = msg;
    syncAutonomy(msg);
  } else if (msg.type === "result" && !msg.ok) {
    const failed = (msg.failed || []).map((k) => CHECK[k] || k).join(", ");
    toast(`${REASON[msg.reason] || msg.reason}${failed ? `: ${failed}` : ""}`);
  } else if (msg.type === "detection") {
    onDetection(msg);
  } else if (msg.type === "detection_review") {
    onReview(msg);
  } else if (msg.type === "event") {
    const d = msg.detail || {};
    if (msg.kind === "cmd_ignored") return render();   // 자율이 auto가 아닐 때 보낸 이동 명령(1초에 한 번까지). 기록에는 남기지 않는다
    const text = msg.kind === "mode_changed" ? `${(MODE[d.from] || [d.from])[0]} → ${(MODE[d.to] || [d.to])[0]} · ${reasonText(d.reason)}`
      : msg.kind === "safety_landing" ? `안전 착륙: ${reasonText(d.reason)}`
      : msg.kind === "video_changed" ? `영상 ${d.on ? "켜짐" : "꺼짐"} (${VIDEO_REASON[d.reason] || d.reason})`
      : msg.kind === "autonomy_connected" ? `자율 연결 (${d.ground_origin ? "지상에서" : "공중에서 붙어 자율 시작 불가"})`
      : msg.kind === "autonomy_disconnected" ? "자율 연결 끊김"
      : msg.kind === "mission_finished" ? `임무 끝: ${d.phase === "arrived" ? "도착" : "실패"}${d.reason ? ` · ${reasonText(d.reason)}` : ""}`
      : msg.kind === "command_rejected" ? `${CMD_LABEL[d.name] || d.name} 거절: ${REASON[d.reason] || d.reason}${d.detail ? ` · ${reasonText(d.detail)}` : ""}` : msg.kind;
    const kind = msg.kind === "mode_changed" || msg.kind.startsWith("autonomy") || msg.kind === "mission_finished" ? "mode"
      : msg.kind === "video_changed" ? "video" : "intervention";
    logLine({ time: hhmmss(msg.at), kind, text });
    if (msg.kind === "safety_landing") toast(`안전 착륙: ${reasonText(d.reason)}`);
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
    const n = findingNumberOf(f.finding_id), top = ui.timeline[0], conf = f.best.confidence;
    if (top?.group) {   // 바로 앞 줄도 사람 발견이면 그 줄에 묶는다(시각은 마지막 발견)
      Object.assign(top.group, { last: n, count: top.group.count + 1, best: Math.max(top.group.best, conf) });
      top.time = hhmmss(f.first_seen);
    } else {
      logLine({ time: hhmmss(f.first_seen), kind: "detection", group: { first: n, last: n, count: 1, best: conf } });
    }
  }
}

// 확인·오탐은 게이트웨이가 발견마다 저장하고 모든 화면에 알린다(이 화면에서 누른 것도 이 메시지로 기록에 남는다).
const REVIEW_WORD = { confirmed: "확인", false_positive: "오탐으로 표시", unreviewed: "미확인으로 되돌림" };
function onReview(msg) {
  const f = ui.findings.get(msg.finding_id);
  if (!f) return;   // 아직 받지 못한 발견: 다시 연결할 때 목록(GET /api/detections)으로 함께 받는다
  Object.assign(f, { review: msg.review, review_at: msg.at });
  logLine({ time: hhmmss(msg.at), kind: "review", text: `발견 ${findingNumberOf(msg.finding_id)}번 ${REVIEW_WORD[msg.review] || msg.review}` });
}

// 확인·오탐 보내기. 누르면 바로 바뀌어 보이고(보내는 중), 게이트웨이가 저장하면 그 값으로, 못 하면 되돌리고 알린다.
// 같은 버튼을 다시 누르면 미확인으로 되돌린다. 기록 재생은 기록된 확인·오탐만 보여 주므로 바꿀 수 없다.
async function sendReview(id, pick) {
  const f = ui.findings.get(id);
  if (!f) return;
  if (!ui.wsOpen) return toast("게이트웨이에 연결되지 않았습니다");
  if (ui.state?.replay) return toast("기록 재생 중에는 확인·오탐을 바꿀 수 없습니다 · 기록된 확인·오탐을 보여 줍니다");
  const review = (ui.reviewPending[id] || f.review || "unreviewed") === pick ? "unreviewed" : pick;
  ui.reviewPending[id] = review;
  render();
  try {
    const r = await fetch(`/api/detections/${encodeURIComponent(id)}/review`, {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ review }) });
    const out = await r.json().catch(() => ({}));
    if (!r.ok || !out.ok) throw new Error(out.reason || String(r.status));
    Object.assign(f, { review: out.review, review_at: out.review_at });
  } catch (e) {
    toast("확인·오탐을 저장하지 못했습니다 · 게이트웨이 연결을 확인하고 다시 눌러 주세요");
  }
  if (ui.reviewPending[id] === review) delete ui.reviewPending[id];   // 그사이 다시 누른 값이 있으면 그것을 기다린다
  render();
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
  const now = () => hhmmss(new Date().toISOString());
  ws.onopen = async () => {
    if (ui.everOpen && !ui.wsOpen) logLine({ time: now(), kind: "link", text: "게이트웨이 다시 연결" });   // 끊긴 사이는 기록이 비어 있다
    ui.wsOpen = ui.everOpen = true; await loadFindings(); render();
  };
  ws.onmessage = (ev) => { try { onMessage(JSON.parse(ev.data)); } catch (e) { /* 잘못된 메시지는 무시 */ } };
  ws.onclose = () => {
    if (ui.wsOpen) logLine({ time: now(), kind: "link", text: "게이트웨이 끊김 · 다시 연결하는 중" });   // 다시 붙기를 시도할 때마다 적지 않는다
    ui.wsOpen = false; resetOperator(); render(); setTimeout(connect, 1000);
  };
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
  if (ev.target.closest('[data-live-map-save]')) { liveSave?.save(); return; }
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
  const ly = ev.target.closest("[data-map-layer]");
  if (ly) {
    const key = ly.dataset.mapLayer;
    if (!ui.mapHidden.delete(key)) ui.mapHidden.add(key);
    savePref("controller.map.hidden", [...ui.mapHidden].join(","));
    return render();
  }
  const rv = ev.target.closest("[data-review]");
  if (rv) {
    const id = rv.closest("[data-event]").dataset.event;
    if (ui.live) return sendReview(id, rv.dataset.review);   // 실시간: 게이트웨이에 저장(모든 화면이 같이 본다)
    ui.reviews[id] = ui.reviews[id] === rv.dataset.review ? "unreviewed" : rv.dataset.review;   // 샘플: 이 브라우저 안에서만
    return render();
  }
  const pick = ev.target.closest(".det[data-event], .mark[data-event]");
  if (pick) {
    document.querySelectorAll("[data-event]").forEach((el) => el.classList.toggle("sel", el.dataset.event === pick.dataset.event));
    return;
  }
  const ol = ev.target.closest("[data-outline]");
  if (ol) {
    if (ol.classList.contains("off")) return toast("사람 윤곽: AI 합성 영상에는 윤곽선이 들어 있습니다");
    ui.outline = !ui.outline;
    savePref("controller.outline", ui.outline ? "on" : "off");
    return render();
  }
  const fb = ev.target.closest("[data-feed]");
  if (fb) {
    if (fb.classList.contains("off")) return toast("AI 합성: 사람 탐지 프로그램이 합성 영상을 보내고 있지 않습니다");
    ui.feed = fb.dataset.feed;
    savePref("controller.feed", ui.feed);
    return render();
  }
  const tk = ev.target.closest("[data-takeoff]");
  if (tk) {
    const a = tk.dataset.takeoff;
    if (a === "cancel") { closeTakeoff(); return render(); }
    if (a === "area") return setArea(!areaChecked(current().st));
    if (a === "go") return requestTakeoff();
    return;
  }
  const b = ev.target.closest("[data-cmd]");
  if (!b) return;
  const name = b.dataset.cmd;
  if (name === "return_home") return toast("복귀는 자율 비행 쪽과 맞춘 뒤 연결합니다 (화면 자리만)");
  if (b.classList.contains("off")) {
    const why = ui.live && !ui.wsOpen ? "게이트웨이에 연결되지 않았습니다"
      : name === "auto_start" && ui.live ? "자율 비행이 연결되지 않았습니다"
      : name.startsWith("video_") ? REASON.video_disabled : "지금 상태에서는 할 수 없습니다";
    return toast(`${CMD_LABEL[name]}: ${why}`);
  }
  if (name === "takeoff") {
    // 키보드로 누른 클릭(Enter·Space)은 칸 열기까지만 한다. 이륙은 칸 안의 이륙 버튼이나 T 키로.
    if (!ui.live || ev.detail > 0 || !ui.takeoff.open) requestTakeoff();
    return;
  }
  sendCmd(name);
});

let keyboardEnabled = false;
function setArea(confirmed) {   // 주변 확인 체크. 실시간은 게이트웨이에 보내고, 칸은 게이트웨이 점검 결과로 체크를 그린다
  if (!ui.live) { ui.takeoff.sampleChecked = confirmed; return render(); }
  ui.takeoff.confirmedAt = confirmed ? Date.now() : null;
  sendCmd("confirm_area", { confirmed });
}
function openTakeoff() {
  ui.takeoff.open = true;
  ui.takeoff.focus = "area";
  if (areaChecked(current().st)) setArea(false);   // 전에 해 둔 확인은 풀고 새로 확인한다
  render();
}
function closeTakeoff(keepConfirm = false) {   // keepConfirm: 이륙해서 닫을 때(게이트웨이가 확인을 이미 썼다)
  if (!ui.takeoff.open) return;
  ui.takeoff.open = false;
  if (!keepConfirm && areaChecked(current().st)) setArea(false);
  ui.takeoff.confirmedAt = null;
  ui.takeoff.sampleChecked = false;
}
function requestTakeoff() {   // 이륙 버튼·T 키: 칸이 닫혀 있으면 열고, 열려 있으면 준비됐을 때만 이륙(아니면 까닭을 알림)
  if (!ui.takeoff.open) return openTakeoff();
  const why = takeoffBlocker(current().st);
  if (why) return toast(why);
  sendCmd("takeoff");
}
// 지도 콘솔 수동 조종 모드: 주변 확인 뒤 30초 안에 이륙해야 한다(게이트웨이 규칙). 남은 시간을 보이고, 지나면 확인을 푼다.
setInterval(() => {
  const el = $("[data-countdown]");
  if (!el) return;
  const left = Math.ceil((Number(el.dataset.countdown) - Date.now()) / 1000);
  if (left > 0) { el.textContent = ` · ${left}초 안에 이륙`; return; }
  el.remove();
  setArea(false);
  toast("주변 확인 뒤 30초가 지났습니다. 다시 확인하세요");
}, 250);

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
  closeTakeoff();
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
window.addEventListener('pagehide', () => liveSave?.dispose());
window.addEventListener('pageshow', () => { if (ui.live) liveSave?.setTarget(consoleTarget(ui.state).origin || null); });
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
  const badge = $("#conn");
  if (ui.live) {
    $("#scenario-wrap").hidden = true;
    badge.classList.replace("sample", ui.state.source === "fake" || ui.state.source === "replay" ? "live" : "real");
    badge.title = ui.state.source === "replay" ? "지난 회차 기록을 재생하는 화면입니다. 명령은 보내지 않습니다."
      : "게이트웨이에 연결된 화면입니다. 버튼이 실제로 명령을 보냅니다.";
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
