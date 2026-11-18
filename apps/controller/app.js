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
  alertFirst: 0,                        // 사람 발견 알림이 묶고 있는 새 발견의 첫 번호(알림이 닫힌 뒤 오는 발견부터 새로 센다)
  live: false, ws: null, wsOpen: false, everOpen: false, state: null, timeline: [], seq: 0, cache: {}, view: null,
  findings: new Map(), lastDet: null, newFindingAt: 0,   // 실시간 사람 탐지: finding_id → 발견, 영상 위 윤곽선용 최근 탐지
  hold: null,                           // 처음 연결해 이번 회차 기록(GET /api/events)을 채우는 동안 잡아 둔 WS 메시지
  feed: loadPref("controller.feed", "raw") === "ai" ? "ai" : "raw",   // 영상 칸: raw(원본) / ai(사람 탐지 합성 영상). 합성이 끊기면 원본을 보여 준다
  outline: loadPref("controller.outline", "on") !== "off",   // 영상 칸: 원본 위에 사람 윤곽선을 그릴지(보기 고르기, 이 브라우저에 기억)
  map3d: { mode: loadPref("map-mode", "3d"), host: null, view: null, revision: undefined, loading: false, has: false,  // 3D 점군(지도 연결기)
    data: null, plan: null, planOf: null,   // 받은 지도 한 버전과 그것으로 만든 평면도(2D 지도 칸, floorplan.js)
    photo: loadPref("map3d-photo", "0") === "1", ceiling: loadPref("map3d-ceiling", "0") === "1" },   // 3D 보기: 사진 색(아니면 높이 색)·천장 보기(아니면 뺌)
  mapBig: loadPref("map-big", "0") === "1",   // 지도 칸을 가장 큰 칸(기본 프리셋은 영상)과 바꿔 크게 본다
  mapHidden: new Set(loadPref("controller.map.hidden", "").split(",").filter(Boolean)),   // 지도 칸에서 끈 표시(flown·planned·people)
  mapView: { zoom: 1, px: 0, py: 0, rot: 0 }, mapFrame: null,   // 2D 지도 보기(mapview.js): 늘 다 보이게 맞춘 그림 위의 확대·이동·회전(웹과 같음)
  mapHeights: loadPref("controller.map.heights", "0") === "1",   // 막힌 칸을 높이로 나눠 칠하기(우리 것, 기본 꺼짐: 웹과 같은 한 색)
  preset: new URLSearchParams(location.search).get("preset") || loadPref("controller.preset", ""),   // 실시간 감독 칸 배치(presets.js)
  edit: null,   // 화면 편집 중이면 { draft: 고치는 배치, from: 시작한 프리셋 id }
  console: { frame: null, url: null, slot: null, ro: null, loadedUp: null, height: 0, viewportHeight: 0 },
  autonomy: { connected: false, path: null, people: null, pathRev: 0, peopleRev: 0, loading: false, track: [],  // 자율 지도(/api/autonomy)
              grid: null, gridCodes: null, gridVer: undefined, gridLoading: false, gridAt: 0, found: null, foundVer: undefined,
              world: null, worldVer: undefined },   // world: 자율 비행 웹이 그리는 지도 파일(시뮬레이터 벽·가구·목적지, 실기체 빈 지도)
  selFinding: null,                     // 고른 발견(event_id): 그 발견 카드를 강조한다
  takeoff: { open: false, confirmedAt: null, sampleChecked: false, focus: null,   // 이륙 전 확인 칸(열림, 주변 확인 보낸 시각, 샘플 체크, 다시 그린 뒤 초점)
    armedUntil: 0 },   // 자율 비행 보기: 시작을 한 번 누른 뒤 4초 안에 한 번 더(자율 비행 웹과 같은 두 번 누르기)
  goto: { pick: null, view: null },   // 자율 비행 보기: 2D 지도에서 찍은 자리(보내기 전, map m)와 지금 지도 좌표 변환
  fell: null,   // 드론이 스스로 내려앉은 까닭(flipped·crash·grounded)과 판단한 곳. 다음 이륙·새 회차까지 빨간 띠로 둔다
};
if (!["2d", "3d"].includes(ui.map3d.mode)) ui.map3d.mode = "3d";   // 지도 콘솔은 따로 화면(#/map)
const liveSave = window.createLiveMapSave?.({onChange: () => render()});
// 지도 그림(board.js): 2D 자율 지도 칸의 진입로·사람 동선. 그리기를 켜면 실시간 감독 지도를 크게(끄면 되돌림)
const mapBoard = window.createMapBoard?.({ render: () => render(), toast: (msg) => toast(msg), ready: () => ui.live && ui.wsOpen,
  setBig: (on) => { if (on === ui.mapBig || (on && !ui.mapCanBig)) return false; ui.mapBig = on; return true; } });
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
// 자율 비행(ROS) 비행 노드의 단계 이름 → 쉬운 말. 모르는 이름은 PHASE로 보인다(코드를 그대로 적지 않는다).
const MISSION_STATE = {
  WAIT_POSE: "위치 잡는 중", READY: "출발 대기", TAKEOFF: "비행 높이로", PLAN: "경로 계산", FOLLOW: "경로 따라 이동",
  LOOK: "둘러보기", REPLAN: "경로 다시 계산", BACKOFF: "물러나기", RELOC: "위치 다시 잡기", DOOR: "문 지나기", HOLD: "제자리",
  EXIT: "되짚어 나오기", HOMESPIN: "출발 방향 확인", LAND: "착륙", DONE: "끝", ABORT: "중단",
  UNSTICK: "닿은 곳에서 떼어 내기", WANDER: "둘러보기 비행", TURN: "방향 바꾸기", GAP: "틈(문) 지나기",
};
const missionWords = (m) => !m ? "없음" : MISSION_STATE[m.state] || PHASE[m.phase] || m.phase || "없음";
const SOURCES = { drone: "드론 상태", autonomy: "자율 비행", detect: "사람 탐지", map: "지도", video: "드론 영상" };
const REASON = {
  not_allowed_in_mode: "지금 상태에서는 할 수 없습니다", preflight_failed: "이륙 전 점검을 통과하지 못했습니다",
  invalid_command_args: "잘못된 조종 요청입니다", confirm_area_required: "이 화면에서 주변을 다시 확인하세요",
  takeoff_video_required: "이륙 전에 카메라 영상을 켜고 확인하세요",
  video_required_in_flight: "비행 중에는 조종 영상을 유지합니다",
  drone_io_error: "기체와 통신할 수 없습니다",
  client_not_allowed: "자율 시작은 PC 감독 화면에서만 할 수 있습니다", autonomy_not_connected: "자율 비행이 연결되지 않았습니다",
  autonomy_joined_in_air: "이륙 뒤에 연결된 자율이라 시작할 수 없습니다", replay_source: "기록 재생 중에는 명령을 보내지 않습니다",
  ros_view_read_only: "자율 비행 보기에서는 명령을 보내지 않습니다(조종은 자율 비행 화면에서)",
  ros_view_not_available: "자율 비행 보기에는 없는 조작입니다(자율 비행 프로그램이 조종합니다)",
  ros_view_unreachable: "자율 비행 보기 노드에 닿지 않습니다", bad_xy: "보낼 자리 좌표가 잘못됐습니다",
  commands_disabled: "보기 노드가 명령을 받지 않게 켜져 있습니다(commands:=false)",
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
  // 자율 비행 보기(--ros-view): 모드는 자율 비행 드라이버 상태를 따른다
  autonomy_flying: "자율 비행이 띄움", autonomy_landing: "자율 비행이 착륙 중", autonomy_ground: "자율 비행 드론이 땅에 있음",
  ros_view_unreachable: "자율 비행 보기 노드 끊김", autonomy_driver_silent: "자율 비행 드라이버 응답 없음",
  drone_not_connected: "자율 비행 드라이버가 드론과 끊김",
  // 기체가 스스로 내린 까닭(비행 제어 _crash_check, 자율 비행 드라이버). 말은 자율 비행 웹과 같게
  crash: "충돌로 보임(크게 기운 뒤 바닥)", flipped: "뒤집힘(모터를 끔)", grounded: "스스로 내려앉음(배터리 약함 의심)",
  cmd_timeout: "비행 노드 속도 명령 끊김",
};
const reasonText = (code) => (code == null ? "" : WHY[code] || code);

const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const pct = (v) => (v == null ? "없음" : `${Math.round(v * 100)}%`);
// 분:초. 초는 버린다(반올림하면 59.6초가 "00:60"으로 나왔다)
const clock = (s) => (s == null ? "없음" : `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(Math.floor(s % 60)).padStart(2, "0")}`);
const meters = (v) => (v == null ? "없음" : `${v.toFixed(2)}m`);
// 시간 이름: 게이트웨이가 비행 시간을 센 것(자율 비행 보기: 시뮬레이터 Tello는 모터 시간이 늘 0)이면 "비행",
// 실기체 게이트웨이는 Tello가 준 모터 동작 시간이라 "모터". 샘플은 비행 시간.
const timeLabel = (long) => (!ui.live || ui.state?.flight_time_s != null ? (long ? "비행 시간" : "비행") : (long ? "모터 동작 시간" : "모터"));
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
    const alert = !ui.wsOpen ? (ui.everOpen ? "gateway" : "connecting") : fellNow(st) ? "fell"
      : w.includes("height_mismatch") ? "height_gap" : w.includes("low_altitude") ? "height_low" : w.includes("link_weak") ? "link"
      : w.includes("autonomy_disconnected") && st.mode === "paused" ? "autonomy_lost" : w.includes("autonomy_timeout") ? "autonomy_timeout"
      : w.includes("autonomy_stale") ? "stale" : done ? "mission_done" : null;
    const events = liveFindings();
    const person = !alert && events[0] && Date.now() - ui.newFindingAt < PERSON_ALERT_MS && events[0].review === "unreviewed";
    const doing = !alert && !person && doingText(st) ? "doing" : null;   // 자율 비행 보기: 다른 알림이 없을 때 지금 하는 일과 이유
    return { sc: { alert: alert || (person ? "person" : doing) }, st, events, track: [] };
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
  const fell = ui.live && fellNow(st);
  const [mode, modeTone] = MODE[st.mode] || [st.mode, "neutral"];
  const tone = fell ? "danger" : modeTone;
  // 조종권 말("제자리 대기" 등)은 떠 있을 때만. 땅·끊김·이륙 중에 "제자리 대기"는 떠서 기다리는 것처럼 읽혔다(11/17)
  const air = AIR_MODES.includes(st.mode);
  const who = [fell ? reasonText(ui.fell.reason) : "", air ? AUTHORITY[st.authority] : "",
    rosTakeoffWords(st) || MISSION_STATE[st.mission?.state] || "",
    st.mission?.progress != null ? `진행 ${pct(st.mission.progress)}` : ""].filter(Boolean).join(" · ");
  const b = st.battery_pct;
  const btone = b == null ? "" : b <= 15 ? "danger" : b < 30 ? "warning" : "";
  const d = st.down_tof;
  const hw = ui.live ? st.height_watch : null;   // 게이트웨이 높이 지켜보기(자율 비행 보기): 어긋남·아래 가까움이면 주의 색
  const hsub = hw?.gap ? `하방 센서 · 자율 추정 ${meters(hw.gap.est_m)}(어긋남)` : hw?.low ? "하방 센서 · 아래가 가까움"
    : d.valid ? "하방 센서" : d.reason === "below_min_or_on_ground" ? "땅이거나 너무 가까움" : "하방 센서 값 없음";
  const n = events.length, unreviewed = events.filter((e) => e.review === "unreviewed").length, k = autonomyPeople();
  const fsub = [n ? `미확인 ${unreviewed}${n > unreviewed ? ` · 정리 ${n - unreviewed}` : ""}` : k ? "" : "아직 없음",
    k ? `자율 비행 ${k}명` : ""].filter(Boolean).join(" · ");   // 자율 비행이 찾은 사람은 사람 탐지 발견과 따로 센다
  // 연결 전·꺼짐(빈 점)은 말을 줄이고 마우스를 올리면 설명을 보인다. 주의·끊김·모름은 말을 붙인다.
  const parts = partStates(st, events).map(([name, [ptone, word, tip]]) => {
    const say = word && (ptone !== "neutral" || word === "모름");
    return `<span class="part ${ptone}" title="${esc(`${name}: ${word ? `${word} · ` : ""}${tip}`)}">${esc(name)}${say ? ` <em>${esc(word)}</em>` : ""}</span>`;
  }).join("");
  const [linkText] = LINK[st.link.level] || ["통신 측정 전"];
  const recv = st.link.state_hz != null ? ` · ${st.link.state_hz}Hz · ${st.link.rtt_ms}ms` : "";
  return `<section class="status ${compact ? "compact" : ""}" data-part="strip" aria-label="비행 상태">
    ${statCard("모드", esc(mode), esc(who), tone === "neutral" ? "" : tone)}
    ${statCard("배터리", b == null ? "없음" : `${b}%`, `${timeLabel(false)} ${clock(st.flight_time_s)}`, btone)}
    ${statCard("높이", d.valid ? meters(d.m) : "없음", hsub, hw?.gap || hw?.low ? "warning" : "")}
    ${statCard("발견", n ? `${n}건` : "없음", fsub, n || k ? "person" : "")}
    <div class="stat wide"><span>연결</span><p class="parts">${parts}</p><small>${esc(linkText + recv)}</small></div>
  </section>`;
}

// 사람 발견 알림(분홍: 위험 빨강과 나눔). 사진·번호·신뢰도를 보이고 확인·오탐을 바로 누르게 한다(누르면 알림이 닫힌다).
// 알림이 떠 있는 동안 이어진 새 발견은 알림 하나로 묶어 "새 발견 3건"이라고 적는다. 사진·버튼은 가장 최근 발견 것이다.
// "여기서 멈추기"는 게이트웨이가 멈추기(pause)를 받을 때(자율 탐색 중)만 둔다. 땅·수동 비행에서는 멈출 것이 없다.
function personAlert(events, st) {
  const e = events[0], n = findingNo(e);
  const burst = ui.live ? events.filter((x) => x.review === "unreviewed" && x.n >= ui.alertFirst).length : 1;
  const stop = st.allowed.includes("pause") ? `<button data-cmd="pause">여기서 멈추기</button>` : "";
  const photo = e.photo_url ? `<a class="alert-photo" href="${esc(e.photo_url)}" target="_blank" rel="noopener" title="사진 크게 보기"><img src="${esc(e.photo_url)}" alt="발견 ${n}번 사진"></a>` : "";
  const more = [`신뢰도 ${e.confidence.toFixed(2)}`, findingTime(e), e.people > 1 ? `최대 ${e.people}명` : "",
    burst > 1 ? `새 발견 ${burst}건(${ui.alertFirst}~${n}번)` : "", stop ? "자동으로 멈추지 않습니다" : ""].filter(Boolean).join(" · ");
  return `<div class="alert person" role="alert">${photo}
    <div class="alert-text"><b>사람 발견 ${n}번</b><span>${esc(more)}</span></div>
    <div class="alert-actions" data-event="${esc(e.event_id)}">${reviewButtons(e)}${stop}</div></div>`;
}

// 자율 비행 보기: 지금 하는 일과 이유(비행 노드 /tt/mission_note → 게이트웨이 mission.note). 자율 비행 웹 머리글 밑 띠와 같은 때·같은 말.
// 11/16 실기체: 탐색 시간 120초가 다 돼 출발점으로 돌아와 내렸는데, 화면에는 단계만 나와 왜 내렸는지 몰랐다.
// 알림 띠의 맨 아래 차례다(게이트웨이 연결·사람 발견 알림이 먼저). [굵은 말, 나머지, 색, 매초 바뀜(남은 시간)] 또는 null
const ROS_AIRBORNE = ["TAKEOFF", "PLAN", "FOLLOW", "REPLAN", "BACKOFF", "HOLD", "UNSTICK", "EXIT", "LOOK", "RELOC", "HOMESPIN",
  "DOOR", "WANDER", "TURN", "GAP", "LAND"];   // 게이트웨이 AIRBORNE·자율 비행 웹과 같은 공중 단계
function doingText(st) {
  const m = st.source === "ros" ? st.mission : null, n = m?.note;
  if (!n) return null;
  const home = n.return_home === false ? "그 자리에 내립니다" : "출발점으로 돌아와 내립니다";
  if (m.state === "DONE") return ["비행 끝", `내린 이유: ${n.why_end || "기록 없음"}`, "success"];
  if (m.state === "ABORT") return ["중단", `이유: ${n.why_end || "기록 없음"}`, "danger"];
  if (n.stale) return null;   // 비행 노드가 3초 넘게 조용하다(꺼짐): 끝·중단 이유만 남긴다
  if (m.state === "LAND") return ["내리는 중", `이유: ${n.why_end || "기록 없음"}`, "warning"];
  if (n.returning) {
    return ["출발점으로 돌아가는 중", `이유: ${n.why_return || "기록 없음"} · 더 보내려면 지도를 찍고 [여기로 보내기](다녀온 뒤 다시 돌아옵니다)`, "warning"];
  }
  if (!n.search || !ROS_AIRBORNE.includes(m.state)) return null;
  const left = n.search.limit - n.search.elapsed;
  if (left > 0) {
    return ["자율 탐색 중", `남은 탐색 시간 ${clock(left)}(전체 ${clock(n.search.limit)}) · 다 되면 ${home}`, left <= 30 ? "warning" : "neutral", true];
  }
  return [`탐색 시간(${Math.round(n.search.limit)}초)이 다 됐습니다`, `${activeGoal(st) ? "찍은 자리를 다녀온 뒤" : "하던 동작을 마치면"} ${home}`, "warning"];
}
// 비행 기록 한 줄: 돌아가기 시작한 이유와 내리는(중단한) 이유(게이트웨이 mission_note, 자율 비행 웹 기록과 같은 때)
const missionNoteText = (d) => (d.what === "returning" ? `자율 비행: ${d.why_return} 출발점으로 돌아갑니다`
  : `자율 비행: ${d.state === "ABORT" ? "중단한" : "내리는"} 이유 · ${d.why_end}`);

// 드론이 스스로 내려앉았다: 비행 제어·자율 비행 드라이버의 충돌 판단(_crash_check)이 모드를 땅으로 바꾼 까닭.
// 11/17 실기체: 뒤집혀 모터가 꺼졌는데 비행 기록 한 줄뿐이었고, 그 10초 동안 알림 띠는 사람 발견 알림이 차지했다.
// 그래서 사람 발견보다 앞에 빨간 띠로 두고, 다음 이륙이나 새 회차까지 남긴다(그사이 드론이 꺼져 끊겨도).
const FELL = {
  flipped: ["드론이 뒤집혔습니다", "모터를 껐습니다"],
  crash: ["충돌로 보입니다", "크게 기운 뒤 바닥에 내려앉았습니다"],
  grounded: ["드론이 스스로 내려앉았습니다", "충돌은 아니고 배터리 약함이 의심됩니다(완충 배터리로 바꾸세요)"],
};
// 실시간 게이트웨이는 안전 착륙(landing, 까닭 flipped 등) → 땅(landed), 자율 비행 보기는 바로 땅(까닭 flipped 등)으로 바뀐다
const FLYING_MODES = ["manual", "auto", "paused"];
function noteFell(st) {
  const t = st.last_transition;
  if (t && ["landing", "grounded"].includes(t.to) && FELL[t.reason]) ui.fell = { reason: t.reason, source: st.source };
  else if (FLYING_MODES.includes(st.mode) || st.flight_busy === "takeoff") ui.fell = null;   // 다시 떴다
}
const fellNow = (st) => !!ui.fell && !FLYING_MODES.includes(st.mode);
function fellAlert() {
  const [head, text] = FELL[ui.fell.reason];
  const who = ui.fell.source === "ros" ? "자율 비행 드라이버가 판단" : "비행 제어가 판단";
  return `<div class="alert danger" role="alert"><b>${head}</b><span>${text}(${who}). 드론을 확인하세요: 프로펠러·가드·모터. 다음 이륙 전까지 보입니다.</span></div>`;
}

function alertBox(sc, events, st) {
  // 알림 띠의 버튼은 게이트웨이가 지금 받는 명령일 때만 보인다. 눌러도 거절 알림만 뜨는 버튼을 두지 않는다(사람 발견의 멈추기는 personAlert).
  const act = (name, label) => (st.allowed.includes(name) ? `<button data-cmd="${name}">${label}</button>` : "");
  let body = "";
  if (sc.alert === "connecting") {
    body = `<div class="alert warning slim" role="status"><b>게이트웨이 연결 중</b><span>연결되면 버튼이 열립니다.</span></div>`;
  } else if (sc.alert === "gateway") {
    body = `<div class="alert danger" role="alert"><b>게이트웨이 연결 끊김</b><span>다시 연결하는 중입니다. 이 화면에서 보낸 조종 입력은 0으로 처리됩니다.</span></div>`;
  } else if (sc.alert === "fell") {
    body = fellAlert();
  } else if (sc.alert === "height_gap" && st.height_watch?.gap) {
    // 게이트웨이 높이 지켜보기(height_watch.py): 비행에 끼어들지 않고 알리기만. 11/17 충돌은 이 상태로 바닥까지 내려갔다
    const g = st.height_watch.gap, high = g.gap_m > 0;
    body = `<div class="alert warning" role="alert"><b>자율 비행이 높이를 실제보다 ${Math.abs(g.gap_m).toFixed(1)} m ${high ? "높게" : "낮게"} 보고 있습니다</b>
      <span>추정 ${meters(g.est_m)} · 기압 ${meters(g.baro_m)} · 하방 ${meters(g.tof_m)}. ${high ? "실제보다 낮게 내려갈" : "실제보다 높이 오를"} 수 있습니다. 위험하면 착륙하세요.</span>
      ${act("land", "착륙")}</div>`;
  } else if (sc.alert === "height_low" && st.height_watch?.low) {
    const t = st.height_watch.low.tof_m;
    body = `<div class="alert warning" role="alert"><b>드론 아래가 가깝습니다(하방 ${t == null ? "0.1 m 아래" : meters(t)})</b>
      <span>바닥이나 가구 위 0.5 m 안에서 2초 넘게 날고 있습니다. 위험하면 착륙하세요.</span>${act("land", "착륙")}</div>`;
  } else if (sc.alert === "person" && events[0]) {
    body = personAlert(events, st);
  } else if (sc.alert === "link") {
    body = `<div class="alert warning" role="alert"><b>통신 약함</b><span>더 진행하지 않고 멈췄습니다. 다시 시작하면 연결이 끊길 수 있습니다.</span>
      <button data-cmd="return_home">출발점으로 복귀</button></div>`;
  } else if (sc.alert === "autonomy_lost") {
    body = `<div class="alert danger" role="alert"><b>자율 연결 끊김</b><span>자율 비행 연결이 끊겨 제자리에서 멈췄습니다. 착륙하거나 넘겨받아 조종하세요.</span>
      ${act("takeover", "넘겨받기")}</div>`;
  } else if (sc.alert === "autonomy_timeout") {
    body = `<div class="alert warning" role="alert"><b>자율 명령 끊김</b><span>2초 넘게 이동 명령이 없어 일시정지했습니다. 자율 상태를 확인하고 다시 시작하거나 넘겨받으세요.</span>
      ${act("takeover", "넘겨받기")}</div>`;
  } else if (sc.alert === "mission_done") {
    const failed = ui.state?.mission?.phase === "failed";
    const why = failed ? esc(ui.state.mission.reason || "자율이 임무를 끝내지 못했습니다") : "자율 탐색을 마쳤습니다";
    body = `<div class="alert ${failed ? "danger" : "warning"} slim" role="status"><b>${failed ? "임무 실패" : "임무 끝"}</b><span>${why}. 제자리에서 기다립니다. 착륙하거나 넘겨받으세요.</span>
      ${act("land", "착륙")}</div>`;
  } else if (sc.alert === "stale") {
    body = `<div class="alert warning" role="alert"><b>자율 응답 지연</b><span>0.5초 넘게 명령이 없어 제자리에서 기다립니다. 2초가 지나면 일시정지합니다.</span></div>`;
  } else if (sc.alert === "doing") {
    const [head, text, tone, ticking] = doingText(st);   // 남은 시간은 매초 바뀌어 화면 읽기 프로그램에 알리지 않는다
    body = `<div class="alert ${tone} slim" ${ticking ? "" : 'role="status"'}><b>${esc(head)}</b><span>${esc(text)}</span></div>`;
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
  const off = lost || v.enabled === false || st.source === "ros" || (on && st.mapping_console_url &&   // 자율 비행 보기: 영상은 자율 드라이버가 정한다
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
  const spot = e.drone?.pose?.frame === "map" ? e.drone.pose : null;   // 그때 자율 위치(사진을 찍을 때 드론 자리)
  const where = !big ? ""
    : e.seen_at ? `<div class="muted">${spot ? `그때 드론 위치 (${spot.x_m.toFixed(1)}, ${spot.y_m.toFixed(1)}) m` : "그때 드론 위치 없음(자율 비행 연결 전)"} · ${(MODE[e.drone?.mode] || ["없음"])[0]} · 높이 ${meters(e.drone?.height_m)} · 사람 위치는 아직 없음</div>`
    : `<div class="muted">발견 위치: 그때 드론 위치 (${poseAt(e.pts_us / 1e6).x_m.toFixed(1)}, ${poseAt(e.pts_us / 1e6).y_m.toFixed(1)}) m · 사람 위치는 아직 없음</div>`;
  const more = e.seen_at ? ` · ${e.people > 1 ? `최대 ${e.people}명 · ` : ""}${e.events}번 봄` : ` · ${esc(e.frame_id)}`;   // 이 발견으로 묶인 탐지 장면 수
  return `<article class="det ${big ? "big" : ""} ${e.review}${ui.selFinding === e.event_id ? " sel" : ""}" data-event="${esc(e.event_id)}" tabindex="0">
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
  const k = autonomyPeople();
  const found = k ? `<span class="chip person src-inline" title="자율 비행이 지도에 찍은 사람(2D 지도의 원). 사람 탐지 발견(사진)과 따로 셉니다">자율 비행이 찾은 사람 ${k}명</span>` : "";
  return `<section class="panel dets ${big ? "big" : ""}" data-part="dets" aria-label="발견 목록">
    <header><h2>발견</h2>${src("detect")}${ds ? chip(ds.text, ds.tone, "src-inline") : ""}${found}</header>
    <div class="det-list">${body}</div>
  </section>`;
}

const MAP_MODES = { "2d": "2D 지도", "3d": "3D 점군" };
const MAP_MODE_TIP = { "2d": "자율 비행이 보낸 드론 위치·경로·찾은 사람(1칸 1 m). 자율 비행이 없으면 3D 점군을 위에서 본 평면도", "3d": "지도 파트가 드론 영상으로 만든 3D 점군" };

// 지도 칸 표시 고르기: 범례가 곧 켜고 끄는 버튼이다. 끈 표시는 이 브라우저에 기억한다(controller.map.hidden).
// 드론 위치와 출발은 늘 그린다. 3D 점군은 지나온 길(카메라 경로)만 고른다(사람 점은 지도 파트가 점군에 칠한다).
const LAYER_SWATCH = { flown: "flown", planned: "planned" };   // 샘플 지도. 3D·평면도는 "cam"을 준다
const shown = (key) => !ui.mapHidden.has(key);

// 발견 지점(사람 탐지가 사진을 찍을 때 드론이 있던 자리)은 2D 지도에 그리지 않는다(11/17 "추가된 게 많아 가시성이 안 좋다":
// 사람 위치가 아니라 구조에 쓸모가 적고 사람 원 옆에 겹쳤다). 그때 드론 위치는 발견 카드·탐색 결과에 글로 남는다.

function layerButton(key, label, tip, swatch = LAYER_SWATCH[key]) {
  const on = shown(key);
  return `<button class="layer" data-map-layer="${key}" aria-pressed="${on}" title="${esc(`${tip} · 누르면 ${on ? "숨깁니다" : "다시 보입니다"}`)}"><i class="lg ${swatch}"></i>${esc(label)}</button>`;
}

// 찾은 장애물(비행 노드 discovered.json, 지도에 없던 것): 자율 비행 웹과 같이 빨간 면에 빨간 실선 테두리, 사람만으로 생긴 것도 그린다.
// 이름(비행 노드 sources: YOLO가 피하는 물체 AVOID_CLASSES·닿음·센서를 쉬운 말로)은 마우스를 올리면 보인다.
const FOUND_NAME = { "yolo:chair": "의자", "yolo:dining table": "탁자", "yolo:couch": "소파", "yolo:bed": "침대", "yolo:bench": "벤치",
  "yolo:potted plant": "화분", "yolo:refrigerator": "냉장고", "yolo:toilet": "변기", "yolo:suitcase": "여행 가방", "yolo:backpack": "가방",
  "yolo:bicycle": "자전거", "yolo:dog": "개", "yolo:cat": "고양이", "yolo:person": "사람", contact: "닿음", tof: "앞 센서", under: "밑 센서" };
function foundTip(w) {
  const src = (Array.isArray(w.sources) ? w.sources : []).filter((x) => typeof x === "string");
  const name = [...new Set(src.map((x) => FOUND_NAME[x] || x.replace(/^yolo:/, "")))].join("·");
  return `찾은 장애물(지도에 없던 것)${name ? ` · ${name}${src.some((x) => x.startsWith("yolo:")) ? "(YOLO)" : ""}` : ""} · ${w.sx.toFixed(1)}×${w.sy.toFixed(1)} m`;
}

// 자율 비행 웹 지도 범례 그대로(autonomy_web/static/index.html .legend): 카메라 지도는 지도가 있을 때, 사람은 찾았을 때만
function webLegend(hasGrid, hasPeople, hasTruth = false) {
  const i = (cls) => `<i class="wl ${cls}"></i>`;
  return `<span class="wlegend">`
    + `<span>${i("wall")}벽</span><span>${i("obst")}가구·장애물</span><span>${i("found")}찾은 장애물(지도에 없던 것)</span>`
    + (hasGrid ? `<span>${i("cam")}카메라 지도 막힘 ${i("cam-sus")}의심(경로는 피함) ${i("seen")}본 곳</span>` : "")
    + (hasPeople ? `<span>${i("person")}찾은 사람(길 막음) ${i("person-gone")}지나간 사람(다시 봤는데 없음)</span>` : "")
    + `<span>${i("path")}계획 경로</span><span>${i("trail")}지나온 길(추정)</span><span>${i("drone")}드론(추정)</span>`
    + `<span>${i("goal")}목적지</span><span>${i("pick")}찍은 자리(운용자)</span>`
    + (hasTruth ? `<span>${i("truth")}정답 위치(시뮬레이션)</span>` : "") + `</span>`;
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
  // 출처는 칩 하나: 2D는 자율 비행이 보낸 위치·경로·찾은 사람, 3D와 평면도는 지도 파트의 점군. 샘플은 지어낸 회차다.
  const source = mode === "3d" || plan ? "지도 파트" : ui.live ? "자율 비행" : "자율 비행 · 사람 탐지";   // 제목 "지도"와 겹치지 않게 "지도 파트"
  const head = `<header><h2>지도</h2><span class="src" title="이 칸을 채우는 파트">${source}</span><span class="spacer"></span>${seg || '<a class="link" href="#/map">지도 콘솔 열기</a>'}</header>`;
  if (mode === "3d") return map3dPanel(st, head);
  if (plan) return floorPlanPanel(st, head, plan, tall);
  if (ui.live) return autonomyMapPanel(st, head, tall);
  return sampleMapPanel(st, events, track, head, tall);
}

// 계획 경로(자율 비행이 가려는 길): 자율 비행 웹과 같이 2 px 점선(6·4)과 처음·끝을 포함한 모든 점에 점(반지름 3)
function plannedSvg(pts, X, Y) {
  if (!pts.length) return "";
  const xy = pts.map(([x, y]) => [X(x), Y(y)]);
  const line = xy.length > 1 ? `<polyline class="planned" points="${xy.map((p) => p.join(",")).join(" ")}"/>` : "";
  return `<g class="plan"><title>계획 경로: 자율 비행이 가려는 길</title>${line}`
    + `${xy.map(([x, y]) => `<circle class="plan-pt" cx="${x}" cy="${y}" r="3"/>`).join("")}</g>`;
}
// 드론(웹 drawMap): 끝 (r, 0)·뒤 (−0.7r, ±0.6r)·오목 (−0.4r, 0) 화살촉을 기수 방향으로 돌린다(지도 y가 위라 −yaw)
function droneArrow(x, y, yawRad, r) {
  const f = (v) => v.toFixed(1);
  return `<path transform="translate(${x},${y}) rotate(${f(-yawRad * 180 / Math.PI)})" `
    + `d="M${f(r)},0 L${f(-0.7 * r)},${f(0.6 * r)} L${f(-0.4 * r)},0 L${f(-0.7 * r)},${f(-0.6 * r)} Z"/>`;
}
// 출발(웹 spawn): 14×14 회색 네모, 글 없음
const spawnSvg = (x, y, tip) => `<rect class="spawn" x="${(+x - 7).toFixed(1)}" y="${(+y - 7).toFixed(1)}" width="14" height="14"><title>${esc(tip)}</title></rect>`;

// 샘플 2D 지도(게이트웨이 없이): 방 구획·지나온 길·계획 경로·지금 드론 위치.
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
    ? plannedSvg(S.plan.map((p) => [p.x_m, p.y_m]), X, Y) : "";
  const now = track[track.length - 1] || S.track[0];
  const drone = `<g class="drone">${droneArrow(X(now.x_m), Y(now.y_m), Math.PI / 2, 9)}</g>`;
  const start = spawnSvg(X(0), Y(0), "출발");
  const layers = layerButton("flown", "지나온 길", "드론이 지나온 길") + layerButton("planned", "계획 경로", "자율 비행이 가려는 길");
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="샘플 지도: 지나온 길, 계획 경로, 지금 드론 위치">
      ${rooms}${path}${planned}${start}${drone}
    </svg>
    ${mapFoot(null, "", layers)}
  </section>`;
}

// 자율 지도(2D): 자율 비행이 보낸 위치·계획 경로·찾은 사람(frame map: 켠 자리 원점, x 앞·y 왼쪽, m).
// 화면에는 x를 오른쪽, y를 위로 그린다. 지나온 길은 화면이 위치를 모아 그린다(자율 연결이 새로 붙으면 비운다).
// 경로·찾은 사람은 길어서 상태에는 버전만 오고, 버전이 바뀌면 GET /api/autonomy로 읽는다.
function syncAutonomy(st) {
  const a = ui.autonomy, s = st.autonomy;
  if (!s) return;
  if (s.connected && !a.connected) { a.track = []; loadTrail(); }   // 새로 연 화면·새 연결: 게이트웨이가 모아 둔 지나온 길로 채운다
  a.connected = s.connected;
  const p = st.pose;
  if (p && p.valid) {
    const last = a.track[a.track.length - 1];
    if (!last || Math.hypot(p.x_m - last[0], p.y_m - last[1]) > 0.03) a.track.push([p.x_m, p.y_m]);
    if (a.track.length > 3000) a.track.splice(0, a.track.length - 3000);
  }
  if ((s.path_revision !== a.pathRev || s.people_revision !== a.peopleRev) && !a.loading) loadAutonomy();
  // 자율 비행이 만드는 지도(벽·장애물): 번호가 바뀔 때만, 1초에 한 번까지. 새 자율 연결이면 지난 지도는 지운다
  if (s.grid && s.grid.ver !== a.gridVer && !a.gridLoading && performance.now() - a.gridAt > GRID_MIN_MS) loadGrid();
  else if (!s.grid && a.gridVer !== undefined) clearGrid();
  if ((s.discovered?.version ?? null) !== (a.foundVer ?? null)) loadFound(s.discovered);
  if ((s.world_map?.version ?? null) !== (a.worldVer ?? null)) loadWorldMap(s.world_map);
}

// 지나온 길은 화면이 받은 위치로 그린다. 비행 중에 새로 연 화면은 처음 길이 없어서(11/17), 게이트웨이가 이번 연결에서
// 모아 둔 길(GET /api/autonomy/trail)이 더 길면 그것으로 바꾼다(그 뒤로는 받은 위치를 이어 붙인다).
async function loadTrail() {
  try {
    const r = await fetch("/api/autonomy/trail", { cache: "no-store" });
    if (!r.ok) return;
    const pts = ((await r.json()).points || []).filter((p) => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]));
    if (pts.length > ui.autonomy.track.length) { ui.autonomy.track = pts.slice(-3000); render(); }
  } catch (e) { /* 지나온 길을 내주지 않는 게이트웨이: 연 뒤부터 그린다 */ }
}

// ---------- 자율 비행 지도(비행 노드 /tt/obstacle_grid, 자율 비행 웹 /api/grid와 같은 칸 코드 0 모름·1 본 곳·2 의심·3 막힘) ----------
const GRID_MIN_MS = 1000;
async function loadGrid() {
  const a = ui.autonomy;
  a.gridLoading = true;
  a.gridAt = performance.now();
  try {
    const r = await fetch("/api/autonomy/grid", { cache: "no-store" });
    if (r.ok) {
      const g = await r.json(), bin = atob(g.cells), codes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) codes[i] = bin.charCodeAt(i);
      if (codes.length === g.w * g.h) {
        a.gridCodes = { x0: g.x0, y0: g.y0, w: g.w, h: g.h, res: g.res, counts: g.counts, codes, heights: gridHeights(g) };
        a.gridVer = g.ver;
        await paintGrid();
      }
    }
  } catch (e) { /* 다음 상태 때 다시 */ }
  a.gridLoading = false;
  render();
}
// 칸 코드 → 그림(한 칸 = 한 화소, 테마 색). 지도 y는 위가 +라 그림 줄은 뒤집는다. 본 곳이 있는 범위(bbox)도 셈한다
function cssRgba(name) {
  const t = document.createElement("canvas").getContext("2d");
  t.fillStyle = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#000";
  t.fillRect(0, 0, 1, 1);
  return t.getImageData(0, 0, 1, 1).data;
}
async function paintGrid() {
  const a = ui.autonomy, g = a.gridCodes;
  if (!g) return;
  if (g.heights && ui.mapHeights) return paintHeightGrid(a, g);   // "벽·가구 나눠 보기"를 켰을 때만 높이로 나눠 칠한다(아래). 기본은 웹과 같은 한 색
  const cv = document.createElement("canvas");
  cv.width = g.w; cv.height = g.h;
  const cx = cv.getContext("2d"), img = cx.createImageData(g.w, g.h);
  const colors = [null, cssRgba("--map-seen"), cssRgba("--map-suspect"), cssRgba("--map-blocked")];
  let i0 = Infinity, i1 = -1, j0 = Infinity, j1 = -1;
  for (let j = 0; j < g.h; j++) {
    for (let i = 0; i < g.w; i++) {
      const c = colors[g.codes[j * g.w + i]];
      if (!c) continue;
      const k = ((g.h - 1 - j) * g.w + i) * 4;
      img.data[k] = c[0]; img.data[k + 1] = c[1]; img.data[k + 2] = c[2]; img.data[k + 3] = c[3];
      i0 = Math.min(i0, i); i1 = Math.max(i1, i); j0 = Math.min(j0, j); j1 = Math.max(j1, j);
    }
  }
  cx.putImageData(img, 0, 0);
  const blob = await new Promise((done) => cv.toBlob(done));
  if (a.grid?.url) URL.revokeObjectURL(a.grid.url);   // 지도 그림은 blob 주소로: 지도 칸 HTML이 커지지 않게
  a.grid = { x0: g.x0, y0: g.y0, w: g.w, h: g.h, res: g.res, counts: g.counts, url: blob ? URL.createObjectURL(blob) : null,
             bbox: i1 < 0 ? null : [g.x0 + i0 * g.res, g.y0 + j0 * g.res, g.x0 + (i1 + 1) * g.res, g.y0 + (j1 + 1) * g.res] };
}
// 지도 높이 층(11/17): 보기 노드가 비행 노드 3D 카메라 지도(camera_map.npz)로 칸마다 0 없음·1 낮은 물체·2 벽·높은 것·3 높이 모름을 준다.
// 칸 수가 지도와 같을 때만 쓴다. 없으면 null(옛 보기 노드·가짜 노드·시뮬레이터): 지금처럼 막힘 한 색.
function gridHeights(g) {
  if (typeof g.heights !== "string") return null;
  const bin = atob(g.heights);
  if (bin.length !== g.w * g.h) return null;
  const h = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) h[i] = bin.charCodeAt(i);
  return h;
}
// 높이 층으로 칠하기: 막힌 칸만 나눈다. 벽·높은 것 --map-wall, 낮은 물체 --map-low 면에 --map-blocked 테두리, 그 밖(높이 모름, 3D 지도가
// 30초마다 저장돼 아직 모르는 새 칸)은 --map-unsure. 의심·본 곳은 지금 색 그대로. 테두리를 그리려고 칸마다 GRID_PX화소로 그린다.
// 막힌 칸 종류별 수도 센다(높이 층이 있다는 표시).
const GRID_PX = 4;
async function paintHeightGrid(a, g) {
  const S = GRID_PX, W = g.w * S, H = g.h * S, n = g.w * g.h;
  const cv = document.createElement("canvas");
  cv.width = W; cv.height = H;
  const cx = cv.getContext("2d"), img = cx.createImageData(W, H), px = img.data;
  const kind = new Uint8Array(n), kinds = { low: 0, tall: 0, unsure: 0 };   // 칸 종류: 0 안 그림·1 본 곳·2 의심·3 높이 모름·4 낮은 물체·5 벽·높은 것
  let i0 = Infinity, i1 = -1, j0 = Infinity, j1 = -1;
  for (let c = 0; c < n; c++) {
    let k = g.codes[c];
    if (!k || k > 3) continue;
    if (k === 3) {
      k = g.heights[c] === 2 ? 5 : g.heights[c] === 1 ? 4 : 3;
      kinds[k === 5 ? "tall" : k === 4 ? "low" : "unsure"]++;
    }
    kind[c] = k;
    const i = c % g.w, j = (c - i) / g.w;
    i0 = Math.min(i0, i); i1 = Math.max(i1, i); j0 = Math.min(j0, j); j1 = Math.max(j1, j);
  }
  const fill = [null, cssRgba("--map-seen"), cssRgba("--map-suspect"), cssRgba("--map-unsure"), cssRgba("--map-low"), cssRgba("--map-wall")];
  const edge = cssRgba("--map-blocked"), low = (i, j) => i >= 0 && j >= 0 && i < g.w && j < g.h && kind[j * g.w + i] === 4;
  for (let j = 0; j < g.h; j++) {
    for (let i = 0; i < g.w; i++) {
      const k = kind[j * g.w + i];
      if (!k) continue;
      const left = i * S, top = (g.h - 1 - j) * S;   // 지도 y는 위가 +라 그림 줄은 뒤집는다
      const eL = k === 4 && !low(i - 1, j), eR = k === 4 && !low(i + 1, j), eT = k === 4 && !low(i, j + 1), eB = k === 4 && !low(i, j - 1);
      for (let y = 0; y < S; y++) {
        for (let x = 0; x < S; x++) {
          const col = (eL && x === 0) || (eR && x === S - 1) || (eT && y === 0) || (eB && y === S - 1) ? edge : fill[k];
          const o = ((top + y) * W + left + x) * 4;
          px[o] = col[0]; px[o + 1] = col[1]; px[o + 2] = col[2]; px[o + 3] = col[3];
        }
      }
    }
  }
  cx.putImageData(img, 0, 0);
  const blob = await new Promise((done) => cv.toBlob(done));
  if (a.grid?.url) URL.revokeObjectURL(a.grid.url);
  a.grid = { x0: g.x0, y0: g.y0, w: g.w, h: g.h, res: g.res, counts: g.counts, heights: kinds, url: blob ? URL.createObjectURL(blob) : null,
             bbox: i1 < 0 ? null : [g.x0 + i0 * g.res, g.y0 + j0 * g.res, g.x0 + (i1 + 1) * g.res, g.y0 + (j1 + 1) * g.res] };
}
function clearGrid() {
  const a = ui.autonomy;
  if (a.grid?.url) URL.revokeObjectURL(a.grid.url);
  a.grid = a.gridCodes = null;
  a.gridVer = undefined;
}
async function loadFound(info) {
  const a = ui.autonomy;
  a.foundVer = info?.version;
  if (!info) { a.found = null; return; }
  try {
    const r = await fetch("/api/autonomy/discovered", { cache: "no-store" });
    a.found = r.ok ? (await r.json()).obstacles || [] : null;
  } catch (e) { a.foundVer = undefined; }   // 다음 상태 때 다시
  render();
}
// 자율 비행 웹이 그리는 지도 파일(GET /api/autonomy/map): 번호가 바뀔 때만(보통 켤 때 한 번)
async function loadWorldMap(info) {
  const a = ui.autonomy;
  a.worldVer = info?.version;
  if (!info) { a.world = null; return; }
  try {
    const r = await fetch("/api/autonomy/map", { cache: "no-store" });
    a.world = r.ok ? await r.json() : null;
  } catch (e) { a.worldVer = undefined; }   // 다음 상태 때 다시
  render();
}
window.addEventListener("themechange", () => { paintGrid().then(render); });   // 테마 색으로 다시 칠한다

// 자율 비행이 지도에 찍은 사람 수(확정, 다시 봤는데 없음은 뺌 = 2D 지도 범례와 같은 수). 사람 탐지 발견(사진)과 따로 센다.
const autonomyPeople = () => (ui.autonomy.people?.people || []).filter((q) => !q.gone).length;

// 자율 비행이 보낸 것(위치·경로·찾은 사람)이 하나라도 있는지. 없으면 2D 지도 칸은 평면도나 빈 칸 안내를 그린다.
function autonomyHas(st) {
  const a = ui.autonomy;
  return !!(st.pose || a.path?.points?.length || a.people?.people?.length || a.people?.unconfirmed?.length || a.grid?.bbox
    || a.found?.length);
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

// 자율 비행 웹(modules/autonomy/autonomy_web/static/index.html drawMap)과 똑같이 그린다(11/17 송승우 "똑같이 맞추기"):
// 칸 범위·1 m 눈금·그리는 차례(눈금 → 카메라 지도 → 출발 → 찾은 장애물 → 사람 → 지나온 길 → 계획 경로 → 찍은 자리 → 드론 → 축척)·
// 모양·색·글·범례가 웹과 같다. 우리 것은 회전(요청)·지도 그림(쓸 때만)·막힌 칸 높이로 나눠 칠하기(기본 꺼짐)뿐.
const WEB_FIT = { pad: 1, minSpan: 6, px: 16 };   // 웹: 데이터 둘레 1 m(가장 좁아도 6 m), 가장자리 16 화소
const PIN_LABEL = { pending: "받음", arrived: "둘러보는 중" };   // 비행 노드가 받은 찍은 자리 글(나머지는 "가는 중"), 웹과 같음
function webBox(pts) {   // 웹 drawMap의 b(눈금을 긋는 범위): 점들 가운데 ± (반폭 + 1 m, 가장 작아도 3 m)
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const cx = (Math.min(...xs) + Math.max(...xs)) / 2, cy = (Math.min(...ys) + Math.max(...ys)) / 2;
  const hx = Math.max(3, (Math.max(...xs) - Math.min(...xs)) / 2 + 1), hy = Math.max(3, (Math.max(...ys) - Math.min(...ys)) / 2 + 1);
  return { xmin: cx - hx, xmax: cx + hx, ymin: cy - hy, ymax: cy + hy };
}
function autonomyMapPanel(st, head, tall) {
  const a = ui.autonomy, pose = st.pose, s = st.autonomy || {};
  const path = a.path?.points || [], people = a.people?.people || [];
  if (!autonomyHas(st)) {
    // 빈 칸: 무엇을 기다리는지 쉬운 말로. 지도 파트의 3D 점군이 들어오고 있으면 바로 바꿔 볼 수 있게 한다.
    const why = !ui.wsOpen ? "게이트웨이에 연결되지 않았습니다 · 연결되면 자율 비행이 보낸 지도를 그립니다"
      : s.connected ? "자율 비행이 연결됐습니다 · 위치가 오면 바로 그립니다"
      : "자율 비행이 아직 연결되지 않았습니다 · 연결되면 드론 위치·지나온 길·계획 경로·찾은 사람을 1 m 눈금 위에 그립니다";
    const to3d = !tall && ui.wsOpen && st.map?.alive && st.map.points > 0 ? `<button data-map-mode="3d">3D 점군 보기</button>` : "";
    return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${head}
      <div class="map-empty"><span>${esc(why)}</span>${to3d}</div>${mapSaveBar()}</section>`;
  }
  // 자율 비행 보기: 지도를 한 번 클릭하면 보낼 자리를 찍고, [여기로 보내기]·G로 비행 노드에 보낸다(자율 비행 웹과 같음)
  const rosGoto = ui.live && st.source === "ros", canGoto = rosGoto && st.allowed.includes("goto");
  const goal = rosGoto ? activeGoal(st) : null, pick = rosGoto ? ui.goto.pick : null;
  // 출발(웹 spawn): 실기체 빈 지도는 원점. 게이트웨이가 이륙한 자리를 주면 거기(시뮬레이터는 세계 좌표라 원점이 출발이 아니다)
  const sp = s.start_pose, took = sp && Number.isFinite(sp.x_m) && Number.isFinite(sp.y_m), o = took ? [sp.x_m, sp.y_m] : [0, 0];
  const occ = a.grid, found = (a.found || []).filter((w) => [w.x, w.y, w.sx, w.sy].every(Number.isFinite));
  // 지도 파일(웹 mapData): 시뮬레이터는 세계 지도(벽·가구·목적지·출발), 실기체는 빈 지도(open, 출발만). 출발은 지도 파일 것을 쓴다(웹과 같음)
  const wm = a.world, wb0 = wm && !wm.open ? wm.bounds : null, sp0 = wm?.spawn ? [wm.spawn.x, wm.spawn.y] : o;
  const truth = st.autonomy?.truth;
  // 칸 범위(웹과 같음): 지도 파일이 빈 지도가 아니면 그 범위, 아니면 지나온 길·드론·찾은 사람·출발·본 곳 둘레.
  // 계획 경로·찍은 자리·찾은 장애물은 넣지 않는다. 지도 그림(우리)은 빈 지도일 때만 넣는다
  const pts = wb0 ? [[wb0.xmin, wb0.ymin], [wb0.xmax, wb0.ymin], [wb0.xmin, wb0.ymax], [wb0.xmax, wb0.ymax]]
    : [...a.track, ...(pose ? [[pose.x_m, pose.y_m]] : []), ...people.map((q) => [q.x_m, q.y_m]), sp0,
      ...(occ?.bbox ? [[occ.bbox[0], occ.bbox[1]], [occ.bbox[2], occ.bbox[3]]] : []), ...(mapBoard?.points() || [])];
  const H = tall ? 360 : 260, W = fitWidth(H);
  const fitFor = (r) => MapView.fit(pts, W, H, r, wb0 ? { pad: 0, minSpan: 0, px: 16 } : WEB_FIT), F = fitFor(ui.mapView.rot);
  const v = MapView.absolute(ui.mapView, F);   // 늘 다 보이게 맞춘 그림(F) 위의 확대·이동(웹 view.zoom·px·py)
  v.base = F.k;
  v.fitFor = fitFor;
  ui.mapFrame = v;
  const k = v.k, rot = v.rot, b = wb0 || webBox(pts);
  const X = (x) => MapView.X(v, x).toFixed(1), Y = (y) => MapView.Y(v, y).toFixed(1);
  const wb = mapBoard?.frame({ X, Y, k, minX: b.xmin, maxX: b.xmax, minY: b.ymin, maxY: b.ymax, W, H, start: o, hidden: !shown("board"),   // 지도 그림(board.js)
    fromClient: mapLocal, people: people.map((q, i) => ({ n: i + 1, x: q.x_m, y: q.y_m, gone: q.gone })) });
  // 1 m 눈금은 칸 범위 안에만(웹과 같음)
  const grid = [];
  for (let gx = Math.ceil(b.xmin); gx <= b.xmax; gx++) grid.push(`<line x1="${X(gx)}" y1="${Y(b.ymin)}" x2="${X(gx)}" y2="${Y(b.ymax)}"/>`);
  for (let gy = Math.ceil(b.ymin); gy <= b.ymax; gy++) grid.push(`<line x1="${X(b.xmin)}" y1="${Y(gy)}" x2="${X(b.xmax)}" y2="${Y(gy)}"/>`);
  // 자율 비행이 만드는 지도(카메라 지도 단면): 칸 그림(blob 주소)을 눈금 위에
  const occImage = occ?.url ? `<image class="occ" href="${occ.url}" x="${X(occ.x0)}" y="${Y(occ.y0 + occ.h * occ.res)}" `
    + `width="${(occ.w * occ.res * k).toFixed(1)}" height="${(occ.h * occ.res * k).toFixed(1)}" preserveAspectRatio="none"/>` : "";
  // 지도 파일의 가구(옅은 회색)·벽(진한 회색): 웹 box처럼 가운데·크기·방향(yaw). 출발 네모, 목적지(초록 원 + "목적지")
  const wbox = (q, cls) => `<rect class="${cls}" x="${(-q.sx * k / 2).toFixed(1)}" y="${(-q.sy * k / 2).toFixed(1)}" width="${(q.sx * k).toFixed(1)}" `
    + `height="${(q.sy * k).toFixed(1)}" transform="translate(${X(q.x)},${Y(q.y)}) rotate(${(-(q.yaw || 0) * 180 / Math.PI).toFixed(1)})"/>`;
  const worldBoxes = !wm ? "" : (wm.obstacles || []).map((q) => wbox(q, "wobst")).join("") + (wm.walls || []).map((q) => wbox(q, "wwall")).join("");
  const spawn = spawnSvg(X(sp0[0]), Y(sp0[1]), wm?.spawn ? "출발(지도 파일)" : took ? "출발: 이륙한 자리" : "출발: 자율 비행 지도의 원점(켠 자리)");
  const goalMark = !wm?.goal ? "" : `<g class="wgoal"><circle cx="${X(wm.goal.x)}" cy="${Y(wm.goal.y)}" r="8"/>`
    + `<text x="${(+X(wm.goal.x) + 11).toFixed(1)}" y="${(+Y(wm.goal.y) + 4).toFixed(1)}">목적지</text></g>`;
  // 정답 위치(시뮬레이션, 웹 truth): 회색 원
  const truthMark = !truth ? "" : `<circle class="truth" cx="${X(truth.x_m)}" cy="${Y(truth.y_m)}" r="7"><title>정답 위치(시뮬레이션)</title></circle>`;
  const foundBoxes = found.map((w) => `<rect class="found" x="${X(w.x - w.sx / 2)}" y="${Y(w.y + w.sy / 2)}" width="${(w.sx * k).toFixed(1)}" `
    + `height="${(w.sy * k).toFixed(1)}"><title>${esc(foundTip(w))}</title></rect>`).join("");
  // 찾은 사람(웹): 칠한 원은 길을 막는 사람, 점선 원은 다시 봤는데 없던 사람. 반지름 실제 0.25 m(가장 작아도 5)
  const pr = Math.max(5, 0.25 * k);
  const marks = people.map((q, i) => `<g class="mark${q.gone ? " gone" : ""}"><title>${esc(`사람 ${i + 1} · ${q.gone ? "다시 봤는데 없음" : "찾은 사람(길 막음)"} · `
    + `${q.hits ?? "?"}번 봄${q.conf != null ? ` · 신뢰도 ${q.conf.toFixed(2)}` : ""}`)}</title><circle cx="${X(q.x_m)}" cy="${Y(q.y_m)}" r="${pr.toFixed(1)}"/></g>`).join("");
  const trail = a.track.length > 1 ? `<polyline class="flown" points="${a.track.map((p) => `${X(p[0])},${Y(p[1])}`).join(" ")}"/>` : "";
  // 운용자가 찍은 자리(웹): 비행 노드가 받은 것은 칠한 점(갈 자리, 받기만 했으면 빈 원)과 글, 찍은 곳과 갈 자리가 다르면 점선으로 잇고 찍은 곳에 빈 원.
  // 아직 안 보낸 자리는 점선 원·십자와 "보낼 자리"
  const pin = (q, on, text) => `<circle class="pin${on ? " on" : ""}" cx="${X(q[0])}" cy="${Y(q[1])}" r="7"/>`
    + (text ? `<text class="pin-label" x="${(+X(q[0]) + 11).toFixed(1)}" y="${(+Y(q[1]) + 4).toFixed(1)}">${text}</text>` : "");
  let opMarks = "";
  if (goal?.click) {
    const g = goal.goal || goal.click;
    const apart = Math.hypot(g[0] - goal.click[0], g[1] - goal.click[1]) > 0.15
      ? `<line class="pin-link" x1="${X(goal.click[0])}" y1="${Y(goal.click[1])}" x2="${X(g[0])}" y2="${Y(g[1])}"/>${pin(goal.click, false, "")}` : "";
    opMarks = `<g class="goto-goal"><title>${esc(`찍은 자리 · ${GOAL_STATE[goal.state] || goal.state}`)}</title>${apart}`
      + `${pin(g, goal.state !== "pending", PIN_LABEL[goal.state] || "가는 중")}</g>`;
  }
  const pickMark = !pick ? "" : (() => {
    const px = +X(pick.x), py = +Y(pick.y), f = (n) => n.toFixed(1);
    return `<g class="goto-pick"><title>보낼 자리(아직 안 보냄)</title><circle cx="${f(px)}" cy="${f(py)}" r="9"/>`
      + `<line x1="${f(px - 14)}" y1="${f(py)}" x2="${f(px + 14)}" y2="${f(py)}"/><line x1="${f(px)}" y1="${f(py - 14)}" x2="${f(px)}" y2="${f(py + 14)}"/>`
      + `<text x="${f(px + 13)}" y="${f(py - 8)}">보낼 자리</text></g>`;
  })();
  // 드론(웹): 위치 오차가 2 cm 넘으면 점선 원, 화살촉 반지름 실제 0.25 m(가장 작아도 9). 게이트웨이와 끊기면 흐리게(마지막 위치)
  const drone = !pose ? "" : `<g class="drone${ui.wsOpen ? "" : " stale"}">`
    + (Number.isFinite(pose.sigma_m) && pose.sigma_m > 0.02 ? `<circle class="sigma" cx="${X(pose.x_m)}" cy="${Y(pose.y_m)}" r="${(pose.sigma_m * k).toFixed(1)}"/>` : "")
    + `${droneArrow(X(pose.x_m), Y(pose.y_m), pose.yaw_rad, Math.max(9, 0.25 * k))}</g>`;
  // 축척(웹): 왼쪽 아래 1 m 막대. 돌리지 않는다
  const scale = `<g class="scale"><line x1="16" y1="${H - 16}" x2="${(16 + k).toFixed(1)}" y2="${H - 16}"/><text x="16" y="${H - 21}">1 m</text></g>`;
  // 막힌 칸 높이로 나눠 칠하기(우리 것, 기본 꺼짐): 자율 비행 3D 카메라 지도의 높이가 있을 때만 버튼을 보인다
  const heights = a.gridCodes?.heights ? `<button class="layer" data-map-heights aria-pressed="${ui.mapHeights}" title="${esc(ui.mapHeights
    ? "막힌 칸을 자율 비행 3D 카메라 지도의 높이로 나눠 칠했습니다: 진회색 벽·높은 것(1.6 m 위까지), 옅은 보라 면에 테두리 낮은 물체(꼭대기 1.4 m 아래: 책상·의자, 카메라 깊이로 잰 높이라 틀릴 수 있음), 보라 높이 모름 · 누르면 자율 비행 웹과 같은 한 색으로"
    : "막힌 칸을 자율 비행 3D 카메라 지도의 높이로 나눠 벽·높은 것과 낮은 물체(책상·의자)를 다른 색으로 칠합니다(우리 화면만, 자율 비행 웹은 한 색)")}">벽·가구 나눠 보기</button>` : "";
  const status = autonomyStatus(st);
  // 지도 그림의 도구 막대·목록은 지도와 한 몸통(.map-body)에 둔다: 넓은 칸이면 지도 옆 기둥, 좁으면 지도 위·아래(style.css)
  const side = wb && (wb.bar || wb.list);
  const tools = `<div class="map-tools" role="group" aria-label="지도 보기">`
    + `<button type="button" data-map-view="ccw" title="왼쪽으로 15° 돌리기 · Shift+끌기·두 손가락으로도 돌립니다" aria-label="지도를 왼쪽으로 돌리기">↺</button>`
    + `<button type="button" data-map-view="cw" title="오른쪽으로 15° 돌리기" aria-label="지도를 오른쪽으로 돌리기">↻</button>`
    + `<button type="button" data-map-view="reset" title="확대·이동·회전을 처음대로(지도를 두 번 눌러도 됩니다)">처음대로</button></div>`;
  return `<section class="panel map ${tall ? "tall" : ""}" data-part="map" aria-label="2D 지도">${wb ? wb.head(head) : head}
    <div class="map-body${side ? " wb-side" : ""}">${wb?.bar || ""}<div class="map-canvas"><svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="자율 지도(자율 비행 웹과 같음): 드론 위치, 지나온 길, 계획 경로, 찾은 사람(눈금 1칸 1 m). 휠·두 손가락: 확대 · 끌기: 이동 · Shift+끌기: 회전 · 두 번 클릭: 처음대로 · 한 번 클릭: 보낼 자리 찍기" data-map2d="1"${rosGoto ? ' data-goto-map="1"' : ""}${wb?.attr || ""}>
      <g class="map-view" transform="rotate(${rot.toFixed(2)} ${W / 2} ${H / 2})" style="--map-rot:${rot.toFixed(2)}"><g class="grid">${grid.join("")}</g>${occImage}${worldBoxes}${spawn}${goalMark}${foundBoxes}${marks}${trail}${plannedSvg(path, X, Y)}${opMarks}${pickMark}${truthMark}${drone}${wb?.svg || ""}</g>${scale}
    </svg>${tools}</div>${wb?.list || ""}</div>
    ${mapFoot(status, status[2], webLegend(!!occ, people.length > 0, !!truth) + heights + (wb?.layer || ""))}${rosGoto ? gotoBar(st, canGoto, goal) : ""}${mapSaveBar()}
  </section>`;
}

// 화면 자리 → 2D 지도 m와 그림 좌표(회전 전). 지도 칸의 <g class="map-view">(회전) 변환을 거꾸로 쓴다(mapview.js 보기).
// 보낼 자리 찍기(mapPoint)와 지도 그림(board.js fromClient)이 같이 쓴다
function mapLocal(svg, clientX, clientY) {
  const v = ui.mapFrame, m = svg?.querySelector("g.map-view")?.getScreenCTM?.();
  if (!v || !m) return null;
  const q = new DOMPoint(clientX, clientY).matrixTransform(m.inverse());
  const [x, y] = MapView.toMap(v, q.x, q.y);
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y, sx: q.x, sy: q.y } : null;
}
function resetMapView() {   // 처음대로(두 번 클릭·버튼): 다 보이게 맞추고 돌리지 않은 보기. 두 번 클릭이 찍으려던 보낼 자리는 버린다
  clearTimeout(ui.goto.tapTimer);
  ui.mapView = { zoom: 1, px: 0, py: 0, rot: 0 };
  render();
}
// 2D 지도 손동작(mapview.js): 휠·두 손가락 벌리기 확대·축소, 끌기 이동, Shift+끌기·두 손가락 돌리기 회전, 두 번 클릭 처음대로.
// 놓으면 맞춘 그림 위의 확대·이동(relative)으로 남긴다: 새 위치·지도가 와도 웹처럼 맞춤이 따라 바뀌고 확대·이동은 그대로
window.MapView?.attach({
  frame: () => ui.mapFrame,
  commit: (v) => { ui.mapView = MapView.relative(v, v.fitFor ? v.fitFor(v.rot) : v); render(); },
  reset: resetMapView,
  drawing: () => !!document.querySelector("svg[data-map2d].wb-drawing"),
});

// 한 화면 배치(WIDE)에서는 2D 지도 그림의 가로를 칸 비율에 맞춘다. 바로 앞에 그린 지도의 실제 크기를 재서 다음 그림에 쓴다.
// (11/17: 800×260 고정이라 낮고 넓은 칸·높은 칸 모두 위아래나 양옆이 비어 지도가 작았다.) 쌓이는 배치는 그림 높이가 가로를 따라가므로 800 그대로
function fitWidth(H) {
  // 한 화면 배치(.live-main)만 칸이 그림 높이를 정한다. 탐색 결과처럼 높이가 그림 비율을 따르는 곳에서 재면 앞 그림 비율이 그대로 굳는다
  const el = matchMedia(WIDE).matches ? document.querySelector(".live-main svg[data-map2d]") : null;
  const r = el?.getBoundingClientRect();
  return r && r.width > 40 && r.height > 40 ? Math.round(H * Math.min(4, Math.max(0.6, r.width / r.height))) : 800;
}

// ---------- 자율 비행 보기: 지도에서 찍은 자리 보내기 (자율 비행 웹 renderGoto와 같은 흐름·말) ----------
const GOAL_STATE = { pending: "비행 노드가 받음", going: "가는 중", arrived: "도착해 둘러보는 중" };
function activeGoal(st) {   // 비행 노드가 받아 기다리거나 가는 중·도착해 둘러보는 찍은 자리
  const g = st.mission?.goal;
  return g && GOAL_STATE[g.state] ? g : null;
}
// 말은 자율 비행 웹 renderGoto와 같다. 찍은 자리에 대한 비행 노드의 마지막 답(받음·거절·그만둠·도착, 게이트웨이 mission.goal_note)은
// 가는 자리가 없을 때 20초 동안 보인다(11/16 실기체: 거절 세 번이 맨 아래 기록에만 한 줄씩 나와 까닭을 몰랐다).
const GOAL_REPLY_S = 20;
function gotoBar(st, canGoto, goal) {
  const p = ui.goto.pick, n = st.mission?.note, reply = st.mission?.goal_note;
  const xy = (q) => (Array.isArray(q) ? `(${q[0].toFixed(1)}, ${q[1].toFixed(1)})` : "");
  const over = !!(n?.search && n.search.elapsed >= n.search.limit);   // 탐색 시간이 다 됨(웹 searchOver)
  const after = !over ? "자율 탐색을 잇습니다" : n.return_home === false ? "그 자리에 내립니다(탐색 시간이 다 됨)" : "출발점으로 돌아갑니다(탐색 시간이 다 됨)";
  const far = goal?.goal && goal.click && Math.hypot(goal.goal[0] - goal.click[0], goal.goal[1] - goal.click[1]) > 0.3;
  let text, warn = false;
  if (p) {
    text = `보낼 자리 (${p.x.toFixed(1)}, ${p.y.toFixed(1)})` + (canGoto ? ` · [여기로 보내기]나 G를 누르면 그리로 갑니다. 둘러본 뒤 ${after}`
      : " · 지금 단계에서는 보낼 수 없습니다(탐색 비행 중에만)");
  } else if (goal?.state === "pending") text = `찍은 자리 ${xy(goal.click)}를 자율 비행이 받았습니다. 하던 동작을 마치면 갑니다`;
  else if (goal?.state === "going") {
    text = `찍은 자리 ${xy(goal.goal || goal.click)}로 가는 중입니다`
      + (far ? ` (찍은 곳 ${xy(goal.click)}까지는 바로 갈 수 없어, 그쪽으로 가장 가까운 빈 자리로 가서 그쪽을 봅니다)` : "");
  } else if (goal?.state === "arrived") text = `찍은 자리에 도착해 둘러보는 중입니다. 마치면 ${after}`;
  else if (reply?.text && reply.age_s < GOAL_REPLY_S) { text = `자율 비행: ${reply.text}`; warn = !!reply.warn; }
  else text = canGoto ? "지도를 한 번 클릭하면 보낼 자리가 찍힙니다(본 곳 밖을 찍으면 그쪽으로 갈 수 있는 데까지 갑니다)"
    // 땅에서는 자율 비행 지도가 아직 없다(비행 노드가 이륙 뒤부터 낸다, 11/17): 빈 지도가 고장처럼 보이지 않게 까닭을 적는다
    : !ui.autonomy.grid && st.mode === "grounded" ? "벽·장애물 지도는 이륙 뒤 자율 비행이 만들기 시작합니다 · 탐색 비행 중에 지도를 한 번 클릭하면 보낼 자리가 찍힙니다"
    : "탐색 비행 중에 지도를 한 번 클릭하면 보낼 자리가 찍힙니다";
  const sendOk = p && canGoto, cancelOff = !p && goal && !st.allowed.includes("goto_cancel");
  const send = `<button data-goto="send" class="${sendOk ? "" : "off"}" ${sendOk ? "" : 'aria-disabled="true"'}>여기로 보내기 <small>G</small></button>`;
  // 웹 .goto와 같이: 점(비행 노드가 받은 찍은 자리가 있으면 칠함)·글·[여기로 보내기]·[지우기/가기 취소](할 것이 없으면 흐림)
  const clearOff = !(p || goal) || cancelOff;
  const clear = `<button data-goto="clear" class="${clearOff ? "off" : ""}" ${clearOff ? 'aria-disabled="true"' : ""}>${!p && goal ? "가기 취소" : "지우기"}</button>`;
  return `<div class="goto-bar" data-part="goto"><span class="goto-dot${!p && goal ? " on" : ""}" aria-hidden="true"></span><span class="${warn ? "goto-warn" : "muted"}" title="${esc(text)}">${esc(text)}</span><span class="spacer"></span>${send}${clear}</div>`;
}
function mapPoint(svg, ev) {   // 클릭한 화면 자리 → 지도 m(돌리고 확대한 보기 그대로 거꾸로)
  const p = mapLocal(svg, ev.clientX, ev.clientY);
  return p ? { x: p.x, y: p.y } : null;
}
function sendGoto() {
  const st = current().st, p = ui.goto.pick;
  if (!p) return toast("여기로 보내기: 먼저 지도를 한 번 클릭해 보낼 자리를 찍으세요");
  if (!st.allowed.includes("goto")) return toast("여기로 보내기: 탐색 비행 중에만 보낼 수 있습니다");
  ui.goto.pick = null;
  sendCmd("goto", { x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 });
  render();
}
function clearGoto() {
  if (ui.goto.pick) { ui.goto.pick = null; return render(); }
  const st = current().st;
  if (!activeGoal(st)) return;
  if (!st.allowed.includes("goto_cancel")) return toast("가기 취소: 지금 상태에서는 할 수 없습니다");
  sendCmd("goto_cancel");
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
  const flown = shown("flown") && route.length > 1 ? `<polyline class="flown cam" points="${route.map((q) => q.join(",")).join(" ")}"/>` : "";
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
  const layers = layerButton("flown", "지나온 길", "지도 파트가 추정한 카메라 경로. 마지막 자리(큰 점)는 늘 보입니다", "cam");
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
  const layers = has ? layerButton("flown", "지나온 길", "지도 파트가 추정한 카메라 경로. 지금 위치(큰 점)는 늘 보입니다", "cam") + map3dOptions() : "";
  return `<section class="panel ${cls}" data-part="map3d" aria-label="3D 지도">${head}
    <div class="map3d-slot"${has ? ` title="${keys}"` : ""}>${has ? "" : `<div class="map-empty">${esc(info[2] || "점군을 기다리는 중입니다.")}</div>`}</div>
    ${mapFoot(info, detail, layers, extra)}${mapSaveBar()}</section>`;
}

// 3D 보기 고르기(11/17 3D 지도가 흐릿하다는 말에): 점 색은 기본 높이 색(낮음 청록 → 높음 주황), 누르면 사진 색.
// 천장(카메라가 지난 가장 높은 곳보다 꽤 위의 점)은 기본으로 빼서 위에서 방 안이 보이게 하고, 누르면 다시 보인다. 지도 데이터는 그대로다.
function map3dOptions() {
  const m = ui.map3d;
  const key = m.photo ? "" : `<span class="key" title="점 색 = 높이(위쪽 방향은 카메라 화면 위쪽 평균이라 대략)"><i class="lg h3d"></i>높이 낮음→높음</span>`;
  return `<button class="layer" data-map3d-opt="photo" aria-pressed="${m.photo}" title="${esc(m.photo ? "사진 색입니다 · 누르면 높이 색(낮음 청록 → 높음 주황)으로 칠합니다" : "높이 색입니다 · 누르면 카메라가 본 사진 색으로 칠합니다")}">사진 색</button>`
    + `<button class="layer" data-map3d-opt="ceiling" aria-pressed="${m.ceiling}" title="${esc(m.ceiling ? "천장까지 다 보입니다 · 누르면 드론이 지난 높이보다 꽤 위(천장·위쪽 튀는 점)를 숨깁니다" : "드론이 지난 높이보다 꽤 위(천장·위쪽 튀는 점)를 숨겨 방 안이 보이게 했습니다 · 누르면 다시 보입니다")}">천장</button>${key}`;
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
  m.view?.setStyle?.({ photo: m.photo, ceiling: m.ceiling });   // 3D 보기 고르기(사진 색·천장)
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
  video_on: "영상 켜기", video_off: "영상 끄기", goto: "여기로 보내기", goto_cancel: "가기 취소",
};
const ROS_CMDS = ["takeoff", "land", "emergency", "confirm_area", "goto", "goto_cancel"];   // 자율 비행 보기에서 보내는 명령(자율 비행 웹과 같은 것)

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
// Esc가 비상 정지를 보낼 때: 떠 있거나 이륙 중. 자율 비행 보기는 상태가 잠깐 끊겨도(connected:false) 바로 앞이 공중이었고
// 게이트웨이가 허락하면(allowed에 emergency, 보기 노드는 닿음) 보낸다(11/16 점검 A2). 땅에서는 보내지 않는다(임무가 끝난다)
const AIR_MODES = ["manual", "auto", "paused", "landing"];
const maybeAirborne = (st) => !!st && (st.flight_busy === "takeoff" || AIR_MODES.includes(st.mode) ||
  (st.source === "ros" && st.mode === "disconnected" && ui.lastAir && st.allowed?.includes("emergency")));
const areaChecked = (st) => (ui.live ? areaConfirmed(st) : ui.takeoff.sampleChecked);
// 자율 비행 보기: 시작을 받은 자율 비행이 이륙하는 중(임무 TAKEOFF). 드론이 답하지 않거나 점검을 못 넘으면 자율 비행이
// 20초 뒤 다시 요청한다(3번까지, 비행 노드 TAKEOFF_RETRY·TAKEOFF_TRIES). 그동안 확인 칸을 다시 열면 확인이 풀려 다시 요청이
// 막히므로 칸은 닫아 두고, 이륙 버튼 자리는 이륙 취소(착륙 명령. 자율 비행 웹도 이때 착륙을 연다)로 바꾼다(11/16 실기체).
// 드론이 바로 거절하면(error) 비행 노드(11/17)는 다시 보내지 않고 시작 대기(READY)로 돌아간다(기수 값이 흐를 때 거절해 왔다).
const rosTakingOff = (st) => st?.source === "ros" && st.mission?.state === "TAKEOFF";
function rosTakeoffWords(st) {   // 땅에서 이륙을 기다리는 동안의 임무 말. 아니면 null
  if (!rosTakingOff(st) || st.mode !== "grounded") return null;
  if (st.flight_busy === "takeoff") return "이륙 중";
  const r = st.ros_view?.driver?.last_result;
  return r?.cmd === "takeoff" && r.ok === false && (r.result?.reason ?? r.reason) !== "error_response"
    ? "드론이 이륙 못 함 · 다시 요청 기다림" : "이륙 요청 중";
}

function takeoffBlocker(st) {   // 지금 이륙할 수 없는 까닭. 이륙할 수 있으면 null
  if (ui.live && !ui.wsOpen) return "게이트웨이에 연결되지 않았습니다";
  if (ui.live && !st.connected) return "드론 상태가 오지 않습니다";
  if (st.source === "ros") return rosStartBlocker(st);
  if (!onGround(st) || !st.allowed.includes("takeoff")) return "지금 상태에서는 이륙할 수 없습니다";
  const failed = (st.preflight || []).filter((c) => c.key !== "operator" && !c.ok && c.blocking !== false);
  if (failed.length) return `기체 점검을 통과하지 못했습니다: ${failed.map((c) => c.title).join(", ")}`;
  if (st.mapping_console_url && !st.video?.available) return "이륙 전에 영상을 켜세요";
  if (!areaChecked(st)) return "주변을 확인하고 체크하세요";
  return null;
}

// 자율 비행 보기: 시작(이륙)을 못 하는 까닭. 자율 비행 웹이 시작 버튼을 끄는 까닭과 같은 순서·같은 뜻이다.
// 게이트웨이가 보기 노드에서 받은 임무 단계·드라이버 점검·YOLO 준비(ros_view.start)를 쓴다. 주변 확인은 이 칸의 체크로 한다.
function rosStartBlocker(st) {
  const s = st.ros_view?.start || {};
  if (!st.ros_view?.commands) return "보기 노드가 명령을 받지 않게 켜져 있습니다(commands:=false)";
  if (!onGround(st)) return "지금 상태에서는 시작할 수 없습니다";
  if (s.mission !== "READY") return !s.mission || s.mission === "WAIT_POSE" ? "자율 비행 노드가 위치를 기다리는 중입니다"
    : `자율 비행이 지금 단계(${MISSION_STATE[s.mission] || s.mission})에서는 시작할 수 없습니다`;
  const failed = (st.preflight || []).filter((c) => c.key !== "operator" && !c.ok && c.blocking !== false);
  if (failed.length) return `기체 점검을 통과하지 못했습니다: ${failed.map((c) => c.title).join(", ")}`;
  if (!s.yolo) return "YOLO 준비 전입니다(사람 탐지 모델을 올리는 데 시작 뒤 20초쯤 걸립니다)";
  if (!areaChecked(st)) return "주변을 확인하고 체크하세요";
  if (!st.allowed.includes("takeoff")) return "지금 상태에서는 시작할 수 없습니다";
  return null;
}
const rosArmed = () => ui.takeoff.armedUntil > Date.now();
// 이륙 버튼은 이륙 전 확인 칸을 여는 버튼이다. 자율 비행 보기는 주변 확인을 칸 안에서 하므로, 확인을 보낼 수 있으면(confirm_area) 칸을 연다.
// 실제 시작(칸의 이륙, 두 번째 누름)은 rosStartBlocker가 자율 비행 웹과 같은 조건으로 막는다.
const canOpenTakeoff = (st) => !rosTakingOff(st) &&
  (st.allowed.includes("takeoff") || (st.source === "ros" && st.allowed.includes("confirm_area")));

// inline: 폰(현장 조작)은 큰 버튼 위에 그대로, PC는 조종 줄 위 오른쪽에 뜬다.
// 자율 비행 보기: 이륙 = 자율 비행 시작(이륙부터 비행 노드가 한다). 자율 비행 웹처럼 한 번 누르면 4초 동안 "한 번 더"로 바뀐다.
// 버튼 글은 짧게 둔다(11/17: 긴 글이 폰 375 px에서 줄을 바꿔 버튼이 화면 아래로 밀려 4초 안에 다시 누르기 어려웠다). 풀이는 옆 안내 글에
function takeoffPanel(st, inline = false) {
  const wrap = (html) => (inline ? `<div data-part="takeoff">${html}</div>` : html);
  if (!ui.takeoff.open || !onGround(st)) return wrap("");
  const ros = st.source === "ros";
  const checked = areaChecked(st), why = takeoffBlocker(st);
  if (why && ros) ui.takeoff.armedUntil = 0;   // 웹처럼: 시작을 못 하게 되면 "한 번 더"를 푼다
  const armed = ros && !why && rosArmed(), drifting = ros && !!st.ros_view?.start?.yaw_drifting;
  const video = !st.mapping_console_url ? "" : `<div class="to-row"><span class="to-name">영상</span>${st.video?.available
    ? chip("✓ 영상 켜짐", "success") : `${chip("✗ 영상 꺼짐", "danger")}<button data-cmd="video_on" class="mini">영상 켜기</button>`}</div>`;
  const deadline = st.mapping_console_url && checked && ui.takeoff.confirmedAt ? ui.takeoff.confirmedAt + AREA_CONFIRM_MS : null;
  const hint = why || (!ui.live ? "샘플 화면: 이륙 명령은 보내지 않습니다"
    : armed ? (drifting ? "4초 안에 한 번 더 누르면 시작을 보냅니다(기수 값이 흐르는 동안 자율 비행은 첫 시작을 받지 않습니다)"
      : "4초 안에 한 번 더 누르면 자율 비행이 이륙합니다")
    : ros ? (drifting ? "이륙 버튼이나 T를 두 번 누르면 시작을 보냅니다" : "이륙 버튼이나 T를 두 번 누르면 자율 비행이 이륙합니다(자율 비행 화면과 같음)")
    : "이륙 버튼이나 T를 누르면 이륙합니다");
  return wrap(`<section class="takeoff-pop ${inline ? "inline" : ""}" role="dialog" aria-label="이륙 전 확인">
    <header><h2>${ros ? "자율 비행 시작 전 확인" : "이륙 전 확인"}</h2><span class="spacer"></span><button data-takeoff="cancel">취소</button></header>
    <div class="to-row"><span class="to-name">기체 점검</span><span class="checks">${preflightChips(st)}</span></div>
    ${ros ? yawRow(st) : ""}
    ${video}
    <button type="button" class="area-check" role="checkbox" aria-checked="${checked}" data-takeoff="area">주변에 사람·장애물이 없고, 프로펠러 가드와 안전 담당을 확인했습니다</button>
    <div class="to-go"><span class="muted">${esc(hint)}${deadline ? `<span data-countdown="${deadline}"></span>` : ""}</span>
      <button class="primary ${why ? "off" : ""} ${armed ? "armed" : ""}" data-takeoff="go" ${why ? 'aria-disabled="true"' : ""}>${armed ? "한 번 더" : "이륙"}</button></div>
  </section>`);
}

// 자율 비행 보기: 땅에서 기수 값이 흐르는지(게이트웨이 ros_view.start.yaw_*, 자율 비행 웹 시작 칸과 같은 잣대: 최근 10초에 2° 이상).
// 흐르는 동안은 드론이 이륙을 거절해 와서 비행 노드가 시작을 받지 않는다(11/17). 막지는 않는다: 30초 안에 한 번 더 시작하면 그대로 요청한다.
function yawRow(st) {
  const s = st.ros_view?.start || {}, r = st.attitude?.yaw_rad;
  const val = Number.isFinite(r) ? `${Math.round(r * 180 / Math.PI)}°` : "값 없음";
  const c = s.yaw_drifting ? chip(`! ${val} · 흐름(10초에 ${Math.round(s.yaw_span_deg)}°)`, "warning")
    : s.yaw_span_deg == null ? chip(`${val} · 확인 중`) : chip(`✓ ${val} · 멈춤`, "success");
  const warn = !s.yaw_drifting ? "" : `<p class="to-warn">기체 기수 값이 땅에서 흐르고 있습니다. 이 상태에서는 드론이 이륙을 거절해 왔습니다 — `
    + "지금은 시작을 눌러도 자율 비행이 받지 않습니다. 기체를 살짝 들었다 평평한 곳에 다시 내려놓고 값이 멈추면 시작하세요"
    + "(그래도 띄워 보려면 30초 안에 시작을 한 번 더).</p>";
  return `<div class="to-row"><span class="to-name">기수 값</span><span class="checks">${c}</span></div>${warn}`;
}

// 비상 정지(0.6초 길게). 자율 비행 보기에서도 같다(보기 노드가 자율 비행 웹처럼 emergency를 낸다). 보기 노드가 명령을 못 받으면 잠근다.
function estopButton(st, wide = false) {
  const cls = `estop${wide ? " wide" : ""}`;
  if (st.source === "ros" && !st.allowed.includes("emergency")) {
    return `<button class="${cls} off" data-cmd="emergency" aria-disabled="true">비상 정지 <small>보기 노드 연결 전</small></button>`;
  }
  return `<button class="${cls}" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>${wide ? "0.6초 길게" : "길게"}</small></button>`;
}

function controlBar(st) {
  const can = (n) => (n === "takeoff" ? canOpenTakeoff(st) : st.allowed.includes(n));
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
    ${!ui.live ? "" : st.source === "ros" ? '<span class="muted keys-hint">자율 비행 보기: T 시작(두 번) · L 착륙 · Esc 비상 정지 · 2D 지도 클릭 뒤 G 보내기</span>'
      : '<span class="muted keys-hint">키보드: WASD 이동 · ↑↓ 높이 · ←→ 회전 · T 이륙 · L 착륙 · Esc 비상 정지</span>'}
    <span class="safety">${grounded && rosTakingOff(st) ? `<button data-cmd="land" class="${can("land") ? "" : "off"}" ${can("land") ? "" : 'aria-disabled="true"'}
        title="자율 비행이 이륙을 그만둡니다(떠오르는 중이면 다 뜬 뒤 바로 내립니다)">이륙 취소</button>`
      : grounded ? btn("takeoff", "", `aria-haspopup="dialog" aria-expanded="${ui.takeoff.open}"`) : btn("land")}
      ${estopButton(st)}</span>
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
    ["개입", st.operator_interventions == null ? "모름" : `${st.operator_interventions}회`],   // 자율 비행 보기는 개입 수를 받지 않는다
    ["임무", rosTakeoffWords(st) || missionWords(st.mission)],   // 알려진 단계 이름은 쉬운 말로, 모르는 코드는 진행 상태(phase)로
    ...(st.mission?.goal?.text ? [["목적지", st.mission.goal.text]] : []),   // 자율 비행 쪽에서 찍은 자리(그쪽 말 그대로)
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
  return `${statusStrip(ctx.st, ctx.events)}${alertBox(ctx.sc, ctx.events, ctx.st)}${board(ctx, livePreset())}${controlBar(ctx.st)}`;
}

function renderMap() {
  const target = consoleTarget(ui.state);
  return `<div class="console-slot" data-part="video-reconstruction">${target.url ? "" : `<p class="map-empty">${esc(target.note)}</p>`}</div>`;
}

function renderReview() {
  const { st, events, track } = current();
  const all = events;
  const count = (r) => all.filter((e) => e.review === r).length;
  const n = st.operator_interventions;   // 자율 비행 보기는 개입 수를 받지 않는다(null: 지어내지 않고 모름)
  const cards = [
    [timeLabel(true), clock(st.flight_time_s)],
    ["탐색 진행", st.mission ? pct(st.mission.progress) : "없음"],
    ["발견", `${all.length}건`, `확인 ${count("confirmed")} · 오탐 ${count("false_positive")} · 미확인 ${count("unreviewed")}`],
    ["자율 판정", !st.autonomy_used ? "자율 안 씀" : n == null ? "개입 모름" : n === 0 ? "개입 없음" : `개입 ${n}회`,
      !st.autonomy_used ? "" : n == null ? "자율 비행 보기는 개입 수를 받지 않음" : n === 0 ? "" : "자율 성공으로 세지 않음"],
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
  : `사람 발견 ${g.count}건 · ${g.first}~${g.last}번${spanText(g)} · 가장 높은 신뢰도 ${g.best.toFixed(2)}`);

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
  const can = (n) => (n === "takeoff" ? canOpenTakeoff(st) : st.allowed.includes(n));
  const big = (n, label) => `<button class="big ${can(n) ? "" : "off"}" data-cmd="${n}">${label}</button>`;
  const grounded = onGround(st);
  // 자율 비행 보기(실행기 autonomy·sim --phone, 11/17): 이륙 = 자율 비행 시작(PC와 같은 확인·두 번 누르기), 착륙·비상 정지.
  // 손 조종(조이스틱·넘겨받기)과 일시정지·복귀는 자율 비행에 없는 명령이라 그리지 않는다. 조이스틱은 칸(data-part)으로 둬서
  // 보기 모드가 바뀌면 화면 전체를 다시 그리고 조이스틱도 다시 묶는다
  const ros = st.source === "ros";
  return `<div class="phone">
    ${statusStrip(st, events, true)}
    ${alertBox(sc, events, st)}
    <div data-part="latest">${latest ? `<div class="alert person slim"><b>최근 발견</b><span>사람 ${latest.confidence.toFixed(2)} · ${findingTime(latest)}</span></div>` : ""}</div>
    <p class="note" data-part="note">${ros ? "자율 비행 보기 화면입니다. 이륙을 누르고 주변을 확인한 뒤 4초 안에 두 번 누르면 자율 비행이 시작합니다(PC 화면과 같음). "
      + "손 조종은 없습니다(자율 비행이 조종). 지도는 실시간 감독 화면에서 봅니다."
      : "현장에서 드론 옆에 두는 화면입니다. 자율 시작은 PC 감독 화면에서만 할 수 있습니다."}</p>
    ${takeoffPanel(st, true)}
    <div class="big-grid${ros ? " solo" : ""}" data-part="big">
      ${ros ? "" : st.mode === "paused" ? big("resume", "다시 시작") : big("pause", "일시정지")}
      ${ros ? "" : big("return_home", "복귀")}
      ${grounded && !rosTakingOff(st) ? big("takeoff", "이륙") : big("land", grounded ? "이륙 취소" : "착륙")}
      ${ros ? "" : big("takeover", "넘겨받기")}
    </div>
    ${ros ? "" : `<div class="sticks" aria-label="조이스틱" data-part="sticks">
      <div class="stick" data-axes="ud,yaw"><span class="knob"></span><small>상하 · 회전</small></div>
      <div class="stick" data-axes="lr,fb"><span class="knob"></span><small>좌우 · 앞뒤</small></div>
    </div>
    <p class="muted center" id="axes" data-part="axes">입력 없음 · 자율 중에 움직이면 넘겨받기</p>`}
    <div data-part="estop">${estopButton(st, true)}</div>
  </div>`;   // 비상 정지도 칸으로: 자율 비행 보기에서는 허용(allowed)에 따라 바뀌는데, 칸 밖이면 처음 그린 "보기 노드 연결 전"에 멈췄다(11/17)
}

// ---------- 그리기: 바뀐 칸만 바꾼다 (영상·조이스틱·길게 누르기가 10Hz 갱신에 끊기지 않게) ----------
function view() {
  const v = (location.hash.replace("#/", "") || "live");
  return ["live", "map", "review", "control"].includes(v) ? v : "live";
}

// 머리줄 연결 표시: 어디에 붙었는지(실기체·가짜 Tello·기록 재생·자율 비행 보기)와 게이트웨이 연결(연결됨·연결 중·끊김).
const SOURCE_NAME = { tello: "실기체", fake: "가짜 Tello", replay: "기록 재생", ros: "자율 비행 보기" };
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
const KEEP_FOCUS = ["takeoff", "feed", "outline", "mapMode", "mapLayer", "map3dOpt", "mapView", "mapHeights", "mapBig", "wbToggle", "wbTool"];

function focusToKeep() {
  if (ui.takeoff.focus) return { sel: `[data-takeoff="${ui.takeoff.focus}"]`, i: 0 };
  const el = document.activeElement, key = el?.dataset && KEEP_FOCUS.find((k) => k in el.dataset);
  if (!key) return null;
  const sel = `[data-${key.replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`)}="${el.dataset[key]}"]`;
  return { sel, i: Math.max(0, [...document.querySelectorAll(sel)].indexOf(el)) };
}

// 누르고 있는 칸(data-part)은 다시 그리지 않는다. 지도 칸은 위치가 올 때마다(초당 약 10번) 바뀌는데, 누르는 사이 칸이
// 바뀌면 클릭이 사라지고(지도 찍기·여기로 보내기) 비상 정지를 0.6초 길게 눌러도 나가지 않았다(11/16 점검 A1).
// 놓으면(클릭 처리가 끝난 뒤) 밀린 것을 그린다. 놓은 것을 못 받으면(창 밖에서 놓음 등) 10초 뒤 푼다.
const pressing = new Map();   // 누른 것(포인터·키) → 그 칸과 바깥 칸 이름들
let pressTimer = null;
function pressParts(el, id) {
  const keys = [];
  for (let p = el?.closest?.("[data-part]"); p; p = p.parentElement?.closest("[data-part]")) keys.push(p.dataset.part);
  if (!keys.length) return;
  pressing.set(id, keys);
  clearTimeout(pressTimer);
  pressTimer = setTimeout(releaseAllParts, 10000);
}
function releaseParts(id) {
  if (pressing.delete(id)) setTimeout(render, 0);
}
function releaseAllParts() {
  if (!pressing.size) return;
  pressing.clear();
  render();
}
document.addEventListener("pointerdown", (ev) => pressParts(ev.target, `pointer${ev.pointerId}`), true);
for (const event of ["pointerup", "pointercancel"]) {
  document.addEventListener(event, (ev) => releaseParts(`pointer${ev.pointerId}`), true);
}

function render() {
  liveSave?.setTarget(ui.live ? consoleTarget(ui.state).origin || null : null);
  syncHeader();
  syncTitle();
  const v = view();
  const html = { live: renderLive, map: renderMap, review: renderReview, control: renderControl }[v]();
  const app = $("#app");
  const tpl = document.createElement("template");
  tpl.innerHTML = html;
  const parts = [...tpl.content.querySelectorAll("[data-part]")];
  const layout = tpl.content.querySelector("[data-layout]")?.dataset.layout ?? "";
  const keys = `${parts.map((p) => p.dataset.part).join(",")}|${layout}`;
  const pressed = new Set([...pressing.values()].flat());
  if (pressed.size && ui.view === v && ui.cache.__keys !== keys) return;   // 칸 배치가 바뀌는 것은 놓은 뒤에 한꺼번에
  const keep = focusToKeep();
  ui.takeoff.focus = null;
  const refocus = () => {
    const all = keep ? document.querySelectorAll(keep.sel) : [];
    const el = all[Math.min(keep?.i ?? 0, all.length - 1)];   // 누른 버튼이 사라졌으면(빈 칸의 "3D 점군 보기") 같은 값의 남은 버튼으로
    if (el && document.activeElement !== el) el.focus({ preventScroll: true });
  };
  if (ui.view !== v || ui.cache.__keys !== keys) {
    if (ui.view && ui.view !== v) window.scrollTo(0, 0);   // 다른 화면으로 가면 맨 위에서 시작한다(같은 화면을 다시 그릴 때·처음 열 때는 스크롤을 그대로 둔다)
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
    refitMap();
    return placeConsole();
  }
  for (const p of parts) {
    const k = p.dataset.part, s = p.outerHTML;
    if (ui.cache[k] === s || pressed.has(k)) continue;   // 누르는 중인 칸은 놓은 뒤에 그린다(캐시를 안 바꿔 다음에 바뀐 것으로 본다)
    ui.cache[k] = s;
    const el = app.querySelector(`[data-part="${k}"]`);
    if (el) el.replaceWith(document.importNode(p, true));
  }
  refocus();
  attachMap3d();
  placeConsole();
  refitMap();
}

// 2D 지도 그림 가로(fitWidth)가 지금 칸 비율과 다르면(처음 그림·창 크기·칸 배치가 바뀜) 한 번 더 그린다.
// 새 상태가 안 와도(땅·기록 끝) 칸에 맞게. 칸 크기는 그림과 상관없이 정해지므로 한 번이면 맞고, 혹시 몰라 0.5초에 한 번만
let refitAt = 0, refitTimer = 0;
function refitMap() {
  requestAnimationFrame(() => {
    const vb = document.querySelector(".live-main svg[data-map2d]")?.viewBox?.baseVal;
    if (!vb?.height || Math.abs(fitWidth(vb.height) - vb.width) <= vb.width * 0.03) return;
    const wait = refitAt + 500 - performance.now();
    if (wait > 0) { clearTimeout(refitTimer); refitTimer = setTimeout(refitMap, wait); return; }   // 창을 끄는 중이면 끝난 뒤 한 번
    refitAt = performance.now();
    render();
  });
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

// 화면을 연 채 게이트웨이만 새 회차로 다시 켰다: 앞 회차 발견·지나온 길·찍은 자리를 화면에서 지운다(발견 번호가 섞이지 않게).
// 앞 회차 것은 그 회차 폴더에 남아 있다. 비행 기록은 시각이 있어 두고 "새 회차" 줄을 넣는다. 새 회차 발견·기록은 다시 읽는다.
function newRun(msg) {
  ui.findings.clear();
  ui.reviewPending = {}; ui.alertFirst = 0; ui.newFindingAt = 0; ui.lastDet = null; ui.selFinding = null;
  Object.assign(ui.autonomy, { track: [], path: null, people: null, pathRev: undefined, peopleRev: undefined });
  ui.goto.pick = null;
  ui.fell = null;
  ui.mapView = { zoom: 1, px: 0, py: 0, rot: 0 };   // 새 회차는 다 보이게
  logLine({ time: hhmmss(msg.at), kind: "mode", text: "게이트웨이 새 회차 · 앞 회차 발견·지나온 길은 화면에서 지움(회차 폴더에 남음)" });
  loadFindings().then(render);
  mapBoard?.reset();   // 지도 그림도 새 회차 것으로
  mapBoard?.load().then(render);
}

function onMessage(msg) {
  if (msg.type === "drone_state") {
    if (ui.state?.run_id && msg.run_id && msg.run_id !== ui.state.run_id) newRun(msg);
    // 끊길 때 한 번만 조종 입력을 푼다. 끊긴 동안 상태마다 풀면 비상 정지 길게 누르기도 계속 취소됐다(11/16 점검 A2)
    if ((ui.state?.connected && !msg.connected) || (ui.state?.mode === "manual" && msg.mode !== "manual") ||
        (msg.mapping_console_url && ui.state?.video?.available && !msg.video?.available)) resetOperator();
    if (msg.mode !== "disconnected") ui.lastAir = AIR_MODES.includes(msg.mode) || msg.flight_busy === "takeoff";
    noteFell(msg);
    // 이륙했거나(확인은 게이트웨이가 이미 씀) 땅이 아니다. 자율 비행 보기는 시작을 받으면 닫는다(다시 요청에 확인이 남아야 한다)
    if (ui.takeoff.open && (!onGround(msg) || rosTakingOff(msg))) closeTakeoff(true);
    ui.state = msg;
    syncAutonomy(msg);
  } else if (msg.type === "result" && !msg.ok) {
    const failed = (msg.failed || []).map((k) => CHECK[k] || k).join(", ");
    toast(`${REASON[msg.reason] || msg.reason}${failed ? `: ${failed}` : ""}`);
  } else if (msg.type === "detection") {
    onDetection(msg);
  } else if (msg.type === "detection_review") {
    onReview(msg);
  } else if (msg.type === "board") {   // 지도 그림(진입로·동선): 그림을 바꾸고, 비행 기록 줄이 되는 것이면 적는다
    const text = mapBoard?.onMessage(msg);
    if (text) logLine({ time: hhmmss(msg.at), kind: "board", text });
  } else if (msg.type === "event") {
    const d = msg.detail || {}, line = eventLine(msg);
    if (!line) return render();
    logLine(line);
    if (msg.kind === "safety_landing") toast(`안전 착륙: ${reasonText(d.reason)}`);
    if (msg.kind === "video_changed" && d.reason === "ground_timeout") toast("지상에서 영상이 오래 켜져 있어 껐습니다 (기체 발열)");
    if (msg.kind === "command_rejected" && d.by === "autonomy_driver") {
      // 알림이 임무 단계(TAKEOFF)보다 먼저 올 수 있어 단계는 보지 않는다. 비행 노드(11/17): 드론이 바로 거절하면(error)
      // 다시 보내지 않고 시작 대기로 돌아간다. 답이 없거나 점검을 못 넘으면 20초 뒤 다시 요청한다(3번까지)
      toast(driverFailText(d) + (d.name !== "takeoff" ? "" : d.reason === "error_response"
        ? " · 자율 비행은 다시 보내지 않고 시작 대기로 돌아갑니다" : " · 자율 비행이 20초 뒤 다시 요청합니다(3번까지)"));
    }
  }
  render();
}

// 자율 비행 노드 이름(/rosout) → 쉬운 말(비행 기록의 경고·오류 줄). 모르는 이름은 그대로 적는다
const NODE_NAME = { tt_mission: "비행 노드", tt_tello_driver: "자율 비행 드라이버", tt_pose: "위치 추정", tt_depth: "깊이 추정",
  tt_detector: "자율 비행 사람 탐지", tt_autonomy_web: "자율 비행 웹" };

// 게이트웨이 높이 지켜보기의 켜짐·풀림(비행 기록 한 줄). 땅·착륙 단계로 끝난 것(end)은 null(적지 않음)
function heightWatchText(d) {
  if (!d.on && d.end) return null;
  if (d.what === "low") {
    return d.on ? `주의: 드론 아래가 가깝습니다(하방 ${d.tof_m == null ? "0.1 m 아래" : meters(d.tof_m)})`
      : `드론 아래 거리가 다시 넉넉합니다(하방 ${meters(d.tof_m)})`;
  }
  return d.on ? `주의: 자율 비행이 높이를 실제보다 ${Math.abs(d.gap_m).toFixed(1)} m ${d.gap_m > 0 ? "높게" : "낮게"} 봄`
    + `(추정 ${meters(d.est_m)} · 기압 ${meters(d.baro_m)} · 하방 ${meters(d.tof_m)})` : "자율 비행 높이 추정이 다시 맞습니다";
}

// 자율 비행 드라이버가 이륙·착륙을 하지 못했다(게이트웨이 ros_view가 드라이버 last_result로 알린다, by: autonomy_driver).
// 드론 응답은 그대로 보인다(Tello는 거절 까닭을 말하지 않고 error만 답한다).
function driverFailText(d) {
  const what = CMD_LABEL[d.name] || d.name;
  if (d.reason === "error_response") return `드론이 ${what}을 거절했습니다(응답 ${d.response || "없음"})`;
  if (d.reason === "timeout") return `드론이 ${what} 명령에 답하지 않았습니다`;
  if (d.reason === "preflight_failed") return `${what} 전 점검을 통과하지 못했습니다${d.failed?.length ? `: ${d.failed.map((k) => CHECK[k] || k).join(", ")}` : ""}`;
  return `${what} 실패: ${REASON[d.reason] || reasonText(d.reason)}`;
}

// 게이트웨이 알림 한 줄(비행 기록). 처음 연 화면이 이번 회차 기록(GET /api/events)을 채울 때도 같은 글을 쓴다.
function eventLine(msg) {
  const d = msg.detail || {};
  if (msg.kind === "cmd_ignored") return null;   // 자율이 auto가 아닐 때 보낸 이동 명령(1초에 한 번까지). 기록에는 남기지 않는다
  const text = msg.kind === "mode_changed" ? `${(MODE[d.from] || [d.from])[0]} → ${(MODE[d.to] || [d.to])[0]} · ${reasonText(d.reason)}`
    : msg.kind === "safety_landing" ? `안전 착륙: ${reasonText(d.reason)}`
    : msg.kind === "video_changed" ? `영상 ${d.on ? "켜짐" : "꺼짐"} (${VIDEO_REASON[d.reason] || d.reason})`
    : msg.kind === "autonomy_connected" ? `자율 연결 (${d.ground_origin ? "지상에서" : "공중에서 붙어 자율 시작 불가"})`
    : msg.kind === "autonomy_disconnected" ? "자율 연결 끊김"
    : msg.kind === "mission_finished" ? `임무 끝: ${d.phase === "arrived" ? "도착" : "실패"}${d.reason ? ` · ${reasonText(d.reason)}` : ""}`
    : msg.kind === "command_rejected" && d.by === "autonomy_driver" ? driverFailText(d)
    : msg.kind === "mission_note" ? missionNoteText(d)
    : msg.kind === "goal_note" ? `자율 비행: ${d.text}`   // 찍은 자리에 대한 비행 노드의 답(받음·거절·그만둠·도착)
    : msg.kind === "height_watch" ? heightWatchText(d)
    : msg.kind === "autonomy_log" ? `${NODE_NAME[d.node] || d.node} ${d.level === "error" ? "오류" : "경고"}: ${d.text}`
    : msg.kind === "command_rejected" ? `${CMD_LABEL[d.name] || d.name} 거절:${REASON[d.reason] || d.reason}${d.detail ? ` · ${reasonText(d.detail)}` : ""}` : msg.kind;
  if (text == null) return null;   // 땅·착륙으로 끝난 높이 알림(end)은 적지 않는다(다시 정상이 된 것이 아니다)
  const kind = msg.kind === "goal_note" ? (d.warn ? "intervention" : "mode")
    : msg.kind === "height_watch" ? (d.on ? "intervention" : "mode")
    : msg.kind === "autonomy_log" ? (d.level === "error" ? "link" : "intervention")   // 오류는 빨강 줄, 경고는 주의 줄
    : msg.kind === "mode_changed" || msg.kind.startsWith("autonomy") || msg.kind === "mission_finished"
    || msg.kind === "mission_note" ? "mode"
    : msg.kind === "video_changed" ? "video" : "intervention";
  return { time: hhmmss(msg.at), kind, text };
}

function onDetection(msg) {
  ui.lastDet = { event: msg.event, latency_ms: msg.latency_ms, at: performance.now() };
  const f = msg.finding, isNew = !ui.findings.has(f.finding_id);
  ui.findings.set(f.finding_id, f);
  if (isNew) {
    const n = findingNumberOf(f.finding_id);
    if (Date.now() - ui.newFindingAt >= PERSON_ALERT_MS) ui.alertFirst = n;   // 알림이 떠 있지 않았으면 새 묶음의 첫 번호
    ui.newFindingAt = Date.now();
    flashPerson();
    beep();
    logFinding(f);
  }
}

function logFinding(f) {   // 바로 앞 줄도 사람 발견이면 그 줄에 묶는다(시각은 마지막 발견, 묶인 동안 시간도 적는다)
  const n = findingNumberOf(f.finding_id), top = ui.timeline[0], conf = f.best.confidence, at = Date.parse(f.first_seen);
  if (top?.group) {
    Object.assign(top.group, { last: n, count: top.group.count + 1, best: Math.max(top.group.best, conf), to: at });
    top.time = hhmmss(f.first_seen);
  } else {
    logLine({ time: hhmmss(f.first_seen), kind: "detection", group: { first: n, last: n, count: 1, best: conf, from: at, to: at } });
  }
}
// 묶인 발견이 걸친 시간(11/17: "사람 발견 8건"이 2분에 걸친 것인데 한 줄이라 순간처럼 보였다)
function spanText(g) {
  const s = Math.round((g.to - g.from) / 1000);
  return !Number.isFinite(s) || s < 5 ? "" : ` · ${s >= 60 ? `${Math.floor(s / 60)}분 ${s % 60}초` : `${s}초`} 동안`;
}

// 화면을 처음 열면 이번 회차 비행 기록을 채운다: 게이트웨이가 모아 둔 알림(GET /api/events)과 받아 둔 발견(처음 본 시각)을
// 시각 순으로 적는다(알림 소리·깜빡임 없이). 그동안 WS로 온 메시지는 잡아 두었다가 뒤에 처리하고, 기록에 이미 있는 알림은 건너뛴다.
async function loadHistory() {
  let events = [];
  try {
    const r = await fetch("/api/events", { cache: "no-store" });
    if (r.ok) events = (await r.json()).events || [];
  } catch (e) { /* 기록을 내주지 않는 게이트웨이: 지금처럼 연 뒤부터 쌓는다 */ }
  const when = (iso) => { const t = Date.parse(iso); return Number.isFinite(t) ? t : 0; };
  const items = [...events.map((m) => ({ t: when(m.at), m })), ...[...ui.findings.values()].map((f) => ({ t: when(f.first_seen), f }))];
  ui.timeline = [];
  for (const x of items.sort((a, b) => a.t - b.t)) {
    if (x.f) logFinding(x.f);
    else if (x.m.type === "detection_review") onReview(x.m);
    else if (x.m.type === "board") { const text = mapBoard?.lineText(x.m); if (text) logLine({ time: hhmmss(x.m.at), kind: "board", text }); }
    else { const line = eventLine(x.m); if (line) logLine(line); }
  }
  return new Set(events.map(historyKey));
}
const historyKey = (m) => `${m.type}|${m.at}|${m.kind || m.finding_id || m.item?.id || ""}|${m.review || m.op || ""}`;

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

// 브라우저 탭 제목에 미확인 사람 발견 수: 다른 창을 보고 있어도 알 수 있게(확인·오탐을 하면 줄어든다).
const BASE_TITLE = document.title;
function syncTitle() {
  const n = ui.live ? liveFindings().filter((e) => e.review === "unreviewed").length : 0;
  const title = n ? `(${n}) 사람 발견 · ${BASE_TITLE}` : BASE_TITLE;
  if (document.title !== title) document.title = title;
}

// 새 사람 발견 때 화면 가장자리를 한 번 분홍으로 깜빡인다(#flash는 다시 그리는 칸 밖에 있어 깜빡임이 끊기지 않는다).
// 움직임 줄이기 설정이면 CSS가 깜빡이지 않는다.
function flashPerson() {
  const el = $("#flash");
  if (!el) return;
  el.classList.remove("on");
  void el.offsetWidth;   // 이어진 발견이면 처음부터 다시
  el.classList.add("on");
}

// 새 사람 발견 소리(위 막대 "소리", 기본 꺼짐, 이 브라우저에 기억). 짧은 두 음. 브라우저는 사람이 화면을 한 번 눌러야 소리를 내게 하므로
// 켤 때와 화면을 누를 때 소리 장치를 깨운다. 소리를 못 내도 알림 띠·탭 제목은 그대로다.
const sound = { on: loadPref("controller.sound", "off") === "on", ctx: null };
function soundCtx() {
  const Ctx = window.AudioContext || window.webkitAudioContext;
  if (!sound.ctx && Ctx) sound.ctx = new Ctx();
  if (sound.ctx?.state === "suspended") sound.ctx.resume().catch(() => {});
  return sound.ctx;
}
function beep() {
  if (!sound.on) return;
  try {
    const ctx = soundCtx(), t = ctx.currentTime;
    [[880, 0], [1175, 0.18]].forEach(([hz, d]) => {
      const o = ctx.createOscillator(), g = ctx.createGain();
      o.frequency.value = hz;
      g.gain.setValueAtTime(0.0001, t + d);
      g.gain.exponentialRampToValueAtTime(0.25, t + d + 0.02);
      g.gain.exponentialRampToValueAtTime(0.0001, t + d + 0.15);
      o.connect(g).connect(ctx.destination);
      o.start(t + d);
      o.stop(t + d + 0.16);
    });
  } catch (e) { /* 소리 장치가 없거나 막힘 */ }
}
function syncSound() {
  const b = $("#sound");
  if (!b) return;
  b.setAttribute("aria-pressed", String(sound.on));
  b.textContent = sound.on ? "소리 켜짐" : "소리 꺼짐";
}
function toggleSound() {
  sound.on = !sound.on;
  savePref("controller.sound", sound.on ? "on" : "off");
  syncSound();
  if (sound.on) beep();   // 켠 것을 바로 들려준다(누른 순간이라 브라우저가 소리를 허용한다)
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
    const first = !ui.everOpen;
    if (ui.everOpen && !ui.wsOpen) logLine({ time: now(), kind: "link", text: "게이트웨이 다시 연결" });   // 끊긴 사이는 기록이 비어 있다
    ui.wsOpen = ui.everOpen = true;
    if (first) ui.hold = [];   // 처음 열 때: 이번 회차 기록을 채우는 동안 온 메시지는 잡아 두었다가 뒤에 처리한다
    let had = new Set();
    try {
      await loadFindings();
      if (first) had = await loadHistory();
      await mapBoard?.load();   // 지도 그림: 기록을 채운 뒤에 받아야 그사이 바뀐 것을 놓치지 않는다
    } finally {
      const held = ui.hold || [];
      ui.hold = null;
      for (const m of held) if (!had.has(historyKey(m))) onMessage(m);
    }
    render();
  };
  ws.onmessage = (ev) => {
    try {
      const m = JSON.parse(ev.data);
      if (ui.hold) ui.hold.push(m); else onMessage(m);
    } catch (e) { /* 잘못된 메시지는 무시 */ }
  };
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
  const o3 = ev.target.closest("[data-map3d-opt]");   // 3D 보기: 사진 색·천장
  if (o3) {
    const k = o3.dataset.map3dOpt;
    ui.map3d[k] = !ui.map3d[k];
    savePref(`map3d-${k}`, ui.map3d[k] ? "1" : "0");
    return render();
  }
  if (ev.target.closest("[data-map-heights]")) {   // 막힌 칸 높이로 나눠 칠하기(우리 것, 기본 꺼짐). 칸 그림을 다시 칠한다
    ui.mapHeights = !ui.mapHeights;
    savePref("controller.map.heights", ui.mapHeights ? "1" : "0");
    return paintGrid().then(render);
  }
  const mv = ev.target.closest("[data-map-view]");   // 2D 지도 돌리기·처음대로 버튼
  if (mv) {
    if (mv.dataset.mapView === "reset") return resetMapView();
    const step = mv.dataset.mapView === "cw" ? 15 : -15, f = ui.mapFrame;
    // 처음 보기(확대·이동 없음)면 돌린 채로 다시 다 보이게, 확대·이동한 보기면 지금 가운데를 축으로 돌린다
    if (MapView.isDefault(ui.mapView) || !f?.fitFor) ui.mapView = { ...ui.mapView, rot: MapView.rotateBy({ rot: ui.mapView.rot }, step).rot };
    else { const turned = MapView.rotateBy(f, step); ui.mapView = MapView.relative(turned, f.fitFor(turned.rot)); }
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
  const pick = ev.target.closest(".det[data-event]");
  if (pick) {   // 발견 카드 고르기: 누른 카드를 강조한다(한 번 더 누르면 풂)
    const id = pick.dataset.event;
    ui.selFinding = ui.selFinding === id ? null : id;
    return render();
  }
  const gb = ev.target.closest("[data-goto]");
  if (gb) return gb.dataset.goto === "send" ? sendGoto() : clearGoto();
  const gm = ev.target.closest("svg[data-goto-map]");
  if (gm) {   // 자율 비행 보기: 2D 지도를 한 번 클릭하면 보낼 자리를 찍는다(아직 보내지 않는다, 자율 비행 웹과 같음).
    // 두 번 클릭(처음대로)과 겹치지 않게 조금 기다렸다 찍는다(자율 비행 웹과 같은 0.28초)
    const p = mapPoint(gm, ev);
    clearTimeout(ui.goto.tapTimer);
    if (p) ui.goto.tapTimer = setTimeout(() => { ui.goto.pick = p; render(); }, 280);
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
      : ui.state?.source === "ros" ? (ROS_CMDS.includes(name) ? "지금 상태에서는 할 수 없습니다" : REASON.ros_view_not_available)
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
  ui.takeoff.armedUntil = 0;
  if (!keepConfirm && areaChecked(current().st)) setArea(false);
  ui.takeoff.confirmedAt = null;
  ui.takeoff.sampleChecked = false;
}
function requestTakeoff() {   // 이륙 버튼·T 키: 칸이 닫혀 있으면 열고, 열려 있으면 준비됐을 때만 이륙(아니면 까닭을 알림)
  if (!ui.takeoff.open) {
    if (rosTakingOff(current().st)) return toast("자율 비행이 이륙하는 중입니다 · 그만두려면 이륙 취소(L)");
    return openTakeoff();
  }
  const st = current().st;
  const why = takeoffBlocker(st);
  if (why) return toast(why);
  if (ui.live && st.source === "ros") {   // 자율 비행 웹과 같은 두 번 누르기: 첫 번째는 4초 동안 "한 번 더"
    if (!rosArmed()) {
      ui.takeoff.armedUntil = Date.now() + 4000;
      setTimeout(() => { if (ui.takeoff.armedUntil && !rosArmed()) { ui.takeoff.armedUntil = 0; render(); } }, 4100);
      return render();
    }
    ui.takeoff.armedUntil = 0;
  }
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
    pressParts(hold, `key${k}`);   // 키로 길게 누르는 동안에도 버튼 칸을 다시 그리지 않는다
    return startHold(hold, k);
  }
  if (!controlPageFocused()) return;
  if (k === "Escape") {
    ev.preventDefault();
    resetOperator();
    if (maybeAirborne(ui.state)) sendCmd("emergency");
    render();
  } else if (k === "KeyT" && keysActive()) {
    ev.preventDefault();
    requestTakeoff();
  } else if (k === "KeyL" && ui.state?.allowed?.includes("land")) {
    ev.preventDefault();
    resetOperator();
    sendCmd("land");
    render();
  } else if (k === "KeyG" && ui.state?.source === "ros" && ui.goto.pick) {   // 자율 비행 보기: 찍은 자리 보내기(자율 비행 웹 G)
    ev.preventDefault();
    sendGoto();
  } else if (AXES[k] && keysActive() && ui.state?.connected && ui.state.mode === "manual") {
    ev.preventDefault();
    held.add(k);
    send({ type: "rc", axes: keyAxes() });
  }
});
document.addEventListener("keyup", (ev) => {
  const k = ev.code;
  releaseParts(`key${k}`);
  if (emergencyHold?.release === k) cancelEmergencyHold();
  if (AXES[k] && held.delete(k)) send({ type: "rc", axes: keyAxes() });
});
const leaveControls = () => { resetOperator(); pressing.clear(); render(); };
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
  $("#sound")?.addEventListener("click", toggleSound);
  syncSound();
  document.addEventListener("pointerdown", () => { if (sound.on) soundCtx(); }, { passive: true });   // 다음 알림 소리를 위해 장치를 깨워 둔다
  const sel = $("#scenario");
  const badge = $("#conn");
  if (ui.live) {
    $("#scenario-wrap").hidden = true;
    badge.classList.replace("sample", ui.state.source === "fake" || ui.state.source === "replay" ? "live" : "real");
    badge.title = ui.state.source === "replay" ? "지난 회차 기록을 재생하는 화면입니다. 명령은 보내지 않습니다."
      : ui.state.source === "ros" ? "자율 비행 프로그램이 드론에 붙어 납니다. 이 화면의 시작·착륙·비상 정지·지도 찍은 자리는 자율 비행 화면과 같은 명령을 보냅니다."
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
  // 주소에 ?preset=을 붙여 열면(실행기가 여는 화면) 그 프리셋을 고른 것과 같다: 프리셋이 정한 지도 보기(2D/3D)로 시작한다.
  // 이 브라우저에 기억한 보기는 그대로 둔다(주소 없이 열면 기억한 보기).
  if (new URLSearchParams(location.search).get("preset")) {
    const mode = livePreset().panels.find((c) => c.id === "map")?.mode;
    if (MAP_MODES[mode]) ui.map3d.mode = mode;
  }
  pick.onchange = () => { choosePreset(pick.value); pick.blur(); };
  window.addEventListener("resize", () => { if (ui.edit && editLocked()) render(); });
  window.addEventListener("hashchange", leaveControls);
  window.addEventListener("resize", placeConsole);
  window.addEventListener("resize", refitMap);   // 2D 지도 그림 비율을 새 칸에(fitWidth)
  render();
})();
