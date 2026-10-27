// 실내 탐색 관제 화면. 서버 연결 전이라 SAMPLE을 그리기만 하고 명령은 보내지 않는다.
// 화면 3개: #/live(PC 실시간 감독), #/review(임무 후 탐색 결과), #/control(폰 현장 조작).
"use strict";

const S = window.SAMPLE;
const $ = (sel, root = document) => root.querySelector(sel);
const ui = { scenario: "person", reviews: {} };  // reviews: event_id → confirmed / false_positive

const MODE = {
  disconnected: ["연결 끊김", "danger"], grounded: ["지상", "neutral"], manual: ["수동 비행", "success"],
  auto: ["자율 탐색", "accent"], paused: ["일시정지", "warning"], landing: ["착륙 중", "warning"], emergency: ["비상 정지", "danger"],
};
const AUTHORITY = { operator: "사람이 조종 중", autonomy: "자율이 조종 중", none: "제자리 대기" };
const LINK = { good: ["통신 좋음", "success"], fair: ["통신 주의", "warning"], weak: ["통신 약함", "warning"], critical: ["통신 위험", "danger"] };
const PHASE = { idle: "대기", running: "진행 중", paused: "멈춤", arrived: "도착", failed: "실패", stopped: "종료" };
const SOURCES = {
  drone: "드론 상태", autonomy: "자율 비행", detect: "사람 탐지",
  map: "지도", video: "드론 영상",
};

const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const pct = (v) => (v == null ? "없음" : `${Math.round(v * 100)}%`);
const clock = (s) => (s == null ? "없음" : `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(Math.round(s % 60)).padStart(2, "0")}`);
const meters = (v) => (v == null ? "없음" : `${v.toFixed(2)}m`);
const chip = (text, tone = "neutral", cls = "") => `<span class="chip ${tone} ${cls}">${esc(text)}</span>`;
const src = (key) => `<span class="src" title="이 칸에 들어올 팀원 작업물">${esc(SOURCES[key])}</span>`;

function current() {
  const sc = S.scenarios[ui.scenario];
  const events = S.events.slice(0, sc.detections).map((e) => ({ ...e, review: ui.reviews[e.event_id] || "unreviewed" }));
  const tNow = sc.state.mode === "grounded" ? 0 : S.track[S.track.length - 1].t_s;
  return { sc, st: sc.state, events: events.reverse(), track: S.track.filter((p) => p.t_s <= tNow) };
}

function poseAt(t_s) {  // 탐지 시각에 가장 가까운 드론 위치. 사람의 위치가 아니다.
  return S.track.reduce((a, b) => (Math.abs(b.t_s - t_s) < Math.abs(a.t_s - t_s) ? b : a));
}

function toast(msg) {
  const el = $("#toast");
  el.textContent = msg;
  el.classList.add("show");
  clearTimeout(toast.t);
  toast.t = setTimeout(() => el.classList.remove("show"), 2200);
}

// ---------- 공통 조각 ----------
function statusStrip(st, events, compact = false) {
  const [mode, tone] = MODE[st.mode];
  const [link, ltone] = LINK[st.link.level];
  const found = events.length;
  return `<section class="strip ${compact ? "compact" : ""}" aria-label="비행 상태">
    <span class="mode ${tone}">${mode}</span>
    <span class="who">${AUTHORITY[st.authority]}</span>
    ${found ? chip(`사람 ${found}건 발견`, "danger") : chip("발견 없음")}
    ${st.mission ? chip(`탐색 ${pct(st.mission.progress)}`) : ""}
    <span class="spacer"></span>
    ${chip(link, ltone)}
    ${chip(`배터리 ${st.battery_pct ?? "없음"}%`, st.battery_pct != null && st.battery_pct < 30 ? "warning" : "neutral")}
    ${compact ? "" : chip(`비행 ${clock(st.flight_time_s)}`)}
  </section>`;
}

function alertBox(sc, events) {
  if (sc.alert === "person" && events[0]) {
    return `<div class="alert danger" role="alert"><b>사람 발견</b><span>신뢰도 ${events[0].confidence.toFixed(2)} · 지도 ${S.events.indexOf(S.events.find((e) => e.event_id === events[0].event_id)) + 1}번 표시. 자동으로 멈추지 않습니다.</span>
      <button data-cmd="pause">여기서 멈추기</button></div>`;
  }
  if (sc.alert === "link") {
    return `<div class="alert warning" role="alert"><b>통신 약함</b><span>더 진행하지 않고 멈췄습니다. 다시 시작하면 연결이 끊길 수 있습니다.</span>
      <button data-cmd="return_home">출발점으로 복귀</button></div>`;
  }
  if (sc.alert === "stale") {
    return `<div class="alert warning" role="alert"><b>자율 응답 지연</b><span>0.5초 넘게 명령이 없어 제자리에서 기다립니다. 2초가 지나면 일시정지합니다.</span></div>`;
  }
  return "";
}

function videoPanel(st, events, big = true) {
  const latest = events[0];
  const box = latest && st.mode !== "grounded" ? (() => {
    const [x1, y1, x2, y2] = latest.bbox_xyxy_px;
    const w = latest.image_width, h = latest.image_height;
    return `<div class="bbox" style="left:${(x1 / w) * 100}%;top:${(y1 / h) * 100}%;width:${((x2 - x1) / w) * 100}%;height:${((y2 - y1) / h) * 100}%">
      <span>사람 ${latest.confidence.toFixed(2)}</span></div>`;
  })() : "";
  const tof = st.front_tof.valid ? `앞 ${meters(st.front_tof.m)}` : "앞 거리 없음";
  return `<section class="panel video ${big ? "" : "small"}" aria-label="드론 영상">
    <header><h2>영상</h2>${src("video")}${chip("탐지 상자: 사람 탐지", "neutral", "src-inline")}</header>
    <div class="frame-wrap"><div class="frame">
      <div class="placeholder">영상 연결 전 · 960×720</div>
      ${box}
      <span class="overlay bl">${tof} · 높이 ${meters(st.height_m)}</span>
      <span class="overlay br">${st.video.available ? `${st.video.fps}fps` : "영상 없음"}</span>
    </div></div>
    ${big ? detailLine(st) : ""}
  </section>`;
}

function detectionCard(e, i, big = false) {
  const t = e.pts_us / 1e6;
  const state = { confirmed: ["확인됨", "success"], false_positive: ["오탐", "neutral"], unreviewed: ["미확인", "warning"] }[e.review];
  const n = S.events.findIndex((x) => x.event_id === e.event_id) + 1;
  return `<article class="det ${big ? "big" : ""} ${e.review}" data-event="${esc(e.event_id)}" tabindex="0">
    <div class="thumb" aria-hidden="true"><span>${n}</span></div>
    <div class="det-body">
      <div class="det-title">사람 · ${e.confidence.toFixed(2)} ${chip(state[0], state[1])}</div>
      <div class="muted">${clock(t)} · ${esc(e.frame_id)}</div>
      ${big ? `<div class="muted">발견 위치: 그때 드론 위치 (${poseAt(t).x_m.toFixed(1)}, ${poseAt(t).y_m.toFixed(1)}) m · 사람 위치는 아직 없음</div>` : ""}
      <div class="det-actions">
        <button data-review="confirmed" ${e.review === "confirmed" ? 'aria-pressed="true"' : ""}>확인</button>
        <button data-review="false_positive" ${e.review === "false_positive" ? 'aria-pressed="true"' : ""}>오탐</button>
      </div>
    </div>
  </article>`;
}

function detectionList(events, big = false) {
  const body = events.length
    ? events.map((e, i) => detectionCard(e, i, big)).join("")
    : `<p class="empty">아직 발견한 사람이 없습니다. 탐지 결과가 오면 최신순으로 쌓입니다.</p>`;
  return `<section class="panel dets ${big ? "big" : ""}" aria-label="발견 목록">
    <header><h2>발견</h2>${src("detect")}</header>
    <div class="det-list">${body}</div>
  </section>`;
}

function mapPanel(st, events, track, tall = false) {
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
  return `<section class="panel map ${tall ? "tall" : ""}" aria-label="2D 지도">
    <header><h2>지도</h2>${src("map")}${chip("경로: 자율 비행", "neutral", "src-inline")}
      <span class="spacer"></span><a class="link" href="http://127.0.0.1:8790/" target="_blank" rel="noopener">3D 지도 열기 (apps/console)</a></header>
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="지나온 경로, 계획 경로, 발견 지점, 현재 드론 위치">
      ${rooms}${path}${planned}${start}${marks}${drone}
    </svg>
    <p class="legend"><i class="lg flown"></i>지나온 경로 <i class="lg planned"></i>계획 경로 <i class="lg mark"></i>발견 지점 = 그때 드론 위치(사람 위치 아님)</p>
  </section>`;
}

const CMD_LABEL = {
  takeoff: "이륙", land: "착륙", pause: "일시정지", resume: "다시 시작", takeover: "넘겨받기",
  auto_start: "자율 시작", auto_stop: "자율 종료", return_home: "복귀", confirm_area: "주변 확인",
};

function controlBar(st) {
  const can = (n) => st.allowed.includes(n);
  const btn = (n, extra = "") => `<button data-cmd="${n}" class="${can(n) ? "" : "off"} ${extra}" ${can(n) ? "" : 'aria-disabled="true"'}>${CMD_LABEL[n]}</button>`;
  const manualOn = st.mode === "manual", autoOn = st.mode === "auto";
  const flying = ["manual", "auto", "paused"].includes(st.mode);
  return `<section class="controls" aria-label="조종">
    <span class="muted">조종</span>
    <div class="seg" role="group" aria-label="수동 자율 전환">
      <button data-cmd="takeover" class="${manualOn ? "on" : ""} ${flying ? "" : "off"}" aria-pressed="${manualOn}">수동</button>
      <button data-cmd="auto_start" class="${autoOn ? "on" : ""} ${can("auto_start") || can("resume") || autoOn ? "" : "off"}" aria-pressed="${autoOn}">자율</button>
    </div>
    ${st.mode === "paused" ? btn("resume") : btn("pause")}
    ${btn("return_home", "soon")}
    ${st.mode === "grounded" ? btn("takeoff") : btn("land")}
    <span class="spacer"></span>
    <button class="estop" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>길게</small></button>
  </section>`;
}

function detailItems(st) {
  return [
    ["온도", st.temp_high_c == null ? "없음" : `${st.temp_high_c}°C`],
    ["하방", st.down_tof.valid ? meters(st.down_tof.m) : "없음"],
    ["전방", st.front_tof.valid ? meters(st.front_tof.m) : "없음"],
    ["상태 수신", `${st.link.state_hz}Hz · ${st.link.rtt_ms}ms`],
    ["개입", `${st.operator_interventions}회`],
    ["미션", st.mission ? `${st.mission.state} · ${PHASE[st.mission.phase]}` : "없음"],
  ];
}

function detailLine(st) {
  return `<p class="detail-line" title="${esc(SOURCES.drone)}">${detailItems(st).map(([k, v]) => `<span>${k} <b>${esc(v)}</b></span>`).join("")}</p>`;
}

// ---------- 화면 ----------
function renderLive() {  // 모니터 한 화면(1366×768 이상)에 스크롤 없이 들어가게 배치한다.
  const { sc, st, events, track } = current();
  return `${statusStrip(st, events)}<div class="alert-slot">${alertBox(sc, events)}</div>
    <div class="live-main">${videoPanel(st, events)}<div class="side">${detectionList(events)}${mapPanel(st, events, track)}</div></div>
    ${controlBar(st)}`;
}

function renderReview() {
  const { st, events, track } = current();
  const all = events;
  const count = (r) => all.filter((e) => e.review === r).length;
  const autonomyOk = st.autonomy_used && st.operator_interventions === 0;
  const cards = [
    ["비행 시간", clock(st.flight_time_s)],
    ["탐색 진행", st.mission ? pct(st.mission.progress) : "없음"],
    ["발견", `${all.length}건`, `확인 ${count("confirmed")} · 오탐 ${count("false_positive")} · 미확인 ${count("unreviewed")}`],
    ["자율 판정", !st.autonomy_used ? "자율 안 씀" : autonomyOk ? "개입 없음" : `개입 ${st.operator_interventions}회`,
      st.autonomy_used && !autonomyOk ? "자율 성공으로 세지 않음" : ""],
  ];
  const tl = S.timeline.filter((x) => x.t_s <= (track[track.length - 1]?.t_s ?? 0));
  return `<section class="summary" aria-label="임무 요약">${cards.map(([k, v, sub]) => `<div class="metric"><span>${k}</span><b>${v}</b>${sub ? `<small>${sub}</small>` : ""}</div>`).join("")}</section>
    <p class="note">임무 후 보고용 화면입니다. 발견마다 확인·오탐을 정하면 요약에 바로 반영됩니다.</p>
    <div class="review-grid">${detectionList(all, true)}${mapPanel(st, all, track, true)}</div>
    <div class="review-grid two">
      <section class="panel" aria-label="3D 지도"><header><h2>3D 지도</h2>${src("map")}</header>
        <div class="map3d"><p>팀 지도 화면(<code>apps/console</code>)의 점군·카메라 경로를 여기에 띄울 자리입니다.</p>
        <p class="muted">지도 화면이 실행 중이면 새 창으로 엽니다. 단위·좌표계는 지도 쪽 표시를 따릅니다.</p>
        <a class="link" href="http://127.0.0.1:8790/" target="_blank" rel="noopener">3D 지도 열기</a></div></section>
      <section class="panel" aria-label="비행 기록"><header><h2>비행 기록</h2>${src("drone")}</header>
        <ol class="timeline">${tl.map((x) => `<li class="${x.kind}"><time>${clock(x.t_s)}</time>${esc(x.text)}</li>`).join("") || '<li class="empty">기록 없음</li>'}</ol></section>
    </div>`;
}

function renderControl() {
  const { st, events } = current();
  const latest = events[0];
  const flying = ["manual", "auto", "paused"].includes(st.mode);
  const big = (n, label) => `<button class="big ${st.allowed.includes(n) ? "" : "off"}" data-cmd="${n}">${label}</button>`;
  return `<div class="phone">
    ${statusStrip(st, events, true)}
    ${latest ? `<div class="alert danger slim"><b>최근 발견</b><span>사람 ${latest.confidence.toFixed(2)} · ${clock(latest.pts_us / 1e6)}</span></div>` : ""}
    <p class="note">현장에서 드론 옆에 두는 화면입니다. 자율 시작은 PC 감독 화면에서만 할 수 있습니다.</p>
    <div class="big-grid">
      ${st.mode === "paused" ? big("resume", "다시 시작") : big("pause", "일시정지")}
      ${big("return_home", "복귀")}
      ${st.mode === "grounded" ? big("takeoff", "이륙") : big("land", "착륙")}
      <button class="big ${flying ? "" : "off"}" data-cmd="takeover">넘겨받기</button>
    </div>
    <div class="sticks" aria-label="조이스틱 (화면 시안)">
      <div class="stick" data-axes="ud,yaw"><span class="knob"></span><small>상하 · 회전</small></div>
      <div class="stick" data-axes="lr,fb"><span class="knob"></span><small>좌우 · 앞뒤</small></div>
    </div>
    <p class="muted center" id="axes">입력 없음 · 자율 중에 움직이면 넘겨받기</p>
    <button class="estop wide" data-hold="emergency" aria-label="비상 정지, 길게 누르기"><span class="fill"></span>비상 정지 <small>0.6초 길게</small></button>
  </div>`;
}

// ---------- 동작 ----------
function view() {
  const v = (location.hash.replace("#/", "") || "live");
  return ["live", "review", "control"].includes(v) ? v : "live";
}

function render() {
  const v = view();
  document.body.dataset.view = v;
  document.querySelectorAll(".views a").forEach((a) => a.setAttribute("aria-current", a.dataset.view === v ? "page" : "false"));
  $("#app").innerHTML = { live: renderLive, review: renderReview, control: renderControl }[v]();
  bindSticks();
}

document.addEventListener("click", (ev) => {
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
  const b = ev.target.closest("[data-cmd]");
  if (!b) return;
  const name = b.dataset.cmd;
  if (b.classList.contains("off")) return toast(`지금은 ${CMD_LABEL[name]}을(를) 할 수 없습니다`);
  if (name === "return_home") return toast("복귀는 자율 쪽과 맞춘 뒤 연결합니다 (화면 자리만)");
  toast(`화면 시안: ${CMD_LABEL[name]} 명령은 서버 연결 뒤 보냅니다`);
});

// 비상 정지는 길게 눌러야 한다(0.6초). 실수로 눌러 공중에서 모터가 멈추지 않게. 마우스·터치·키보드(Space·Enter) 모두.
const HOLD_MS = 600;
function startHold(b, endEvents) {
  if (b.classList.contains("holding")) return;
  b.classList.add("holding");
  const t = setTimeout(() => { b.classList.remove("holding"); toast("화면 시안: 비상 정지는 서버 연결 뒤 보냅니다"); }, HOLD_MS);
  const cancel = () => { clearTimeout(t); b.classList.remove("holding"); };
  endEvents.forEach((name) => b.addEventListener(name, cancel, { once: true }));
}
document.addEventListener("pointerdown", (ev) => {
  const b = ev.target.closest("[data-hold]");
  if (b) startHold(b, ["pointerup", "pointerleave", "pointercancel"]);
});
document.addEventListener("keydown", (ev) => {
  const b = ev.target.closest && ev.target.closest("[data-hold]");
  if (b && (ev.key === " " || ev.key === "Enter") && !ev.repeat) { ev.preventDefault(); startHold(b, ["keyup", "blur"]); }
});

function bindSticks() {
  document.querySelectorAll(".stick").forEach((stick) => {
    const knob = $(".knob", stick);
    const move = (ev) => {
      const r = stick.getBoundingClientRect(), rad = r.width / 2;
      let dx = ev.clientX - (r.left + rad), dy = ev.clientY - (r.top + rad);
      const d = Math.hypot(dx, dy), lim = rad - 22;
      if (d > lim) { dx *= lim / d; dy *= lim / d; }
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
      const [a, b] = stick.dataset.axes.split(",");
      $("#axes").textContent = `${a} ${(dx / lim).toFixed(2)} · ${b} ${(-dy / lim).toFixed(2)} (보내지 않음)`;
    };
    stick.onpointerdown = (ev) => { stick.setPointerCapture(ev.pointerId); move(ev); stick.onpointermove = move; };
    stick.onpointerup = stick.onpointercancel = () => {
      stick.onpointermove = null; knob.style.transform = ""; $("#axes").textContent = "입력 없음 · 손 떼면 제자리";
    };
  });
}

const sel = $("#scenario");
sel.innerHTML = Object.entries(S.scenarios).map(([k, v]) => `<option value="${k}">${v.label}</option>`).join("");
sel.value = ui.scenario;
sel.onchange = () => { ui.scenario = sel.value; render(); };
window.addEventListener("hashchange", render);
render();
