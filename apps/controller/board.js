// 지도 그림(화이트보드 1단계): 2D 자율 지도 칸의 진입로와 사람 동선.
// - 그림은 게이트웨이(board.py, GET·POST /api/board)에 두고 모든 화면이 같이 본다(ws board). 회차 폴더 board.jsonl에 남는다.
// - 동선은 게이트웨이가 자율 비행 지도(벽·장애물) 위에서 계산한다(route_plan.py): 몸 폭 0.6 m, 못 본 곳은 지나가지 않음,
//   길이 없으면 갈 수 있는 데까지와 남은 거리, 걸리는 시간은 조심 걸음 0.5 m/s. 드론 비행 높이 지도라 추정이다.
// - 그림은 드론 명령이 아니다. 드론 보낼 자리 찍기(자율 비행 보기)는 "보기" 도구에서만 된다(그리기 도구일 때 지도 클릭은 여기서 끝).
// - 그리기를 켜 둔 동안 Esc는 그리기 취소다(app.js의 Esc 비상 정지보다 먼저 받는다). 비상 정지는 화면의 비상 정지 버튼으로.
// app.js가 createMapBoard({render, toast, ready, setBig})로 만들고, 2D 자율 지도 칸을 그릴 때 frame()을 부른다.
"use strict";

(function () {
  const TOOLS = [["view", "보기"], ["entry", "진입로"], ["route", "동선"], ["erase", "지우개"]];
  const HIT = 16;   // 지도 위 진입로·출발·사람을 누른 것으로 보는 거리(지도 그림 단위, 가로 800)
  const HINT = {   // 도구 막대 안내(짧게). 자세한 말은 버튼 설명(TIP)
    view: "보기: 지도 클릭은 그리기 전과 같습니다",
    entry: "진입로: 대원이 들어갈 곳을 누르세요",
    route: "동선: 출발 → 도착 순서로 누르세요(사람 원만 누르면 진입 1에서)",
    erase: "지우개: 지울 진입로·동선을 누르세요",
  };
  const TIP = {
    view: "보기: 지도를 누르면 그리기 전과 같습니다(자율 비행 보기는 드론 보낼 자리 찍기)",
    entry: "진입로: 대원이 들어갈 문을 누르면 진입 n을 찍습니다",
    route: "동선: 출발(진입로·출발·아무 곳)을 누르고 도착(사람 원·아무 곳)을 누르면, 게이트웨이가 자율 비행 지도 위에서 사람이 걸을 길을 찾습니다. "
      + "사람 원을 바로 누르면 진입 1(없으면 출발)에서 찾습니다",
    erase: "지우개: 지울 진입로나 동선을 누르면 지웁니다(모든 화면에서)",
  };
  const NO_PATH = { start_blocked: "출발 둘레가 막혔습니다", boxed_in: "출발 둘레가 막혔습니다", start_off_map: "출발이 지도 밖입니다" };
  const NO_MAP = { no_grid: "자율 비행 지도가 아직 없습니다(지도가 오면 계산)", invalid_grid: "자율 비행 지도를 읽지 못했습니다",
                   grid_too_big: "자율 비행 지도가 너무 커 계산하지 않습니다" };
  const FAIL = { no_board: "이 화면에서는 지도 그림을 쓸 수 없습니다", replay_source: "기록 재생 중에는 그릴 수 없습니다",
                 too_many_items: "지도 그림이 너무 많습니다(회차마다 300개)", invalid_item: "그릴 수 없는 자리입니다(지운 진입로이거나 값이 잘못됨)",
                 not_found: "이미 지워진 그림입니다" };
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
  const meters = (v) => `${(+v).toFixed(1)} m`;
  const duration = (s) => { const t = Math.round(+s); return t >= 60 ? `${Math.floor(t / 60)}분${t % 60 ? ` ${t % 60}초` : ""}` : `${t}초`; };
  const typing = (t) => t?.isContentEditable || ["INPUT", "SELECT", "TEXTAREA", "IFRAME"].includes(t?.tagName);
  const round2 = (v) => Math.round(v * 100) / 100;
  const stop = (ev) => { ev.preventDefault(); ev.stopPropagation(); };

  // 동선 결과를 쉬운 말로(목록·설명·비행 기록이 같이 쓴다)
  function resultWords(r) {
    if (!r || r.state === "pending") return "계산 중";
    if (r.state === "ok") return `${meters(r.length_m)} · 조심 걸음 약 ${duration(r.walk_s)}`;
    if (r.state === "partial") return `갈 수 있는 데까지 ${meters(r.length_m)} · ${meters(r.gap_m)} 남음(지도에서 막힘)`;
    if (r.state === "no_path") return `길을 찾지 못함 · ${NO_PATH[r.reason] || r.reason || ""}`;
    return NO_MAP[r.reason] || "자율 비행 지도가 없습니다";
  }
  const warnOf = (r) => !!r && ["partial", "no_path", "no_map"].includes(r.state);

  // 비행 기록 한 줄(게이트웨이가 logged로 보낸 것만): 진입로·동선을 넣고 지운 것, 지도가 바뀌어 동선 상태가 바뀐 것
  function lineText(msg) {
    const it = msg?.item;
    if (!it) return null;
    if (it.kind === "entry") return msg.op === "add" ? `${it.text} 표시` : msg.op === "delete" ? `${it.text} 지움` : null;
    if (it.kind !== "route") return null;
    if (msg.op === "delete") return `동선 ${it.n} 지움`;
    const words = resultWords(it.result);
    return msg.op === "add" ? `동선 ${it.n}: ${it.from.label} → ${it.to.label} · ${words}` : `동선 ${it.n}: 지도가 바뀌어 ${words}`;
  }

  window.createMapBoard = function ({ render, toast, ready, setBig }) {
    const S = { available: false, on: false, tool: "view", rev: 0, items: new Map(), from: null, frame: null, enlarged: false, tmp: 0 };
    const live = () => !!ready?.();
    const client = () => (location.hash.replace("#/", "") === "control" ? "control" : "monitor");
    const list = (kind) => [...S.items.values()].filter((it) => it.kind === kind && !it.pending);

    async function load(again = true) {   // 처음 열 때·다시 연결할 때·새 회차: 전부 받는다(없으면 기록 재생 등이라 그리기 버튼을 감춘다)
      try {
        const r = await fetch("/api/board", { cache: "no-store" });
        if (!r.ok) { S.available = false; S.items.clear(); return; }
        const d = await r.json();
        if (again && S.available && S.rev > (d.rev || 0)) return load(false);   // 받는 사이 ws로 더 새것을 받았다: 한 번 더
        S.available = true;
        S.rev = d.rev || 0;
        S.items = new Map((d.items || []).map((it) => [it.id, it]));
        if (S.from?.type === "entry" && !S.items.has(S.from.id)) S.from = null;
      } catch (e) { /* 다음 연결 때 다시 */ }
    }
    function reset() { S.items.clear(); S.rev = 0; S.from = null; }
    function apply(item) {   // 받은 그림 하나(판 번호가 더 새것일 때만)
      const old = S.items.get(item.id);
      if (old && (old.rev || 0) > (item.rev || 0)) return;
      if (item.deleted) S.items.delete(item.id); else S.items.set(item.id, item);
      if (item.deleted && S.from?.id === item.id) S.from = null;
    }
    function onMessage(msg) {   // ws board. 돌려주는 글은 비행 기록 줄(없으면 null)
      if (!msg?.item) return null;
      if (S.available && msg.rev > S.rev + 1) load().then(render);   // 놓친 바꿈이 있다: 전부 다시 받는다
      apply(msg.item);
      S.rev = Math.max(S.rev, msg.rev || 0);
      return msg.logged ? lineText(msg) : null;
    }

    // ---------- 그리기 ----------
    function toggle() {
      if (!S.on && !live()) return toast("그리기: 게이트웨이에 연결되지 않았습니다");
      S.on = !S.on;
      S.tool = !S.on ? "view" : list("entry").length ? "route" : "entry";   // 처음이면 진입로부터
      S.from = null;
      if (location.hash.replace("#/", "") === "live" || !location.hash) {   // 실시간 감독: 켜면 지도를 크게, 끄면 되돌린다
        if (S.on) S.enlarged = !!setBig?.(true);
        else if (S.enlarged) { setBig?.(false); S.enlarged = false; }
      }
      render();
    }
    function setTool(tool) {
      S.tool = tool;
      S.from = null;
      render();
    }
    function cancel() {   // Esc: 고르던 출발 → 도구 → 그대로(그리기는 버튼·B로 끈다)
      if (S.from) S.from = null;
      else if (S.tool !== "view") S.tool = "view";
      toast("그리기 취소 · 그리기를 켜 둔 동안 Esc는 비상 정지가 아닙니다(비상 정지는 화면 버튼)");
      render();
    }
    async function post(body) {
      const r = await fetch("/api/board", { method: "POST", headers: { "Content-Type": "application/json" },
                                            body: JSON.stringify({ client: client(), ...body }) });
      const out = await r.json().catch(() => ({}));
      if (!r.ok || !out.ok) throw out.reason || String(r.status);
      return out;
    }
    const failText = (why) => `지도 그림: ${FAIL[why] || "저장하지 못했습니다 · 게이트웨이 연결을 확인하고 다시 해 주세요"}`;
    async function add(item, shown) {   // 보내는 동안에도 바로 보이고(흐리게), 못 하면 지우고 알린다
      const tmp = { ...shown, id: `tmp-${++S.tmp}`, pending: true };
      S.items.set(tmp.id, tmp);
      render();
      try {
        const out = await post({ op: "add", item });
        S.items.delete(tmp.id);
        apply(out.item);
        S.rev = Math.max(S.rev, out.rev || 0);
      } catch (why) {
        S.items.delete(tmp.id);
        toast(failText(why));
      }
      render();
    }
    async function remove(id) {
      const it = S.items.get(id);
      if (!it || it.pending) return;
      if (!live()) return toast("지도 그림: 게이트웨이에 연결되지 않았습니다");
      S.items.delete(id);
      if (S.from?.id === id) S.from = null;
      render();
      try {
        const out = await post({ op: "delete", id });
        S.rev = Math.max(S.rev, out.rev || 0);
      } catch (why) {
        if (why !== "not_found") { S.items.set(id, it); toast(failText(why)); }   // 다른 화면이 먼저 지웠으면 그대로 지운 것
      }
      render();
    }
    function addEntry(p) {
      const n = Math.max(0, ...[...S.items.values()].filter((it) => it.kind === "entry").map((it) => it.n || 0)) + 1;
      add({ kind: "entry", pts: [[round2(p.x), round2(p.y)]] }, { kind: "entry", pts: [[p.x, p.y]], text: `진입 ${n}` });
    }
    function addRoute(from, to) {
      const wire = (e) => (e.type === "entry" ? { type: "entry", id: e.id } : e.type === "person"
        ? { type: "person", n: e.n, x: round2(e.x), y: round2(e.y) } : { type: e.type, x: round2(e.x), y: round2(e.y) });
      add({ kind: "route", from: wire(from), to: wire(to) }, { kind: "route", from, to, result: { state: "pending" } });
    }
    function entryPoint(e) { return { type: "entry", id: e.id, x: e.pts[0][0], y: e.pts[0][1], label: e.text }; }
    function defaultFrom() {   // 사람 원을 바로 눌렀을 때: 진입 1(가장 앞 번호), 없으면 출발
      const e = list("entry").sort((a, b) => a.n - b.n)[0];
      if (e) return entryPoint(e);
      const s = S.frame?.start || [0, 0];
      return { type: "start", x: s[0], y: s[1], label: "출발" };
    }
    function near(p, candidates) {   // 누른 자리에서 HIT 안의 가장 가까운 것(지도 그림 단위로 잰다)
      const v = S.frame;
      let best = null, bestD = HIT;
      for (const c of candidates) {
        const d = Math.hypot(+v.X(c.x) - p.sx, +v.Y(c.y) - p.sy);
        if (d <= bestD) { best = c; bestD = d; }
      }
      return best;
    }
    function routeClick(p) {
      const v = S.frame;
      const person = near(p, (v.people || []).filter((q) => !q.gone));
      const goal = person ? { type: "person", n: person.n, x: person.x, y: person.y, label: `사람 ${person.n}` }
        : { type: "point", x: p.x, y: p.y, label: "지정 위치" };
      if (S.from) {
        const from = S.from;
        S.from = null;
        return addRoute(from, goal);
      }
      const entry = near(p, list("entry").map((e) => ({ ...entryPoint(e), e })));
      if (entry) { S.from = entryPoint(entry.e); return render(); }
      if (v.start && near(p, [{ x: v.start[0], y: v.start[1] }])) {
        S.from = { type: "start", x: v.start[0], y: v.start[1], label: "출발" };
        return render();
      }
      if (person) return addRoute(defaultFrom(), goal);
      S.from = { type: "point", x: p.x, y: p.y, label: "찍은 자리" };
      render();
    }
    function point(svg, ev) {   // 누른 화면 자리 → 지도 m와 지도 그림 자리(돌리기 전)
      const v = S.frame;
      if (v?.fromClient) return v.fromClient(svg, ev.clientX, ev.clientY);   // app.js: 확대·이동·회전한 보기를 거꾸로(mapview.js)
      if (!v || !svg.getScreenCTM?.()) return null;
      const pt = svg.createSVGPoint();
      pt.x = ev.clientX;
      pt.y = ev.clientY;
      const s = pt.matrixTransform(svg.getScreenCTM().inverse());
      const x = v.minX + (s.x - (v.W - (v.maxX - v.minX) * v.k) / 2) / v.k;
      const y = v.minY + (v.H - (v.H - (v.maxY - v.minY) * v.k) / 2 - s.y) / v.k;
      return Number.isFinite(x) && Number.isFinite(y) ? { x, y, sx: s.x, sy: s.y } : null;
    }

    // 버튼·지도 클릭은 app.js보다 먼저 받는다(그리기 도구일 때 지도 클릭이 드론 보낼 자리를 찍지 않게)
    document.addEventListener("click", (ev) => {
      const t = ev.target;
      if (!t?.closest) return;
      if (t.closest("[data-wb-toggle]")) { stop(ev); return toggle(); }
      const tool = t.closest("[data-wb-tool]");
      if (tool) { stop(ev); return setTool(tool.dataset.wbTool); }
      const del = t.closest("[data-wb-del]");
      if (del) { stop(ev); return remove(del.dataset.wbDel); }
      const svg = t.closest("svg[data-wb-map]");
      if (!svg || !S.on || S.tool === "view") return;   // 보기: 그리기 전과 같다(app.js가 받는다)
      stop(ev);
      if (!live()) return toast("지도 그림: 게이트웨이에 연결되지 않았습니다");
      const p = point(svg, ev);
      if (!p) return;
      if (S.tool === "entry") return addEntry(p);
      if (S.tool === "route") return routeClick(p);
      const hit = t.closest("[data-wb-id]");
      if (hit) remove(hit.dataset.wbId);
      else toast("지우개: 지울 진입로나 동선을 누르세요");
    }, true);

    // 키: B 그리기 켜고 끄기, 1~4 도구, Esc 그리기 취소(그리기를 켜 둔 동안은 비상 정지로 가지 않는다)
    document.addEventListener("keydown", (ev) => {
      if (ev.ctrlKey || ev.altKey || ev.metaKey || ev.isComposing || typing(ev.target)) return;
      if (ev.code === "KeyB" && !ev.repeat && document.querySelector("[data-wb-toggle]")) { stop(ev); return toggle(); }
      if (!S.on || !document.querySelector("svg[data-wb-map]")) return;   // 그리기 칸이 안 보이면(다른 화면·3D) 지금처럼
      if (ev.code === "Escape") { stop(ev); return cancel(); }
      const digit = /^Digit([1-4])$/.exec(ev.code);
      if (digit && !ev.repeat) { stop(ev); setTool(TOOLS[+digit[1] - 1][0]); }
    }, true);

    // ---------- 그림 ----------
    function arrowHead(P) {   // 마지막 꺾인 점에서 끝으로 향하는 화살촉
      if (P.length < 2) return "";
      const [a, b] = [P[P.length - 2], P[P.length - 1]], th = Math.atan2(b[1] - a[1], b[0] - a[0]);
      const at = (r, d) => `${(b[0] + r * Math.cos(th + d)).toFixed(1)},${(b[1] + r * Math.sin(th + d)).toFixed(1)}`;
      return `<path class="wb-head" d="M${b[0].toFixed(1)},${b[1].toFixed(1)} L${at(11, Math.PI - 0.45)} L${at(11, Math.PI + 0.45)} Z"/>`;
    }
    function routeSvg(it, X, Y) {
      const r = it.result || {}, pts = r.pts || [];
      const tip = `동선${it.n ? ` ${it.n}` : ""} · ${it.from.label} → ${it.to.label} · ${resultWords(r)}`
        + " · 드론 비행 높이(약 1 m) 지도로 계산한 추정 동선입니다. 바닥의 낮은 물체·계단은 지도에 없을 수 있습니다";
      const fx = X(it.from.x), fy = Y(it.from.y), tx = X(it.to.x), ty = Y(it.to.y);
      let body;
      if (it.pending || r.state === "pending") {
        body = `<line class="wb-route pending" x1="${fx}" y1="${fy}" x2="${tx}" y2="${ty}"/>`
          + `<text class="wb-label" x="${(+tx + 10).toFixed(1)}" y="${(+ty - 8).toFixed(1)}">계산 중</text>`;
      } else if (pts.length) {
        const P = pts.map(([x, y]) => [+X(x), +Y(y)]), end = P[P.length - 1];
        const line = P.map((q) => `${q[0].toFixed(1)},${q[1].toFixed(1)}`).join(" ");
        const label = r.state === "ok" ? `${it.to.label} · ${meters(r.length_m)} · 약 ${duration(r.walk_s)}` : `${it.to.label} · ${meters(r.gap_m)} 남음`;
        const gap = (a, b) => `<line class="wb-gap" x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}"/>`;
        body = (r.snap_start_m > 0.05 ? gap([fx, fy], P[0].map((q) => q.toFixed(1))) : "")
          + `<polyline class="wb-route" points="${line}"/>${arrowHead(P)}`
          + (r.state === "partial" ? gap(end.map((q) => q.toFixed(1)), [tx, ty]) : "")
          + `<polyline class="wb-hit" points="${line}"/>`
          + `<text class="wb-label${r.state === "ok" ? "" : " warn"}" x="${(end[0] + 10).toFixed(1)}" y="${(end[1] - 8).toFixed(1)}">${esc(label)}</text>`;
      } else {   // 길 없음·지도 없음: 출발 옆에 까닭(주황 글)
        body = `<circle class="wb-hit-dot" cx="${fx}" cy="${fy}" r="10"/>`
          + `<text class="wb-label warn" x="${(+fx + 10).toFixed(1)}" y="${(+fy + 18).toFixed(1)}">${esc(`동선 ${it.n}: ${r.state === "no_map" ? "지도 없음" : "길 없음"}`)}</text>`;
      }
      return `<g class="wb-item${it.pending ? " pending" : ""}" data-wb-id="${esc(it.id)}"><title>${esc(tip)}</title>${body}</g>`;
    }
    function entrySvg(it, X, Y) {
      const [[x, y]] = it.pts, sx = +X(x), sy = +Y(y);
      return `<g class="wb-item wb-entry${it.pending ? " pending" : ""}" data-wb-id="${esc(it.id)}"><title>${esc(`${it.text} · 진입로(대원이 들어갈 곳)`)}</title>`
        + `<rect x="${(sx - 8).toFixed(1)}" y="${(sy - 8).toFixed(1)}" width="16" height="16" rx="3"/>`
        + `<path d="M${(sx - 4.5).toFixed(1)},${sy.toFixed(1)} h8 M${(sx + 0.5).toFixed(1)},${(sy - 3.5).toFixed(1)} l3.5,3.5 l-3.5,3.5"/>`
        + `<text class="wb-label" x="${(sx + 12).toFixed(1)}" y="${(sy + 4).toFixed(1)}">${esc(it.text)}</text></g>`;
    }
    function layerSvg(v) {
      const X = v.X, Y = v.Y, items = [...S.items.values()];
      const pick = S.on && S.tool === "route" && S.from
        ? `<g class="wb-pick"><title>${esc(`동선 출발: ${S.from.label} · 도착을 누르세요`)}</title><circle cx="${X(S.from.x)}" cy="${Y(S.from.y)}" r="12"/></g>` : "";
      return `<g class="wb${S.on && S.tool === "erase" ? " wb-erase" : ""}">`
        + items.filter((it) => it.kind === "route").map((it) => routeSvg(it, X, Y)).join("")
        + items.filter((it) => it.kind === "entry").map((it) => entrySvg(it, X, Y)).join("") + pick + "</g>";
    }
    function toolbar() {
      const tools = TOOLS.map(([t, label], i) => `<button data-wb-tool="${t}" class="${S.tool === t ? "on" : ""}" aria-pressed="${S.tool === t}" `
        + `title="${esc(TIP[t])} (${i + 1})">${label}<small>${i + 1}</small></button>`).join("");
      const hint = !live() ? "게이트웨이에 연결되지 않아 그릴 수 없습니다"
        : S.tool === "route" && S.from ? `동선: ${S.from.label}에서 출발 · 도착(사람 원·아무 곳)을 누르세요` : HINT[S.tool];   // 출발을 고른 뒤
      return `<div class="wb-bar" role="toolbar" aria-label="지도 그리기 도구"><span class="seg wb-tools">${tools}</span>`
        + `<span class="wb-note">${esc(hint)}</span><span class="wb-esc">Esc는 그리기 취소 · 비상 정지는 화면 버튼</span></div>`;
    }
    function listHtml(hidden) {   // 동선 목록(그리기를 켜면 진입로도, 지우기 버튼과 함께). 지도를 못 누르는 사람도 읽고 지운다
      const routes = list("route"), entries = S.on ? list("entry") : [];
      if ((hidden && !S.on) || (!routes.length && !entries.length)) return "";
      const del = (it, name) => (S.on ? `<button data-wb-del="${esc(it.id)}" aria-label="${esc(`${name} 지우기`)}">지우기</button>` : "");
      const rows = entries.map((it) => `<li><b>${esc(it.text)}</b><span>진입로 (${it.pts[0][0].toFixed(1)}, ${it.pts[0][1].toFixed(1)})</span>${del(it, it.text)}</li>`)
        .concat(routes.map((it) => `<li class="${warnOf(it.result) ? "warn" : ""}"><b>동선 ${it.n}</b>`
          + `<span>${esc(`${it.from.label} → ${it.to.label} · ${resultWords(it.result)}`)}</span>${del(it, `동선 ${it.n}`)}</li>`));
      return `<ol class="wb-list" id="wb-list" aria-label="지도 그림: 진입로와 사람 동선(추정)">${rows.join("")}</ol>`;
    }

    return {
      load, reset, onMessage, lineText,
      // 지도 칸이 넣을 자리(칸 범위를 정하기 전에): 진입로·동선·고르던 출발도 칸 안에 보이게
      points() {
        if (!S.available) return [];
        const out = [];
        for (const it of S.items.values()) {
          if (it.kind === "entry") out.push(it.pts[0]);
          else for (const q of [...(it.result?.pts || []), [it.from.x, it.from.y], [it.to.x, it.to.y]]) out.push(q);
        }
        if (S.from) out.push([S.from.x, S.from.y]);
        return out;
      },
      // 2D 자율 지도 칸이 부른다. v: {X, Y, k, minX, maxX, minY, maxY, W, H, start, people, hidden, fromClient}
      frame(v) {
        S.frame = v;
        if (!S.available) return null;
        const drawing = S.on && S.tool !== "view";
        const button = `<button class="wb-btn${live() ? "" : " off"}" data-wb-toggle aria-pressed="${S.on}" `
          + `title="${esc(S.on ? "그리기를 끕니다(B)" : "지도에 진입로·사람 동선을 그립니다(B). 그림은 모든 화면이 같이 보고, 드론에는 아무것도 보내지 않습니다")}">✎ 그리기</button>`;
        const n = S.items.size;
        // 도구 막대(bar)·목록(list)은 지도 칸이 넓으면 지도 옆 기둥에, 좁으면 지도 위·아래에 둔다(app.js .map-body)
        return {
          head: (head) => head.replace(/<\/header>$/, `${button}</header>`),
          bar: S.on ? toolbar() : "",
          attr: ` data-wb-map="1"${drawing ? ' class="wb-drawing"' : ""}${n ? ' aria-describedby="wb-list"' : ""}`,
          svg: v.hidden && !S.on ? "" : layerSvg(v),
          layer: n ? `<button class="layer" data-map-layer="board" aria-pressed="${!v.hidden}" title="${esc(`진입로·사람 동선(지도 그림) · 누르면 ${v.hidden ? "다시 보입니다" : "숨깁니다"}`)}"><i class="lg wb"></i>그림·동선 ${n}</button>` : "",
          list: listHtml(v.hidden),
        };
      },
    };
  };
})();
