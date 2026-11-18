// 2D 지도 보기: 확대·축소·이동·회전(11/17 송승우 "2D 지도는 따로 확대 축소 회전이 되어야", 자율 비행 웹과 같은 휠·끌기).
// 지도 좌표(x 앞, y 왼쪽, m) → 그림 좌표(SVG viewBox, 돌리기 전): X = W/2 + (x − cx)·k, Y = H/2 − (y − cy)·k.
// 회전은 그림을 담은 <g class="map-view">가 가운데(W/2, H/2)를 축으로 rot°(화면에서 시계 방향 +) 돌린다.
// 자율 비행 웹처럼 확대·이동은 "늘 다 보이게 맞춘 그림"(fit) 위에 얹는다(relative: zoom·px·py, 화면 단위). 새 데이터가 오면
// 맞춤이 따라 바뀌고 확대·이동은 그대로다(11/17 "똑같이 맞추기"). 손동작은 절대 보기(cx·cy·k)로 셈하고 놓을 때 relative로 바꾼다.
// 지도 칸은 0.1초마다 새로 그려지므로 손동작은 문서에 한 번만 걸고, 누르는 동안은 그 칸을 다시 그리지 않는(app.js pressParts)
// 사이에 <g>의 transform만 바꿔 미리 보여 준 뒤, 손을 떼면 새 보기를 넘겨 다시 그린다.
(function (root) {
  "use strict";
  const ZOOM_MIN = 0.5, ZOOM_MAX = 30;    // 맞춘 배율(base)의 몇 배까지(자율 비행 웹과 같음)
  const TAP_MOVE = 6, TAP_TIME = 600;     // 이보다 적게 움직이고 짧게 누르면 클릭(보낼 자리 찍기·그리기), 웹과 같음
  const WHEEL_STEP = 1.2;                 // 마우스 휠 한 칸(deltaY 100)에 1.2배(웹과 같음). 터치패드는 잘게 와서 그만큼 조금씩
  const rad = (d) => (d * Math.PI) / 180;
  const wrapDeg = (d) => (((d + 180) % 360) + 360) % 360 - 180;

  // 점들이 rot°로 돌린 그림 안에 다 들어오게 가운데·배율을 맞춘다(여백 pad m, 가장 좁아도 minSpan m, 가장자리 px 그림 단위)
  function fit(points, W, H, rot = 0, opts = {}) {
    const pad = opts.pad ?? 0.6, minSpan = opts.minSpan ?? 3, px = opts.px ?? 0;
    const pts = points.length ? points : [[0, 0]];
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const [x, y] of pts) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
    const c0x = (x0 + x1) / 2, c0y = (y0 + y1) / 2, cs = Math.cos(rad(rot)), sn = Math.sin(rad(rot));
    let a0 = Infinity, a1 = -Infinity, b0 = Infinity, b1 = -Infinity;
    for (const [x, y] of pts) {   // 그림 방향(u 오른쪽, v 아래)으로 바꾼 뒤 SVG rotate와 같이 돌린다
      const u = x - c0x, v = -(y - c0y), a = u * cs - v * sn, b = u * sn + v * cs;
      a0 = Math.min(a0, a); a1 = Math.max(a1, a); b0 = Math.min(b0, b); b1 = Math.max(b1, b);
    }
    const sa = Math.max(minSpan, a1 - a0 + 2 * pad), sb = Math.max(minSpan, b1 - b0 + 2 * pad);
    const ma = (a0 + a1) / 2, mb = (b0 + b1) / 2, u = ma * cs + mb * sn, v = -ma * sn + mb * cs;
    return { cx: c0x + u, cy: c0y - v, k: Math.min(Math.max(1, W - 2 * px) / sa, Math.max(1, H - 2 * px) / sb), rot: wrapDeg(rot), W, H };
  }
  // 맞춘 보기 F 위의 확대·이동(rel: zoom, 화면 단위 px·py, rot) ↔ 절대 보기(cx·cy·k). 웹 drawMap과 같은 셈:
  // 화면 = 가운데 + zoom·(맞춘 그림 − 가운데) + (px, py). 돌린 그림에서는 이동을 돌리기 전 그림 방향으로 바꿔 더한다
  function absolute(rel, F) {
    const zoom = rel?.zoom ?? 1, px = rel?.px ?? 0, py = rel?.py ?? 0, k = F.k * zoom, a = rad(F.rot);
    const u = px * Math.cos(a) + py * Math.sin(a), v = -px * Math.sin(a) + py * Math.cos(a);
    return { ...F, cx: F.cx - u / k, cy: F.cy + v / k, k };
  }
  function relative(V, F) {
    const u = (F.cx - V.cx) * V.k, v = (V.cy - F.cy) * V.k, a = rad(V.rot);
    return { zoom: V.k / F.k, px: u * Math.cos(a) - v * Math.sin(a), py: u * Math.sin(a) + v * Math.cos(a), rot: wrapDeg(V.rot) };
  }
  const isDefault = (rel) => Math.abs((rel?.zoom ?? 1) - 1) < 1e-6 && Math.abs(rel?.px ?? 0) < 0.5 && Math.abs(rel?.py ?? 0) < 0.5;
  const X = (v, x) => v.W / 2 + (x - v.cx) * v.k;
  const Y = (v, y) => v.H / 2 - (y - v.cy) * v.k;
  const toMap = (v, lx, ly) => [v.cx + (lx - v.W / 2) / v.k, v.cy - (ly - v.H / 2) / v.k];
  // 돌린 뒤 그림 자리(svg 좌표) ↔ 돌리기 전 그림 자리: 가운데를 축으로 rot°
  function unrotate(v, sx, sy) {
    const a = rad(-v.rot), ux = sx - v.W / 2, uy = sy - v.H / 2;
    return [v.W / 2 + ux * Math.cos(a) - uy * Math.sin(a), v.H / 2 + ux * Math.sin(a) + uy * Math.cos(a)];
  }
  const fromSvg = (v, sx, sy) => toMap(v, ...unrotate(v, sx, sy));   // svg 좌표(화면에 보이는 자리) → 지도 m
  // 지도 점 (mx, my)가 svg 좌표 (sx, sy)에 보이도록 가운데를 옮긴 보기(배율·회전은 v 그대로)
  function pinTo(v, mx, my, sx, sy) {
    const [lx, ly] = unrotate(v, sx, sy);
    return { ...v, cx: mx - (lx - v.W / 2) / v.k, cy: my + (ly - v.H / 2) / v.k };
  }
  const clampK = (k, base) => Math.min(base * ZOOM_MAX, Math.max(base * ZOOM_MIN, k));
  // (lx, ly) 그림 자리(돌리기 전) 밑의 지도 점이 그대로 있게 f배(배율은 base의 ZOOM_MIN~ZOOM_MAX배 안)
  function zoomAt(v, lx, ly, f, base = v.k) {
    const k = clampK(v.k * f, base);
    const [mx, my] = toMap(v, lx, ly);
    return { ...v, k, cx: mx - (lx - v.W / 2) / k, cy: my + (ly - v.H / 2) / k };
  }
  const panBy = (v, dlx, dly) => ({ ...v, cx: v.cx - dlx / v.k, cy: v.cy + dly / v.k });
  const rotateBy = (v, d) => ({ ...v, rot: wrapDeg(v.rot + d) });
  // 휠 한 번의 확대 배율: deltaY에 비례(마우스 한 칸 100 → 1.2배, 줄 단위·쪽 단위도 화소로 바꿔 셈, 한 번에 3칸까지)
  function wheelFactor(deltaY, deltaMode = 0) {
    const px = deltaY * (deltaMode === 1 ? 33 : deltaMode === 2 ? 100 : 1);
    return Math.pow(WHEEL_STEP, -Math.max(-300, Math.min(300, px)) / 100);
  }

  // 손동작: 휠·두 손가락 벌리기 = 확대·축소(누른 자리 기준), 끌기 = 이동, Shift+끌기 = 가운데를 축으로 회전,
  // 두 손가락 돌리기 = 손가락 가운데를 축으로 회전, 두 번 클릭 = 처음대로.
  // opts.frame(): 지금 그린 보기 {cx, cy, k, rot, W, H, base} 또는 null, opts.commit(view): 새 보기로 다시 그림,
  // opts.reset(): 처음대로, opts.drawing(): 그리기 도구를 쓰는 중이면 true(두 번 클릭으로 처음대로 가지 않음)
  function attach(opts) {
    const doc = root.document;
    const svgOf = (t) => (t && t.closest ? t.closest("svg[data-map2d]") : null);
    const groupOf = (svg) => svg && svg.querySelector("g.map-view");
    const svgPoint = (m, x, y) => {   // 화면 → svg 좌표: 누른 때의 변환을 계속 쓴다(누르는 동안 칸을 다시 그리지 않아 svg는 그대로)
      const p = new DOMPoint(x, y).matrixTransform(m);
      return [p.x, p.y];
    };
    doc.addEventListener("wheel", (e) => {
      const svg = svgOf(e.target), v = svg && groupOf(svg) && opts.frame(), c = svg && svg.getScreenCTM();
      if (!v || !c) return;
      e.preventDefault();   // 쪽 넘기기·터치패드 확대(ctrl+휠)가 브라우저 화면을 움직이지 않게
      const [sx, sy] = svgPoint(c.inverse(), e.clientX, e.clientY), [mx, my] = fromSvg(v, sx, sy);
      opts.commit(pinTo({ ...v, k: clampK(v.k * wheelFactor(e.deltaY, e.deltaMode), v.base) }, mx, my, sx, sy));
    }, { passive: false });

    // 손동작 하나: drawn 지금 그려 둔 보기(미리 보기의 기준), v 지금 손동작의 출발 보기, m 화면 → svg 변환,
    // ptrs 손가락(id → 화면 자리), start 그 손가락이 이 단계에서 누른 자리, moved 가장 멀리 움직인 화소
    let s = null;
    let eatClick = false;
    function rebase(v) {   // 손가락 수가 바뀌면 지금 보기에서 새로 시작한다
      s.v = v;
      for (const [id, p] of s.ptrs) s.start.set(id, p);
      s.base2 = s.ptrs.size >= 2 ? twoFinger(s.ptrs) : null;
    }
    doc.addEventListener("pointerdown", (e) => {
      const svg = svgOf(e.target), c = svg && groupOf(svg) && svg.getScreenCTM();
      if (!c || (e.pointerType === "mouse" && e.button !== 0)) return;
      if (!s) {
        const v = opts.frame();
        if (!v) return;
        s = { svg, g: groupOf(svg), m: c.inverse(), drawn: v, v, ptrs: new Map(), start: new Map(), moved: 0, shift: e.shiftKey, base2: null,
              t0: Date.now() };
      }
      if (s.svg !== svg) return;
      const now = s.ptrs.size ? preview() : s.v;
      s.ptrs.set(e.pointerId, [e.clientX, e.clientY]);
      try { svg.setPointerCapture(e.pointerId); } catch (_) { /* 이미 놓았으면 그냥 둔다 */ }
      rebase(now);
    });
    function twoFinger(ptrs) {   // 앞 두 손가락 사이 거리·각도·가운데(화면)
      const [a, b] = [...ptrs.values()];
      return { d: Math.hypot(b[0] - a[0], b[1] - a[1]), ang: Math.atan2(b[1] - a[1], b[0] - a[0]), mid: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] };
    }
    // 지금까지의 손동작을 보기로. 누른 자리(두 손가락이면 가운데) 밑의 지도 점이 손을 따라온다
    function preview() {
      const v = s.v;
      if (s.ptrs.size >= 2 && s.base2) {   // 두 손가락: 벌린 만큼 확대, 돌린 만큼 회전, 가운데가 옮긴 만큼 이동
        const now = twoFinger(s.ptrs), b = s.base2;
        const [s0x, s0y] = svgPoint(s.m, b.mid[0], b.mid[1]), [s1x, s1y] = svgPoint(s.m, now.mid[0], now.mid[1]);
        const [mx, my] = fromSvg(v, s0x, s0y);
        const turned = { ...rotateBy(v, ((now.ang - b.ang) * 180) / Math.PI), k: clampK(v.k * (now.d / Math.max(1, b.d)), v.base) };
        return pinTo(turned, mx, my, s1x, s1y);
      }
      const id = [...s.ptrs.keys()][0], p = s.ptrs.get(id), p0 = s.start.get(id);
      if (!p || !p0) return v;
      if (s.shift) {   // Shift+끌기: 그림 가운데를 축으로 돌린다
        const r = s.svg.getBoundingClientRect(), cx = r.left + r.width / 2, cy = r.top + r.height / 2;
        return rotateBy(v, ((Math.atan2(p[1] - cy, p[0] - cx) - Math.atan2(p0[1] - cy, p0[0] - cx)) * 180) / Math.PI);
      }
      const [s0x, s0y] = svgPoint(s.m, p0[0], p0[1]), [s1x, s1y] = svgPoint(s.m, p[0], p[1]);
      const [mx, my] = fromSvg(v, s0x, s0y);
      return pinTo(v, mx, my, s1x, s1y);
    }
    // 미리 보기: 그려 둔 보기(drawn)를 새 보기로 옮기는 변환을 <g> 하나에(다시 그리지 않고)
    function show(v) {
      const a = s.drawn, f = v.k / a.k, W = a.W, H = a.H;
      // 새 그림 좌표 = 가운데 + f·(그려 둔 그림 좌표 − 가운데) + 옮김(그려 둔 보기 가운데의 지도 점이 새로 놓이는 자리)
      const tx = (a.cx - v.cx) * v.k, ty = -(a.cy - v.cy) * v.k;
      s.g.setAttribute("transform", `rotate(${v.rot.toFixed(2)} ${W / 2} ${H / 2}) translate(${(tx + W / 2).toFixed(2)} ${(ty + H / 2).toFixed(2)}) `
        + `scale(${f.toFixed(4)}) translate(${(-W / 2).toFixed(2)} ${(-H / 2).toFixed(2)})`);
      s.g.style.setProperty("--map-rot", v.rot.toFixed(2));
    }
    doc.addEventListener("pointermove", (e) => {
      if (!s || !s.ptrs.has(e.pointerId)) return;
      s.ptrs.set(e.pointerId, [e.clientX, e.clientY]);
      const p0 = s.start.get(e.pointerId);
      s.moved = Math.max(s.moved, Math.hypot(e.clientX - p0[0], e.clientY - p0[1]));
      if (s.moved < TAP_MOVE && s.ptrs.size < 2) return;
      s.svg.classList.add("map-dragging");
      show(preview());
    });
    const end = (e) => {
      if (!s || !s.ptrs.has(e.pointerId)) return;
      const v = preview(), moved = s.moved >= TAP_MOVE || s.ptrs.size >= 2 || s.wasTwo;
      s.ptrs.delete(e.pointerId);
      s.start.delete(e.pointerId);
      if (s.ptrs.size) {   // 두 손가락 중 하나를 뗌: 남은 손가락으로 이어서 이동
        s.wasTwo = true;
        rebase(v);
        return;
      }
      const long = Date.now() - s.t0 > TAP_TIME;   // 오래 누른 것은 클릭이 아니다(웹 TAP_TIME): 보낼 자리를 찍지 않는다
      s.svg.classList.remove("map-dragging");
      s = null;
      if (moved || long) { eatClick = true; setTimeout(() => { eatClick = false; }, 0); }
      if (moved) opts.commit(v);
    };
    doc.addEventListener("pointerup", end);
    doc.addEventListener("pointercancel", end);
    // 끈 뒤 따라오는 클릭은 버린다(끌기를 보낼 자리 찍기·그리기로 받지 않게). 다른 클릭 처리보다 먼저(창 붙잡기 단계)
    root.addEventListener("click", (e) => {
      if (eatClick && svgOf(e.target)) { e.stopImmediatePropagation(); e.preventDefault(); eatClick = false; }
    }, true);
    doc.addEventListener("dblclick", (e) => {
      if (!svgOf(e.target) || (opts.drawing && opts.drawing())) return;
      e.preventDefault();
      opts.reset();
    });
  }

  const api = { fit, absolute, relative, isDefault, X, Y, toMap, unrotate, fromSvg, pinTo, zoomAt, panBy, rotateBy, wheelFactor, attach,
                TAP_MOVE, TAP_TIME, ZOOM_MIN, ZOOM_MAX };
  if (typeof module === "object" && module.exports) module.exports = api;
  root.MapView = api;
})(typeof window !== "undefined" ? window : globalThis);
