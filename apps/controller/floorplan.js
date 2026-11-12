// 평면도: 3D 점군(게이트웨이 GET /api/map, gateway-map/0.1)을 위에서 내려다본 2D 그림(외부 라이브러리 없이).
// 자율 비행 지도가 없을 때 2D 지도 칸이 그린다(app.js floorPlanPanel). 좌표는 지도 파트 것 그대로 쓴다.
// - 바닥 면: up(카메라 화면 위쪽 평균)에 수직인 면. 앞 = 지도 좌표 z(지도를 시작한 카메라가 본 쪽)를 그 면에 눌러 편 방향
//   (z가 위쪽과 거의 같으면 y, 그다음 x), 왼쪽 = 위 × 앞. 원점 = 지도를 시작한 카메라 자리.
//   자율 2D 지도(x 앞·y 왼쪽)와 같은 방향으로 그리므로, 나중에 두 지도를 겹칠 때 축을 다시 바꾸지 않는다.
// - 높이 띠: 카메라(드론)가 지나간 높이 ±0.6 m 안의 점만 누른다(m 단위가 아니면 점 높이 폭의 1/4).
//   바닥·천장 점이 빠져 벽 윤곽이 남는다. 띠 안 점이 전체의 15%보다 적으면 모든 점을 누른다.
// - 칸(m 단위면 5 cm, 아니면 범위의 1/160)마다 점 수를 세어 진하기로 바꾼 그림 한 장(흰색, 투명도 = 진하기)을 만든다.
//   색은 화면이 CSS 토큰(--map-plan)으로 칠한다(그림은 SVG mask로만 쓴다). 그래서 테마가 바뀌어도 다시 만들지 않는다.
// - 크기 단위는 지도 쪽 scale.unit 그대로다. m가 아니면 화면이 눈금을 그리지 않는다.
(function (root) {
  const BAND_M = 0.6, CELL_M = 0.05, CELLS = 160, MAX_CELLS = 512, MIN_KEEP = 0.15, PAD_M = 0.3, PAD = 0.03;

  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
  const finite3 = (v) => Array.isArray(v) && v.length >= 3 && Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
  const quantile = (sorted, q) => sorted[Math.min(sorted.length - 1, Math.max(0, Math.floor(sorted.length * q)))];

  // 지도 좌표 → (앞, 왼쪽, 위) 축. up이 없거나 이상하면 카메라 좌표처럼 y 아래(-y가 위)로 본다.
  function frame(up) {
    const u = norm(finite3(up) && Math.hypot(up[0], up[1], up[2]) > 1e-6 ? up : [0, -1, 0]);
    for (const e of [[0, 0, 1], [0, 1, 0], [1, 0, 0]]) {   // 어떤 위쪽이든 셋 중 하나는 면에 0.8 이상 남는다
      const d = dot(e, u), p = [e[0] - d * u[0], e[1] - d * u[1], e[2] - d * u[2]];
      if (Math.hypot(p[0], p[1], p[2]) >= 0.5) {
        const f = norm(p);
        return { u, f, l: cross(u, f) };
      }
    }
  }

  // 지도 한 버전 → 평면도 숫자(그림 제외). 점이 없으면 null.
  function compute(map) {
    const raw = (map?.points || []).filter(finite3);
    if (!raw.length) return null;
    const { u, f, l } = frame(map.up);
    const metric = map.scale?.unit === "m";
    const P = raw.map((p) => [dot(p, f), dot(p, l), dot(p, u)]);
    const C = (map.cameras || []).filter(finite3).map((c) => [dot(c, f), dot(c, l), dot(c, u)]);
    const hs = P.map((p) => p[2]).sort((a, b) => a - b), ch = C.length ? C.map((c) => c[2]) : [0];
    const half = metric ? BAND_M : Math.max((quantile(hs, 0.95) - quantile(hs, 0.05)) / 4, 1e-6);
    const lo = Math.min(...ch) - half, hi = Math.max(...ch) + half;
    let sel = P.filter((p) => p[2] >= lo && p[2] <= hi), band = [lo, hi];
    if (sel.length < P.length * MIN_KEEP) { sel = P; band = null; }
    // 범위: 튀는 점 1%씩은 빼고, 카메라 경로와 원점은 늘 넣는다.
    const as = sel.map((p) => p[0]).sort((a, b) => a - b), bs = sel.map((p) => p[1]).sort((a, b) => a - b);
    let a0 = Math.min(quantile(as, 0.01), 0, ...C.map((c) => c[0])), a1 = Math.max(quantile(as, 0.99), 0, ...C.map((c) => c[0]));
    let b0 = Math.min(quantile(bs, 0.01), 0, ...C.map((c) => c[1])), b1 = Math.max(quantile(bs, 0.99), 0, ...C.map((c) => c[1]));
    const span = Math.max(a1 - a0, b1 - b0, 1e-6), pad = metric ? PAD_M : span * PAD;
    a0 -= pad; a1 += pad; b0 -= pad; b1 += pad;
    const cell = Math.max(metric ? CELL_M : span / CELLS, (a1 - a0) / MAX_CELLS, (b1 - b0) / MAX_CELLS);
    const nx = Math.max(1, Math.ceil((a1 - a0) / cell)), ny = Math.max(1, Math.ceil((b1 - b0) / cell));
    a1 = a0 + nx * cell; b0 = b1 - ny * cell;   // 그림 한 칸이 딱 cell이 되게 범위를 맞춘다
    const counts = new Uint16Array(nx * ny);
    for (const [a, b] of sel) {
      const i = Math.floor((a - a0) / cell), j = Math.floor((b1 - b) / cell);   // 그림 위쪽 = 왼쪽 끝(b1)
      if (i >= 0 && i < nx && j >= 0 && j < ny && counts[j * nx + i] < 65535) counts[j * nx + i]++;
    }
    const filled = [];
    for (const n of counts) if (n) filled.push(n);
    filled.sort((a, b) => a - b);
    return {
      metric, unit: metric ? "m" : map.scale?.unit || "arbitrary", a0, a1, b0, b1, cell, nx, ny, counts,
      ref: Math.max(1, filled.length ? quantile(filled, 0.95) : 1),   // 이만큼 쌓인 칸부터 가장 진하게
      path: C.map((c) => [c[0], c[1]]), band, half, kept: sel.length, total: P.length,
    };
  }

  // 칸 점 수 → 진하기 0~1. 한 점짜리 칸도 보이게 바닥을 0.25로 둔다.
  function alphaOf(n, ref) {
    return n ? 0.25 + 0.75 * Math.min(1, Math.log1p(n) / Math.log1p(ref)) : 0;
  }

  // 평면도 숫자 → PNG data URL(흰색, 투명도 = 진하기). 브라우저가 아니면 null.
  function image(plan) {
    if (typeof document === "undefined") return null;
    const canvas = document.createElement("canvas");
    canvas.width = plan.nx; canvas.height = plan.ny;
    const ctx = canvas.getContext("2d");
    if (!ctx) return null;
    const img = ctx.createImageData(plan.nx, plan.ny);
    plan.counts.forEach((n, k) => {
      if (!n) return;
      img.data.set([255, 255, 255, Math.round(255 * alphaOf(n, plan.ref))], k * 4);   // 흰색이라 mask가 밝기로 읽어도 같다
    });
    ctx.putImageData(img, 0, 0);
    return canvas.toDataURL("image/png");
  }

  function build(map) {
    const plan = compute(map);
    if (plan) plan.png = image(plan);
    return plan;
  }

  const api = { frame, compute, alphaOf, build, BAND_M };
  if (typeof module === "object" && module.exports) module.exports = api;
  else root.FloorPlan = api;
})(typeof window !== "undefined" ? window : globalThis);
