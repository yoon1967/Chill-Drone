// 3D 점군 보기(외부 라이브러리 없이 WebGL). 게이트웨이 GET /api/map(gateway-map/0.1)을 그린다.
// 점(색), 카메라 경로(선), 마지막 카메라 위치(큰 점). 왼쪽 드래그 회전, 오른쪽 드래그 이동, 휠·두 손가락 확대.
// 카메라 경로는 지도 칸의 "지나온 길" 고르기로 숨길 수 있다(setPathVisible). 마지막 위치는 늘 그린다.
// 경로와 마지막 위치는 색 토큰 --map-path(2D 지도 경로와 같은 파랑)를 따르고, 테마가 바뀌면(theme.js) 다시 그린다.
// 사용자가 시점을 건드리기 전까지는 지도가 커질 때마다 전체가 보이게 다시 맞춘다.
// 좌표는 지도 표시 좌표 그대로(단위는 지도 쪽 scale.unit). up은 카메라 화면 위쪽 평균이며 중력 방향이 아니다.
(function () {
  const VS = `attribute vec3 p; attribute vec3 c; uniform mat4 m; uniform float s; varying vec3 v;
    void main() { gl_Position = m * vec4(p, 1.0); gl_PointSize = s; v = c; }`;
  const FS = `precision mediump float; varying vec3 v; uniform vec4 u; uniform float rnd;
    void main() {
      if (rnd > 0.5) { vec2 d = gl_PointCoord - 0.5; if (dot(d, d) > 0.25) discard; }
      gl_FragColor = u.a > 0.0 ? u : vec4(v, 1.0);
    }`;
  const PATH_FALLBACK = [0.12, 0.37, 0.6, 1];   // --map-path를 못 읽을 때(밝은 테마의 파랑)
  const FOV = 0.9;

  // CSS 색 토큰 → WebGL용 [r, g, b, a](0~1). 어떤 CSS 색이든 캔버스가 #rrggbb나 rgba(…)로 바꿔 준다.
  function tokenColor(el, name, fallback) {
    const value = getComputedStyle(el).getPropertyValue(name).trim();
    const ctx = tokenColor.ctx || (tokenColor.ctx = document.createElement("canvas").getContext("2d"));
    if (!value || !ctx) return fallback;
    ctx.fillStyle = "#000000";
    ctx.fillStyle = value;
    const s = ctx.fillStyle, hex = s.match(/^#([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i);
    if (hex) return [parseInt(hex[1], 16) / 255, parseInt(hex[2], 16) / 255, parseInt(hex[3], 16) / 255, 1];
    const n = s.match(/[\d.]+/g);
    return n && n.length >= 3 ? [n[0] / 255, n[1] / 255, n[2] / 255, n.length > 3 ? +n[3] : 1] : fallback;
  }

  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dotv = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

  function viewDirection({yaw, pitch}, {u, a, b}) {
    const horizontal = Math.cos(pitch), vertical = Math.sin(pitch);
    const alongA = horizontal * Math.cos(yaw), alongB = horizontal * Math.sin(yaw);
    return u.map((value, axis) => alongA * a[axis] + alongB * b[axis] + vertical * value);
  }

  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return [f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0];
  }
  function lookAt(eye, target, up) {
    const z = norm(sub(eye, target)), x = norm(cross(up, z)), y = cross(z, x);
    return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dotv(x, eye), -dotv(y, eye), -dotv(z, eye), 1];
  }
  function mul(a, b) {   // 열 우선 4×4
    const o = new Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
    return o;
  }

  class PointView {
    constructor(canvas) {
      this.canvas = canvas;
      const gl = (this.gl = canvas.getContext("webgl", { antialias: true, alpha: true }));
      this.ok = !!gl;
      this.n = 0; this.pathN = 0; this.showPath = true;
      this.view = { yaw: -0.9, pitch: 0.55, dist: 5, target: [0, 0, 0] };
      this.basis = { u: [0, 0, 1], a: [1, 0, 0], b: [0, 1, 0] };
      this.home = null; this.key = null; this.touched = false;
      if (!gl) return;
      const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); return s; };
      const prog = (this.prog = gl.createProgram());
      gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
      gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
      gl.linkProgram(prog);
      this.loc = { p: gl.getAttribLocation(prog, "p"), c: gl.getAttribLocation(prog, "c"),
        m: gl.getUniformLocation(prog, "m"), s: gl.getUniformLocation(prog, "s"),
        u: gl.getUniformLocation(prog, "u"), rnd: gl.getUniformLocation(prog, "rnd") };
      this.buf = { p: gl.createBuffer(), c: gl.createBuffer(), path: gl.createBuffer(), last: gl.createBuffer() };
      this._input();
      new ResizeObserver(() => this.draw()).observe(canvas);
      window.addEventListener("themechange", () => { this.pathColor = null; this.draw(); });
    }

    // 지도 한 버전. 사용자가 시점을 건드렸고 같은 세션·구간이면 그 시점을 유지하고, 아니면 전체가 보이게 맞춘다.
    setData(map) {
      if (!this.ok) return;
      const gl = this.gl, pts = map.points || [], cams = map.cameras || [];
      const pos = new Float32Array(pts.length * 3), col = new Float32Array(pts.length * 3);
      pts.forEach((p, i) => { pos.set(p.slice(0, 3), i * 3); col.set([p[3] / 255, p[4] / 255, p[5] / 255], i * 3); });
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buf.p); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buf.c); gl.bufferData(gl.ARRAY_BUFFER, col, gl.STATIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buf.path); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(cams.flat()), gl.STATIC_DRAW);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.buf.last); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(cams.at(-1) || [0, 0, 0]), gl.STATIC_DRAW);
      this.n = pts.length; this.pathN = cams.length;
      const key = `${map.source?.session_id}|${map.source?.segment_id}`;
      if (key !== this.key || !this.home || !this.touched) {
        if (key !== this.key) this.touched = false;
        this.key = key;
        this._fit(pts, cams, map.up);
      }
      this.draw();
    }

    _fit(pts, cams, up) {
      const u = norm(up || [0, 0, 1]), e = Math.abs(u[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
      const a = norm(sub(e, u.map((v) => v * dotv(e, u)))), b = cross(u, a);
      this.basis = { u, a, b };
      const all = pts.length ? pts : cams;
      if (!all.length) return;
      const med = (k) => all.map((p) => p[k]).sort((x, y) => x - y)[all.length >> 1];
      const target = [med(0), med(1), med(2)];
      const d = all.map((p) => Math.hypot(p[0] - target[0], p[1] - target[1], p[2] - target[2])).sort((x, y) => x - y);
      const r = d[Math.floor(d.length * 0.9)] || 1;   // 튀는 점 10%는 무시하고 맞춘다
      this.home = { yaw: -0.9, pitch: 0.55, dist: Math.max(r * 2.4, 0.5), target };
      this.view = { ...this.home, target: [...target] };
    }

    // 지도 칸 "지나온 길" 고르기. 바뀔 때만 다시 그린다.
    setPathVisible(on) {
      if (this.showPath === on) return;
      this.showPath = on;
      this.draw();
    }

    reset() { if (this.home) { this.touched = false; this.view = { ...this.home, target: [...this.home.target] }; this.draw(); } }

    draw() {
      if (!this.ok || this._raf) return;
      this._raf = requestAnimationFrame(() => { this._raf = 0; this._draw(); });
    }

    _draw() {
      const gl = this.gl, c = this.canvas, dpr = window.devicePixelRatio || 1;
      const w = Math.max(1, Math.round(c.clientWidth * dpr)), h = Math.max(1, Math.round(c.clientHeight * dpr));
      if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
      gl.viewport(0, 0, w, h);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      if (!this.n && !this.pathN) return;
      const { dist, target } = this.view, { u } = this.basis;
      const dir = viewDirection(this.view, this.basis);
      const eye = [0, 1, 2].map((i) => target[i] + dist * dir[i]);
      const m = mul(perspective(FOV, w / h, dist * 0.01, dist * 50), lookAt(eye, target, u));
      gl.useProgram(this.prog);
      gl.enable(gl.DEPTH_TEST);
      gl.uniformMatrix4fv(this.loc.m, false, m);
      const draw = (buf, mode, count, size, color, round) => {
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.enableVertexAttribArray(this.loc.p);
        gl.vertexAttribPointer(this.loc.p, 3, gl.FLOAT, false, 0, 0);
        if (color) { gl.disableVertexAttribArray(this.loc.c); gl.vertexAttrib3f(this.loc.c, 0, 0, 0); }
        else {
          gl.bindBuffer(gl.ARRAY_BUFFER, this.buf.c);
          gl.enableVertexAttribArray(this.loc.c);
          gl.vertexAttribPointer(this.loc.c, 3, gl.FLOAT, false, 0, 0);
        }
        gl.uniform1f(this.loc.s, size * dpr);
        gl.uniform4fv(this.loc.u, color || [0, 0, 0, 0]);
        gl.uniform1f(this.loc.rnd, round ? 1 : 0);
        gl.drawArrays(mode, 0, count);
      };
      const path = this.pathColor || (this.pathColor = tokenColor(this.canvas, "--map-path", PATH_FALLBACK));
      if (this.n) draw(this.buf.p, gl.POINTS, this.n, 2, null, false);
      if (this.showPath && this.pathN > 1) draw(this.buf.path, gl.LINE_STRIP, this.pathN, 1, path, false);
      if (this.pathN) draw(this.buf.last, gl.POINTS, 1, 11, path, true);
    }

    _pan(dx, dy) {
      const backward = viewDirection(this.view, this.basis);
      const right = norm(cross(this.basis.u, backward)), up = cross(backward, right);
      const scale = 2 * this.view.dist * Math.tan(FOV / 2) / Math.max(1, this.canvas.clientHeight);
      this.view.target = this.view.target.map((value, i) => value + scale * (dy * up[i] - dx * right[i]));
    }

    _input() {
      const c = this.canvas, ptrs = new Map();
      let pinch = 0;
      c.addEventListener("contextmenu", (e) => e.preventDefault());
      c.addEventListener("pointerdown", (e) => {
        c.setPointerCapture(e.pointerId);
        ptrs.set(e.pointerId, {x: e.clientX, y: e.clientY, pan: e.button === 2});
        pinch = 0;
        this.touched = true;
      });
      c.addEventListener("pointermove", (e) => {
        const prev = ptrs.get(e.pointerId);
        if (!prev) return;
        const dx = e.clientX - prev.x, dy = e.clientY - prev.y;
        prev.x = e.clientX; prev.y = e.clientY;
        if (ptrs.size === 1) {
          if (prev.pan) this._pan(dx, dy);
          else {
            this.view.yaw -= dx * 0.008;
            this.view.pitch = Math.max(-1.5, Math.min(1.5, this.view.pitch + dy * 0.008));
          }
        } else if (ptrs.size === 2) {
          const [p, q] = [...ptrs.values()], d = Math.hypot(p.x - q.x, p.y - q.y);
          if (pinch) this.view.dist = Math.max(0.05, this.view.dist * (pinch / d));
          pinch = d;
        }
        this.draw();
      });
      const up = (e) => { ptrs.delete(e.pointerId); pinch = 0; };
      c.addEventListener("pointerup", up);
      c.addEventListener("pointercancel", up);
      c.addEventListener("lostpointercapture", up);
      c.addEventListener("wheel", (e) => { e.preventDefault(); this.touched = true; this.view.dist = Math.max(0.05, this.view.dist * Math.exp(e.deltaY * 0.001)); this.draw(); }, { passive: false });
      c.addEventListener("dblclick", () => this.reset());
    }
  }

  window.PointView = PointView;
})();
