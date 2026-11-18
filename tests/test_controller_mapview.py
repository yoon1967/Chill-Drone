"""관제 화면 2D 지도 보기(apps/controller/mapview.js): 확대·축소·이동·회전(11/17 송승우 "2D 지도는 따로 확대 축소 회전").

- 보기 계산(fit·X·Y·toMap·zoomAt·panBy·rotateBy·scaleLen)은 Node로 불러 숫자로 확인한다. Node가 없으면 그 부분은 건너뛴다.
- app.js·board.js·index.html·style.css가 보기를 지도 칸에 끼우는지는 글로 확인한다
  (보낼 자리 찍기와 지도 그림이 돌리고 확대한 보기를 거꾸로 쓰는지, 지도 칸을 자율 비행 웹처럼 정리했는지).
"""
import json
import math
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / 'apps' / 'controller'
MAPVIEW = CONTROLLER / 'mapview.js'
APP = (CONTROLLER / 'app.js').read_text(encoding='utf-8')
BOARD = (CONTROLLER / 'board.js').read_text(encoding='utf-8')
CSS = (CONTROLLER / 'style.css').read_text(encoding='utf-8')
NODE = shutil.which('node')

SCRIPT = r"""
const mv = require(process.argv[1]);
const pts = [[0, 0], [4, -2], [6.5, 3], [-1, 2.2], [2, 5]];
const out = { fits: [] };
for (const rot of [0, 30, 90, -45, 170]) {
  const v = mv.fit(pts, 800, 300, rot);
  // 돌린 그림 자리(SVG rotate(rot W/2 H/2)와 같이)
  const shown = pts.map(([x, y]) => {
    const lx = mv.X(v, x) - v.W / 2, ly = mv.Y(v, y) - v.H / 2, a = rot * Math.PI / 180;
    return [v.W / 2 + lx * Math.cos(a) - ly * Math.sin(a), v.H / 2 + lx * Math.sin(a) + ly * Math.cos(a)];
  });
  out.fits.push({ rot, v, shown, back: pts.map(([x, y]) => mv.toMap(v, mv.X(v, x), mv.Y(v, y))) });
}
const v = mv.fit(pts, 800, 300, 0);
const z = mv.zoomAt(v, 600, 80, 1.2, v.k);
out.zoom = { before: mv.toMap(v, 600, 80), after: mv.toMap(z, 600, 80), k: z.k / v.k };
let far = v;
for (let i = 0; i < 100; i++) far = mv.zoomAt(far, 400, 150, 1.2, v.k);
let near = v;
for (let i = 0; i < 100; i++) near = mv.zoomAt(near, 400, 150, 1 / 1.2, v.k);
out.clamp = [far.k / v.k, near.k / v.k];
const p = mv.panBy(v, 50, -20);
out.pan = [mv.X(p, 1) - mv.X(v, 1), mv.Y(p, 1) - mv.Y(v, 1)];
out.rot = [mv.rotateBy({ rot: 170 }, 20).rot, mv.rotateBy({ rot: -170 }, -20).rot, mv.rotateBy({ rot: 10 }, -15).rot];
out.empty = mv.fit([], 800, 300, 0);
out.tap = [mv.TAP_MOVE, mv.TAP_TIME];
// 자율 비행 웹과 같은 맞춤(둘레 1 m, 가장 좁아도 6 m, 가장자리 16): 점 하나면 6 m가 (300 − 32)에 들어간다
out.web = mv.fit([[2, 1]], 800, 300, 0, { pad: 1, minSpan: 6, px: 16 });
// 맞춘 그림 위의 확대·이동(웹 view): 화면 = 가운데 + zoom·(맞춘 그림 − 가운데) + 이동, 돌린 그림에서도
out.rel = [0, 30, -75].map((rot) => {
  const F = mv.fit(pts, 800, 300, rot, { pad: 1, minSpan: 6, px: 16 }), rel = { zoom: 2.5, px: 40, py: -25, rot };
  const V = mv.absolute(rel, F), shownAt2 = (v, x, y) => {
    const lx = mv.X(v, x) - 400, ly = mv.Y(v, y) - 150, a = v.rot * Math.PI / 180;
    return [400 + lx * Math.cos(a) - ly * Math.sin(a), 150 + lx * Math.sin(a) + ly * Math.cos(a)];
  };
  const f = shownAt2(F, 4, -2), s2 = shownAt2(V, 4, -2);
  return { back: mv.relative(V, F), web: [400 + 2.5 * (f[0] - 400) + 40, 150 + 2.5 * (f[1] - 150) - 25], shown: s2 };
});
out.isDefault = [mv.isDefault({ zoom: 1, px: 0, py: 0 }), mv.isDefault({ zoom: 1.2, px: 0, py: 0 }), mv.isDefault({ zoom: 1, px: 3, py: 0 })];
// 돌린 보기에서 화면 자리 ↔ 지도: fromSvg는 보이는 자리의 지도 점, pinTo는 지도 점을 그 자리로 옮긴 보기
const shownAt = (v, x, y) => {
  const lx = mv.X(v, x) - v.W / 2, ly = mv.Y(v, y) - v.H / 2, a = v.rot * Math.PI / 180;
  return [v.W / 2 + lx * Math.cos(a) - ly * Math.sin(a), v.H / 2 + lx * Math.sin(a) + ly * Math.cos(a)];
};
const r30 = { ...mv.fit(pts, 800, 300, 30), base: 1 };
const s = shownAt(r30, 6.5, 3);
const pinned = mv.pinTo({ ...r30, k: r30.k * 2, rot: 75 }, 6.5, 3, 120, 40);
out.rotated = { back: mv.fromSvg(r30, s[0], s[1]), pinned: shownAt(pinned, 6.5, 3) };
out.wheel = [mv.wheelFactor(-100), mv.wheelFactor(100), mv.wheelFactor(-3, 1), mv.wheelFactor(-5000), mv.wheelFactor(-4)];
console.log(JSON.stringify(out));
"""


@unittest.skipUnless(NODE, 'node가 없어 2D 지도 보기 계산을 확인하지 못함')
class ViewMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        run = subprocess.run([NODE, '-e', SCRIPT, str(MAPVIEW)], capture_output=True, text=True, encoding='utf-8', timeout=30)
        if run.returncode:
            raise AssertionError(run.stderr)
        cls.out = json.loads(run.stdout)

    def test_fit_shows_every_point_at_any_rotation(self):
        for f in self.out['fits']:
            with self.subTest(rot=f['rot']):
                for x, y in f['shown']:
                    self.assertTrue(-1e-6 <= x <= 800 + 1e-6 and -1e-6 <= y <= 300 + 1e-6, (x, y))
                self.assertGreater(f['v']['k'], 0)

    def test_drawing_and_map_round_trip(self):   # 보낼 자리 찍기·지도 그림이 쓰는 거꾸로 계산
        pts = [[0, 0], [4, -2], [6.5, 3], [-1, 2.2], [2, 5]]
        for f in self.out['fits']:
            for (x, y), (bx, by) in zip(pts, f['back']):
                self.assertAlmostEqual(x, bx, places=9)
                self.assertAlmostEqual(y, by, places=9)

    def test_zoom_keeps_the_point_under_the_cursor(self):
        z = self.out['zoom']
        self.assertAlmostEqual(z['before'][0], z['after'][0], places=9)
        self.assertAlmostEqual(z['before'][1], z['after'][1], places=9)
        self.assertAlmostEqual(z['k'], 1.2)

    def test_zoom_is_limited(self):   # 맞춘 배율의 0.5 ~ 30배(자율 비행 웹과 같음)
        far, near = self.out['clamp']
        self.assertAlmostEqual(far, 30)
        self.assertAlmostEqual(near, 0.5)

    def test_fit_like_the_autonomy_web(self):   # 웹 drawMap: 둘레 1 m, 가장 좁아도 6 m, 가장자리 16 화소
        v = self.out['web']
        self.assertAlmostEqual(v['k'], (300 - 32) / 6)
        self.assertAlmostEqual(v['cx'], 2)
        self.assertAlmostEqual(v['cy'], 1)

    def test_zoom_and_pan_sit_on_the_fit_like_the_web(self):   # 새 데이터가 와도 맞춤은 따라 바뀌고 확대·이동은 그대로
        for r in self.out['rel']:
            self.assertAlmostEqual(r['web'][0], r['shown'][0], places=6)
            self.assertAlmostEqual(r['web'][1], r['shown'][1], places=6)
            self.assertAlmostEqual(r['back']['zoom'], 2.5)
            self.assertAlmostEqual(r['back']['px'], 40, places=6)
            self.assertAlmostEqual(r['back']['py'], -25, places=6)
        self.assertEqual(self.out['isDefault'], [True, False, False])

    def test_pan_moves_the_drawing_by_the_drag(self):
        dx, dy = self.out['pan']
        self.assertAlmostEqual(dx, 50)
        self.assertAlmostEqual(dy, -20)

    def test_rotation_wraps(self):
        self.assertEqual(self.out['rot'], [-170, 170, -5])

    def test_empty_map_still_fits(self):
        v = self.out['empty']
        self.assertTrue(all(math.isfinite(v[k]) for k in ('cx', 'cy', 'k')))
        self.assertEqual((v['cx'], v['cy']), (0, 0))

    def test_tap_threshold(self):   # 6 화소보다 적게 움직이고 0.6초 안에 떼면 클릭(보낼 자리 찍기·그리기), 웹과 같음
        self.assertEqual(self.out['tap'], [6, 600])

    def test_rotated_view_keeps_the_point_under_the_hand(self):   # 휠·끌기·두 손가락이 돌린 보기에서도 누른 자리 기준
        back, pinned = self.out['rotated']['back'], self.out['rotated']['pinned']
        self.assertAlmostEqual(back[0], 6.5, places=9)
        self.assertAlmostEqual(back[1], 3, places=9)
        self.assertAlmostEqual(pinned[0], 120, places=6)
        self.assertAlmostEqual(pinned[1], 40, places=6)

    def test_wheel_zoom_follows_the_wheel_amount(self):   # 마우스 한 칸 1.2배, 터치패드처럼 잘게 오면 조금씩, 한 번에 3칸까지
        one_in, one_out, line, huge, small = self.out['wheel']
        self.assertAlmostEqual(one_in, 1.2)
        self.assertAlmostEqual(one_out, 1 / 1.2)
        self.assertAlmostEqual(line, 1.2 ** 0.99)
        self.assertAlmostEqual(huge, 1.2 ** 3)
        self.assertLess(small, 1.01)


def function_body(name):
    start = APP.index(f'\nfunction {name}(')
    return APP[start:APP.index('\n}\n', start)]


class WiringTest(unittest.TestCase):
    def test_script_loads_before_the_app(self):
        html = (CONTROLLER / 'index.html').read_text(encoding='utf-8')
        self.assertLess(html.index('<script src="mapview.js"></script>'), html.index('<script src="app.js"></script>'))

    def test_map_panel_uses_the_view(self):
        panel = function_body('autonomyMapPanel')
        self.assertIn('MapView.fit(pts, W, H, r, wb0 ? { pad: 0, minSpan: 0, px: 16 } : WEB_FIT)', panel)   # 웹과 같은 맞춤(지도 파일이면 그 범위)
        self.assertIn('MapView.absolute(ui.mapView, F)', panel)       # 늘 맞춘 그림 위에 확대·이동(웹 view)
        self.assertIn('<g class="map-view" transform="rotate(', panel)
        self.assertIn('fromClient: mapLocal', panel)              # 지도 그림도 같은 거꾸로 계산
        self.assertIn('data-map-view="reset"', panel)
        self.assertIn('>1 m</text>', panel)                       # 축척은 웹처럼 늘 1 m
        # 눈금 막대는 돌리는 묶음 밖
        self.assertLess(panel.index('${wb?.svg || ""}</g>'), panel.index('${scale}'))
        self.assertIn('MapView.relative(v, v.fitFor ? v.fitFor(v.rot) : v)', APP)   # 놓으면 맞춤 위의 확대·이동으로 남긴다

    def test_clicks_use_the_rotated_view(self):
        self.assertIn('mapLocal(svg, ev.clientX, ev.clientY)', function_body('mapPoint'))
        self.assertIn('querySelector("g.map-view")?.getScreenCTM', function_body('mapLocal'))
        self.assertIn('if (v?.fromClient) return v.fromClient(svg, ev.clientX, ev.clientY);', BOARD)
        self.assertIn('setTimeout(() => { ui.goto.pick = p; render(); }, 280)', APP)   # 두 번 누르기와 겹치지 않게
        self.assertIn('clearTimeout(ui.goto.tapTimer)', function_body('resetMapView'))
        self.assertIn('"mapView"', APP[APP.index('const KEEP_FOCUS'):APP.index('\n', APP.index('const KEEP_FOCUS'))])
        self.assertIn('ui.mapView = { zoom: 1, px: 0, py: 0, rot: 0 }', function_body('newRun'))   # 새 회차는 다 보이게

    def test_gestures_are_not_taken_by_the_browser(self):
        self.assertIn('.map svg[data-map2d]{touch-action:none}', CSS)
        self.assertIn('.map-view text{transform-box:fill-box;transform-origin:center;transform:rotate(calc(var(--map-rot,0) * -1deg))}', CSS)

    def test_map_is_drawn_like_the_autonomy_web(self):   # 11/17 "똑같이 맞추기": autonomy_web/static/index.html drawMap
        panel = function_body('autonomyMapPanel')
        self.assertNotIn('findMark', APP)                        # 발견 지점 네모는 2D 지도에 없다
        self.assertNotIn('.find[data-event]', APP)
        self.assertNotIn('layerButton(', panel)                  # 범례는 웹처럼 고정(켜고 끄기 없음)
        self.assertIn('webLegend(!!occ, people.length > 0, !!truth)', panel)
        legend = function_body('webLegend')
        for word in ('벽', '가구·장애물', '찾은 장애물(지도에 없던 것)', '카메라 지도 막힘', '의심(경로는 피함)', '본 곳',
                     '찾은 사람(길 막음)', '지나간 사람(다시 봤는데 없음)', '계획 경로', '지나온 길(추정)', '드론(추정)', '목적지',
                     '찍은 자리(운용자)'):
            self.assertIn(word, legend)
        self.assertNotIn('unconfirmed', panel)                   # 웹은 미확정 사람을 그리지 않는다
        self.assertNotIn('<text', panel[panel.index('const marks ='):panel.index('const trail')])   # 사람 원에 번호 없음
        # 그리는 차례: 눈금 → 카메라 지도 → 출발 → 찾은 장애물 → 사람 → 지나온 길 → 계획 경로 → 찍은 자리 → 드론
        order = ['<g class="grid">', '${occImage}', '${spawn}', '${foundBoxes}', '${marks}', '${trail}', '${plannedSvg(path, X, Y)}',
                 '${opMarks}', '${pickMark}', '${drone}']
        at = [panel.index(x) for x in order]
        self.assertEqual(at, sorted(at))
        self.assertIn('Math.max(5, 0.25 * k)', panel)            # 사람 반지름(웹)
        self.assertIn('Math.max(9, 0.25 * k)', panel)            # 드론 반지름(웹)
        self.assertIn('PIN_LABEL[goal.state] || "가는 중"', panel) # 찍은 자리 글(받음·가는 중·둘러보는 중)
        self.assertIn('>보낼 자리</text>', panel)
        self.assertIn('class="sigma"', panel)                    # 위치 오차 원
        self.assertIn('ui.mapHeights', panel)                    # 높이로 나눠 칠하기는 고르는 버튼(기본 꺼짐)
        self.assertIn('if (g.heights && ui.mapHeights) return paintHeightGrid(a, g);', APP)
        self.assertIn('loadPref("controller.map.heights", "0") === "1"', APP)
        # 지도 파일(시뮬레이터 세계 지도): 빈 지도가 아니면 그 범위로 맞추고 가구 → 벽 → 출발 → 목적지 차례, 정답 위치는 드론 앞에
        self.assertIn('const wm = a.world, wb0 = wm && !wm.open ? wm.bounds : null', panel)
        world = ['${occImage}', '${worldBoxes}', '${spawn}', '${goalMark}', '${foundBoxes}', '${pickMark}', '${truthMark}', '${drone}']
        at = [panel.index(x) for x in world]
        self.assertEqual(at, sorted(at))
        self.assertIn('(wm.obstacles || []).map((q) => wbox(q, "wobst")).join("") + (wm.walls || []).map((q) => wbox(q, "wwall"))', panel)
        self.assertIn('>목적지</text>', panel)
        self.assertIn('loadWorldMap(s.world_map)', APP)
        self.assertIn('class="plan-pt"', function_body('plannedSvg'))
        self.assertNotIn('plan-end', function_body('plannedSvg'))   # 끝 고리 없이 모든 점에 점


if __name__ == '__main__':
    unittest.main()
