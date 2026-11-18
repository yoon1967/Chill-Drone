"""관제 화면 지도 그림(apps/controller/board.js, 화이트보드 1단계): 진입로·사람 동선.

- app.js가 2D 자율 지도 칸·ws·비행 기록·새 회차·다시 연결에 board.js를 끼우는지(글로 확인).
- board.js 동작은 Node로 불러 확인한다(document·fetch는 흉내). Node가 없으면 그 부분은 건너뛴다.
  그리기를 켜 둔 동안 Esc는 그리기 취소이고 app.js의 Esc 비상 정지로 넘어가지 않는다(송승우 11/17).
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / 'apps' / 'controller'
APP = (CONTROLLER / 'app.js').read_text(encoding='utf-8')
NODE = shutil.which('node')


def function_body(name, prefix='function'):
    start = APP.index(f'\n{prefix} {name}(')
    return APP[start:APP.index('\n}\n', start)]


class HooksTest(unittest.TestCase):
    def test_board_is_wired_into_the_screen(self):
        html = (CONTROLLER / 'index.html').read_text(encoding='utf-8')
        self.assertLess(html.index('<script src="board.js"></script>'), html.index('<script src="app.js"></script>'))
        panel = function_body('autonomyMapPanel')
        self.assertIn('...(mapBoard?.points() || [])', panel)   # 그림도 칸 안에
        self.assertIn('mapBoard?.frame({', panel)
        self.assertIn('wb ? wb.head(head) : head', panel)
        self.assertIn('${wb?.svg || ""}', panel)
        self.assertIn('${wb?.list || ""}', panel)
        # 도구 막대·지도·목록은 한 몸통에: 넓은 칸이면 지도 옆 기둥(11/17: 위아래로 쌓여 지도가 16 px까지 줄었다)
        self.assertIn('<div class="map-body${side ? " wb-side" : ""}">${wb?.bar || ""}<div class="map-canvas"><svg', panel)
        self.assertLess(panel.index('${wb?.list || ""}</div>'), panel.index('${mapFoot('))
        self.assertIn('W = fitWidth(H)', panel)   # 그림 가로를 칸 비율에 맞춤
        css = (CONTROLLER / 'style.css').read_text(encoding='utf-8')
        self.assertIn('.live-main .map-body.wb-side{display:grid', css)
        self.assertIn('.live-main .map-body.wb-side>.map-canvas{grid-column:1;grid-row:1/3}', css)   # 지도(보기 도구 포함)가 기둥 왼쪽 전체
        self.assertIn('mapBoard?.onMessage(msg)', function_body('onMessage'))
        self.assertIn('mapBoard?.lineText(x.m)', function_body('loadHistory', 'async function'))
        self.assertIn('mapBoard?.reset()', function_body('newRun'))
        onopen = APP[APP.index('ws.onopen = async'):APP.index('ws.onclose')]
        self.assertLess(onopen.index('had = await loadHistory()'), onopen.index('await mapBoard?.load()'))   # 기록 다음에 그림
        self.assertIn('"wbToggle", "wbTool"', APP)   # 다시 그려도 도구 버튼 초점을 지킨다


SCRIPT = r"""
const listeners = {};
const found = new Set();   // document.querySelector가 찾았다고 할 것
global.window = {};
global.location = { hash: "#/live" };
global.document = { addEventListener: (type, fn, capture) => { listeners[type] = { fn, capture }; },
                    querySelector: (sel) => (found.has(sel) ? {} : null) };
let snapshot = { rev: 0, items: [] }, fetches = 0;
global.fetch = async () => { fetches++; return { ok: true, json: async () => snapshot }; };
require(process.argv[1]);
const toasts = [], out = {};
let big = null;
const b = window.createMapBoard({ render() {}, toast: (m) => toasts.push(m), ready: () => true,
  setBig: (on) => { big = on; return true; } });
const ok = { state: "ok", pts: [[0, 0], [2, 0], [4, 1]], length_m: 4.24, walk_s: 8.5, gap_m: null, snap_start_m: 0 };
const route = { id: "b-0002", kind: "route", n: 1, rev: 2, from: { type: "entry", id: "b-0001", x: 0, y: 0, label: "진입 1" },
  to: { type: "person", n: 2, x: 4.2, y: 1.1, label: "사람 2" }, result: ok };
const entry = { id: "b-0001", kind: "entry", n: 1, rev: 1, pts: [[0, 0]], text: "진입 1" };
const event = (target) => { const e = { target, prevented: false, stopped: false, code: "",
  preventDefault() { this.prevented = true; }, stopPropagation() { this.stopped = true; } }; return e; };
const el = (attrs) => ({ dataset: attrs.dataset || {}, closest: (sel) => (attrs.match && attrs.match(sel) ? attrs : null) });

(async () => {
  snapshot = { rev: 2, items: [entry, route] };
  await b.load();
  const X = (x) => (x * 10 + 100).toFixed(1), Y = (y) => (200 - y * 10).toFixed(1);
  const v = { X, Y, k: 10, minX: -10, maxX: 70, minY: -6, maxY: 20, W: 800, H: 260, start: [0, 0], hidden: false,
              people: [{ n: 2, x: 4.2, y: 1.1, gone: false }] };
  let f = b.frame(v);
  out.head = f.head('<header><h2>지도</h2></header>');
  out.attr = f.attr; out.svg = f.svg; out.layer = f.layer; out.list = f.list; out.points = b.points().length; out.barOff = f.bar;
  out.lines = [
    b.lineText({ op: "add", item: entry }), b.lineText({ op: "add", item: route }),
    b.lineText({ op: "route", item: { ...route, result: { state: "partial", pts: [[0, 0]], length_m: 3.0, gap_m: 1.34 } } }),
    b.lineText({ op: "add", item: { ...route, result: { state: "no_map", reason: "no_grid", pts: [] } } }),
    b.lineText({ op: "add", item: { ...route, result: { state: "ok", pts: [[0, 0], [30, 0]], length_m: 30, walk_s: 60 } } }),
    b.lineText({ op: "delete", item: { ...route, deleted: true } }),
  ];
  // ws: 조용한 다시 계산은 기록 줄이 없고, 지운 것은 사라지고, 판 번호가 건너뛰면 다시 받는다
  out.quiet = b.onMessage({ type: "board", op: "route", rev: 3, logged: false, item: { ...route, rev: 3 } });
  out.deleted = b.onMessage({ type: "board", op: "delete", rev: 4, logged: true, item: { ...entry, rev: 4, deleted: true } });
  out.afterDelete = b.frame(v).svg.includes('data-wb-id="b-0001"');
  const before = fetches;
  b.onMessage({ type: "board", op: "add", rev: 9, logged: true, item: { ...entry, id: "b-0009", rev: 9 } });
  out.reloaded = fetches > before;
  // Esc: 그리기가 꺼져 있으면 지나가고(app.js 비상 정지), 켜져 있으면 여기서 멈춘다(그리기 취소)
  found.add("svg[data-wb-map]"); found.add("[data-wb-toggle]");
  const key = (code) => { const e = event({ tagName: "BODY" }); e.code = code; listeners.keydown.fn(e); return e.stopped; };
  out.escOff = key("Escape");
  out.capture = [listeners.keydown.capture, listeners.click.capture];
  key("KeyB");   // 그리기 켜기
  out.big = big;
  out.escOn = key("Escape");
  out.toastEsc = toasts[toasts.length - 1];
  out.digit = key("Digit3");
  f = b.frame(v);
  out.toolbar = f.bar;
  out.headOn = f.head('<header></header>');
  out.drawingAttr = f.attr;
  // 그리기 도구일 때 지도 클릭은 여기서 멈춘다(app.js가 드론 보낼 자리를 찍지 않게), 보기 도구면 지나간다
  const svgEl = { dataset: {}, getScreenCTM: () => null, closest: (sel) => (sel === "svg[data-wb-map]" ? svgEl : null) };
  const mapClick = () => { const e = event(svgEl); e.clientX = 10; e.clientY = 10; listeners.click.fn(e); return e.stopped; };
  out.clickDrawing = mapClick();
  key("Digit1");
  out.clickView = mapClick();
  key("KeyB");   // 끄기
  out.bigAfter = big;
  out.escAfterOff = key("Escape");
  console.log(JSON.stringify(out));
})();
"""


@unittest.skipUnless(NODE, 'node가 없어 board.js 동작을 확인하지 못함')
class BoardScriptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        run = subprocess.run([NODE, '-e', SCRIPT, str(CONTROLLER / 'board.js')], capture_output=True, text=True,
                             encoding='utf-8', timeout=60)
        if run.returncode:
            raise AssertionError(run.stderr)
        cls.out = json.loads(run.stdout)

    def test_drawn_on_the_map(self):
        o = self.out
        self.assertIn('data-wb-toggle', o['head'])
        self.assertTrue(o['head'].endswith('</header>'))
        self.assertEqual(o['barOff'], '')                           # 꺼져 있으면 도구 막대 없음
        self.assertIn('data-wb-map="1"', o['attr'])
        self.assertIn('data-wb-id="b-0002"', o['svg'])              # 동선
        self.assertIn('class="wb-route"', o['svg'])
        self.assertIn('사람 2 · 4.2 m · 약 9초', o['svg'])           # 끝 글: 도착 · 거리 · 조심 걸음 시간
        self.assertIn('진입 1', o['svg'])
        self.assertIn('data-map-layer="board"', o['layer'])
        self.assertIn('동선 1', o['list'])
        self.assertNotIn('data-wb-del', o['list'])                  # 그리기를 켜야 지우기 버튼
        self.assertEqual(o['points'], 6)                            # 진입로 1 + 동선 점 3 + 출발·도착

    def test_flight_log_words(self):
        self.assertEqual(self.out['lines'], [
            '진입 1 표시',
            '동선 1: 진입 1 → 사람 2 · 4.2 m · 조심 걸음 약 9초',
            '동선 1: 지도가 바뀌어 갈 수 있는 데까지 3.0 m · 1.3 m 남음(지도에서 막힘)',
            '동선 1: 진입 1 → 사람 2 · 자율 비행 지도가 아직 없습니다(지도가 오면 계산)',
            '동선 1: 진입 1 → 사람 2 · 30.0 m · 조심 걸음 약 1분',
            '동선 1 지움',
        ])

    def test_messages_update_the_board(self):
        o = self.out
        self.assertIsNone(o['quiet'])
        self.assertEqual(o['deleted'], '진입 1 지움')
        self.assertFalse(o['afterDelete'])
        self.assertTrue(o['reloaded'])

    def test_escape_cancels_drawing_instead_of_emergency(self):
        o = self.out
        self.assertEqual(o['capture'], [True, True])   # app.js(버블)보다 먼저 받는다
        self.assertFalse(o['escOff'])                  # 그리기가 꺼져 있으면 지금처럼(app.js 비상 정지)
        self.assertTrue(o['escOn'])                    # 켜져 있으면 여기서 멈춘다
        self.assertIn('비상 정지가 아닙니다', o['toastEsc'])
        self.assertFalse(o['escAfterOff'])
        self.assertEqual((o['big'], o['bigAfter']), (True, False))   # 켜면 지도를 크게, 끄면 되돌림

    def test_tools_and_map_clicks(self):
        o = self.out
        self.assertTrue(o['digit'])
        self.assertIn('role="toolbar"', o['toolbar'])
        self.assertIn('data-wb-tool="route" class="on" aria-pressed="true"', o['toolbar'])
        self.assertIn('Esc는 그리기 취소 · 비상 정지는 화면 버튼', o['toolbar'])
        self.assertTrue(o['headOn'].endswith('</header>'))          # 도구 막대는 머리줄이 아니라 지도 몸통에(지도 칸이 둔다)
        self.assertIn('wb-drawing', o['drawingAttr'])
        self.assertTrue(o['clickDrawing'])             # 그리기 도구: 드론 보낼 자리를 찍지 않는다
        self.assertFalse(o['clickView'])               # 보기 도구: 지금처럼


if __name__ == '__main__':
    unittest.main()
