"""관제 화면 3D 점군 보기(apps/controller/map3d.js)의 높이 색 범위와 천장 빼기 높이를 본다(11/17).

map3d.js는 브라우저 파일이라 Node로 불러 숫자만 확인한다(WebGL 그리기는 브라우저에서만). Node가 없으면 건너뛴다.
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

CONTROLLER = Path(__file__).resolve().parents[1] / 'apps' / 'controller'
NODE = shutil.which('node')

SCRIPT = r"""
global.window = {};
require(process.argv[1]);
const cases = JSON.parse(require('fs').readFileSync(0, 'utf8')), out = {};   // 점이 많아 명령줄 대신 표준 입력
for (const [name, c] of Object.entries(cases)) out[name] = window.PointView.heightRange(c.pts, c.cams, c.up);
console.log(JSON.stringify(out));
"""


def room():
    """위가 +y인 방: 바닥 −1 m(많음), 벽 −1~1.7 m, 천장 1.7 m(많음). 카메라는 0~0.2 m 높이로 지나감."""
    pts = [[x * 0.1, -1.0, z * 0.1, 0, 0, 0] for x in range(30) for z in range(30)]                 # 바닥 900점
    pts += [[0.0, -1.0 + k * 0.03, z * 0.1, 0, 0, 0] for k in range(91) for z in range(10)]      # 벽 910점
    pts += [[x * 0.1, 1.7, z * 0.1, 0, 0, 0] for x in range(30) for z in range(20)]                # 천장 600점
    cams = [[0.5 * i, 0.1 * (i % 3), 1.0] for i in range(6)]                                       # 높이 0~0.2 m
    return {'pts': pts, 'cams': cams, 'up': [0, 1, 0]}


@unittest.skipUnless(NODE, 'node가 없어 map3d.js 높이 범위를 확인하지 못함')
class HeightRangeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        r = room()
        cases = {
            'room': r,
            'no_cameras': {**r, 'cams': []},
            'empty': {'pts': [], 'cams': r['cams'], 'up': [0, 1, 0]},
            'low_cameras': {**r, 'cams': [[0, -5.0, 0]]},   # 카메라가 지도 아래로 튀어도 가운데보다 아래는 자르지 않는다
        }
        run = subprocess.run([NODE, '-e', SCRIPT, str(CONTROLLER / 'map3d.js')], input=json.dumps(cases),
                             capture_output=True, text=True, encoding='utf-8', timeout=60)
        if run.returncode:
            raise AssertionError(run.stderr)
        cls.out = json.loads(run.stdout)

    def test_colors_span_floor_to_ceiling_without_outliers(self):
        o = self.out['room']
        self.assertAlmostEqual(o['lo'], -1.0, places=6)
        self.assertAlmostEqual(o['hi'], 1.7, places=6)

    def test_ceiling_is_cut_a_little_above_the_flight(self):   # 카메라 가장 높은 곳 0.2 m + 높이 폭 2.7 m의 30%
        o = self.out['room']
        self.assertAlmostEqual(o['cut'], 0.2 + 2.7 * 0.3, places=6)
        self.assertLess(o['cut'], 1.7)   # 천장(1.7 m)은 빠진다
        self.assertGreater(o['cut'], 0.2)

    def test_without_cameras_only_the_top_outliers_go(self):
        o = self.out['no_cameras']
        self.assertAlmostEqual(o['cut'], o['hi'], places=6)

    def test_no_points_no_range(self):
        self.assertIsNone(self.out['empty'])

    def test_cut_never_goes_below_the_middle(self):
        o = self.out['low_cameras']
        self.assertAlmostEqual(o['cut'], o['lo'] + (o['hi'] - o['lo']) * 0.5, places=6)


class WiringTest(unittest.TestCase):
    def test_style_choices_reach_the_view(self):
        app = (CONTROLLER / 'app.js').read_text(encoding='utf-8')
        self.assertIn('m.view?.setStyle?.({ photo: m.photo, ceiling: m.ceiling })', app)
        self.assertIn('data-map3d-opt="photo"', app)
        self.assertIn('data-map3d-opt="ceiling"', app)
        self.assertIn('"map3dOpt"', app)   # 다시 그려도 고르기 버튼 초점을 지킨다
        map3d = (CONTROLLER / 'map3d.js').read_text(encoding='utf-8')
        self.assertIn('setStyle({ photo = false, ceiling = false } = {})', map3d)
        self.assertIn('gl_PointSize = att > 0.0 ? clamp(att / max(gl_Position.w, 0.0001), sz.x, sz.y) : s;', map3d)   # 가까울수록 크게


if __name__ == '__main__':
    unittest.main()
