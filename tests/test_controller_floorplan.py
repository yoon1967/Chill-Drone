"""관제 화면 평면도(apps/controller/floorplan.js): 3D 점군을 위에서 본 2D 그림의 축·높이 띠·칸을 본다.

floorplan.js는 브라우저 파일이라 Node로 불러 숫자만 확인한다(그림 PNG는 브라우저에서만 만든다). Node가 없으면 건너뛴다.
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

FLOORPLAN = Path(__file__).resolve().parents[1] / 'apps' / 'controller' / 'floorplan.js'
NODE = shutil.which('node')

# 지도 이름 → gateway-map/0.1 모양 일부(points [x,y,z,r,g,b], cameras [x,y,z], up, scale)
SCRIPT = r"""
const fp = require(process.argv[1]);
const maps = JSON.parse(process.argv[2]);
const out = {};
for (const [name, map] of Object.entries(maps)) {
  const p = fp.compute(map);
  if (!p) { out[name] = null; continue; }
  let best = 0;   // 점이 가장 많이 쌓인 칸의 가운데(앞, 왼쪽)
  p.counts.forEach((n, k) => { if (n > p.counts[best]) best = k; });
  const i = best % p.nx, j = Math.floor(best / p.nx);
  out[name] = { metric: p.metric, band: p.band, half: p.half, kept: p.kept, total: p.total, cell: p.cell,
    nx: p.nx, ny: p.ny, a0: p.a0, a1: p.a1, b0: p.b0, b1: p.b1, path: p.path, png: fp.build(map).png,
    densest: [p.a0 + (i + 0.5) * p.cell, p.b1 - (j + 0.5) * p.cell], sum: p.counts.reduce((s, n) => s + n, 0) };
}
out.alpha = [fp.alphaOf(0, 5), fp.alphaOf(1, 5), fp.alphaOf(5, 5), fp.alphaOf(50, 5)];
console.log(JSON.stringify(out));
"""


def camera_room():
    """카메라 좌표 지도(x 오른쪽·y 아래·z 앞, m): 앞 3 m에 벽, 왼쪽 1 m에 벽, 바닥(카메라보다 0.8 m 아래)."""
    pts = []
    for s in range(41):
        x = -1 + s * 0.05
        for h in (-0.4, -0.2, 0.0, 0.2, 0.4):
            pts.append([x, -h, 3.0, 200, 200, 200])          # 앞벽: 앞 3 m, 높이 -0.4~0.4 m
    for s in range(31):
        pts.append([-1.0, 0.0, s * 0.1, 200, 200, 200])      # 왼벽: 카메라 왼쪽 1 m(x = -1)
    for s in range(60):
        pts.append([(s % 10) * 0.2 - 1, 0.8, (s // 10) * 0.5, 120, 110, 100])   # 바닥: 카메라 0.8 m 아래 → 띠 밖
    return {'points': pts, 'cameras': [[0, 0, 0], [0, 0, 0.5], [0, 0, 1.0]], 'up': [0, -1, 0],
            'scale': {'status': 'tello_metric_estimate', 'unit': 'm'}}


@unittest.skipUnless(NODE, 'node가 없어 평면도 계산을 확인하지 못함')
class FloorPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        room = camera_room()
        z_up = {'points': [[-1.0, 2.0, 1.0, 1, 1, 1]] * 20 + [[0.5, 4.0, 1.1, 1, 1, 1]] * 5,   # z 위 세계: 앞은 y, 왼쪽은 -x
                'cameras': [[0, 0, 1.0], [0, 1.0, 1.0]], 'up': [0, 0, 1], 'scale': {'unit': 'arbitrary'}}
        tall = {'points': [[0, -3.0 - 0.01 * i, 2.0, 1, 1, 1] for i in range(40)] + [[0, 0, 2.0, 1, 1, 1]] * 2,
                'cameras': [[0, 0, 0]], 'up': [0, -1, 0], 'scale': {'unit': 'm'}}   # 띠 안 점이 15%보다 적음
        maps = {'room': room, 'z_up': z_up, 'tall': tall, 'empty': {'points': [], 'cameras': [[0, 0, 0]]},
                'no_up': {**room, 'up': None}}
        run = subprocess.run([NODE, '-e', SCRIPT, str(FLOORPLAN), json.dumps(maps)],
                             capture_output=True, text=True, encoding='utf-8', timeout=30)
        if run.returncode:
            raise AssertionError(run.stderr)
        cls.out = json.loads(run.stdout)

    def test_forward_is_camera_z_and_left_is_minus_x(self):
        room = self.out['room']
        self.assertTrue(room['metric'])
        self.assertEqual([[round(a, 3), round(b, 3)] for a, b in room['path']], [[0, 0], [0.5, 0], [1.0, 0]])
        a, b = room['densest']   # 가장 진한 칸 = 높이 다섯 줄이 쌓인 앞벽(앞 3 m)
        self.assertAlmostEqual(a, 3.0, delta=room['cell'])
        self.assertLessEqual(abs(b), 1.0 + room['cell'])
        self.assertLessEqual(room['a0'], -0.3 + 1e-9)   # 원점(지도 시작)과 왼벽(왼쪽 +1 m)까지 범위에 든다
        self.assertGreaterEqual(room['b1'], 1.0)

    def test_band_keeps_drone_height_and_drops_floor(self):
        room = self.out['room']
        self.assertEqual([round(v, 3) for v in room['band']], [-0.6, 0.6])
        self.assertEqual(room['kept'], 41 * 5 + 31)    # 바닥 60점은 빠진다
        self.assertEqual(room['total'], 41 * 5 + 31 + 60)
        self.assertEqual(room['sum'], room['kept'])     # 띠 안 점은 모두 칸에 들어간다
        self.assertAlmostEqual(room['cell'], 0.05)
        self.assertAlmostEqual(room['a1'] - room['a0'], room['nx'] * room['cell'], places=6)

    def test_z_up_map_uses_y_as_forward(self):
        z_up = self.out['z_up']
        self.assertFalse(z_up['metric'])
        self.assertEqual([[round(a, 3), round(b, 3)] for a, b in z_up['path']], [[0, 0], [1.0, 0]])
        a, b = z_up['densest']
        self.assertAlmostEqual(a, 2.0, delta=z_up['cell'])
        self.assertAlmostEqual(b, 1.0, delta=z_up['cell'])

    def test_few_points_in_band_falls_back_to_all_points(self):
        tall = self.out['tall']
        self.assertIsNone(tall['band'])
        self.assertEqual(tall['kept'], tall['total'])

    def test_missing_up_means_camera_coordinates(self):
        self.assertEqual(self.out['no_up']['densest'], self.out['room']['densest'])

    def test_empty_map_has_no_plan_and_node_has_no_picture(self):
        self.assertIsNone(self.out['empty'])
        self.assertIsNone(self.out['room']['png'])   # PNG는 브라우저(canvas)에서만

    def test_density_alpha(self):
        zero, one, ref, more = self.out['alpha']
        self.assertEqual(zero, 0)
        self.assertGreaterEqual(one, 0.25)
        self.assertLess(one, ref)
        self.assertEqual(ref, 1)
        self.assertEqual(more, 1)


class FloorPlanPageTest(unittest.TestCase):
    def test_page_loads_floorplan_before_app(self):
        html = (FLOORPLAN.parent / 'index.html').read_text(encoding='utf-8')
        self.assertLess(html.index('src="floorplan.js"'), html.index('src="app.js"'))


if __name__ == '__main__':
    unittest.main()
