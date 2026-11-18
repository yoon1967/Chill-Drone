"""tools/make_room_map.py: 손으로 만드는 시험 공간 지도와 지도 없이 띄울 때의 빈 지도(open_map)."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.make_room_map import make_map, open_map  # noqa: E402


class OpenMapTest(unittest.TestCase):
    def test_open_map_is_square_around_start_with_nothing_in_it(self):
        # 2026-10-30 복도 비행: 실습실 지도를 그대로 써서 방 크기·탐색 목표가 맞지 않았다 → map:= 없이 띄우면 이 지도
        m = open_map(20.0, 1.2)
        b = m["bounds"]
        self.assertEqual((b["xmin"], b["ymin"], b["xmax"], b["ymax"]), (-10.0, -10.0, 10.0, 10.0))
        self.assertEqual((m["spawn"]["x"], m["spawn"]["y"], m["spawn"]["yaw"]), (0.0, 0.0, 0.0))
        self.assertEqual(m["flight_height"], 1.2)
        self.assertEqual((m["walls"], m["obstacles"], m["goal"], m["stage"]), ([], [], None, 4))
        self.assertTrue(m["open"])

    def test_open_map_can_be_centered_elsewhere(self):
        # Gazebo 실습실 월드로 시험할 때: 월드의 출발점·방향에 맞춰 둔다
        m = open_map(10.0, center=(5.5, 4.45), yaw=3.1416)
        b = m["bounds"]
        self.assertAlmostEqual(b["xmin"], 0.5)
        self.assertAlmostEqual(b["ymax"], 9.45)
        self.assertEqual(m["spawn"]["x"], 5.5)
        self.assertAlmostEqual(m["spawn"]["yaw"], 3.1416)

    def test_cli_open_writes_file_without_size_or_start(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "open.json"
            r = subprocess.run([sys.executable, str(ROOT / "tools" / "make_room_map.py"), "--open", "16", "--out", str(out)],
                               capture_output=True, text=True, encoding="utf-8")
            self.assertEqual(r.returncode, 0, r.stderr)
            m = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(m["bounds"]["xmax"], 8.0)
            self.assertEqual(m["walls"], [])

    def test_room_map_keeps_walls_unless_explore(self):
        m = make_map(4.0, 3.0, (0.6, 1.5), 0.0, (3.2, 1.5), [])
        self.assertEqual(len(m["walls"]), 4)
        self.assertEqual(m["stage"], 1)
        e = make_map(4.0, 3.0, (0.6, 1.5), 0.0, None, [], explore=True)
        self.assertEqual((e["walls"], e["stage"]), ([], 4))


if __name__ == "__main__":
    unittest.main()
