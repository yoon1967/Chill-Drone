"""자율 비행 보기 노드의 지도 높이 층(11/17): 비행 노드 3D 카메라 지도(camera_map.npz)로 막힌 칸을 벽·높은 것과 낮은 물체로 나눈다.

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_ros_view_heights.py" -v
"""

import base64
import builtins
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import ros_view_core  # noqa: E402
from ros_view_core import (HEIGHT_LOW, HEIGHT_NONE, HEIGHT_TALL, HEIGHT_UNSURE, ViewState,  # noqa: E402
                           camera_map_heights, make_server, same_grid, watch_camera_map)

try:
    import numpy as np
except ImportError:   # 보기 노드는 numpy가 없어도 돈다(높이 층만 빠진다)
    np = None

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NX, NY, NZ = 10, 8, 25           # 1.0 × 0.8 m, 바닥~2.5 m (0.1 m 칸)
ORIGIN = (-1.0, -2.0, 0.0)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def fake_camera_map(path: str) -> None:
    """비행 노드 CameraMap.save와 같은 모양(logodds·seen·hard·origin·res)으로 기둥 몇 개를 만든다."""
    lo = np.zeros((NX, NY, NZ), dtype=np.float32)
    seen = np.zeros((NX, NY, NZ), dtype=bool)

    def column(i, j, hit, free=()):
        for k in hit:
            lo[i, j, k], seen[i, j, k] = 0.85, True
        for k in free:
            lo[i, j, k], seen[i, j, k] = -0.5, True

    column(1, 1, range(3, 22))                 # 벽: 0.3–2.2 m 막힘 → 높음
    column(2, 2, [7], range(8, 14))            # 책상: 꼭대기 0.8 m, 그 위 빈 칸 → 낮음
    column(3, 3, [7])                          # 꼭대기 0.8 m인데 위를 못 봄 → 모름
    column(4, 4, [14], range(15, 20))          # 꼭대기 1.5 m(1.4–1.6) → 모름
    column(5, 5, [13], range(14, 20))          # 꼭대기 딱 1.4 m, 위가 빔 → 낮음
    column(6, 6, [15])                         # 꼭대기 딱 1.6 m → 높음
    column(7, 7, [1], range(2, 10))            # 바닥(0.3 m 아래)만 → 없음
    column(8, 1, [24], range(10, 24))          # 천장(2.4 m 위)만 → 없음
    lo[9, 7, 7], seen[9, 7, 7] = 0.4, True     # 의심(0.3–0.5)도 막힌 칸으로 본다
    column(9, 7, [], range(8, 12))             # 위가 빔 → 낮음
    np.savez_compressed(path, logodds=lo, seen=seen, hard=np.zeros_like(seen), origin=np.array(ORIGIN), res=0.1)


def code_at(codes: bytes, i: int, j: int) -> int:
    return codes[j * NX + i]   # /api/grid cells와 같은 순서(y 줄마다 x)


@unittest.skipIf(np is None, "numpy 없음")
class HeightsTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)

    def test_columns_split_by_top_height(self):
        path = os.path.join(self.dir, "camera_map.npz")
        fake_camera_map(path)
        codes, info = camera_map_heights(path)
        self.assertEqual(len(codes), NX * NY)
        self.assertEqual((info["w"], info["h"], info["x0"], info["y0"], info["res"]), (NX, NY, -1.0, -2.0, 0.1))
        want = {(1, 1): HEIGHT_TALL, (2, 2): HEIGHT_LOW, (3, 3): HEIGHT_UNSURE, (4, 4): HEIGHT_UNSURE, (5, 5): HEIGHT_LOW,
                (6, 6): HEIGHT_TALL, (7, 7): HEIGHT_NONE, (8, 1): HEIGHT_NONE, (9, 7): HEIGHT_LOW, (0, 0): HEIGHT_NONE}
        self.assertEqual({ij: code_at(codes, *ij) for ij in want}, want)
        self.assertEqual(info["counts"], {"low": 3, "tall": 2, "unsure": 2})

    def test_broken_file_raises(self):   # 비행 노드가 쓰는 중(압축이 덜 됨)이면 예외: 부르는 쪽이 다음에 다시 읽는다
        path = os.path.join(self.dir, "camera_map.npz")
        with open(path, "wb") as f:
            f.write(b"PK\x03\x04 not finished")
        with self.assertRaises(Exception):
            camera_map_heights(path)

    def test_view_attaches_heights_to_the_same_grid_only(self):
        clock = Clock()
        v = ViewState(clock=clock)
        v.on_grid(0.1, NX, NY, -1.0, -2.0, bytes(NX * NY))
        self.assertNotIn("heights", v.grid())
        path = os.path.join(self.dir, "camera_map.npz")
        fake_camera_map(path)
        codes, info = camera_map_heights(path)
        v.on_heights(codes, info, path)
        clock.t += 2.5
        g = v.grid()
        self.assertEqual((g["ver"], base64.b64decode(g["heights"]), g["heights_counts"], g["heights_age_s"]),
                         (2, codes, {"low": 3, "tall": 2, "unsure": 2}, 2.5))   # 높이 층이 오면 지도 번호도 오른다
        self.assertEqual(v.snapshot()["topics"]["camera_map.npz"]["count"], 1)
        v.on_heights(codes, info, path)
        self.assertEqual(v.grid()["ver"], 2)                                       # 같으면 그대로
        v.on_grid(0.1, NX, NY, 0.0, 0.0, bytes(NX * NY))                           # 원점이 다른 지도(시뮬레이터 등)에는 안 싣는다
        self.assertNotIn("heights", v.grid())
        v.on_grid(0.1, NX, NY, -1.0, -2.0, bytes(NX * NY))
        self.assertIn("heights", v.grid())
        v.on_heights(None, None)                                                   # 새 회차에 아직 없음
        self.assertNotIn("heights", v.grid())
        self.assertTrue(same_grid({"w": 2, "h": 2, "x0": 0.0, "y0": 1.0, "res": 0.1}, {"w": 2, "h": 2, "x0": 1e-9, "y0": 1.0, "res": 0.1}))
        self.assertFalse(same_grid({"w": 2, "h": 2, "x0": 0.0, "y0": 1.0, "res": 0.1}, {"w": 2, "h": 3, "x0": 0.0, "y0": 1.0, "res": 0.1}))

    def test_http_grid_carries_heights(self):
        v = ViewState()
        server = make_server(v, "127.0.0.1", 0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(lambda: (server.stopping.set(), server.shutdown(), server.server_close()))
        path = os.path.join(self.dir, "camera_map.npz")
        fake_camera_map(path)
        v.on_grid(0.1, NX, NY, -1.0, -2.0, bytes([100]) * (NX * NY))
        v.on_heights(*camera_map_heights(path), path)
        with urllib.request.urlopen(f"http://127.0.0.1:{server.server_address[1]}/api/grid", timeout=5) as r:
            g = json.loads(r.read())
        self.assertEqual(code_at(base64.b64decode(g["heights"]), 1, 1), HEIGHT_TALL)
        self.assertEqual(code_at(base64.b64decode(g["cells"]), 1, 1), 3)

    def test_watch_newest_run_and_wait_for_writing(self):
        root = os.path.join(self.dir, "flights")
        old, new = os.path.join(root, "20261117_100000"), os.path.join(root, "20261117_110000")
        os.makedirs(old)
        os.makedirs(new)
        fake_camera_map(os.path.join(old, "camera_map.npz"))
        v = ViewState()
        v.on_grid(0.1, NX, NY, -1.0, -2.0, bytes(NX * NY))
        stop = threading.Event()
        t = threading.Thread(target=watch_camera_map, args=(v, root, None, stop), kwargs={"period": 0.02, "settle": 0.5})
        t.start()
        self.addCleanup(lambda: (stop.set(), t.join(5)))

        def wait_for(cond, timeout=5.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if cond():
                    return True
                time.sleep(0.02)
            return False

        time.sleep(0.2)
        self.assertNotIn("heights", v.grid())                    # 가장 새 회차에는 아직 없다: 지난 회차 것을 쓰지 않는다
        target = os.path.join(new, "camera_map.npz")
        with open(target, "wb") as f:
            f.write(b"PK\x03\x04 half")
        past = time.time() - 30
        os.utime(target, (past, past))                           # 오래돼도 못 읽는 판은 버리고 다음 저장을 기다린다
        time.sleep(0.2)
        self.assertNotIn("heights", v.grid())
        fake_camera_map(target)                                  # 방금 쓴 파일: settle 동안은 안 읽는다
        self.assertTrue(wait_for(lambda: "heights" in v.grid()))
        self.assertEqual(code_at(base64.b64decode(v.grid()["heights"]), 2, 2), HEIGHT_LOW)
        os.remove(target)                                        # 회차가 바뀌어 파일이 없으면 높이 층을 비운다
        self.assertTrue(wait_for(lambda: "heights" not in v.grid()))


class NodeTest(unittest.TestCase):
    def test_node_watches_the_camera_map(self):   # rclpy가 없어 불러오지 않고 읽기만 한다
        with open(os.path.join(HERE, "ros_view_node.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn('self.declare_parameter("camera_map", "")', src)
        self.assertIn("target=watch_camera_map", src)

    def test_core_imports_without_numpy_at_module_level(self):   # numpy는 높이 층 함수 안에서만 불러온다
        with open(os.path.join(HERE, "ros_view_core.py"), encoding="utf-8") as f:
            top = [line for line in f.read().splitlines() if line.startswith(("import ", "from "))]
        self.assertFalse([line for line in top if "numpy" in line])
        self.assertTrue(callable(ros_view_core.camera_map_heights))

    def test_watch_without_numpy_only_drops_heights(self):   # WSL 파이썬에 numpy가 없으면 경고 한 줄 남기고 지켜보기만 그만둔다
        real_import = builtins.__import__

        def no_numpy(name, *args, **kwargs):
            if name == "numpy" or name.startswith("numpy."):
                raise ImportError("numpy 없음")
            return real_import(name, *args, **kwargs)

        logs, v = [], ViewState()
        with mock.patch("builtins.__import__", no_numpy):
            watch_camera_map(v, tempfile.gettempdir(), None, threading.Event(), log=logs.append)   # 곧바로 돌아온다
        self.assertEqual(len(logs), 1)
        self.assertIn("numpy", logs[0])
        v.on_grid(0.1, 2, 1, 0.0, 0.0, bytes([100, 0]))
        self.assertNotIn("heights", v.grid())                                                   # 지도 단면은 그대로


if __name__ == "__main__":
    unittest.main()
