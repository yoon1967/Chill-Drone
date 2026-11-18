"""4단계 비행 기록 검사. 실행: python -m unittest discover -s tests"""
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.flight_record import FlightRecorder, load_record  # noqa: E402


class FlightRecordTest(unittest.TestCase):
    def test_round_trip_across_chunks(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FlightRecorder(d, chunk=3, meta={"hfov": 1.44})
            for i in range(7):   # 3 + 3 + 1장: 마지막 덩어리는 flush에서 쓴다
                rec.add_depth(10.0 + i, (i, 2.0, 1.0, 0.1 * i), np.full((4, 6), 1.5 + i, np.float32))
                rec.add_tof(10.05 + i, (i, 2.0, 1.0, 0.0), 0.8 if i % 2 else np.inf)
            rec.flush()
            r = load_record(d)
            self.assertEqual(r["depth"].shape, (7, 4, 6))
            np.testing.assert_allclose(r["t"], 10.0 + np.arange(7))
            self.assertAlmostEqual(float(r["depth"][6, 0, 0]), 7.5, places=2)
            self.assertAlmostEqual(float(r["pose"][4, 3]), 0.4)
            self.assertEqual(len(r["tof_t"]), 7)
            self.assertTrue(np.isinf(r["tof_range"][0]))
            self.assertEqual(r["meta"]["hfov"], 1.44)

    def test_broken_chunk_is_skipped(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FlightRecorder(d, chunk=2)
            for i in range(4):
                rec.add_depth(float(i), (0, 0, 1, 0), np.ones((2, 2), np.float32))
            rec.flush()
            (Path(d) / "depth_0001.npz").write_bytes(b"PK\x03\x04 cut off")   # 쓰다가 꺼진 파일
            r = load_record(d)
            self.assertEqual(len(r["t"]), 2)
            self.assertEqual(r["skipped"], ["depth_0001.npz"])
            self.assertFalse(list(Path(d).glob("*.part")), "임시 파일이 남지 않는다")

    def test_new_recording_replaces_old_chunks(self):
        with tempfile.TemporaryDirectory() as d:
            rec = FlightRecorder(d, chunk=2)
            for i in range(4):
                rec.add_depth(float(i), (0, 0, 1, 0), np.ones((2, 2), np.float32))
            rec.flush()
            rec = FlightRecorder(d, chunk=2)   # 같은 폴더에 다시 날리면 앞 비행 기록은 지운다
            rec.add_depth(0.0, (0, 0, 1, 0), np.ones((2, 2), np.float32))
            rec.flush()
            self.assertEqual(len(load_record(d)["t"]), 1)


if __name__ == "__main__":
    unittest.main()
