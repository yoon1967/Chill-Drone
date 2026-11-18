from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_publisher import load_calibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        import rospkg
        source = Path(rospkg.RosPack().get_path("scale_recovery")) / "config/camera_info.yaml"
        self.data = yaml.safe_load(source.read_text())

    def load(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.yaml"
            path.write_text(yaml.safe_dump(self.data))
            return load_calibration(path)

    def test_intrinsics_are_preserved_and_stamp_is_static(self):
        message = self.load()
        self.assertEqual(list(message.K), self.data["camera_matrix"]["data"])
        self.assertEqual(list(message.D), self.data["distortion_coefficients"]["data"])
        self.assertEqual(message.header.stamp.to_sec(), 0)

    def test_virtual_baseline_sets_orb_bf_only_when_requested(self):
        self.assertEqual(self.load().P[3], self.data["projection_matrix"]["data"][3])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.yaml"
            path.write_text(yaml.safe_dump(self.data))
            message = load_calibration(path, 0.075)
            for bad in (-0.01, 0.6, float("nan")):
                with self.assertRaises(ValueError):
                    load_calibration(path, bad)
        self.assertAlmostEqual(message.P[3], 0.075 * self.data["camera_matrix"]["data"][0])
        self.assertEqual(list(message.K), self.data["camera_matrix"]["data"])

    def test_wrong_image_size_is_rejected(self):
        self.data["image_width"] = 1280
        with self.assertRaises(ValueError):
            self.load()

    def test_bad_focal_length_is_rejected(self):
        self.data["camera_matrix"]["data"][0] = 0
        with self.assertRaises(ValueError):
            self.load()

    def test_short_distortion_vector_is_rejected(self):
        self.data["distortion_coefficients"]["data"] = [0]
        with self.assertRaises(ValueError):
            self.load()
