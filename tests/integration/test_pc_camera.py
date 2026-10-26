"""Real camera configuration boundary; no physical camera or Docker required."""
from copy import deepcopy
import unittest
from unittest.mock import patch

from modules.mapping.localization.orbslam3_live import LiveManager, live_camera


class PCCameraConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.camera = dict(width=1920, height=1080, fx=1200., fy=1190.,
                           cx=960., cy=540., distortion=[.01, -.02, 0., 0.])
        self.config = dict(width=960, height=540, fps=29.97, camera=self.camera)

    def test_intrinsics_scale_without_mutating_the_uploaded_calibration(self):
        original = deepcopy(self.camera)
        value = live_camera(self.config)
        self.assertEqual((value["width"], value["height"]), (960, 540))
        self.assertEqual((value["fx"], value["fy"], value["cx"], value["cy"]), (600., 595., 480., 270.))
        self.assertEqual(value["distortion"], self.camera["distortion"])
        self.assertIn("not_independently_verified", value["calibration_status"])
        self.assertEqual(self.camera, original)

    def test_missing_calibration_is_explicitly_an_estimate(self):
        value = live_camera(dict(width=640, height=480, fps=30, fov=60))
        self.assertEqual(value["calibration_status"], "fov_estimated; not_calibrated")
        self.assertEqual(value["distortion"], [0.] * 4)

    def test_invalid_calibration_never_starts_docker_or_replaces_current_session(self):
        invalid = [dict(self.config, width=960.5), dict(self.config, fps=float("nan")),
                   dict(self.config, width=True), dict(self.config, camera=[]),
                   dict(self.config, camera={}), dict(self.config, camera="camera.json"),
                   dict(width=640, height=480, fov=float("inf"))]
        for change in [dict(width=1080, height=1920), dict(fx="1200"), dict(cy=2000),
                       dict(distortion={}), dict(distortion=[0, 0, 0, False]),
                       dict(distortion=[0, 0, 0, float("inf")]), dict(calibration_status={})]:
            invalid.append(dict(self.config, camera={**self.camera, **change}))
        manager = LiveManager(".")
        sentinel = manager.current = object()
        with patch("modules.mapping.localization.orbslam3_live.subprocess.check_output") as docker:
            for config in invalid:
                with self.subTest(config=config), self.assertRaises((ValueError, KeyError)):
                    manager.start(config)
                self.assertIs(manager.current, sentinel)
            docker.assert_not_called()


if __name__ == "__main__":
    unittest.main()
