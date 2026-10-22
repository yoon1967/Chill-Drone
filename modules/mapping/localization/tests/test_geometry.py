import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import pycolmap

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reconstruct import scale_transform, triangulate_anchor


class GeometryTest(unittest.TestCase):
    def setUp(self):
        camera = pycolmap.Camera(model="SIMPLE_RADIAL", width=1920, height=1080,
                                 params=[1000, 960, 540, 0.04], camera_id=1)
        images = {}
        self.observations = []
        for i, x in enumerate([-1.0, 0, 1, 2]):
            center = np.array([x, 0.2, 0.0])
            pose = pycolmap.Rigid3d(pycolmap.Rotation3d(), -center)
            name = f"frame_{i:04d}.jpg"
            images[i] = SimpleNamespace(name=name, has_pose=True, camera_id=1,
                cam_from_world=lambda p=pose: p, projection_center=lambda c=center: c)
            for aid, point in [("A", [0., 0., 5.]), ("B", [1., 0., 5.])]:
                xy = camera.img_from_cam(pose * np.array(point))
                self.observations.append({"anchor_id": aid, "frame_id": name,
                    "x_px": str(xy[0]), "y_px": str(xy[1]), "metric_validation_eligible": "True"})
        self.model = SimpleNamespace(images=images, cameras={1: camera})

    def test_distorted_pixels_triangulate_known_world_point(self):
        point, quality = triangulate_anchor(self.model, [o for o in self.observations if o["anchor_id"] == "A"])
        np.testing.assert_allclose(point, [0, 0, 5], atol=1e-8)
        self.assertLess(quality["max_reprojection_error_px"], 1e-6)

    def test_ineligible_measurements_do_not_supply_geometry(self):
        observations = [dict(o, metric_validation_eligible="False") for o in self.observations]
        with self.assertRaisesRegex(ValueError, "eligible"):
            triangulate_anchor(self.model, observations)

    def test_degenerate_camera_baseline_is_rejected(self):
        for image in self.model.images.values():
            image.cam_from_world = lambda: pycolmap.Rigid3d()
            image.projection_center = lambda: np.zeros(3)
        with self.assertRaises(ValueError):
            triangulate_anchor(self.model, [o for o in self.observations if o["anchor_id"] == "A"])

    def test_metric_transform_preserves_camera_projection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "anchors.json").write_text(json.dumps({"segments": [
                {"segment_id": "width_44cm", "length_m": .44, "role": "scale_reference", "correspondence_ready": True},
                {"segment_id": "height_136cm", "correspondence_ready": False}]}), encoding="utf-8")
            with (root / "obs.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.observations[0]))
                writer.writeheader()
                writer.writerows(self.observations)
            factor, rotation, translation, info = scale_transform(self.model, {"paths": {
                "measurement_anchors": root / "anchors.json", "measurement_observations": root / "obs.csv"}})
        self.assertEqual(info["status"], "reference_scaled")
        self.assertAlmostEqual(factor, .44)
        point = np.array([0.3, -.1, 5.1])
        image = self.model.images[0]
        transformed = factor * rotation @ point + translation
        center = factor * rotation @ image.projection_center() + translation
        world_from_camera = rotation @ image.cam_from_world().rotation.matrix().T
        np.testing.assert_allclose(world_from_camera.T @ (transformed - center), factor * (image.cam_from_world() * point))
        self.assertEqual(info["independent_length_validation"], "not_performed_unconfirmed_floor_C")


if __name__ == "__main__":
    unittest.main()
