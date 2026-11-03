import unittest

import numpy as np
import torch
import torch.nn.functional as functional

from modules.perception.inference.ip_webcam_semantic import (parser, resize_for_display,
                                                             smooth_filtered_label_map)
from modules.perception.inference.semantic_overlay import (filtered_label_map, render_overlay,
                                                            resolve_class_ids)


class SemanticOverlayTests(unittest.TestCase):
    def test_filters_classes_and_low_confidence_pixels(self):
        probabilities = np.array([
            [[.8, .2], [.4, .1]],
            [[.1, .7], [.3, .2]],
            [[.1, .1], [.3, .7]],
        ], dtype=np.float32)
        actual = filtered_label_map(probabilities, {0, 1}, .6)
        np.testing.assert_array_equal(actual, [[0, 1], [-1, -1]])

    def test_renders_only_visible_pixels_and_reports_coverage(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        labels = np.array([[0, -1], [1, 1]], dtype=np.int32)
        output, coverage = render_overlay(frame, labels, {0: "wall", 1: "floor"}, alpha=.5)
        self.assertTrue(np.any(output[0, 0]))
        self.assertTrue(np.all(output[0, 1] == 0))
        self.assertEqual(coverage, [
            {"class_id": 0, "label": "wall", "pixels": 1, "coverage": .25},
            {"class_id": 1, "label": "floor", "pixels": 2, "coverage": .5},
        ])

    def test_resolves_labels_case_insensitively(self):
        self.assertEqual(resolve_class_ids({0: "wall", 3: "Table"}, ["WALL", "table"]), {0, 3})
        with self.assertRaisesRegex(ValueError, "unknown_classes:desk"):
            resolve_class_ids({0: "wall"}, ["desk"])

    def test_parses_performance_options(self):
        defaults = parser().parse_args(["--source", "http://camera/video"])
        self.assertEqual((defaults.input_size, defaults.display_width, defaults.inference_every),
                         (512, 0, 1))
        self.assertEqual(defaults.classes, "wall,floor,ceiling,table,chair,door,windowpane")
        optimized = parser().parse_args([
            "--source", "http://camera/video",
            "--input-size", "384",
            "--display-width", "960",
            "--inference-every", "2",
        ])
        self.assertEqual((optimized.input_size, optimized.display_width, optimized.inference_every),
                         (384, 960, 2))

    def test_resizes_display_frame_while_preserving_aspect_ratio(self):
        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        self.assertIs(resize_for_display(frame, 0), frame)
        self.assertEqual(resize_for_display(frame, 640).shape, (360, 640, 3))

    def test_smooths_selected_class_scores_before_choosing_labels(self):
        probabilities = torch.tensor([
            [[.8, .8], [.8, .1]],
            [[.1, .1], [.1, .8]],
            [[.1, .1], [.1, .1]],
        ])
        actual = smooth_filtered_label_map(
            probabilities,
            torch.tensor([1]),
            torch.tensor([0, 2]),
            torch.tensor([-1, 1]),
            .4,
            (4, 4),
            functional,
        )
        self.assertEqual(actual.shape, (4, 4))
        self.assertEqual(actual[0, 0], -1)
        self.assertEqual(actual[-1, -1], 1)
        self.assertTrue(set(np.unique(actual)).issubset({-1, 1}))


if __name__ == "__main__":
    unittest.main()
