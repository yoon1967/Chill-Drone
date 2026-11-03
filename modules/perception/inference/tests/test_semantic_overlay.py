import unittest

import numpy as np

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


if __name__ == "__main__":
    unittest.main()
