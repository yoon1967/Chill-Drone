import unittest

import numpy as np

from modules.perception.inference.tello_dual_segmentation import parser, render_people


class _Tensor:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def tolist(self):
        return self.value


class _Boxes:
    xyxy = _Tensor([[2, 2, 8, 8]])
    conf = _Tensor([.91])


class _Masks:
    data = None
    xy = [np.array([[2, 2], [8, 2], [8, 8], [2, 8]], dtype=np.float32)]


class TelloDualSegmentationTests(unittest.TestCase):
    def test_defaults_limit_semantic_refresh_rate(self):
        args = parser().parse_args([])
        self.assertEqual(args.semantic_every, 3)
        self.assertEqual(args.input_size, 384)
        self.assertEqual(args.yolo_model, "yolo26n-seg.pt")

    def test_draws_person_contour_over_existing_scene_overlay(self):
        frame = np.full((12, 12, 3), 40, dtype=np.uint8)
        result = type("Result", (), {"boxes": _Boxes(), "masks": _Masks()})()
        output, count = render_people(frame, result)
        self.assertEqual(count, 1)
        self.assertFalse(np.array_equal(output, frame))
        self.assertTrue(np.array_equal(frame, np.full_like(frame, 40)))

    def test_rejects_person_without_segmentation_mask(self):
        frame = np.zeros((12, 12, 3), dtype=np.uint8)
        result = type("Result", (), {"boxes": _Boxes(), "masks": None})()
        with self.assertRaisesRegex(RuntimeError, "person_segmentation_mask_missing"):
            render_people(frame, result)


if __name__ == "__main__":
    unittest.main()
