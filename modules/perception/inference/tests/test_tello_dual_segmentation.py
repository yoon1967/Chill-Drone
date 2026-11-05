import unittest
from unittest.mock import MagicMock, call, patch

import numpy as np

from modules.perception.inference.tello_dual_segmentation import DualSegmentationProcessor, parser, render_people


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
    def model_dependencies(self):
        torch, transformers, ultralytics = MagicMock(), MagicMock(), MagicMock()
        torch.cuda.is_available.return_value = False
        yolo = ultralytics.YOLO.return_value
        yolo.names, yolo.task = {0: 'person'}, 'segment'
        semantic = transformers.SegformerForSemanticSegmentation.from_pretrained.return_value
        semantic.to.return_value.eval.return_value.config.id2label = dict(enumerate(parser().parse_args([]).classes.split(',')))
        return torch, transformers, ultralytics

    def test_cached_scene_model_never_requests_online_metadata(self):
        torch, transformers, ultralytics = self.model_dependencies()
        args = parser().parse_args([])
        with patch.dict('sys.modules', {'torch': torch, 'torch.nn': torch.nn, 'torch.nn.functional': torch.nn.functional,
                                             'transformers': transformers, 'ultralytics': ultralytics}):
            DualSegmentationProcessor(args)
        transformers.SegformerImageProcessor.from_pretrained.assert_called_once_with(args.semantic_model, local_files_only=True)
        transformers.SegformerForSemanticSegmentation.from_pretrained.assert_called_once_with(args.semantic_model, local_files_only=True)

    def test_missing_processor_or_weights_can_download_on_first_use(self):
        for missing in ('SegformerImageProcessor', 'SegformerForSemanticSegmentation'):
            with self.subTest(missing=missing):
                torch, transformers, ultralytics = self.model_dependencies()
                loader = getattr(transformers, missing).from_pretrained
                model = loader.return_value
                loader.side_effect = [OSError('not cached'), model]
                args = parser().parse_args([])
                with patch.dict('sys.modules', {'torch': torch, 'torch.nn': torch.nn, 'torch.nn.functional': torch.nn.functional,
                                             'transformers': transformers, 'ultralytics': ultralytics}):
                    DualSegmentationProcessor(args)
                self.assertEqual(loader.call_args_list, [call(args.semantic_model, local_files_only=True), call(args.semantic_model)])

    def test_model_failure_is_not_retried_as_a_download(self):
        torch, transformers, ultralytics = self.model_dependencies()
        loader = transformers.SegformerForSemanticSegmentation.from_pretrained
        loader.side_effect = RuntimeError('invalid model')
        with patch.dict('sys.modules', {'torch': torch, 'torch.nn': torch.nn, 'torch.nn.functional': torch.nn.functional,
                                             'transformers': transformers, 'ultralytics': ultralytics}):
            with self.assertRaisesRegex(RuntimeError, 'invalid model'):
                DualSegmentationProcessor(parser().parse_args([]))
        self.assertEqual(loader.call_count, 1)

    def test_mapping_mode_does_not_download_when_local_scene_weights_are_missing(self):
        torch, transformers, ultralytics = self.model_dependencies()
        args = parser().parse_args([]); args.local_models_only = True
        loader = transformers.SegformerImageProcessor.from_pretrained
        loader.side_effect = OSError('not cached')
        with patch.dict('sys.modules', {'torch': torch, 'torch.nn': torch.nn, 'torch.nn.functional': torch.nn.functional,
                                       'transformers': transformers, 'ultralytics': ultralytics}):
            with self.assertRaisesRegex(RuntimeError, '로컬 공간 인식 모델'):
                DualSegmentationProcessor(args)
        loader.assert_called_once_with(args.semantic_model, local_files_only=True)

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
