"""Run YOLO person instances and SegFormer scene classes on one Tello stream."""
from __future__ import annotations

import argparse
import sys
import threading
import time

import cv2
import numpy as np

from modules.autonomy.tello_link.tello_link import TelloLink
from modules.autonomy.tello_link.tello_video import TelloVideo
from modules.perception.inference.ip_webcam_semantic import (DEFAULT_MODEL,
                                                              smooth_filtered_label_map)
from modules.perception.inference.semantic_overlay import (DEFAULT_CLASSES, render_overlay,
                                                            resolve_class_ids)
from modules.perception.inference.tello_person_stream import _mask_polygons


def render_people(frame: np.ndarray, result) -> tuple[np.ndarray, int]:
    """Draw every YOLO person mask contour without replacing the semantic overlay."""
    output = frame.copy()
    boxes, masks = result.boxes, result.masks
    if boxes is None:
        return output, 0
    values = list(zip(boxes.xyxy.cpu().tolist(), boxes.conf.cpu().tolist()))
    if values and (masks is None or len(masks.xy) != len(values)):
        raise RuntimeError("person_segmentation_mask_missing")
    for index, (_, confidence) in enumerate(values):
        point_sets = _mask_polygons(masks, index, frame.shape[1], frame.shape[0])
        contours = [np.rint(points).astype(np.int32).reshape(-1, 1, 2)
                    for points in point_sets if len(points) >= 3]
        if not contours:
            raise RuntimeError("person_segmentation_polygon_invalid")
        cv2.polylines(output, contours, True, (0, 255, 0), 2, cv2.LINE_AA)
        all_points = np.concatenate([contour[:, 0, :] for contour in contours])
        anchor_x, anchor_y = all_points.min(axis=0)
        label = f"person {float(confidence):.2f}"
        origin = (int(anchor_x), max(24, int(anchor_y) - 8))
        cv2.putText(output, label, origin, cv2.FONT_HERSHEY_SIMPLEX, .7,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(output, label, origin, cv2.FONT_HERSHEY_SIMPLEX, .7,
                    (0, 255, 0), 2, cv2.LINE_AA)
    return output, len(values)


class DualSegmentationProcessor:
    """Apply YOLO person contours and SegFormer scene masks to BGR frames.

    Model imports and loading stay inside the constructor so the ordinary Tello
    check server does not require the AI dependencies unless explicitly enabled.
    """

    def __init__(self, args):
        try:
            import torch
            import torch.nn.functional as functional
            from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError("dual_segmentation_dependencies_missing: install requirements.txt") from error

        if not 0 <= args.confidence <= 1 or not 0 <= args.semantic_confidence <= 1:
            raise ValueError("confidence_must_be_between_0_and_1")
        if args.semantic_every < 1 or args.input_size < 32:
            raise ValueError("semantic_every_and_input_size_must_be_positive")

        self.torch = torch
        self.functional = functional
        self.device = args.device if args.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu")
        self.args = args
        self.yolo = YOLO(args.yolo_model)
        if self.yolo.names.get(0) != "person" or getattr(self.yolo, "task", None) != "segment":
            raise RuntimeError("yolo_model_must_segment_person_class_zero")
        # Tello Wi-Fi normally has no internet. Cached models must not wait for
        # remote metadata or retry optional files before processing the stream.
        try:
            self.processor = SegformerImageProcessor.from_pretrained(args.semantic_model, local_files_only=True)
            semantic = SegformerForSemanticSegmentation.from_pretrained(args.semantic_model, local_files_only=True)
        except OSError:
            if getattr(args, 'local_models_only', False):
                raise RuntimeError('로컬 공간 인식 모델을 읽지 못했습니다. 인터넷 연결 상태에서 '
                    'start_3d_mapping.cmd --prepare-models로 모델을 준비하세요.') from None
            print('Downloading the scene recognition model for first use. Keep internet available until setup finishes.', flush=True)
            self.processor = SegformerImageProcessor.from_pretrained(args.semantic_model)
            semantic = SegformerForSemanticSegmentation.from_pretrained(args.semantic_model)
        self.semantic = semantic.to(self.device).eval()
        if getattr(args, 'fp16', False) and self.device == 'cuda':
            self.semantic.half()
        self.id2label = {int(key): value for key, value in self.semantic.config.id2label.items()}
        class_ids = sorted(resolve_class_ids(self.id2label, args.classes.split(",")))
        self.class_ids_tensor = torch.tensor(class_ids, device=self.device)
        self.background_ids_tensor = torch.tensor(
            [class_id for class_id in sorted(self.id2label) if class_id not in class_ids], device=self.device
        )
        self.label_lookup = torch.tensor([-1, *class_ids], device=self.device)
        self.semantic_mask = None
        self.semantic_frame = -1
        self.frame_number = 0
        self.yolo_ms = self.semantic_ms = 0.0
        self._lock = threading.Lock()

    def process(self, frame: np.ndarray, *, structured=False):
        """Return an annotated copy. Calls are serialized because model state is shared."""
        with self._lock:
            self.frame_number += 1
            started = time.perf_counter()
            yolo_result = self.yolo.predict(source=frame, classes=[0], conf=self.args.confidence,
                                            imgsz=self.args.yolo_size, verbose=False)[0]
            self.yolo_ms = (time.perf_counter() - started) * 1000

            # Mapping must never project a mask from another camera frame into 3D.
            if structured or self.semantic_mask is None or self.frame_number - self.semantic_frame >= self.args.semantic_every:
                started = time.perf_counter()
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                inputs = self.processor(images=rgb, return_tensors="pt",
                                        size={"height": self.args.input_size, "width": self.args.input_size})
                inputs = {key: value.to(device=self.device, dtype=self.semantic.dtype) for key, value in inputs.items()}
                with self.torch.inference_mode():
                    probabilities = self.torch.softmax(self.semantic(**inputs).logits[0], dim=0)
                    self.semantic_mask, self.semantic_confidence = smooth_filtered_label_map(
                        probabilities, self.class_ids_tensor, self.background_ids_tensor, self.label_lookup,
                        self.args.semantic_confidence, frame.shape[:2], self.functional, return_confidence=True
                    )
                self.semantic_frame = self.frame_number
                self.semantic_ms = (time.perf_counter() - started) * 1000

            if structured:
                return {'labels': self.semantic_mask.copy(), 'confidence': self.semantic_confidence.copy(),
                        'people': yolo_result, 'id2label': self.id2label,
                        'inference_ms': self.yolo_ms + self.semantic_ms}

            output, coverage = render_overlay(frame, self.semantic_mask, self.id2label, self.args.alpha)
            output, people = render_people(output, yolo_result)
            status = (f"YOLO {self.yolo_ms:.0f}ms | SegFormer {self.semantic_ms:.0f}ms/"
                      f"{self.frame_number - self.semantic_frame}f | people {people} | {self.device}")
            cv2.putText(output, status, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, .65,
                        (255, 255, 255), 2, cv2.LINE_AA)
            for row, item in enumerate(sorted(coverage, key=lambda value: -value["coverage"]), 1):
                text = f"{item['label']}: {item['coverage'] * 100:.1f}%"
                cv2.putText(output, text, (16, 30 + row * 24), cv2.FONT_HERSHEY_SIMPLEX,
                            .58, (255, 255, 255), 2, cv2.LINE_AA)
            return output


def run(args, link_factory=TelloLink, video_factory=TelloVideo) -> int:
    inference = DualSegmentationProcessor(args)

    link, video = link_factory().start(), video_factory()
    command = link.send("command", timeout=3)
    if not command.ok:
        link.close()
        raise RuntimeError(f"tello_command_failed:{command.reason or command.response}")
    try:
        video.start()
        stream = link.send("streamon", timeout=3)
        if not stream.ok:
            raise RuntimeError(f"tello_streamon_failed:{stream.reason or stream.response}")
        last_stamp = None
        while True:
            snapshot = video.jpeg_snapshot()
            if snapshot is None or snapshot[1] == last_stamp:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    return 0
                time.sleep(.005)
                continue
            jpeg, last_stamp, _ = snapshot
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            output = inference.process(frame)
            cv2.imshow("Tello YOLO + SegFormer - q/ESC to quit", output)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return 0
    finally:
        try:
            link.send("streamoff", timeout=3)
        except Exception:
            pass
        video.stop()
        link.close()
        cv2.destroyAllWindows()


def parser():
    value = argparse.ArgumentParser(description="Tello YOLO person + SegFormer scene segmentation")
    value.add_argument("--yolo-model", default="yolo26n-seg.pt")
    value.add_argument("--semantic-model", default=DEFAULT_MODEL)
    value.add_argument("--classes", default=",".join(DEFAULT_CLASSES))
    value.add_argument("--confidence", type=float, default=.25, help="YOLO person confidence")
    value.add_argument("--semantic-confidence", type=float, default=.45)
    value.add_argument("--alpha", type=float, default=.5)
    value.add_argument("--device", default="auto")
    value.add_argument("--yolo-size", type=int, default=640)
    value.add_argument("--input-size", type=int, default=384, help="SegFormer square input size")
    value.add_argument("--semantic-every", type=int, default=3,
                       help="refresh the semantic mask every N processed frames")
    return value


def main():
    try:
        return run(parser().parse_args())
    except (RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
