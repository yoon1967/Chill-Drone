"""Display ADE20K semantic segmentation from an Android IP Webcam stream."""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

from modules.perception.inference.semantic_overlay import (DEFAULT_CLASSES, filtered_label_map,
                                                            render_overlay, resolve_class_ids)


DEFAULT_MODEL = "nvidia/segformer-b0-finetuned-ade-512-512"


def open_capture(source: str):
    capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG,
                               [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("ip_webcam_open_failed")
    return capture


def run(args) -> int:
    try:
        import torch
        import torch.nn.functional as functional
        from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
    except ImportError as error:
        raise RuntimeError("semantic_dependencies_missing: install requirements.txt") from error

    if not 0 <= args.confidence <= 1 or not 0 <= args.alpha <= 1:
        raise ValueError("confidence_and_alpha_must_be_between_0_and_1")
    device = args.device if args.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu")
    processor = SegformerImageProcessor.from_pretrained(args.model)
    model = SegformerForSemanticSegmentation.from_pretrained(args.model).to(device).eval()
    id2label = {int(key): value for key, value in model.config.id2label.items()}
    class_ids = resolve_class_ids(id2label, args.classes.split(","))
    capture = open_capture(args.source)
    smoothed_fps, frames = 0.0, 0
    print(f"source={args.source} model={args.model} device={device} classes={args.classes}")
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("ip_webcam_read_failed")
            started = time.perf_counter()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            inputs = processor(images=rgb, return_tensors="pt")
            inputs = {key: value.to(device) for key, value in inputs.items()}
            with torch.inference_mode():
                logits = model(**inputs).logits
                logits = functional.interpolate(logits, size=frame.shape[:2], mode="bilinear",
                                                align_corners=False)
                probabilities = torch.softmax(logits[0], dim=0).cpu().numpy()
            label_map = filtered_label_map(probabilities, class_ids, args.confidence)
            output, coverage = render_overlay(frame, label_map, id2label, args.alpha)
            elapsed = time.perf_counter() - started
            current_fps = 1.0 / max(elapsed, 1e-6)
            smoothed_fps = current_fps if frames == 0 else smoothed_fps * .9 + current_fps * .1
            frames += 1
            cv2.putText(output, f"{elapsed * 1000:.0f} ms | {smoothed_fps:.1f} FPS | {device}",
                        (16, 30), cv2.FONT_HERSHEY_SIMPLEX, .72, (255, 255, 255), 2, cv2.LINE_AA)
            for row, item in enumerate(sorted(coverage, key=lambda value: -value["coverage"]), start=1):
                text = f"{item['label']}: {item['coverage'] * 100:.1f}%"
                cv2.putText(output, text, (16, 30 + row * 26), cv2.FONT_HERSHEY_SIMPLEX, .65,
                            (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("IP Webcam semantic segmentation - q/ESC to quit", output)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return 0
    finally:
        capture.release()
        cv2.destroyAllWindows()


def parser():
    value = argparse.ArgumentParser(description="IP Webcam ADE20K semantic segmentation experiment")
    value.add_argument("--source", required=True,
                       help="IP Webcam MJPEG URL, usually http://PHONE_IP:8080/video")
    value.add_argument("--model", default=DEFAULT_MODEL)
    value.add_argument("--classes", default=",".join(DEFAULT_CLASSES),
                       help="comma-separated ADE20K labels")
    value.add_argument("--confidence", type=float, default=.45)
    value.add_argument("--alpha", type=float, default=.5)
    value.add_argument("--device", default="auto", help="auto, cpu, cuda, or cuda:0")
    return value


def main():
    try:
        return run(parser().parse_args())
    except (RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
