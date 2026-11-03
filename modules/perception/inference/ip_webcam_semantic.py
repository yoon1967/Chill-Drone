"""Display ADE20K semantic segmentation from an Android IP Webcam stream."""
from __future__ import annotations

import argparse
import sys
import threading
import time

import cv2

from modules.perception.inference.semantic_overlay import (DEFAULT_CLASSES, render_overlay,
                                                            resolve_class_ids)


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


def resize_for_display(frame, display_width: int):
    """Resize a frame while preserving its aspect ratio; zero keeps the source size."""
    if not display_width or frame.shape[1] == display_width:
        return frame
    display_height = max(1, round(frame.shape[0] * display_width / frame.shape[1]))
    interpolation = cv2.INTER_AREA if display_width < frame.shape[1] else cv2.INTER_LINEAR
    return cv2.resize(frame, (display_width, display_height), interpolation=interpolation)


def smooth_filtered_label_map(probabilities, class_ids_tensor, background_ids_tensor, label_lookup,
                              threshold: float, output_size: tuple[int, int], functional):
    """Upsample selected-class and background scores before choosing smooth output labels."""
    selected_scores = probabilities.index_select(0, class_ids_tensor)
    if background_ids_tensor.numel():
        background_score = probabilities.index_select(0, background_ids_tensor).amax(
            dim=0, keepdim=True
        )
    else:
        background_score = probabilities.new_zeros((1, *probabilities.shape[1:]))
    scores = probabilities.new_empty((selected_scores.shape[0] + 1, *probabilities.shape[1:]))
    scores[0] = background_score[0]
    scores[1:] = selected_scores
    scores = functional.interpolate(
        scores.unsqueeze(0), size=output_size, mode="bilinear", align_corners=False
    )[0]
    confidence, choices = scores.max(dim=0)
    labels = label_lookup[choices]
    labels = labels.masked_fill((choices == 0) | (confidence < threshold), -1)
    return labels.int().cpu().numpy()


def run(args) -> int:
    try:
        import torch
        import torch.nn.functional as functional
        from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
    except ImportError as error:
        raise RuntimeError("semantic_dependencies_missing: install requirements.txt") from error

    if not 0 <= args.confidence <= 1 or not 0 <= args.alpha <= 1:
        raise ValueError("confidence_and_alpha_must_be_between_0_and_1")
    if args.input_size < 32:
        raise ValueError("input_size_must_be_at_least_32")
    if args.display_width < 0:
        raise ValueError("display_width_must_not_be_negative")
    if args.inference_every < 1:
        raise ValueError("inference_every_must_be_at_least_1")
    device = args.device if args.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu")
    processor = SegformerImageProcessor.from_pretrained(args.model)
    model = SegformerForSemanticSegmentation.from_pretrained(args.model).to(device).eval()
    id2label = {int(key): value for key, value in model.config.id2label.items()}
    class_ids = resolve_class_ids(id2label, args.classes.split(","))
    sorted_class_ids = sorted(class_ids)
    class_ids_tensor = torch.tensor(sorted_class_ids, device=device)
    background_ids_tensor = torch.tensor(
        [class_id for class_id in sorted(id2label) if class_id not in class_ids], device=device
    )
    label_lookup = torch.tensor([-1, *sorted_class_ids], device=device)
    capture = open_capture(args.source)
    stop_event = threading.Event()
    frame_condition = threading.Condition()
    frame_state = {"image": None, "number": -1, "captured_at": 0.0}
    mask_lock = threading.Lock()
    mask_state = {"image": None, "frame_number": -1, "captured_at": 0.0,
                  "inference_ms": 0.0}
    failure_lock = threading.Lock()
    failures = []

    def fail(message: str):
        with failure_lock:
            if not failures:
                failures.append(message)
        stop_event.set()
        with frame_condition:
            frame_condition.notify_all()

    def capture_frames():
        try:
            while not stop_event.is_set():
                ok, source_frame = capture.read()
                if not ok:
                    if not stop_event.is_set():
                        fail("ip_webcam_read_failed")
                    return
                source_frame = resize_for_display(source_frame, args.display_width)
                with frame_condition:
                    frame_state["image"] = source_frame
                    frame_state["number"] += 1
                    frame_state["captured_at"] = time.perf_counter()
                    frame_condition.notify_all()
        except Exception as error:  # pragma: no cover - hardware/backend failures
            fail(f"ip_webcam_capture_failed:{error}")

    def infer_latest_frames():
        last_inferred = None
        try:
            while not stop_event.is_set():
                with frame_condition:
                    ready = frame_condition.wait_for(
                        lambda: stop_event.is_set() or (
                            frame_state["image"] is not None
                            and (last_inferred is None
                                 or frame_state["number"] - last_inferred >= args.inference_every)
                        ), timeout=.1
                    )
                    if stop_event.is_set():
                        return
                    if not ready:
                        continue
                    frame = frame_state["image"].copy()
                    frame_number = frame_state["number"]
                    captured_at = frame_state["captured_at"]

                started = time.perf_counter()
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                inputs = processor(images=rgb, return_tensors="pt",
                                   size={"height": args.input_size, "width": args.input_size})
                inputs = {key: value.to(device) for key, value in inputs.items()}
                with torch.inference_mode():
                    logits = model(**inputs).logits
                    probabilities = torch.softmax(logits[0], dim=0)
                    label_map = smooth_filtered_label_map(
                        probabilities, class_ids_tensor, background_ids_tensor, label_lookup,
                        args.confidence, frame.shape[:2], functional
                    )
                inference_ms = (time.perf_counter() - started) * 1000
                with mask_lock:
                    mask_state["image"] = label_map
                    mask_state["frame_number"] = frame_number
                    mask_state["captured_at"] = captured_at
                    mask_state["inference_ms"] = inference_ms
                last_inferred = frame_number
        except Exception as error:  # pragma: no cover - model/backend failures
            fail(f"semantic_inference_failed:{error}")

    capture_thread = threading.Thread(target=capture_frames, name="semantic-capture", daemon=True)
    inference_thread = threading.Thread(target=infer_latest_frames, name="semantic-inference", daemon=True)
    print(f"source={args.source} model={args.model} device={device} classes={args.classes}")
    capture_thread.start()
    inference_thread.start()
    smoothed_fps, displayed_frames = 0.0, 0
    last_displayed = -1
    last_displayed_at = None
    try:
        while not stop_event.is_set():
            with frame_condition:
                has_frame = frame_condition.wait_for(
                    lambda: stop_event.is_set() or frame_state["number"] != last_displayed,
                    timeout=.05
                )
                if stop_event.is_set():
                    break
                if not has_frame or frame_state["image"] is None:
                    cv2.waitKey(1)
                    continue
                frame = frame_state["image"]
                frame_number = frame_state["number"]
                captured_at = frame_state["captured_at"]
                last_displayed = frame_number

            with mask_lock:
                label_map = mask_state["image"]
                mask_frame_number = mask_state["frame_number"]
                mask_captured_at = mask_state["captured_at"]
                inference_ms = mask_state["inference_ms"]

            if label_map is not None and label_map.shape == frame.shape[:2]:
                output, coverage = render_overlay(frame, label_map, id2label, args.alpha)
                mask_age_frames = max(0, frame_number - mask_frame_number)
                mask_age_ms = max(0.0, (captured_at - mask_captured_at) * 1000)
            else:
                output, coverage = frame.copy(), []
                mask_age_frames, mask_age_ms = 0, 0.0

            displayed_at = time.perf_counter()
            if last_displayed_at is not None:
                current_fps = 1.0 / max(displayed_at - last_displayed_at, 1e-6)
                smoothed_fps = (current_fps if displayed_frames == 0
                                else smoothed_fps * .9 + current_fps * .1)
                displayed_frames += 1
            last_displayed_at = displayed_at
            status = (f"{inference_ms:.0f} ms | {smoothed_fps:.1f} FPS | {device} | "
                      f"mask {mask_age_frames}f/{mask_age_ms:.0f}ms")
            cv2.putText(output, status,
                        (16, 30), cv2.FONT_HERSHEY_SIMPLEX, .72, (255, 255, 255), 2, cv2.LINE_AA)
            for row, item in enumerate(sorted(coverage, key=lambda value: -value["coverage"]), start=1):
                text = f"{item['label']}: {item['coverage'] * 100:.1f}%"
                cv2.putText(output, text, (16, 30 + row * 26), cv2.FONT_HERSHEY_SIMPLEX, .65,
                            (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("IP Webcam semantic segmentation - q/ESC to quit", output)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                stop_event.set()
                with frame_condition:
                    frame_condition.notify_all()
                break
        with failure_lock:
            if failures:
                raise RuntimeError(failures[0])
        return 0
    finally:
        stop_event.set()
        with frame_condition:
            frame_condition.notify_all()
        capture.release()
        capture_thread.join(timeout=1)
        inference_thread.join(timeout=1)
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
    value.add_argument("--input-size", type=int, default=512,
                       help="square model input size; smaller is faster but less detailed")
    value.add_argument("--display-width", type=int, default=0,
                       help="resize output to this width while preserving aspect ratio; 0 keeps source")
    value.add_argument("--inference-every", type=int, default=1,
                       help="run the model every N frames and reuse the latest mask between runs")
    return value


def main():
    try:
        return run(parser().parse_args())
    except (RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
