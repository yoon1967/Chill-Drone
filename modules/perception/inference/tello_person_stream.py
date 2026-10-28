"""Tello video receiver and YOLO person-detection stream for the local console."""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from modules.autonomy.tello_link.tello_link import TelloLink
from modules.autonomy.tello_link.tello_video import TelloVideo
from modules.autonomy.tello_link.flight_control import FlightController


class TelloPersonStream:
    """Own the Tello SDK/video sockets and retain only the latest detection frame."""

    def __init__(self, model="yolo26n-seg.pt", confidence=.25, image_size=640, link_factory=TelloLink,
                 video_factory=TelloVideo, detector_factory=None, flight_enabled=False,
                 flight_factory=FlightController):
        if not 0 <= confidence <= 1:
            raise ValueError("confidence_out_of_range")
        self.model_name, self.confidence, self.image_size = model, confidence, image_size
        self.link_factory, self.video_factory = link_factory, video_factory
        self.detector_factory = detector_factory
        self.flight_enabled, self.flight_factory = flight_enabled, flight_factory
        self.link = self.video = self.detector = self.flight = None
        self.worker = None
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.annotated = None
        self.detections = []
        self.frame_stamp = None
        self.inference_ms = None
        self.processed_frames = 0
        self.error = None

    @property
    def running(self):
        return self.worker is not None and self.worker.is_alive()

    def start(self):
        with self.condition:
            if self.running:
                return self.stats()
            self.error = None
        detector_factory = self.detector_factory
        if detector_factory is None:
            from ultralytics import YOLO
            detector_factory = YOLO
        detector = detector_factory(self.model_name)
        if detector.names.get(0) != "person":
            raise RuntimeError("model_class_zero_is_not_person")
        if getattr(detector, "task", None) != "segment":
            raise RuntimeError("model_task_is_not_segmentation")
        link = self.link_factory(allow_flight=self.flight_enabled).start() if self.flight_enabled else self.link_factory().start()
        command = link.send("command", timeout=3)
        if not command.ok:
            link.close()
            raise RuntimeError(f"tello_command_failed:{command.reason or command.response}")
        video = self.video_factory()
        try:
            video.start()
            stream = link.send("streamon", timeout=3)
            if not stream.ok:
                raise RuntimeError(f"tello_streamon_failed:{stream.reason or stream.response}")
        except Exception:
            try:
                link.send("streamoff", timeout=3)
            except Exception:
                pass
            video.stop()
            link.close()
            raise
        self.link, self.video, self.detector = link, video, detector
        if self.flight_enabled:
            self.flight = self.flight_factory(link).start()
        self.stop_event.clear()
        self.worker = threading.Thread(target=self._run, name="tello-person-detection", daemon=True)
        self.worker.start()
        return self.stats()

    def _run(self):
        last_stamp = None
        try:
            while not self.stop_event.is_set():
                snapshot = self.video.jpeg_snapshot()
                if snapshot is None or snapshot[1] == last_stamp:
                    self.stop_event.wait(.01)
                    continue
                jpeg, stamp, _ = snapshot
                last_stamp = stamp
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                started = time.perf_counter()
                result = self.detector.predict(source=frame, classes=[0], conf=self.confidence,
                                               imgsz=self.image_size, verbose=False)[0]
                output = frame.copy()
                frame_height, frame_width = frame.shape[:2]
                font_scale = max(.85, min(1.25, min(frame_width, frame_height) / 640))
                font_thickness = max(2, round(font_scale * 2))
                boxes = []
                if result.boxes is not None:
                    values = list(zip(result.boxes.xyxy.cpu().tolist(),
                                      result.boxes.conf.cpu().tolist(),
                                      result.boxes.cls.cpu().tolist()))
                    segments = list(result.masks.xy) if result.masks is not None else []
                    if values and len(values) != len(segments):
                        raise RuntimeError("person_segmentation_mask_missing")
                    for (xyxy, confidence, class_id), segment in zip(values, segments):
                        points = np.asarray(segment, dtype=np.float32).reshape(-1, 2)
                        polygon = [[round(float(x), 1), round(float(y), 1)] for x, y in points]
                        if len(polygon) < 3:
                            raise RuntimeError("person_segmentation_polygon_invalid")
                        contour = np.rint(points).astype(np.int32).reshape(-1, 1, 2)
                        cv2.polylines(output, [contour], True, (0, 255, 0), 2, cv2.LINE_AA)
                        anchor_x, anchor_y = contour[:, 0, :].min(axis=0)
                        label = f"person {float(confidence):.2f}"
                        origin = (int(anchor_x), max(round(28 * font_scale), int(anchor_y) - 8))
                        cv2.putText(output, label, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                                    (0, 0, 0), font_thickness + 3, cv2.LINE_AA)
                        cv2.putText(output, label, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                                    (0, 255, 0), font_thickness, cv2.LINE_AA)
                        boxes.append({"class_id": int(class_id), "label": "person",
                                      "confidence": round(float(confidence), 4),
                                      "xyxy": [round(float(value), 1) for value in xyxy],
                                      "polygons": [polygon]})
                ok, encoded = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                with self.condition:
                    self.annotated = encoded.tobytes()
                    self.frame_stamp = stamp
                    self.detections = boxes
                    self.inference_ms = round((time.perf_counter() - started) * 1000, 1)
                    self.processed_frames += 1
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = str(error)
                self.condition.notify_all()

    def latest(self, previous_stamp=None, timeout=1):
        with self.condition:
            self.condition.wait_for(lambda: self.frame_stamp != previous_stamp or self.error or not self.running,
                                    timeout=timeout)
            return self.annotated, self.frame_stamp

    def stats(self):
        video = self.video.stats() if self.video else {}
        state = self.link.latest_state() if self.link else None
        with self.condition:
            return {"enabled": True, "running": self.running, "model": self.model_name,
                    "task": "segment",
                    "flight_enabled": self.flight_enabled,
                    "confidence": self.confidence, "processed_frames": self.processed_frames,
                    "person_count": len(self.detections), "detections": list(self.detections),
                    "inference_ms": self.inference_ms, "error": self.error,
                    "video": video, "drone_state": state,
                    "flight": self.flight.status() if self.flight else None}

    def require_flight(self):
        if not self.flight:
            raise RuntimeError("tello_flight_disabled")
        return self.flight

    def stop(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=3)
        if self.flight:
            self.flight.close()
        if self.link:
            try:
                self.link.send("streamoff", timeout=3)
            except Exception:
                pass
        if self.video:
            self.video.stop()
        if self.link:
            self.link.close()
        self.worker = self.link = self.video = self.detector = self.flight = None
        with self.condition:
            self.condition.notify_all()
        return self.stats()
