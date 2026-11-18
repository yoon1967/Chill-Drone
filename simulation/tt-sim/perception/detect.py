"""YOLO 객체 탐지 도우미. ROS와 무관해서 녹화 영상·실기체 영상에도 그대로 쓴다.

- YoloDetector: ultralytics YOLO를 감싼다 (GPU가 있으면 GPU로).
- draw: 탐지 상자를 그린 그림을 만든다.
- EventThrottle: 같은 대상을 연속으로 잡았을 때 이벤트를 한 번만 내보낸다.
- camera_matrix / project: 드론 카메라 좌표 → 영상 픽셀 (평가·위치 추정용).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

PERSON = 0   # COCO 클래스 번호


@dataclass
class Detection:
    cls: int
    name: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float
    # 분할 모델(yolo26n-seg, develop 콘솔과 같은 모델)이면 보이는 영역의 윤곽(픽셀 좌표, 여러 조각일 수 있다)과
    # 같은 크기의 bool 마스크. 마스크는 깊이 영상에서 그 사람까지의 거리를 재는 데 쓰고 JSON에는 넣지 않는다
    polygons: list = field(default_factory=list)
    mask: object = field(default=None, repr=False, compare=False)

    def contains(self, u: float, v: float, margin: float = 0.0) -> bool:
        return self.x1 - margin <= u <= self.x2 + margin and self.y1 - margin <= v <= self.y2 + margin

    def to_dict(self) -> dict:
        d = {"cls": self.cls, "name": self.name, "conf": round(self.conf, 3)}
        for k in ("x1", "y1", "x2", "y2"):
            d[k] = round(getattr(self, k), 1)
        if self.polygons:
            d["polygons"] = [[[round(float(u), 1), round(float(v), 1)] for u, v in poly] for poly in self.polygons]
        return d


class YoloDetector:
    def __init__(self, model: str | Path, conf: float = 0.4, device: str | None = None):
        import torch
        from ultralytics import YOLO

        self.model = YOLO(str(model))
        self.conf = conf
        self.device = device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.task = getattr(self.model, "task", "detect")   # "segment"면 사람 윤곽·마스크도 낸다

    def detect(self, rgb: np.ndarray) -> list[Detection]:
        """rgb: (H, W, 3) uint8. ultralytics는 BGR을 기대하므로 채널을 뒤집어 넘긴다."""
        result = self.model.predict(rgb[:, :, ::-1], conf=self.conf, device=self.device, verbose=False)[0]
        names = result.names
        h, w = rgb.shape[:2]
        masks = result.masks
        out = []
        for i, ((x1, y1, x2, y2), c, k) in enumerate(zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                                                         result.boxes.cls.tolist())):
            det = Detection(int(k), names[int(k)], float(c), x1, y1, x2, y2)
            if masks is not None and i < len(masks.data):
                det.mask, det.polygons = mask_polygons(masks.data[i].cpu().numpy(), w, h)
            out.append(det)
        return out


def mask_polygons(raw: np.ndarray, width: int, height: int):
    """모델 마스크(모델 입력 크기) → (영상 크기 bool 마스크, 바깥 윤곽 다각형들). develop 콘솔과 같은 방식."""
    import cv2

    binary = cv2.resize(raw.astype(np.float32), (width, height), interpolation=cv2.INTER_NEAREST) > 0.5
    contours, _ = cv2.findContours(binary.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = [c.reshape(-1, 2).astype(np.float32) for c in contours if len(c) >= 3]
    return binary, polys


def draw(rgb: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """사람은 빨강, 나머지는 노랑 상자. 반환은 새 배열."""
    import cv2

    img = np.ascontiguousarray(rgb.copy())
    for d in detections:
        color = (230, 40, 40) if d.cls == PERSON else (240, 200, 0)
        p1, p2 = (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2))
        cv2.rectangle(img, p1, p2, color, 2)
        label = f"{d.name} {d.conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (p1[0], max(0, p1[1] - th - 6)), (p1[0] + tw + 4, p1[1]), color, -1)
        cv2.putText(img, label, (p1[0] + 2, p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


class EventThrottle:
    """같은 종류 탐지는 cooldown초에 한 번만 이벤트로 낸다 (연속 프레임 중복 억제)."""

    def __init__(self, cooldown: float = 2.0):
        self.cooldown = cooldown
        self.last: dict[int, float] = {}

    def ready(self, cls: int, now: float) -> bool:
        if now - self.last.get(cls, -math.inf) < self.cooldown:
            return False
        self.last[cls] = now
        return True


def camera_matrix(width: int, height: int, hfov: float) -> tuple[float, float, float]:
    """(초점거리 fx=fy, cx, cy). 정사각 픽셀 가정."""
    f = (width / 2) / math.tan(hfov / 2)
    return f, width / 2, height / 2


def project(cam_pos, cam_yaw: float, point, width: int, height: int, hfov: float):
    """수평으로 놓인 전방 카메라(위치·요)에서 월드 점이 찍히는 픽셀 (u, v). 뒤에 있으면 None."""
    f, cx, cy = camera_matrix(width, height, hfov)
    dx, dy, dz = (point[i] - cam_pos[i] for i in range(3))
    forward = math.cos(cam_yaw) * dx + math.sin(cam_yaw) * dy
    left = -math.sin(cam_yaw) * dx + math.cos(cam_yaw) * dy
    if forward <= 1e-6:
        return None
    return cx - f * left / forward, cy - f * dz / forward
