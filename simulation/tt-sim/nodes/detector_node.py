#!/usr/bin/env python3
"""드론 카메라 영상에 YOLO를 돌려 사람(과 다른 물체)을 탐지한다.

입력  image_topic (기본 /tt/camera/image_delayed: 실제 TT처럼 175 ms 늦게 도착하는 영상)
      depth_topic (선택, /tt/depth): 깊이 영상. 사람 마스크 안의 거리를 재서 탐지마다 depth_m으로 낸다
출력  /tt/detections         탐지 결과 JSON (std_msgs/String): stamp, width, height, detections[cls, name, conf,
                             x1..y2, polygons(분할 모델일 때), depth_m(깊이 영상이 있을 때, 배율 맞추기 전 값)]
      /tt/person_events      팀 공통 사람 탐지 계약 person-detection-event/1.0 (contracts/perception). 웹·다른 모듈용
      /tt/detections/image   상자·윤곽을 그린 영상 (rqt_image_view 등으로 확인)
      out_dir                사람을 잡을 때마다 사진 PNG + events.jsonl (cooldown초에 한 번)

모델: weights/yolo26n-seg.pt(사람 인스턴스 분할, develop 콘솔과 같은 모델)가 있으면 그것, 없으면 yolo11n.pt(상자만).
분할이면 사람의 보이는 영역 윤곽을 내고, 깊이 영상이 있으면 그 영역의 가까운 쪽 거리(25% 백분위)를 depth_m으로 낸다.
비행 노드는 depth_m × 바닥 배율로 사람 위치를 잰다 (발이 안 보여도 됨).
오래된 프레임 처리: 처리 중에 들어온 프레임은 쌓지 않고 가장 최근 것 하나만 남긴다.

YOLO 가상환경의 파이썬으로 실행한다.
  ~/venvs/tt/bin/python nodes/detector_node.py --ros-args -p use_sim_time:=true -p out_dir:=/tmp/detections
"""
from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
from perception import PERSON, EventThrottle, YoloDetector, draw  # noqa: E402
from worldgen.images import write_png  # noqa: E402

try:   # 팀 공통 계약(develop, modules/perception/inference). 없는 사본이면 계약 토픽만 안 낸다
    sys.path.insert(0, str(REPO))
    from modules.perception.inference.detection_event import build_person_detection_event, utc_from_timestamp
except ImportError:   # pragma: no cover
    build_person_detection_event = None

MODELS = ("yolo26n-seg.pt", "yolo11n.pt")   # 앞의 것부터 있는 것을 쓴다
DEPTH_PERCENTILE = 25                       # 마스크 안 거리 중 가까운 쪽 (팔·몸통 앞면)
DEPTH_MAX_AGE = 1.0                         # 깊이 영상이 이보다 오래됐으면 거리를 안 잰다 (s)


def image_to_rgb(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)


def rgb_to_image(rgb: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def default_model() -> Path:
    for name in MODELS:
        if (ROOT / "weights" / name).exists():
            return ROOT / "weights" / name
    return ROOT / "weights" / MODELS[0]   # 없으면 처음 실행 때 이 이름으로 내려받는다 (인터넷 필요)


def mask_depth(mask: np.ndarray, depth: np.ndarray) -> float | None:
    """사람 마스크(영상 크기) 안 깊이의 가까운 쪽 거리. 깊이 영상이 더 작으면 마스크를 줄여 맞춘다."""
    if mask is None or depth is None:
        return None
    if mask.shape != depth.shape:
        import cv2

        mask = cv2.resize(mask.astype(np.uint8), (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    vals = depth[mask]
    vals = vals[np.isfinite(vals) & (vals > 0.1)]
    if vals.size < 20:
        return None
    return float(np.percentile(vals, DEPTH_PERCENTILE))


class Detector(Node):
    def __init__(self) -> None:
        super().__init__("tt_detector")
        self.declare_parameter("image_topic", "/tt/camera/image_delayed")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("model", str(default_model()))
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("out_dir", "")
        self.declare_parameter("person_cooldown", 2.0)

        model = Path(self.get_parameter("model").value)
        model.parent.mkdir(parents=True, exist_ok=True)   # 처음 실행 때 이 위치로 내려받는다
        self.yolo = YoloDetector(model, conf=float(self.get_parameter("conf").value))
        self.model_info = {"name": model.name, "version": _ultralytics_version(),
                           "weights_sha256": hashlib.sha256(model.read_bytes()).hexdigest() if model.exists() else None}
        self.get_logger().info(f"YOLO {model.name} 준비 ({self.yolo.device}, "
                               f"{'사람 윤곽 분할' if self.yolo.task == 'segment' else '상자만'}"
                               f"{', 계약 /tt/person_events' if build_person_detection_event else ''})")

        out = self.get_parameter("out_dir").value
        self.out_dir = Path(out) if out else None
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)
        self.throttle = EventThrottle(float(self.get_parameter("person_cooldown").value))

        self.latest: Image | None = None
        self.depth: tuple[float, np.ndarray] | None = None   # (받은 시각, 깊이 영상)
        self.dropped = 0
        self.frames = 0
        self.times: list[float] = []
        self.input_id = f"tt-{int(time.time())}"
        self.det_pub = self.create_publisher(String, "/tt/detections", 10)
        self.event_pub = self.create_publisher(String, "/tt/person_events", 10)
        # 신뢰성 있는(reliable) QoS로 내야 rqt_image_view 같은 일반 구독자와도 맞는다
        self.img_pub = self.create_publisher(Image, "/tt/detections/image", 2)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos_profile_sensor_data)
        depth_topic = self.get_parameter("depth_topic").value
        if depth_topic:
            self.create_subscription(Image, depth_topic, self.on_depth, 2)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.step)
        self.create_timer(10.0, self.report)

    def on_image(self, msg: Image) -> None:
        if self.latest is not None:
            self.dropped += 1   # 처리하지 못한 이전 프레임은 버린다
        self.latest = msg

    def on_depth(self, msg: Image) -> None:
        if msg.encoding not in ("32FC1", ""):
            return
        self.depth = (time.monotonic(), np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width))

    def step(self) -> None:
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        rgb = image_to_rgb(msg)
        t0 = time.perf_counter()
        dets = self.yolo.detect(rgb)
        self.times.append(time.perf_counter() - t0)
        self.frames += 1

        now = self.get_clock().now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age_ms = (now.nanoseconds * 1e-9 - stamp) * 1000
        depth = self.depth[1] if self.depth and time.monotonic() - self.depth[0] < DEPTH_MAX_AGE else None
        items = []
        for d in dets:
            item = d.to_dict()
            if d.cls == PERSON and depth is not None:
                dist = mask_depth(d.mask, depth)
                if dist is not None:
                    item["depth_m"] = round(dist, 3)
            items.append(item)
        payload = {"stamp": round(stamp, 3), "frame_age_ms": round(age_ms, 1),
                   "width": int(msg.width), "height": int(msg.height), "detections": items}
        self.det_pub.publish(String(data=json.dumps(payload)))
        self.publish_event(msg, dets)
        annotated = draw(rgb, dets)
        self.img_pub.publish(rgb_to_image(annotated, msg.header))

        people = [d for d in dets if d.cls == PERSON]
        if people and self.throttle.ready(PERSON, now.nanoseconds * 1e-9):
            best = max(people, key=lambda d: d.conf)
            dist = next((i["depth_m"] for i in items if i.get("depth_m") is not None), None)
            self.get_logger().info(f"사람 탐지 {len(people)}명 (최고 신뢰도 {best.conf:.2f}, 프레임 지연 {age_ms:.0f} ms"
                                   f"{f', 깊이 약 {dist:.1f} m' if dist is not None else ''})")
            if self.out_dir:
                name = f"person_{stamp:.2f}.png"
                write_png(self.out_dir / name, annotated.shape[1], annotated.shape[0],
                          [bytearray(r.tobytes()) for r in annotated])
                with open(self.out_dir / "events.jsonl", "a", encoding="utf-8") as f:
                    f.write(json.dumps({**payload, "image": name}, ensure_ascii=False) + "\n")

    def publish_event(self, msg: Image, dets) -> None:
        """팀 공통 계약(person-detection-event/1.0)으로 사람만 낸다. 상자는 영상 안으로 자르고, 윤곽이 없으면(상자 모델)
        상자 네 모서리를 윤곽으로 둔다."""
        if build_person_detection_event is None:
            return
        w, h = int(msg.width), int(msg.height)
        people = []
        for d in dets:
            if d.cls != PERSON:
                continue
            x1, y1 = max(0.0, min(w - 1.0, d.x1)), max(0.0, min(h - 1.0, d.y1))
            x2, y2 = max(x1 + 1.0, min(float(w), d.x2)), max(y1 + 1.0, min(float(h), d.y2))
            polys = [[[min(float(w), max(0.0, float(u))), min(float(h), max(0.0, float(v)))] for u, v in poly]
                     for poly in d.polygons if len(poly) >= 3]
            if not polys:
                polys = [[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]]
            people.append({"confidence": round(min(1.0, max(0.0, d.conf)), 4), "xyxy": [x1, y1, x2, y2], "polygons": polys})
        try:
            event = build_person_detection_event(input_id=self.input_id, frame_id=f"frame-{self.frames:06d}",
                                                 detected_at_utc=utc_from_timestamp(time.time()), image_width=w,
                                                 image_height=h, model=self.model_info, detections=people)
        except ValueError as err:   # 계약에 안 맞는 값은 내지 않는다 (탐지 자체는 /tt/detections로 이미 나갔다)
            self.get_logger().warn(f"사람 탐지 계약 형식 오류: {err}")
            return
        self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def report(self) -> None:
        if self.times:
            ms = 1000 * np.array(self.times)
            self.get_logger().info(f"추론 {len(ms)}회, 평균 {ms.mean():.0f} ms, 최대 {ms.max():.0f} ms, "
                                   f"버린 프레임 {self.dropped}개")
            self.times.clear()


def _ultralytics_version() -> str | None:
    try:
        import ultralytics

        return getattr(ultralytics, "__version__", None)
    except ImportError:   # pragma: no cover
        return None


def main() -> None:
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
