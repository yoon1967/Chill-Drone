"""드론 게이트웨이 영상에 사람 탐지 파트의 YOLO+SegFormer를 붙인다. Tello에는 붙지 않는다.

게이트웨이 `/snapshot.jpg`를 받아 `tello_dual_segmentation.DualSegmentationProcessor`를 그대로 돌리고
- 합성 영상(사람 윤곽선 + 벽·바닥·문 등 장면 분할)은 `PUT /api/perception/overlay.jpg` → 관제 화면 "AI 합성"
- 사람 결과는 person-detection-event/1.0으로 `POST /api/detections` → 발견 목록·사진·영상 위 윤곽선
input_id·frame_id·detected_at_utc는 게이트웨이 헤더 값을 그대로 쓴다(게이트웨이가 같은 프레임의 사진·드론 상태를 붙인다).
Tello에는 게이트웨이만 붙으므로 조종권 관리자가 그대로 유지된다(Tello는 처음 command를 보낸 쪽에만 답한다).

    python -m modules.perception.inference.gateway_ai_client                         # http://127.0.0.1:8010
    python -m modules.perception.inference.gateway_ai_client --max-hz 8 --confidence 0.4
    python -m modules.perception.inference.gateway_ai_client --no-overlay           # 사람 결과만 보내기

모델 옵션(--yolo-model, --semantic-model, --classes, --confidence, --semantic-every 등)은
tello_dual_segmentation.py와 같다. 영상이 꺼져 있으면(이륙 전 등) 기다렸다가 다시 시도한다.
사람이 없어도 결과를 보낸다(탐지가 동작 중이라는 근거).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from modules.perception.inference.detection_event import build_person_detection_event  # noqa: E402
from modules.perception.inference.tello_person_stream import _mask_polygons  # noqa: E402

JPEG_QUALITY = 75


class _KeepLastResult:
    """processor.yolo 대신 끼워 마지막 predict 결과를 남긴다. 같은 프레임에 YOLO를 두 번 돌리지 않으려고."""

    def __init__(self, model):
        self.model, self.last = model, None

    def predict(self, *args, **kwargs):
        self.last = self.model.predict(*args, **kwargs)
        return self.last

    def __getattr__(self, name):
        return getattr(self.model, name)


def people_from_result(result, width: int, height: int) -> list[dict]:
    """YOLO 세그멘테이션 결과 → detection_event 입력(confidence, xyxy, polygons). 화면 밖 좌표는 자른다."""
    if result is None or result.boxes is None or result.masks is None:
        return []
    people = []
    for i, (xyxy, conf) in enumerate(zip(result.boxes.xyxy.cpu().tolist(), result.boxes.conf.cpu().tolist())):
        polys = []
        for pts in _mask_polygons(result.masks, i, width, height):
            pts = cv2.approxPolyDP(np.asarray(pts, np.float32).reshape(-1, 1, 2), 1.5, True).reshape(-1, 2)
            if len(pts) >= 3:   # 점 수 줄이기(1.5px 이내). 이벤트가 작아진다
                polys.append([[round(float(min(max(x, 0), width)), 1), round(float(min(max(y, 0), height)), 1)]
                              for x, y in pts])
        x1, y1, x2, y2 = (round(float(v), 1) for v in xyxy)
        x1, x2 = max(0.0, x1), min(float(width), x2)
        y1, y2 = max(0.0, y1), min(float(height), y2)
        if polys and x2 > x1 and y2 > y1:
            people.append({"confidence": round(float(conf), 3), "xyxy": [x1, y1, x2, y2], "polygons": polys})
    return people


class GatewayAIClient:
    """게이트웨이 프레임 하나를 처리해 결과를 보낸다. 모델은 밖에서 받는다(시험에서는 가짜)."""

    def __init__(self, gateway: str, processor, model_info: dict, *, overlay: bool = True, timeout: float = 3.0):
        self.base = gateway.rstrip("/")
        self.processor = processor
        if not isinstance(processor.yolo, _KeepLastResult):
            processor.yolo = _KeepLastResult(processor.yolo)
        self.model_info = dict(model_info)
        self.overlay = overlay
        self.timeout = timeout
        self.last_frame: str | None = None

    def step(self) -> dict:
        """프레임 1장 처리. 결과: {"status": "no_video"|"no_gateway"|"same_frame"|"bad_frame"|"sent", ...}"""
        try:
            with urllib.request.urlopen(self.base + "/snapshot.jpg", timeout=self.timeout) as r:
                h = {k.lower(): v for k, v in r.headers.items()}
                jpeg = r.read()
        except urllib.error.HTTPError as e:   # 404 --no-video, 503 꺼짐·첫 프레임 전
            return {"status": "no_video", "code": e.code, "detail": e.read().decode("utf-8", "replace")[:200]}
        except OSError as e:
            return {"status": "no_gateway", "detail": str(e)}
        if h.get("x-frame-id") == self.last_frame:
            return {"status": "same_frame"}
        self.last_frame = h.get("x-frame-id")
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return {"status": "bad_frame"}
        t0 = time.perf_counter()
        drawn = self.processor.process(frame)
        infer_ms = (time.perf_counter() - t0) * 1000
        result = self.processor.yolo.last
        people = people_from_result(result[0] if result else None, frame.shape[1], frame.shape[0])
        frame_headers = {"X-Input-Id": h["x-input-id"], "X-Frame-Id": h["x-frame-id"]}
        out = {"status": "sent", "frame_id": h["x-frame-id"], "people": len(people), "infer_ms": round(infer_ms)}
        if self.overlay:
            ok, enc = cv2.imencode(".jpg", drawn, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                out["overlay"] = self._send("PUT", "/api/perception/overlay.jpg", enc.tobytes(),
                                            {"Content-Type": "image/jpeg", **frame_headers})
        event = build_person_detection_event(
            input_id=h["x-input-id"], frame_id=h["x-frame-id"],
            detected_at_utc=h["x-frame-time"].replace("+00:00", "Z"),
            image_width=frame.shape[1], image_height=frame.shape[0], model=self.model_info, detections=people)
        out["detection"] = self._send("POST", "/api/detections", json.dumps(event).encode(),
                                      {"Content-Type": "application/json"})
        return out

    def _send(self, method: str, path: str, body: bytes, headers: dict) -> dict:
        req = urllib.request.Request(self.base + path, body, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return {"ok": False, "code": e.code, "detail": e.read().decode("utf-8", "replace")[:200]}
        except (OSError, ValueError) as e:
            return {"ok": False, "detail": str(e)}


def model_info(args) -> dict:
    """계약의 model 칸. 두 모델을 함께 쓰므로 이름에 둘 다 적는다. 가중치 해시는 아직 계산하지 않는다(null)."""
    version = None
    try:
        import ultralytics
        version = f"ultralytics-{ultralytics.__version__}"
    except (ImportError, AttributeError):
        pass
    return {"name": f"{os.path.basename(str(args.yolo_model))}+{args.semantic_model}",
            "version": version, "weights_sha256": None}


def run(args, dual_args) -> None:
    from modules.perception.inference.tello_dual_segmentation import DualSegmentationProcessor
    processor = DualSegmentationProcessor(dual_args)   # 첫 프레임은 모델을 올리느라 느리다
    client = GatewayAIClient(args.gateway, processor, model_info(dual_args), overlay=not args.no_overlay)
    print(f"사람 탐지 연결: 게이트웨이 {client.base} · 장치 {processor.device} · YOLO {dual_args.yolo_model} · "
          f"장면 {dual_args.semantic_model} · 합성 영상 {'보냄' if client.overlay else '안 보냄'}")
    period, waiting = 1.0 / args.max_hz, None
    stats = {"n": 0, "people": 0, "infer": 0.0, "t": time.monotonic()}
    while True:
        started = time.monotonic()
        out = client.step()
        if out["status"] in ("no_video", "no_gateway"):
            if out["status"] != waiting:
                print("영상 없음, 기다리는 중" if out["status"] == "no_video" else f"게이트웨이 연결 실패: {out['detail']}")
            waiting = out["status"]
            time.sleep(1)
            continue
        if out["status"] == "same_frame":
            time.sleep(0.01)
            continue
        waiting = None
        if out["status"] == "sent":
            stats["n"] += 1
            stats["people"] += bool(out["people"])
            stats["infer"] += out["infer_ms"]
            for key in ("overlay", "detection"):
                if key in out and not out[key].get("ok", True):
                    print(f"전송 거절({key}): {out[key]}")
            if out["people"] and out["detection"].get("finding_id"):
                print(f"{out['frame_id']} 사람 {out['people']}명 → {out['detection']['finding_id']}")
        if time.monotonic() - stats["t"] >= 5:
            dt = time.monotonic() - stats["t"]
            print(f"[5초] {stats['n'] / dt:.1f}장/s · 처리 평균 {stats['infer'] / max(1, stats['n']):.0f}ms · "
                  f"사람 있는 장 {stats['people']}")
            stats.update(n=0, people=0, infer=0.0, t=time.monotonic())
        time.sleep(max(0.0, period - (time.monotonic() - started)))


def main():
    ap = argparse.ArgumentParser(description="게이트웨이 영상으로 사람 탐지 + 장면 분할 (사람 탐지 파트 모델)",
                                 epilog="모델 옵션은 tello_dual_segmentation.py와 같습니다.")
    ap.add_argument("--gateway", default="http://127.0.0.1:8010")
    ap.add_argument("--max-hz", type=float, default=10.0, help="초당 최대 처리 장수")
    ap.add_argument("--no-overlay", action="store_true", help="합성 영상은 보내지 않고 사람 결과만")
    args, rest = ap.parse_known_args()
    from modules.perception.inference.tello_dual_segmentation import parser as dual_parser
    dual_args = dual_parser().parse_args(rest)
    try:
        run(args, dual_args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
