"""가짜 사람 탐지: 게이트웨이 영상을 받아 합성 탐지 결과를 돌려보낸다 (표준 라이브러리만 사용).

실제 탐지(YOLO)를 게이트웨이에 붙일 때 그대로 따라 하면 되는 흐름이다.
1. GET /snapshot.jpg → JPEG + 헤더 X-Input-Id·X-Frame-Id·X-Frame-Time·X-Image-Size
2. 탐지 → person-detection-event/1.0 (input_id·frame_id는 헤더 값 그대로, detected_at_utc는 X-Frame-Time)
3. POST /api/detections (사람이 없어도 보낸다: 탐지가 동작 중이라는 근거)

    python fake_detector.py                       # http://127.0.0.1:8010, 초당 5장, 4초 보이고 5초 안 보임
    python fake_detector.py --hz 10 --on 6 --off 3
영상이 꺼져 있으면(503) 기다렸다가 다시 시도한다. 사람 모양은 영상 내용과 무관한 합성 값이다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..")))

from modules.perception.inference.detection_event import build_person_detection_event  # noqa: E402

MODEL = {"name": "fake-detector", "version": "0.1", "weights_sha256": None}


def fake_people(t: float, width: int, height: int, on_s: float, off_s: float) -> list[dict]:
    """on_s 동안 사람 1명(가끔 2명)이 좌우로 움직이고, off_s 동안은 없다."""
    phase = t % (on_s + off_s)
    if phase >= on_s:
        return []
    people = []
    for k in range(2 if phase > on_s * 0.6 else 1):
        cx = width * (0.3 + 0.4 * (0.5 + 0.5 * math.sin(t * 0.8 + k * 2.0)))
        w, h = width * 0.14, height * 0.55
        x1, y1 = max(0.0, cx - w / 2), height * 0.3
        x2, y2 = min(float(width), cx + w / 2), min(float(height), y1 + h)
        mx = (x1 + x2) / 2
        polygon = [[mx, y1], [x2, y1 + h * 0.25], [x2, y2], [x1, y2], [x1, y1 + h * 0.25]]
        conf = 0.55 + 0.4 * (0.5 + 0.5 * math.sin(t * 1.3 + k))
        people.append({"confidence": round(conf, 3), "xyxy": [round(v, 1) for v in (x1, y1, x2, y2)],
                       "polygons": [[[round(x, 1), round(y, 1)] for x, y in polygon]]})
    return people


def run(base: str, hz: float, on_s: float, off_s: float, count: int | None = None) -> int:
    sent, last_frame, period, t0 = 0, None, 1.0 / hz, time.monotonic()
    while count is None or sent < count:
        started = time.monotonic()
        try:
            with urllib.request.urlopen(base + "/snapshot.jpg", timeout=3) as r:
                h = {k.lower(): v for k, v in r.headers.items()}
                r.read()
        except urllib.error.HTTPError as e:
            print(f"영상 없음({e.code}): {e.read().decode('utf-8', 'replace')}, 1초 뒤 다시")
            time.sleep(1)
            continue
        except OSError as e:
            print(f"게이트웨이 연결 실패: {e}, 1초 뒤 다시")
            time.sleep(1)
            continue
        if h["x-frame-id"] == last_frame:   # 새 프레임이 아직 없다
            time.sleep(0.02)
            continue
        last_frame = h["x-frame-id"]
        width, height = (int(v) for v in h.get("x-image-size", "960x720").split("x"))
        event = build_person_detection_event(
            input_id=h["x-input-id"], frame_id=h["x-frame-id"],
            detected_at_utc=h["x-frame-time"].replace("+00:00", "Z"),
            image_width=width, image_height=height, model=MODEL,
            detections=fake_people(time.monotonic() - t0, width, height, on_s, off_s))
        req = urllib.request.Request(base + "/api/detections", json.dumps(event).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                out = json.loads(r.read())
        except urllib.error.HTTPError as e:
            out = {"error": e.code, **json.loads(e.read() or b"{}")}
        sent += 1
        if event["detections"] or sent % 25 == 1:
            print(f"{h['x-frame-id']} 사람 {len(event['detections'])} → {out}")
        time.sleep(max(0.0, period - (time.monotonic() - started)))
    return sent


def main():
    ap = argparse.ArgumentParser(description="가짜 사람 탐지 (게이트웨이 영상 → 합성 탐지 결과)")
    ap.add_argument("--gateway", default="http://127.0.0.1:8010")
    ap.add_argument("--hz", type=float, default=5.0, help="초당 처리 장수")
    ap.add_argument("--on", type=float, default=4.0, help="사람이 보이는 시간(초)")
    ap.add_argument("--off", type=float, default=5.0, help="사람이 안 보이는 시간(초)")
    args = ap.parse_args()
    try:
        run(args.gateway.rstrip("/"), args.hz, args.on, args.off)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
