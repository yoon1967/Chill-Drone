"""사람 탐지 결과 받기: 탐지 이벤트를 검증·저장하고, 사람이 보이는 동안을 발견 하나로 묶는다 (표준 라이브러리만 사용).

계약: contracts/perception/PERSON_DETECTION_EVENT_V1.md (`person-detection-event/1.0`). 검증은 그 모듈의 함수를 그대로 쓴다.
- 탐지 쪽은 게이트웨이 `/snapshot.jpg`의 X-Input-Id·X-Frame-Id를 이벤트 `input_id`·`frame_id`에 그대로 넣는다.
  게이트웨이는 보관 중인 같은 프레임으로 사진·수신 시각·그때 드론 상태를 붙인다(사진 재전송 불필요).
- 게이트웨이 프레임이 아니면(다른 카메라, 보관 시간 지남) 사진 없이 기록하고, 탐지 쪽이 PUT으로 JPEG를 올릴 수 있다.
- 사람이 없는 이벤트도 모두 기록한다(탐지가 동작 중이었다는 근거). 화면에는 사람이 있을 때만 보낸다.
- 발견: 사람이 GAP_S 넘게 안 보이다 다시 보이면 새 발견. 사람 구분(추적)은 하지 않는다. 대표 사진은 신뢰도가 가장 높은 장면.
- 사진 저장은 발견마다 초당 1장까지, 대표 장면이 바뀌면 항상 저장한다.
- 확인·오탐: 사람(화면)이 발견마다 고른 판단(`review`: unreviewed·confirmed·false_positive)을 여기 둔다.
  모든 화면에 `detection_review`로 알리고 회차 기록에 남긴다. 저장에 실패하면 되돌린다(화면과 기록이 어긋나지 않게).
회차 폴더: detections.jsonl(이벤트마다 한 줄, 확인·오탐은 type review 줄), detections/<event_id>.jpg
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import uuid
from collections import OrderedDict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from modules.perception.inference.detection_event import (  # noqa: E402
    SCHEMA_VERSION as DETECTION_SCHEMA, validate_person_detection_event)
from mode_arbiter import SCHEMA_VERSION  # noqa: E402
from tello_link import now_iso  # noqa: E402

GAP_S = 3.0                 # 이만큼 사람이 안 보이면 다음에 보일 때 새 발견
PHOTO_MIN_INTERVAL_S = 1.0  # 발견마다 사진 저장 최소 간격(대표 장면이 바뀌면 예외)
ALIVE_S = 2.0               # 이벤트가 이 시간 안에 왔으면 탐지 동작 중
MAX_IMAGE_BYTES = 3_000_000
SEEN_IDS = 5000             # 중복 판정에 기억하는 event_id 수
REVIEWS = ("unreviewed", "confirmed", "false_positive")   # 발견마다 사람이 고르는 판단(처음은 unreviewed)


class DetectionInbox:
    def __init__(self, frames, *, run_dir: str | None = None, emit=None, clock=time.monotonic):
        self.frames = frames                    # FrameStore
        self.run_dir = run_dir
        self.emit = emit or (lambda msg: None)
        self.clock = clock
        self._lock = threading.Lock()
        self._seen: OrderedDict[str, dict] = OrderedDict()   # event_id → 기록(사진 올리기용)
        self._recent = deque(maxlen=50)                      # 최근 이벤트 수신 시각(초당 처리 수)
        self.findings: list[dict] = []
        self.events = 0
        self.person_events = 0
        self.rejected = 0
        self.model: dict | None = None
        self._last_person_mono: float | None = None
        self._last_photo_mono: float | None = None
        if run_dir:
            os.makedirs(os.path.join(run_dir, "detections"), exist_ok=True)

    # ---------- 받기 ----------
    def receive(self, event) -> tuple[int, dict]:
        """이벤트 하나. (HTTP 상태, 응답). 201 저장, 200 중복, 400 검증 실패, 500 저장 실패."""
        try:
            if not isinstance(event, dict):
                raise ValueError("event_not_object")
            validate_person_detection_event(event)
            if event["event_id"] != str(uuid.UUID(event["event_id"])):  # 사진 파일 이름에 쓰므로 표준 표기만
                raise ValueError("invalid_event_id")
        except (ValueError, TypeError, KeyError, AttributeError) as e:
            with self._lock:
                self.rejected += 1
            return 400, {"ok": False, "reason": "invalid_event", "detail": str(e)}

        now = self.clock()
        frame = self.frames.get(event["input_id"], event["frame_id"])
        if frame and frame["width"] and (frame["width"], frame["height"]) != (
                event["image"]["width_px"], event["image"]["height_px"]):
            with self._lock:
                self.rejected += 1
            return 400, {"ok": False, "reason": "image_size_mismatch",
                         "detail": f"gateway frame {frame['width']}x{frame['height']}"}

        with self._lock:
            event_id = event["event_id"]
            if event_id in self._seen:
                return 200, {"ok": True, "duplicate": True, "event_id": event_id}
            self.events += 1
            self._recent.append(now)
            self.model = event["model"]
            people = event["detections"]
            record = {
                "received_at": _iso(time.time()),
                "event": event,
                "frame": "gateway" if frame else "unknown",
                "frame_time": _iso(frame["wall"]) if frame else None,
                "latency_ms": round((now - frame["mono"]) * 1000) if frame else None,
                "drone": frame.get("drone") if frame else None,
                "finding_id": None,
                "photo": None,
            }
            finding, best_changed = (self._update_finding(event, record, frame, now) if people else (None, False))
            jpeg = None
            if finding is not None and frame and (best_changed or self._last_photo_mono is None
                                                  or now - self._last_photo_mono >= PHOTO_MIN_INTERVAL_S):
                jpeg = frame["jpeg"]
                record["photo"] = f"detections/{event_id}.jpg"
                self._last_photo_mono = now
            self._seen[event_id] = record
            while len(self._seen) > SEEN_IDS:
                self._seen.popitem(last=False)

        try:
            self._write(record, jpeg)
        except OSError as e:
            return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
        if finding is not None:
            self.emit({"schema_version": SCHEMA_VERSION, "type": "detection", "at": record["received_at"],
                       "event": event, "finding": self._public(finding), "latency_ms": record["latency_ms"],
                       "photo_url": self._photo_url(record)})
        body = {"ok": True, "duplicate": False, "event_id": event_id, "finding_id": record["finding_id"],
                "frame": record["frame"], "photo": record["photo"] is not None}
        if people and not frame:
            body["upload"] = f"/api/detections/{event_id}.jpg"   # 게이트웨이 프레임이 아니면 사진을 따로 올린다
        return 201, body

    def put_photo(self, event_id: str, jpeg: bytes) -> tuple[int, dict]:
        """게이트웨이 프레임이 아닌 이벤트의 사진. 이미 사진이 있으면 덮어쓰지 않는다."""
        if not jpeg.startswith(b"\xff\xd8") or len(jpeg) > MAX_IMAGE_BYTES:
            return 400, {"ok": False, "reason": "invalid_image"}
        with self._lock:
            record = self._seen.get(event_id)
            if record is None:
                return 404, {"ok": False, "reason": "unknown_event"}
            if record["photo"] is not None:
                return 200, {"ok": True, "duplicate": True}
            record["photo"] = f"detections/{event_id}.jpg"
            finding = next((f for f in self.findings if f["best"]["event_id"] == event_id), None)
            if finding is not None:
                finding["best"]["photo"] = record["photo"]
        try:
            self._save_photo(event_id, jpeg)
            self._append("photo_added", {"event_id": event_id})
        except OSError as e:
            return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
        return 201, {"ok": True, "duplicate": False}

    def photo_path(self, event_id: str) -> str | None:
        with self._lock:
            record = self._seen.get(event_id)
        if not record or not record["photo"] or not self.run_dir:
            return None
        path = os.path.join(self.run_dir, record["photo"])
        return path if os.path.isfile(path) else None

    # ---------- 화면용 ----------
    def status(self) -> dict:
        now = self.clock()
        with self._lock:
            recent = [t for t in self._recent if now - t <= ALIVE_S]
            last = self._recent[-1] if self._recent else None
            return {
                "alive": last is not None and now - last <= ALIVE_S,
                "events_per_s": round(len(recent) / ALIVE_S, 1) if recent else 0.0,
                "last_event_age_s": None if last is None else round(now - last, 2),
                "events": self.events,
                "person_events": self.person_events,
                "rejected": self.rejected,
                "findings": len(self.findings),
                "model": self.model,
            }

    def snapshot(self) -> dict:
        with self._lock:
            findings = [self._public(f) for f in reversed(self.findings)]   # 최신순
        return {"schema_version": SCHEMA_VERSION, "detection_schema": DETECTION_SCHEMA,
                "findings": findings, "status": self.status()}

    # ---------- 확인·오탐 ----------
    def set_review(self, finding_id: str, review) -> tuple[int, dict]:
        """화면이 고른 확인·오탐. (HTTP 상태, 응답). 200 저장(같은 값이면 changed false), 400 모르는 값, 404 모르는 발견, 500 저장 실패.
        바뀌면 회차 기록에 남기고 모든 화면에 detection_review로 알린다. 기록에 못 남기면 되돌린다."""
        if review not in REVIEWS:
            return 400, {"ok": False, "reason": "invalid_review", "detail": f"review must be one of {', '.join(REVIEWS)}"}
        at = _iso(time.time())
        with self._lock:
            finding = next((f for f in self.findings if f["finding_id"] == finding_id), None)
            if finding is None:
                return 404, {"ok": False, "reason": "unknown_finding"}
            previous, previous_at = finding["review"], finding["review_at"]
            if previous == review:
                return 200, {"ok": True, "finding_id": finding_id, "review": review, "review_at": previous_at, "changed": False}
            finding["review"], finding["review_at"] = review, at
        try:
            self._append("review", {"at": at, "finding_id": finding_id, "review": review, "previous": previous})
        except OSError as e:
            with self._lock:
                if finding["review_at"] == at:   # 그사이 다른 화면이 다시 바꾸지 않았으면 되돌린다
                    finding["review"], finding["review_at"] = previous, previous_at
            return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
        self.emit({"schema_version": SCHEMA_VERSION, "type": "detection_review", "at": at,
                   "finding_id": finding_id, "review": review, "previous": previous})
        return 200, {"ok": True, "finding_id": finding_id, "review": review, "review_at": at, "changed": True}

    # ---------- 내부 ----------
    def _update_finding(self, event, record, frame, now) -> tuple[dict, bool]:
        self.person_events += 1
        best_det = max(event["detections"], key=lambda d: d["confidence"])
        seen_at = frame["wall"] if frame else time.time()
        current = self.findings[-1] if self.findings else None
        if current is None or self._last_person_mono is None or now - self._last_person_mono > GAP_S:
            current = {"finding_id": f"finding-{len(self.findings) + 1:04d}", "first_seen": _iso(seen_at),
                       "events": 0, "max_people": 0, "best": None, "review": "unreviewed", "review_at": None}
            self.findings.append(current)
        self._last_person_mono = now
        current["last_seen"] = _iso(seen_at)
        current["events"] += 1
        current["max_people"] = max(current["max_people"], len(event["detections"]))
        best_changed = current["best"] is None or best_det["confidence"] > current["best"]["confidence"]
        if best_changed:
            current["best"] = {"event_id": event["event_id"], "frame_id": event["frame_id"],
                               "confidence": best_det["confidence"], "people": len(event["detections"]),
                               "image": event["image"], "detections": event["detections"],
                               "seen_at": _iso(seen_at), "drone": record["drone"],
                               "photo": f"detections/{event['event_id']}.jpg" if frame else None}
        record["finding_id"] = current["finding_id"]
        return current, best_changed

    def _public(self, finding: dict) -> dict:
        out = {k: v for k, v in finding.items() if k != "best"}
        best = dict(finding["best"])
        best["photo_url"] = f"/api/detections/{best['event_id']}.jpg" if best.pop("photo") else None
        out["best"] = best
        return out

    @staticmethod
    def _photo_url(record) -> str | None:
        return f"/api/detections/{record['event']['event_id']}.jpg" if record["photo"] else None

    def _write(self, record: dict, jpeg: bytes | None) -> None:
        if jpeg is not None:
            self._save_photo(record["event"]["event_id"], jpeg)
        self._append("event", record)

    def _save_photo(self, event_id: str, jpeg: bytes) -> None:
        if not self.run_dir:
            return
        with open(os.path.join(self.run_dir, "detections", f"{event_id}.jpg"), "wb") as f:  # 경로는 서버가 만든다
            f.write(jpeg)

    def _append(self, kind: str, data: dict) -> None:
        if not self.run_dir:
            return
        line = json.dumps({"type": kind, **data}, ensure_ascii=False)
        with self._lock, open(os.path.join(self.run_dir, "detections.jsonl"), "a", encoding="utf-8") as f:
            f.write(line + "\n")


def _iso(wall: float) -> str:
    return now_iso(wall)
