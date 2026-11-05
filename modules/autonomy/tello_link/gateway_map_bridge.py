"""3D 지도 연결기: 지도 서버(apps/console/serve.py)의 실시간 점군을 읽어 게이트웨이로 보낸다 (표준 라이브러리만).

사람 탐지 연결기(perception/inference/gateway_ai_client.py)와 같은 자리다. 지도 서버도 게이트웨이도 고치지 않고 둘 사이를 잇는다.
- 읽는 곳: 지도 파트 계약 ONLINE_SPATIAL_V1의 `/api/<live|saved>/state`(버전 확인)와 `/api/<live|saved>/run`(점군·카메라).
  버전(revision)이 바뀔 때만 전체를 읽는다. 차분(`since_revision`)은 쓰지 않는다(같은 PC라 전체도 충분히 빠름).
- 보내는 곳: 게이트웨이 `POST /api/map`(`gateway-map/0.1`). 점이 많으면 오차가 작은 점부터 --max-points개만 고른다.
- 좌표는 바꾸지 않는다(지도 표시 좌표, 단위는 지도 쪽 scale.unit). 화면이 보기 좋게 세우도록 카메라 화면 위쪽 평균(`up`)을 같이 보낸다.
  중력 방향이 아니다(카메라가 대체로 수평이라는 가정).
- 지도 서버가 꺼졌거나 지도가 없어도 2초마다 상태(`status`)를 보내 화면이 이유를 보이게 한다.
- 깊이 지도(`backend=orb_slam3_depth`, -365): 복원이 끝나야 `map_available`이 켜진다. 그 전에는 상태의 `depth_job_id`로
  지도 화면이 쓰는 진행 중 미리보기 `GET /api/import/preview?id=..&revision=..`을 읽는다(점 행 [x,y,z,r,g,b], 최대 약 1.2만 점).
  읽기만 한다. 화면 표시 확인(`presented`)은 지도 화면 몫이라 보내지 않는다(그래서 지도 화면이 열려 있어야 복원이 다음 프레임으로 간다).

    python modules/autonomy/tello_link/gateway_map_bridge.py                                   # 지도 http://127.0.0.1:8790, 실시간
    python modules/autonomy/tello_link/gateway_map_bridge.py --source saved                    # 저장된 지도 결과
    python modules/autonomy/tello_link/gateway_map_bridge.py --map http://<지도 PC>:8790 --gateway http://127.0.0.1:8010

지도 서버는 Tello 영상을 게이트웨이에서 받게 켠다(`--tello-gateway http://127.0.0.1:8010`). 직접 모드는 게이트웨이와 동시에 못 쓴다.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request

SCHEMA = "gateway-map/0.1"
STATUS_EVERY_S = 2.0


def get_json(req, timeout: float) -> tuple[int, dict | None]:
    """주소 또는 Request → (HTTP 상태, 본문). 연결 실패는 OSError로 올린다."""
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read())
        except ValueError:
            return e.code, None


def post_json(url: str, body: dict, timeout: float = 10) -> tuple[int, dict | None]:
    req = urllib.request.Request(url, json.dumps(body, separators=(",", ":")).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    return get_json(req, timeout)


def compact_scene(run: dict, state: dict, map_url: str, source: str, max_points: int) -> dict:
    """지도 서버 scene(phone-sfm-result/1.x) → gateway-map/0.1. 점 행은 [id,x,y,z,r,g,b,error_px,track_length]."""
    rows = [p for p in run.get("points") or [] if len(p) >= 7 and all(_finite(v) for v in p[1:4])]
    total = len(rows)
    if total > max_points:   # 오차 작은 점 우선, 같으면 많이 관측된 점
        rows.sort(key=lambda p: (p[7] if len(p) > 7 and _finite(p[7]) else math.inf,
                                 -(p[8] if len(p) > 8 and _finite(p[8]) else 0)))
        rows = rows[:max_points]
    points = [[round(p[1], 4), round(p[2], 4), round(p[3], 4), *(_byte(c) for c in p[4:7])] for p in rows]
    cams = [c for c in run.get("cameras") or [] if isinstance(c.get("position"), list) and len(c["position"]) == 3]
    cameras = [[round(v, 4) for v in c["position"]] for c in cams if all(_finite(v) for v in c["position"])]
    scale = run.get("scale") or {}
    return {
        "schema_version": SCHEMA, "kind": "snapshot", "upstream": "ok", "source_url": map_url,
        "mapping_state": state.get("mapping_state") or state.get("status"),
        "source": {"kind": source, "schema_version": run.get("schema_version"), "input_run_id": run.get("input_run_id"),
                   "session_id": state.get("session_id") or (run.get("stream") or {}).get("session_id"),
                   "segment_id": run.get("segment_id"), "revision": run.get("revision", state.get("revision")),
                   "pose_source": run.get("pose_source")},
        "scale": {"status": scale.get("status"), "unit": scale.get("unit") or "arbitrary"},
        "points": points, "points_total": total, "cameras": cameras,
        "up": camera_up(cams),
    }


def camera_up(cams: list[dict]) -> list[float] | None:
    """카메라 화면 위쪽(카메라 -y축)의 평균. 카메라 로컬 축은 x 오른쪽·y 아래·z 앞(계약)."""
    return _up_from([c.get("world_from_camera_rotation") for c in cams])


def _up_from(rotations) -> list[float] | None:
    s = [0.0, 0.0, 0.0]
    for rot in rotations:
        if isinstance(rot, list) and len(rot) == 3 and all(isinstance(r, list) and len(r) == 3 for r in rot):
            for i in range(3):   # 열 1 = 카메라 y축(표시 좌표)
                v = rot[i][1]
                if _finite(v):
                    s[i] -= v
    n = math.sqrt(sum(v * v for v in s))
    return [round(v / n, 4) for v in s] if n > 1e-6 else None


def compact_preview(preview: dict, state: dict, map_url: str, max_points: int) -> dict:
    """깊이 지도 진행 중 미리보기 → gateway-map/0.1. 점 행은 [x,y,z,r,g,b](이미 지도 쪽에서 약 1.2만 점으로 솎음).
    좌표는 ORB-SLAM3 세계 좌표(카메라 y 아래)이고 배율은 단안이라 정해지지 않았다."""
    rows = [p for p in preview.get("points") or [] if isinstance(p, list) and len(p) >= 6 and all(_finite(v) for v in p[:3])]
    if len(rows) > max_points:   # 오차 정보가 없으니 고르게 솎는다
        rows = rows[::math.ceil(len(rows) / max_points)]
    points = [[round(p[0], 4), round(p[1], 4), round(p[2], 4), *(_byte(c) for c in p[3:6])] for p in rows]
    cameras = [[round(v, 4) for v in c] for c in preview.get("cameras") or []
               if isinstance(c, list) and len(c) == 3 and all(_finite(v) for v in c)]
    total = preview.get("map_points")
    return {
        "schema_version": SCHEMA, "kind": "snapshot", "upstream": "ok", "source_url": map_url,
        "mapping_state": state.get("mapping_state") or state.get("status"),
        "source": {"kind": "live", "schema_version": "live-depth-preview", "input_run_id": state.get("session_id"),
                   "session_id": state.get("session_id"), "segment_id": preview.get("map_id"),
                   "revision": preview.get("revision"), "pose_source": "orb_slam3",
                   "depth_job_id": state.get("depth_job_id"), "tracking": preview.get("tracking")},
        "scale": {"status": "arbitrary", "unit": "arbitrary"},
        "points": points, "points_total": total if _finite(total) else len(rows), "cameras": cameras,
        "up": _up_from([preview.get("pose_rotation")]) or [0.0, -1.0, 0.0],
    }


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _byte(v) -> int:
    return max(0, min(255, int(round(v)))) if _finite(v) else 128


class MapBridge:
    """지도 서버를 한 번 보고(poll_once) 새 버전이면 deliver(msg)로 넘긴다. 게이트웨이 안(스레드)과 따로 실행에서 같이 쓴다.
    deliver는 (HTTP 상태, 응답)을 돌려준다: 게이트웨이 안에서는 MapInbox.receive, 따로 실행에서는 POST /api/map."""

    def __init__(self, map_url: str, deliver, *, source: str = "live", max_points: int = 20000,
                 log=print, clock=time.monotonic):
        self.base, self.api = map_url.rstrip("/"), f"{map_url.rstrip('/')}/api/{source}"
        self.deliver, self.source, self.max_points, self.log, self.clock = deliver, source, max_points, log, clock
        self.last_key = None
        self._preview = (None, 0)   # (깊이 작업 번호, 마지막으로 보낸 미리보기 버전)
        self._last_status, self._last_said = -1e9, None

    def _tell(self, upstream: str, detail=None, mapping_state=None) -> None:
        if self.clock() - self._last_status < STATUS_EVERY_S and upstream == self._last_said:
            return
        self._last_status, self._last_said = self.clock(), upstream
        self.deliver({"schema_version": SCHEMA, "kind": "status", "upstream": upstream,
                      "source_url": self.base, "mapping_state": mapping_state, "detail": detail})

    def poll_once(self) -> None:
        try:
            code, state = get_json(self.api + "/state", timeout=3)
        except OSError as e:
            return self._tell("unreachable", str(e))
        state = state or {}
        mstate = state.get("mapping_state") or state.get("status")
        if code == 200 and not state.get("map_available") and self.source == "live" and state.get("depth_job_id"):
            return self._poll_preview(state, mstate)
        if code != 200 or not state.get("map_available"):
            return self._tell("no_map", f"HTTP {code}" if code != 200 else None, mstate)
        key = (state.get("session_id"), state.get("segment_id"), state.get("revision"))   # 저장 결과는 늘 버전 0 → 한 번만 읽음
        if key == self.last_key:
            return self._tell("ok", None, mstate)
        try:
            code, scene = get_json(self.api + "/run", timeout=15)
        except OSError as e:
            return self._tell("unreachable", str(e), mstate)
        if code != 200 or not scene or scene.get("points") is None:
            return self._tell("no_map", f"run HTTP {code}", mstate)
        msg = compact_scene(scene, state, self.base, self.source, self.max_points)
        code, out = self.deliver(msg)
        if code in (200, 201):
            self.last_key = key
            self._last_status, self._last_said = self.clock(), "ok"
            self.log(f"버전 {msg['source']['revision']} · 점 {len(msg['points'])}/{msg['points_total']} · "
                     f"카메라 {len(msg['cameras'])} · {msg['scale']['unit']} → {code}")
        else:
            self.log(f"게이트웨이 거절 {code}: {out}")

    def _poll_preview(self, state: dict, mstate) -> None:
        """깊이 지도 복원 중: 지도 화면과 같은 미리보기를 읽어 새 버전이면 보낸다."""
        job = str(state["depth_job_id"])
        since = self._preview[1] if self._preview[0] == job else 0
        url = f"{self.base}/api/import/preview?" + urllib.parse.urlencode({"id": job, "revision": since})
        try:
            code, prev = get_json(url, timeout=10)
        except OSError as e:
            return self._tell("unreachable", str(e), mstate)
        if code != 200 or not prev:
            return self._tell("no_map", f"preview HTTP {code}", mstate)
        if prev.get("unchanged") or prev.get("waiting") or not prev.get("points"):
            return self._tell("ok" if self._preview[0] == job else "no_map", None, mstate)
        msg = compact_preview(prev, state, self.base, self.max_points)
        code, out = self.deliver(msg)
        if code in (200, 201):
            self._preview = (job, int(prev.get("revision") or 0))
            self._last_status, self._last_said = self.clock(), "ok"
            self.log(f"복원 중 버전 {msg['source']['revision']} · 점 {len(msg['points'])}/{msg['points_total']} · "
                     f"카메라 {len(msg['cameras'])} · 추적 {prev.get('tracking')} → {code}")
        else:
            self.log(f"게이트웨이 거절 {code}: {out}")

    def run_forever(self, interval: float, stop=None) -> None:
        """stop: threading.Event 또는 None(Ctrl+C까지)."""
        while stop is None or not stop.is_set():
            started = self.clock()
            try:
                self.poll_once()
            except Exception as e:   # 지도 쪽 응답이 이상해도 연결기는 계속 돈다
                self.log(f"지도 읽기 오류: {e}")
                try:
                    self.deliver({"schema_version": SCHEMA, "kind": "status", "upstream": "error",
                                  "source_url": self.base, "detail": str(e)[:200]})
                except Exception:
                    pass
            wait = max(0.05, interval - (self.clock() - started))
            if stop is None:
                time.sleep(wait)
            elif stop.wait(wait):
                break


def run(args) -> None:
    gw = args.gateway.rstrip("/")
    print(f"지도 {args.map}/api/{args.source} → 게이트웨이 {gw}/api/map (점 최대 {args.max_points}개)")

    def deliver(msg):
        try:
            return post_json(gw + "/api/map", msg, timeout=15)
        except OSError as e:
            print(f"게이트웨이 연결 실패: {e}")
            return 0, None

    MapBridge(args.map, deliver, source=args.source, max_points=args.max_points).run_forever(args.interval)


def main():
    ap = argparse.ArgumentParser(description="지도 서버의 3D 점군을 드론 게이트웨이로 보내기")
    ap.add_argument("--map", default="http://127.0.0.1:8790", help="지도 서버(apps/console/serve.py) 주소")
    ap.add_argument("--gateway", default="http://127.0.0.1:8010")
    ap.add_argument("--source", choices=("live", "saved"), default="live", help="실시간 지도(live, 기본) 또는 저장된 결과(saved)")
    ap.add_argument("--max-points", type=int, default=20000, help="게이트웨이로 보낼 최대 점 수")
    ap.add_argument("--interval", type=float, default=0.5, help="지도 버전 확인 간격(초)")
    args = ap.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
