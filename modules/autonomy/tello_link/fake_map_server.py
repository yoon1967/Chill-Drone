"""가짜 지도 서버(시험용): 지도 서버(apps/console/serve.py, -340)의 드론 지도 부분만 흉내 낸다 (표준 라이브러리만).

- POST /api/live/tello/start · /api/live/tello/heartbeat · /api/live/stop (X-Mapping-Client: 1, Host 127.0.0.1만)
- GET  /api/live/state · /api/live/run (ONLINE_SPATIAL_V1 모양, 점 행 [id,x,y,z,r,g,b,error_px,track_length])
세션이 켜져 있는 동안 1초마다 버전이 오르고, 가상 카메라가 복도를 따라가며 3 m 안의 벽 점이 드러난다.
heartbeat가 15초 없으면 진짜처럼 세션을 끝낸다. 영상·SLAM·Docker는 없다.
--depth: 지금 지도 서버(-365 깊이 지도)처럼 세션이 끝나야 `map_available`이 켜지고, 그 전에는 상태의 `depth_job_id`와
`GET /api/import/preview?id=..&revision=..`(점 행 [x,y,z,r,g,b], 카메라는 위치만)로 진행 중 지도를 내준다.

    python modules/autonomy/tello_link/fake_map_server.py --port 8795
    python modules/autonomy/tello_link/fake_map_server.py --obj <복도 OBJ>    # 벽 모양을 OBJ 상자들에서
    python modules/autonomy/tello_link/fake_map_server.py --start              # 바로 지도 만드는 중(세션 시작 요청 없이)
    python modules/autonomy/tello_link/fake_map_server.py --start --depth      # 깊이 지도 방식(복원 중 미리보기)
"""

from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
import urllib.parse
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LEASE_S = 15.0
# 기본 복도: (x0, x1, y0, y1, z0, z1) m. 폭 2 m·길이 9.5 m 복도 + 홀 일부
BOXES = [(-1.12, -1.0, 0, 9.5, 0, 2.5), (1.0, 1.12, 0, 9.5, 0, 2.5), (-4.5, -1.0, 0, 0.12, 0, 2.5),
         (1.0, 6.0, 0, 0.12, 0, 2.5), (-4.5, 6.0, -3.4, -3.28, 0, 2.5), (-1, 1, 0, 9.5, -0.08, 0)]


def load_obj(path: str) -> list[tuple]:
    boxes, cur = [], []
    for line in open(path, encoding="utf-8"):
        if line.startswith("o "):
            if cur:
                boxes.append(cur)
            cur = []
        elif line.startswith("v "):
            cur.append(tuple(map(float, line.split()[1:4])))
    if cur:
        boxes.append(cur)
    return [(min(p[0] for p in b), max(p[0] for p in b), min(p[1] for p in b), max(p[1] for p in b),
             min(p[2] for p in b), max(p[2] for p in b)) for b in boxes]


def wall_points(boxes, density=40, seed=1) -> list[list]:
    rnd, pts = random.Random(seed), []
    for x0, x1, y0, y1, z0, z1 in boxes:
        floor = z1 - z0 < 0.5
        col = (150, 140, 120) if floor else (200, 200, 190)
        area = (x1 - x0) * (y1 - y0) if floor else max(x1 - x0, y1 - y0) * (z1 - z0) * 2
        for _ in range(int(area * density)):
            x, y, z = rnd.uniform(x0, x1), rnd.uniform(y0, y1), rnd.uniform(z0, z1)
            if not floor:   # 벽은 넓은 두 면 위에
                if x1 - x0 < y1 - y0:
                    x = rnd.choice((x0, x1))
                else:
                    y = rnd.choice((y0, y1))
            g = rnd.randint(-20, 20)
            pts.append([len(pts) + 1, round(x, 3), round(y, 3), round(z, 3), *(max(0, min(255, c + g)) for c in col),
                        round(rnd.uniform(0.3, 3), 2), rnd.randint(2, 9)])
    return pts


class FakeMap:
    ROT = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]   # 카메라 x 오른쪽, y 아래(-z), z 앞(+y)

    def __init__(self, boxes, clock=time.monotonic, depth=False):
        self.pts, self.clock, self.depth = wall_points(boxes), clock, depth
        self.path = [(0.0, 9.0 - t * 0.25) for t in range(37)] + [(-0.2 * t, -0.1 * t) for t in range(1, 15)]
        self.lock = threading.Lock()
        self.session = None   # {id, started, lease, revision_at_stop}

    def start(self) -> dict:
        with self.lock:
            self.session = {"id": f"tello_{uuid.uuid4().hex[:8]}", "started": self.clock(), "lease": self.clock() + LEASE_S,
                            "stopped": None}
            return {"session_id": self.session["id"], "session_status": "starting", "input_mode": "gateway"}

    def heartbeat(self, sid) -> tuple[int, dict]:
        with self.lock:
            s = self.session
            if not s or s["id"] != sid or self._ended(s):
                return 409, {"error": "camera_session_mismatch"}
            s["lease"] = self.clock() + LEASE_S
            return 200, {"session_id": sid}

    def stop(self, sid) -> tuple[int, dict]:
        with self.lock:
            s = self.session
            if not s or s["id"] != sid:
                return 409, {"error": "camera_session_mismatch"}
            s["stopped"] = s["stopped"] or self.clock()
            return 202, {"session_id": sid, "session_status": "stopping"}

    def _ended(self, s) -> bool:
        if s["stopped"] is None and self.clock() > s["lease"]:
            s["stopped"] = s["lease"]
        return s["stopped"] is not None

    def revision(self) -> int:
        s = self.session
        if not s:
            return 0
        self._ended(s)
        end = s["stopped"] or self.clock()
        return min(len(self.path), 1 + int(end - s["started"]))

    def state(self) -> dict:
        with self.lock:
            s, r = self.session, self.revision()
            status = "idle" if not s else "stopped" if s["stopped"] else "mapping"
            value = {"schema_version": "mapping-source-state/1.0", "source": "live", "status": status,
                     "mapping_state": status, "session_id": s["id"] if s else None, "segment_id": 0 if s else None,
                     "revision": r, "map_available": r > 0}
            if self.depth and s:   # 깊이 지도: 끝나야 결과가 생긴다
                value.update(backend="orb_slam3_depth", depth_job_id=f"job_{s['id']}", map_available=bool(s["stopped"]))
            return value

    def preview(self, job: str, since: int) -> tuple[int, dict]:
        """깊이 지도 진행 중 미리보기(session_import.preview 모양)."""
        with self.lock:
            s = self.session
            if not self.depth or not s or job != f"job_{s['id']}":
                return 400, {"error": "unknown_import_job"}
            r, done = self.revision(), bool(s["stopped"])
            if r < 1:
                return 200, {"revision": 0, "waiting": True, "done": done}
            if r == since:
                return 200, {"revision": since, "unchanged": True, "done": done, "needs_presentation": False}
            seen = self.path[:r]
            pts = [[p[1], -p[3], p[2], *p[4:7]] for p in self.pts   # 카메라 세계 좌표처럼 y 아래(z 위 → -y)
                   if min(math.hypot(p[1] - x, p[2] - y) for x, y in seen) < 3.0]
            return 200, {"revision": r, "frame_id": f"f{r}", "tracking": "tracking", "pose_confirmed": True, "map_id": 0,
                         "pose_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "points": pts[::max(1, math.ceil(len(pts) / 12000))],
                         "map_points": len(pts), "cameras": [[x, -1.0, y] for x, y in seen], "done": done,
                         "needs_presentation": False}

    def run(self) -> dict | None:
        with self.lock:
            r = self.revision()
            if r < 1:
                return None
            seen = self.path[:r]
            cams = [{"frame_id": f"f{i}", "position": [x, y, 1.0], "world_from_camera_rotation": self.ROT}
                    for i, (x, y) in enumerate(seen)]
            pts = [p for p in self.pts if min(math.hypot(p[1] - x, p[2] - y) for x, y in seen) < 3.0]
            return {"schema_version": "phone-sfm-result/1.2", "input_run_id": self.session["id"], "segment_id": 0,
                    "revision": r, "pose_source": "fake", "scale": {"status": "arbitrary", "unit": "arbitrary"},
                    "points": pts, "cameras": cams}


def make_handler(fake: FakeMap):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code: int, obj) -> None:
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/api/import/preview?"):
                q = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                return self.send(*fake.preview(q.get("id", [""])[0], int(q.get("revision", ["0"])[0] or 0)))
            if self.path == "/api/live/state":
                return self.send(200, fake.state())
            if self.path == "/api/live/run":
                run = fake.run()
                return self.send(200, run) if run else self.send(503, {"status": "incomplete"})
            self.send(404, {"error": "not_found"})

        def do_POST(self):
            port = self.server.server_port
            if (self.headers.get("Host") not in (f"127.0.0.1:{port}", f"localhost:{port}")
                    or self.headers.get("X-Mapping-Client") != "1"):
                return self.send(403, {"error": "forbidden"})
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
            if self.path == "/api/live/tello/start":
                return self.send(201, fake.start())
            if self.path == "/api/live/tello/heartbeat":
                return self.send(*fake.heartbeat(body.get("session_id")))
            if self.path == "/api/live/stop":
                return self.send(*fake.stop(body.get("session_id")))
            self.send(404, {"error": "not_found"})
    return H


def main():
    ap = argparse.ArgumentParser(description="가짜 지도 서버(드론 지도 세션·점군 흉내)")
    ap.add_argument("--port", type=int, default=8795)
    ap.add_argument("--obj", help="벽 모양을 읽을 OBJ(상자들). 없으면 기본 복도")
    ap.add_argument("--tello-gateway", help="진짜 지도 서버와 같은 인자를 받기 위한 자리(쓰지 않음)")
    ap.add_argument("--depth", action="store_true", help="깊이 지도 방식(-365): 끝나야 결과, 그 전엔 진행 중 미리보기")
    ap.add_argument("--start", action="store_true", help="켜자마자 드론 지도 세션 시작(지도 화면에서 시작한 것처럼). 15초 뒤 끝나지 않게 계속 연장")
    args = ap.parse_args()
    fake = FakeMap(load_obj(args.obj) if args.obj else BOXES, depth=args.depth)
    if args.start:
        sid = fake.start()["session_id"]

        def keep_alive():
            while True:
                time.sleep(LEASE_S / 3)
                fake.heartbeat(sid)
        threading.Thread(target=keep_alive, daemon=True).start()
    print(f"가짜 지도 서버 http://127.0.0.1:{args.port} · 점 {len(fake.pts)}개", flush=True)
    ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(fake)).serve_forever()


if __name__ == "__main__":
    main()
