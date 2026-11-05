"""가짜 자율 비행(시험용): 게이트웨이 `/ws/autonomy`에 붙어 자율 비행 쪽 메시지를 흉내 낸다(AUTONOMY_BRIDGE_V0_1).

다리 노드·ROS 2 없이 게이트웨이 판정(속도 명령, 끊김 0.5초·2초, 넘겨받기)과 화면(2D 지도·찾은 사람)을 시험한다.
- `start`를 받으면 출발점 앞 2 m × 2 m 네모를 돈다. 위치는 자기가 보낸 속도를 더해 만든다(가짜 Tello는 옆으로 움직이지 않는다).
  게이트웨이 모드가 `auto`일 때만 움직인 것으로 친다(그때만 속도 명령이 기체로 간다).
- 정해 둔 두 자리에 사람이 있다. 카메라 시야(앞 ±35°, 2.5 m 안)에 들어오면 찾는다. A는 한 번 흐릿하게 봐서 미확정,
  두 번째에 확정. B는 출발 때 보이고, 한 바퀴 돈 뒤 출발 방향으로 되돌아볼 때(HOMESPIN) 그 자리가 비어 있다(`gone`).
- `pause` 제자리(속도 명령 멈춤, 위치는 계속), `resume` 이어서, `stop` 끝. 한 바퀴 돌고 출발 방향으로 돌면 `arrived`.
- 위치·자세는 자율 지도(`frame: map`): 켠 자리 원점, x 앞·y 왼쪽, m, yaw 반시계 +.

    python modules/autonomy/tello_link/fake_autonomy.py                          # http://127.0.0.1:8010 게이트웨이에
    python modules/autonomy/tello_link/fake_autonomy.py --gateway http://127.0.0.1:8030
    python modules/autonomy/tello_link/fake_autonomy.py --stall-at 5 --stall-for 3   # 달리기 5초 뒤 3초 명령 끊기
    python modules/autonomy/tello_link/fake_autonomy.py --reject-start              # 시작 거절(pose_not_ready)

필요: websockets(게이트웨이 venv에 uvicorn과 함께 있다).
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time

SCHEMA = "0.1.0"
SQUARE = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]   # 출발점(0,0)에서 앞으로 2 m, 왼쪽으로 2 m, 돌아오기
FLY_Z = 1.0
PEOPLE = [{"name": "A", "x": 2.6, "y": 1.2, "conf": [0.62, 0.84], "top": 1.62},
          {"name": "B", "x": 1.6, "y": -0.6, "conf": [0.88, 0.9], "top": 1.1, "leaves_after_s": 8.0}]
HFOV = math.radians(70)
SEE_M = 2.5
SEE_EVERY_S = 1.0           # 같은 사람을 다시 세는 간격
MAX_V = 0.3                 # 게이트웨이 hello의 max_speed_mps와 같다


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class FakeMission:
    """네트워크 없는 임무 흉내. step()이 보낼 속도 명령을 돌려주고, 움직였으면 위치를 더한다."""

    def __init__(self, *, reject_start: bool = False, stall_at: float | None = None, stall_for: float = 0.0,
                 clock=time.monotonic):
        self.clock = clock
        self.reject_start, self.stall_at, self.stall_for = reject_start, stall_at, stall_for
        self.phase, self.state, self.reason = "idle", "WAIT_POSE", None
        self.x = self.y = self.yaw = 0.0
        self.z = 0.0
        self.leg = 0
        self.run_s = 0.0             # running으로 지낸 시간(멈춘 시간 빼고)
        self.found: list[dict] = []  # 찾은 사람(게이트웨이 people 모양의 원본)
        self._seen_at: dict[str, float] = {}
        self.changed_people = False
        self.path_sent = False

    # ---------- 게이트웨이 control ----------
    def control(self, cmd: str) -> tuple[bool, str | None]:
        if cmd == "start":
            if self.reject_start:
                self.reason = "pose_not_ready"
                return False, "pose_not_ready"
            if self.phase in ("running", "paused"):
                return False, "already_running"
            self.phase, self.state, self.reason, self.leg = "running", "FOLLOW", None, 0
            self.path_sent = False
            return True, None
        if cmd == "pause":
            if self.phase != "running":
                return False, "not_running"
            self.phase, self.state = "paused", "PAUSED"
            return True, None
        if cmd == "resume":
            if self.phase != "paused":
                return False, "not_paused"
            self.phase, self.state = "running", "FOLLOW"
            return True, None
        if cmd == "stop":
            self.phase, self.state, self.reason = "stopped", "STOPPED", "operator_stop"
            return True, None
        return False, "unknown_cmd"

    # ---------- 주기 ----------
    def stalled(self) -> bool:
        return self.stall_at is not None and self.stall_at <= self.run_s < self.stall_at + self.stall_for

    def step(self, dt: float, moving: bool) -> dict | None:
        """dt초 지남. moving: 게이트웨이가 auto라 속도 명령이 기체로 가는가. 보낼 cmd_vel(기체 기준) 또는 None."""
        if moving:
            self.z = FLY_Z
        if self.phase != "running":
            return None
        self.run_s += dt
        if self.stalled():
            return None
        if self.leg < len(SQUARE):
            tx, ty = SQUARE[self.leg]
            dist = math.hypot(tx - self.x, ty - self.y)
            if dist < 0.08:
                self.leg += 1
        if self.leg >= len(SQUARE):             # 다 돌았으면 출발 방향으로 되돌아본다
            err = _wrap(0.0 - self.yaw)
            if abs(err) < 0.08:
                self.phase, self.state, self.reason = "arrived", "ARRIVED", "route_done"
                return {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0}
            vx, wz, self.state = 0.0, max(-0.6, min(0.6, 1.2 * err)), "HOMESPIN"
        else:
            tx, ty = SQUARE[self.leg]
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            err = _wrap(math.atan2(dy, dx) - self.yaw)
            if abs(err) > 0.25:                 # 먼저 돌고
                vx, wz, self.state = 0.0, max(-0.6, min(0.6, 1.2 * err)), "LOOK"
            else:                               # 앞으로
                vx, wz, self.state = min(MAX_V, 0.8 * dist + 0.05), 0.8 * err, "FOLLOW"
        if moving:
            self.yaw = _wrap(self.yaw + wz * dt)
            self.x += math.cos(self.yaw) * vx * dt
            self.y += math.sin(self.yaw) * vx * dt
            self._look()
        return {"vx_mps": round(vx, 3), "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": round(wz, 3)}

    def _look(self) -> None:
        now = self.run_s
        for p in PEOPLE:
            dx, dy = p["x"] - self.x, p["y"] - self.y
            d = math.hypot(dx, dy)
            if d > SEE_M or abs(_wrap(math.atan2(dy, dx) - self.yaw)) > HFOV / 2:
                continue
            if now - self._seen_at.get(p["name"], -1e9) < SEE_EVERY_S:
                continue
            self._seen_at[p["name"]] = now
            rec = next((q for q in self.found if q["name"] == p["name"]), None)
            if rec is None:
                self.found.append({"name": p["name"], "x": round(p["x"] + 0.12, 2), "y": round(p["y"] - 0.08, 2),
                                   "conf": p["conf"][0], "t_first": round(now, 2), "hits": 1, "gone": False, "top": p["top"]})
                self.changed_people = True
            elif p.get("leaves_after_s") is not None and now - rec["t_first"] >= p["leaves_after_s"]:
                if not rec["gone"]:
                    rec["gone"], self.changed_people = True, True   # 다시 봤는데 그 자리가 비었다
            else:
                rec["hits"] += 1
                rec["conf"] = max(rec["conf"], p["conf"][min(rec["hits"] - 1, len(p["conf"]) - 1)])
                rec["x"], rec["y"] = round((rec["x"] + p["x"]) / 2, 2), round((rec["y"] + p["y"]) / 2, 2)
                self.changed_people = True

    # ---------- 보낼 메시지 ----------
    def progress(self) -> float | None:
        if self.phase == "idle":
            return None
        return round(min(1.0, self.leg / (len(SQUARE) + 1) + (1 if self.phase == "arrived" else 0)), 2)

    def mission_state(self) -> dict:
        return {"type": "mission_state", "state": self.state, "phase": self.phase, "goal": None,
                "progress": self.progress(), "reason": self.reason}

    def pose(self) -> dict:
        return {"type": "pose", "x_m": round(self.x, 3), "y_m": round(self.y, 3), "z_m": self.z,
                "yaw_rad": round(self.yaw, 3), "sigma_m": 0.05, "frame": "map"}

    def path(self) -> dict:
        return {"type": "path", "points": [[0.0, 0.0, FLY_Z]] + [[x, y, FLY_Z] for x, y in SQUARE], "frame": "map"}

    def people(self) -> dict:
        def out(q):
            return {k: q[k] for k in ("x", "y", "conf", "t_first", "hits", "gone", "top")}
        confirmed = [q for q in self.found if q["hits"] >= 2 or q["conf"] >= 0.7]
        return {"type": "people", "frame": "map", "people": [out(q) for q in confirmed],
                "unconfirmed": [out(q) for q in self.found if q not in confirmed],
                "coverage": self.progress(), "explored_m2": None}


def run(gateway: str, mission: FakeMission, hz: float = 10.0, log=print) -> None:
    from websockets.sync.client import connect

    url = gateway.rstrip("/").replace("http://", "ws://").replace("https://", "wss://") + "/ws/autonomy"
    stamp = lambda: time.strftime("%Y-%m-%dT%H:%M:%S")   # noqa: E731
    with connect(url, open_timeout=5) as ws:
        lock = threading.Lock()
        gw = {"mode": None}

        def send(msg: dict) -> None:
            with lock:
                ws.send(json.dumps({"schema_version": SCHEMA, "at": stamp(), **msg}))

        def receive() -> None:
            for raw in ws:
                msg = json.loads(raw)
                kind = msg.get("type")
                if kind == "hello":
                    log(f"게이트웨이 연결: {msg.get('source')}, 속도 상한 {msg.get('max_speed_mps')} m/s")
                elif kind == "flight_status":
                    if msg.get("mode") != gw["mode"]:
                        log(f"게이트웨이 모드: {gw['mode']} → {msg.get('mode')}")
                    gw["mode"] = msg.get("mode")
                elif kind == "control":
                    ok, reason = mission.control(msg.get("cmd"))
                    log(f"control {msg.get('cmd')} → {'ok' if ok else reason}")
                    send({"type": "ack", "id": msg.get("id"), "ok": ok, "reason": reason})
                elif kind == "error":
                    log(f"게이트웨이가 거절: {msg.get('reason')} ({msg.get('for')})")

        threading.Thread(target=receive, daemon=True).start()
        send({"type": "hello", "node": "fake_autonomy", "version": "0.1"})
        period, last, last_state, state_sent = 1.0 / hz, time.monotonic(), None, -1e9
        seq = 0
        while True:
            now = time.monotonic()
            dt, last = now - last, now
            cmd = mission.step(dt, moving=gw["mode"] == "auto")
            if cmd is not None:
                seq += 1
                send({"type": "cmd_vel", **cmd, "frame": "body", "seq": seq})
            if not mission.path_sent and mission.phase == "running":
                send(mission.path())
                mission.path_sent = True
            send(mission.pose())
            st = mission.mission_state()
            if st != last_state or now - state_sent >= 1.0:
                if st != last_state:
                    log(f"임무 {st['phase']} · {st['state']} · 진행 {st['progress']}")
                send(st)
                last_state, state_sent = st, now
            if mission.changed_people:
                mission.changed_people = False
                p = mission.people()
                log(f"찾은 사람: 확정 {len(p['people'])}(사라짐 {sum(q['gone'] for q in p['people'])}) · 미확정 {len(p['unconfirmed'])}")
                send(p)
            time.sleep(max(0.0, period - (time.monotonic() - now)))


def main() -> None:
    ap = argparse.ArgumentParser(description="가짜 자율 비행(게이트웨이 /ws/autonomy 시험용)")
    ap.add_argument("--gateway", default="http://127.0.0.1:8010")
    ap.add_argument("--reject-start", action="store_true", help="start를 pose_not_ready로 거절")
    ap.add_argument("--stall-at", type=float, help="달린 지 이 초 뒤에 속도 명령을 끊는다(끊김 시험)")
    ap.add_argument("--stall-for", type=float, default=3.0, help="끊는 시간(초). 0.5초 넘으면 제자리, 2초 넘으면 일시정지")
    args = ap.parse_args()
    mission = FakeMission(reject_start=args.reject_start, stall_at=args.stall_at, stall_for=args.stall_for)
    try:
        run(args.gateway, mission, log=lambda text: print(text, flush=True))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
