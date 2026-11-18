"""드론 게이트웨이의 비행 모드 판정과 명령 우선순위 (표준 라이브러리만 사용).

계약: contracts/api/DRONE_CONTROL_V0_1.md 3·4·6장, AUTONOMY_BRIDGE_V0_1.md 4·6장.
네트워크와 기체를 직접 만지지 않는 순수 로직이다. 사건(사람 명령, 자율 메시지, 실행 결과)을 받아
모드를 바꾸고 "할 일"을 `drain()`으로 내준다. 실행(FlightController, WebSocket)은 게이트웨이 서버가 한다.

할 일(`Action.kind`):
- `flight`: FlightController에 시킬 일. `name`: takeoff·land·emergency·confirm_area
- `autonomy`: 자율에 보낼 `control` 메시지. `cmd`: start·pause·resume·stop
- `event`: 화면에 보낼 `event`. `kind`, `detail`

rc는 `tick()`이 매 주기(20Hz) 돌려준다. 안전 착륙·높이 상한·데드맨 0.3초 정지는 FlightController가 계속 맡고,
여기서는 "지금 누구의 입력을 보낼지"만 정한다.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

import rc_mixer
from rc_mixer import Rc, SlewLimiter

SCHEMA_VERSION = "0.1.0"
MODES = ("disconnected", "grounded", "manual", "auto", "paused", "landing", "emergency")
FLYING_MODES = ("manual", "auto", "paused", "landing")
SOURCES = ("tello", "fake", "replay")

OPERATOR_DEADMAN_S = 0.3     # 사람 rc가 이보다 오래 없으면 0
AUTONOMY_STALE_S = 0.5       # 자율 속도 명령이 이보다 오래 없으면 제자리(auto 유지)
AUTONOMY_TIMEOUT_S = 2.0     # 이보다 오래 없으면 paused
CMD_IGNORED_EVENT_S = 1.0    # auto가 아닐 때 온 cmd_vel 알림은 1초에 한 번 이하
AUTO_START_CLIENTS = ("monitor",)  # 0.1.0: 자율 시작은 PC 감독 화면에서만

ZERO: Rc = (0, 0, 0, 0)


@dataclass
class Action:
    kind: str
    payload: dict = field(default_factory=dict)


@dataclass
class RcOut:
    rc: Rc | None      # None이면 rc를 보내지 않는다(지상·착륙 중·비상 정지)
    source: str


class ModeArbiter:
    def __init__(self, source: str = "tello", clock=time.monotonic, *,
                 rc_limit: int = rc_mixer.RC_LIMIT, k_lin: float = rc_mixer.K_LIN, k_yaw: float = rc_mixer.K_YAW,
                 deadzone: float = rc_mixer.DEADZONE, slew_rc_per_s: float = rc_mixer.SLEW_RC_PER_S):
        if source not in SOURCES:
            raise ValueError(f"source는 {SOURCES} 중 하나여야 한다: {source}")
        self.source = source
        self.clock = clock
        self.rc_limit, self.k_lin, self.k_yaw, self.deadzone = rc_limit, k_lin, k_yaw, deadzone
        self.slew = SlewLimiter(slew_rc_per_s)

        self.mode = "disconnected"
        self.busy: str | None = None             # takeoff 진행 중
        self.landing_reason: str | None = None
        self.last_transition: dict | None = None
        self.operator_interventions = 0
        self.autonomy_used = False

        self.autonomy_connected = False
        self.autonomy_ground_origin = False      # 자율이 지상에서 연결됐는가(위치 원점이 출발점)
        self.mission_active = False              # start 뒤 stop·끝남 전까지
        self.mission: dict | None = None
        self.autonomy_warning: str | None = None  # autonomy_stale / autonomy_timeout / autonomy_disconnected 등
        self._pending_start: str | None = None   # ack를 기다리는 start id
        self._control_ids = itertools.count(1)

        self._op_rc: Rc = ZERO
        self._op_rc_t = -1e9
        self._cmd_vel_rc: Rc = ZERO
        self._cmd_vel_t = -1e9
        self._cmd_ignored_t = -1e9
        self.link_ok = False
        self._actions: list[Action] = []

    # ---------- 내보내기 ----------
    def drain(self) -> list[Action]:
        out, self._actions = self._actions, []
        return out

    def _emit(self, kind: str, **detail) -> None:
        self._actions.append(Action("event", {"kind": kind, "detail": detail}))

    def _flight(self, name: str, **args) -> None:
        self._actions.append(Action("flight", {"name": name, **args}))

    def _control(self, cmd: str, goal: dict | None = None) -> str:
        cid = f"g-{next(self._control_ids):04d}"
        payload = {"schema_version": SCHEMA_VERSION, "type": "control", "cmd": cmd, "id": cid}
        if goal is not None:
            payload["goal"] = goal
        self._actions.append(Action("autonomy", payload))
        return cid

    def _set_mode(self, mode: str, by: str, reason: str) -> None:
        if mode == self.mode:
            return
        prev, self.mode = self.mode, mode
        self.last_transition = {"from": prev, "to": mode, "by": by, "reason": reason}
        self.slew.reset()  # 입력 주인이 바뀌면 가감속을 0에서 다시 센다
        self._emit("mode_changed", **self.last_transition)

    def _stop_autonomy(self) -> None:
        """사람·안전 장치가 끼어들 때. 진행 중인 미션이 있으면 stop을 보낸다."""
        if self.mission_active and self.autonomy_connected:
            self._control("stop")
        self.mission_active = False
        self._pending_start = None

    def _count_intervention(self) -> None:
        if self.mode == "auto" or (self.mode == "paused" and self.mission_active):
            self.operator_interventions += 1

    # ---------- 상태 ----------
    @property
    def authority(self) -> str:
        return {"manual": "operator", "auto": "autonomy"}.get(self.mode, "none")

    @property
    def flying(self) -> bool:
        return self.mode in FLYING_MODES

    def allowed(self) -> list[str]:
        if self.source == "replay":
            return []
        names = ["emergency"]
        if self.mode == "grounded":
            names += ["confirm_area"] + ([] if self.busy else ["takeoff"])
        elif self.mode == "manual":
            names += ["land"] + (["auto_start"] if self._auto_start_block() is None else [])
        elif self.mode == "auto":
            names += ["land", "pause", "takeover", "auto_stop"]
        elif self.mode == "paused":
            names += ["land", "takeover"]
            if self.mission_active:
                names += ["auto_stop"] + (["resume"] if self.autonomy_connected else [])
        return names

    def warnings(self) -> list[str]:
        """판정기가 아는 경고만. 배터리·온도·영상·높이 경고는 서버가 상태값으로 붙인다."""
        out = []
        if self.mode in FLYING_MODES and not self.link_ok:
            out.append("state_stale")
        if self.mode in ("grounded", "disconnected") and not self.autonomy_connected:
            out.append("autonomy_not_ready")
        if self.autonomy_warning:
            out.append(self.autonomy_warning)
        return out

    def snapshot(self) -> dict:
        return {
            "mode": self.mode,
            "authority": self.authority,
            "allowed": self.allowed(),
            "warnings": self.warnings(),
            "flying": self.flying,
            "landing_reason": self.landing_reason if self.mode == "landing" else None,
            "mission": self.mission if self.autonomy_connected else None,
            "autonomy": {"connected": self.autonomy_connected, "ground_origin": self.autonomy_ground_origin,
                         "mission_active": self.mission_active},
            "last_transition": self.last_transition,
            "operator_interventions": self.operator_interventions,
            "autonomy_used": self.autonomy_used,
        }

    # ---------- 기체 연결·실행 결과 (서버가 알려 준다) ----------
    def on_link(self, ok: bool) -> None:
        """상태 패킷이 1초 안에 오고 있는가. 비행 중 끊김 처리(안전 착륙)는 FlightController가 한다."""
        self.link_ok = ok
        if not ok and self.mode == "grounded":
            self._set_mode("disconnected", "gateway", "state_lost")
        elif ok and self.mode == "disconnected":
            self._set_mode("grounded", "gateway", "state_ok")

    def on_takeoff_done(self, ok: bool, reason: str | None = None) -> None:
        self.busy = None
        if ok:
            self.landing_reason = None
            self._set_mode("manual", "operator", "takeoff")
        else:
            self._emit("command_rejected", name="takeoff", reason=reason or "error_response")

    def on_safety_landing(self, reason: str) -> None:
        """FlightController가 상태 끊김·배터리·과열로 착륙을 시작했다."""
        self._stop_autonomy()
        self.landing_reason = reason
        self._set_mode("landing", "safety", reason)
        self._emit("safety_landing", reason=reason)

    def on_landed(self, reason: str = "landed") -> None:
        """착륙 완료 또는 비상 정지 뒤 지상 확인."""
        self.mission_active = False
        self.busy = None
        self.autonomy_warning = None if self.autonomy_connected else self.autonomy_warning
        self._set_mode("grounded" if self.link_ok else "disconnected", "gateway", reason)

    # ---------- 사람 ----------
    def _result(self, cmd_id, ok: bool, reason: str | None = None) -> dict:
        return {"schema_version": SCHEMA_VERSION, "type": "result", "id": cmd_id, "ok": ok, "reason": reason,
                "mode": self.mode}

    def _auto_start_block(self) -> str | None:
        if not self.autonomy_connected:
            return "autonomy_not_connected"
        if not self.autonomy_ground_origin:
            return "autonomy_joined_in_air"
        if self._pending_start:
            return "busy"
        return None

    def handle_cmd(self, name: str, args: dict | None = None, client: str = "monitor", cmd_id=None) -> dict:
        args = args or {}
        if self.source == "replay":
            return self._result(cmd_id, False, "replay_source")
        if name == "emergency":  # 우선순위 1: 언제나
            self._count_intervention()
            self._stop_autonomy()
            self._set_mode("emergency", client, "emergency")
            self._flight("emergency")
            return self._result(cmd_id, True)
        if name not in self.allowed():
            reason = "not_allowed_in_mode"
            if name == "auto_start" and self.mode == "manual":
                reason = self._auto_start_block() or reason
            return self._result(cmd_id, False, reason)

        if name == "land":
            self._count_intervention()
            self._stop_autonomy()
            self.landing_reason = "operator"
            self._set_mode("landing", client, "land")
            self._flight("land")
        elif name == "takeoff":
            self.busy = "takeoff"
            self._flight("takeoff")
        elif name == "confirm_area":
            self._flight("confirm_area", confirmed=bool(args.get("confirmed", True)))
        elif name == "pause":
            self._count_intervention()
            self._control("pause")
            self._set_mode("paused", client, "pause")
        elif name == "resume":
            self._control("resume")
            self._cmd_vel_t = self.clock()  # 다시 시작한 순간부터 끊김을 센다
            self.autonomy_warning = None
            self._set_mode("auto", client, "resume")
        elif name == "takeover":
            self._take_over(client, "takeover")
        elif name == "auto_stop":
            self._count_intervention()
            self._stop_autonomy()
            self._set_mode("paused", client, "auto_stop")
        elif name == "auto_start":
            if client not in AUTO_START_CLIENTS:
                return self._result(cmd_id, False, "client_not_allowed")
            goal = args.get("goal")
            self._pending_start = self._control("start", goal if isinstance(goal, dict) else None)
            self.mission_active = True
            self.autonomy_used = True
            self.autonomy_warning = None
            self._cmd_vel_t = self.clock()
            self._cmd_vel_rc = ZERO
            self._set_mode("auto", client, "auto_start")
        return self._result(cmd_id, True)

    def _take_over(self, by: str, reason: str) -> None:
        self._count_intervention()
        self._stop_autonomy()
        self.autonomy_warning = None if self.autonomy_connected else self.autonomy_warning
        self._set_mode("manual", by, reason)

    def on_operator_rc(self, axes, client: str = "control") -> None:
        """사람 rc [좌우, 앞뒤, 상하, 회전] -1~1. 자율 중 데드존 밖 입력은 넘겨받기."""
        if self.source == "replay" or len(axes) != 4:
            return
        if self.mode in ("auto", "paused") and rc_mixer.outside_deadzone(axes, self.deadzone):
            self._take_over(client, "joystick_takeover")
        if self.mode == "manual":
            self._op_rc = rc_mixer.operator_axes_to_rc(axes, rc_limit=self.rc_limit, deadzone=self.deadzone)
            self._op_rc_t = self.clock()

    def on_operator_disconnected(self) -> None:
        """화면 WebSocket이 끊기면 그 화면의 rc를 바로 0으로 본다."""
        self._op_rc, self._op_rc_t = ZERO, -1e9

    # ---------- 자율 ----------
    def on_autonomy_connected(self) -> None:
        self.autonomy_connected = True
        self.autonomy_ground_origin = self.mode in ("grounded", "disconnected")
        if self.autonomy_warning == "autonomy_disconnected":
            self.autonomy_warning = None
        self._emit("autonomy_connected", ground_origin=self.autonomy_ground_origin)

    def on_autonomy_disconnected(self) -> None:
        self.autonomy_connected = False
        self.autonomy_ground_origin = False
        self._pending_start = None
        self.mission = None
        if self.mode == "auto":
            self.autonomy_warning = "autonomy_disconnected"
            self._set_mode("paused", "gateway", "autonomy_disconnected")
        self.mission_active = False
        self._emit("autonomy_disconnected")

    def on_cmd_vel(self, msg: dict) -> bool:
        """자율 속도 명령. 기체로 갈 수 있으면 True."""
        now = self.clock()
        rc = None
        if msg.get("frame", "body") == "body":
            rc = rc_mixer.cmd_vel_to_rc(msg.get("vx_mps"), msg.get("vy_mps"), msg.get("vz_mps"),
                                        msg.get("yaw_rate_rps"), rc_limit=self.rc_limit, k_lin=self.k_lin,
                                        k_yaw=self.k_yaw)
        if self.mode != "auto" or rc is None:
            if now - self._cmd_ignored_t >= CMD_IGNORED_EVENT_S:
                self._cmd_ignored_t = now
                self._emit("cmd_ignored", mode=self.mode, reason="bad_cmd_vel" if rc is None else "not_auto")
            return False
        self._cmd_vel_rc, self._cmd_vel_t = rc, now
        if self.autonomy_warning == "autonomy_stale":
            self.autonomy_warning = None
        return True

    def on_mission_state(self, msg: dict) -> None:
        self.mission = {k: msg.get(k) for k in ("state", "phase", "goal", "progress", "reason")}
        if self.mode == "auto" and msg.get("phase") in ("arrived", "failed"):
            self.mission_active = False
            self._set_mode("paused", "autonomy", "mission_" + msg["phase"])
            self._emit("mission_finished", phase=msg["phase"], reason=msg.get("reason"))

    def on_ack(self, msg: dict) -> None:
        if msg.get("id") != self._pending_start:
            return
        self._pending_start = None
        if not msg.get("ok"):
            self.mission_active = False
            self._emit("command_rejected", name="auto_start", reason="autonomy_rejected", detail=msg.get("reason"))
            if self.mode == "auto":
                self._set_mode("manual", "autonomy", "autonomy_rejected")

    def on_autonomy_request(self, name: str) -> dict:
        """자율의 /tt/flight_cmd. 우선순위 5: 사람이 조종 중이면 따르지 않는다."""
        if name == "land":
            if self.mode in ("auto", "paused") and self.source != "replay":
                self.mission_active = False
                self.landing_reason = "autonomy"
                self._set_mode("landing", "autonomy", "autonomy_land")
                self._flight("land")
                return {"ok": True, "reason": None}
            return {"ok": False, "reason": "not_allowed_in_mode"}
        if name == "takeoff":
            return {"ok": False, "reason": "takeoff_by_operator_only"}
        if name == "emergency":
            if self.mode == "auto":
                self.autonomy_warning = "autonomy_emergency_request"
                self._set_mode("paused", "autonomy", "emergency_request")
            return {"ok": False, "reason": "emergency_by_operator_only"}
        return {"ok": False, "reason": "unknown_request"}

    # ---------- 주기 ----------
    def tick(self) -> RcOut:
        """매 rc 주기(20Hz)에 부른다. 지금 보낼 rc와 출처."""
        now = self.clock()
        if self.mode == "manual":
            if now - self._op_rc_t > OPERATOR_DEADMAN_S:
                target, source = ZERO, "deadman"
            else:
                target, source = self._op_rc, "operator"
        elif self.mode == "auto":
            idle = now - self._cmd_vel_t
            if idle > AUTONOMY_TIMEOUT_S:
                self.autonomy_warning = "autonomy_timeout"
                self._control("pause")
                self._set_mode("paused", "gateway", "autonomy_timeout")
                return RcOut(ZERO, "paused")
            if idle > AUTONOMY_STALE_S:
                self.autonomy_warning = "autonomy_stale"
                target, source = ZERO, "autonomy_stale"
            else:
                target, source = self._cmd_vel_rc, "autonomy"
        elif self.mode == "paused":
            return RcOut(ZERO, "paused")
        else:
            return RcOut(None, self.mode)
        return RcOut(self.slew.step(target, now), source)
