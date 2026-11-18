"""정답 위치 없이 드론 위치를 추정한다. 실기체·시뮬레이션 공통 (ROS와 무관).

1) 속도 적분: Tello 상태값(속도·yaw·높이)을 적분한다. 속도가 0.1 m/s 단위 정수로 오고 배율 오차와
   yaw 누적 오차가 있어, 시간이 갈수록 위치가 어긋난다.
2) 지도 보정(받은 지도가 있을 때): 위치 후보(파티클) 수백 개를 같이 움직이면서
   - 벽·가구 안에 들어간 후보는 버리고 (드론은 벽 속에 있을 수 없다)
   - 전방 ToF 측정이 그 후보 자리에서 지도대로 나와야 할 거리와 맞는지로 점수를 매긴다.
   지도에 없는 장애물은 측정을 '더 가깝게'만 만들므로, 더 가까운 측정에는 약하게, 지도보다 먼
   측정(벽을 뚫고 본 셈)이나 보여야 할 벽이 안 보이는 경우에는 강하게 감점한다.
마커(미션 패드·AprilTag)는 쓰지 않는다. 카메라로 보정하는 일은 따로 한다(Jira 209).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .tello_state import wrap
from .voxel_map import VoxelMap


@dataclass
class EstimatorParams:
    n_particles: int = 300
    use_map: bool = True
    init_xy: float = 0.05          # 출발 위치 불확실성 (m, 1σ). 정해진 자리에 놓고 시작한다
    init_yaw: float = math.radians(2.0)
    scale_sigma: float = 0.05      # 속도 배율 오차 사전분포 (1σ)
    vel_noise: float = 0.05        # 속도 잡음 (m/s, 1σ)
    yaw_walk: float = math.radians(0.3)   # 1초당 방향 오차 변화 (1σ)
    scale_walk: float = 0.0        # 1초당 속도 배율 변화 (1σ). 0이면 비행 내내 같은 배율로 본다.
    #                                0.005·0.01, 속도 잡음 0.08, 방향 변화 0.5°도 기록된 비행 18개로 다시 돌려 봤지만
    #                                모두 지금 값보다 최대 오차가 컸다 (tools/replay_pose.py --inputs)
    tof_sigma: float = 0.08        # ToF가 지도와 맞을 때의 허용 오차
    tof_max: float = 1.2
    tof_offset: float = 0.05       # 기체 중심에서 ToF까지 앞쪽 거리
    tof_min_move: float = 0.05     # 지난 ToF 보정 뒤 이만큼 움직였거나
    tof_min_turn: float = math.radians(3.0)   # 이만큼 돌았을 때만 다시 보정한다 (멈춰서 같은 측정을 거듭 믿지 않게)
    state_latency: float = 0.03    # 상태값이 재진 뒤 도착하기까지 걸리는 시간 (s, 가정. 실기체에서 잰다)
    tof_max_yaw_rate: float = math.radians(10.0)   # 이보다 빨리 돌 때는 ToF로 보정하지 않는다. 상태값 yaw는
    #                                30–130 ms 늦게 오므로 빨리 돌면 측정 방향이 몇 도씩 틀린다. 그보다 느릴 때는
    #                                회전 속도로 ToF 순간의 yaw를 앞당겨 쓴다. 60°/s까지 앞당겨 쓰게 해 봤지만
    #                                기록된 비행 9개를 다시 돌려 보니 10°/s에서 끊는 쪽이 나았다 (tools/replay_pose.py)
    tof_outlier: float = 0.8       # 후보 대부분(점수 합 이 비율 이상)에게 지도보다 확실히 가까운 측정은
    #                                지도에 없는 물체로 보고 보정에 쓰지 않는다 (장애물 처리는 비행 코드 몫)
    block_stop_frac: float = 0.5   # 벽 속으로 들어가는 후보의 점수 합이 이보다 작을 때만 그 후보를 벽 앞에 멈춘다.
    #                                대부분이 막히면 멈추지 않고 그대로 움직인다: 속도는 움직였다는데 지도와 안 맞는
    #                                것은 추정이 옆으로 틀어진 탓이다. 멈춰 세웠더니 실제로는 문틀 옆을 지나 2.9 m
    #                                날아가는 동안 추정만 벽 앞에 남았다 (3단계 seed 3)
    height_source: str = "h"       # "h": Tello h를 높이로(시뮬레이션). "tof": 하방 ToF를 높이로 쓰고, ToF가 없거나
    #                                tof_jump 넘게 튀면(밑에 가구) vz로 이어 간다(실기체: h가 바닥 기준이 아닐 때가 있다)
    tof_jump: float = 0.25         # "tof" 모드: 예측보다 이만큼 넘게 다른 ToF는 높이로 쓰지 않는다 (m)
    tof_settle: float = 1.0        # 그래도 이 시간(s) 넘게 계속 다르면 그 ToF를 믿는다 (예측이 틀어진 경우)
    tof_settle_closer: float = 4.0  # 갑자기 가까워진 쪽(밑에 가구가 들어옴)은 이만큼 기다린다: 가구 위로 넘는 동안(0.25 m/s로 1 m에
    #                                4 s) ToF를 믿으면 높이가 가구 높이만큼 낮게 잡혀 천장 쪽으로 더 오른다 (11/03 수직 공간 쓰기).
    #                                그 뒤에도 Tello h가 ToF 변화의 절반만큼도 같이 내려가지 않았으면 밑에 가구가 있는 것으로 보고 계속 믿지 않는다
    #                                (h는 바닥 기준은 아니지만 변화는 따라간다). h가 없을 때만 시간으로 믿는다
    h_guard_gap: float = 0.4       # "tof" 모드: ToF 높이가 Tello h로 본 높이(h + 평소 차이)보다 이만큼 넘게 낮으면 밑에 가구로 보고
    #                                h로 본 높이를 쓴다(0이면 끔). 위 tof_jump 검사는 한 번에 크게 튈 때만 걸려, 가구 가장자리를
    #                                천천히 지나며 ToF가 조금씩 줄면 못 막는다. 2026-11-09 17:07: SSAFY 글자 조형물 위로 지나며
    #                                ToF가 1.07→0.70 m로 0.1초에 0.04–0.12씩 줄어 높이가 0.67 m로 잡혔고(h는 1.6–1.7 그대로),
    #                                1 m로 오르려다 h 2.0 m에서 40초 넘게 천장에 눌렸다. 바닥 높낮이·h 오차로 평소에도
    #                                ±0.3 m는 어긋나므로 그보다 크게 둔다
    h_guard_track: float = 0.2     # ToF와 h로 본 높이가 이만큼 안에서 맞을 때만 평소 차이를 따라간다
    h_guard_tau: float = 5.0       # 평소 차이를 따라가는 시간 상수 (s)
    h_guard_span: float = 1.5      # 가구로 본 자리에서 이만큼 넘게 옮겨 갔는데도 어긋남이 그대로면 가구가 아니라 h가 어긋난 것
    #                                (17:07 202–207초에 h−ToF가 +0.3에서 −0.3으로 바뀐 뒤 60초 동안 그대로였다) → 새 차이를 받아들인다
    h_guard_min: float = 0.5       # Tello h가 이보다 낮으면 쓰지 않는다 (이륙·착륙 중, 떠 있는데 0~10 cm로 나온 적도 있다)
    under_fresh: float = 1.5       # "tof" 모드: 카메라 지도가 아는 '기체 밑 가구 꼭대기'(on_under, 비행 노드 under_top)를 이 시간(s) 안이면 쓴다.
    #                                하방 ToF는 바로 밑 물건까지의 거리라 가구 위에서는 높이가 아니다 (사용자 지시, 11/09 17:07) —
    #                                밑에 무엇이 있는지는 깊이 영상으로 쌓은 지도(비전)가 안다: 바닥 위 높이 = ToF + 꼭대기.
    #                                단안 깊이로 바닥을 직접 재는 것은 너무 흔들려 못 쓴다 (perception/depth.py 설명).
    #                                지도에 없는 물건이면 위 h_guard가 맡는다
    under_agree: float = 0.2       # 밑에 지도 가구가 있는 동안은 들어올 때의 높이를 기준으로 붙든다(vz로만 이어 감): ToF + 꼭대기가 기준과
    #                                이 안이면 가구 위(높이 = ToF + 꼭대기), ToF가 기준과 under_floor 안이면 가구 옆 바닥(ToF 그대로),
    #                                그 사이(가장자리를 지나며 조금씩 줄어드는 구간)면 높이를 붙든다. 기준을 ToF에 맞춰 가면 조금씩 줄 때
    #                                같이 내려간다 (17:07 0.1초에 0.04–0.12 m)
    under_floor: float = 0.15
    under_min_top: float = 0.15    # 이보다 낮은 꼭대기(바닥 턱·카펫)는 따로 보지 않는다
    no_slip: bool = True          # 옆 미끄럼 없음: 앞뒤로 날 때는 움직인 방향을 기수(yaw)로 본다 (_no_slip 설명)
    slip_max: float = 0.1          # 기수에 수직인 속도가 이 이하(m/s)면 정수 자름 오차로 보고 0으로 둔다
    slip_min_speed: float = 0.1    # 이보다 느리면(0.1 m/s 단위 한 칸 미만) 그대로 쓴다
    slip_max_yaw_rate: float = math.radians(30.0)   # 이보다 빨리 돌 때는 그대로 쓴다 (돌기 전 속도가 남아 미끄러진다)
    drone_half: float = 0.0        # 벽 속 후보를 깎을 때 지도를 부풀리는 폭. 기체 반폭(0.09)만큼 부풀리면
    #                                벽을 스치며 지나간 진짜 위치 후보까지 깎여 추정이 반대쪽으로 튄다


class PoseEstimator:
    """속도 적분 + (선택) 지도 기반 파티클 보정."""

    def __init__(self, start_xy, start_yaw: float, start_z: float = 0.0,
                 occ2d: np.ndarray | None = None, origin=(0.0, 0.0), res: float = 0.05,
                 params: EstimatorParams | None = None, seed: int = 0, solid2d: np.ndarray | None = None):
        """occ2d: 비행 높이 단면 (벽 + 가구). solid2d: 그중 ToF가 뚫고 볼 수 없는 벽·문 위 벽만 (같은 격자).

        가구는 지도에 바닥 면적 상자로 들어 있지만 실제로는 선반 칸, 의자·책상 다리 사이가 비어 있어
        ToF가 상자보다 멀게 잴 수 있다. solid2d를 주면 '가구보다 멀지만 벽보다는 가까운' 측정을 약하게만 감점한다.
        """
        self.p = params or EstimatorParams()
        self.rng = np.random.default_rng(seed)
        self.start_yaw = start_yaw
        self.start_z = start_z
        self.yaw_ref = None            # 첫 상태값의 Tello yaw (이때 기체가 start_yaw를 보고 있다)
        self.t_last = None
        self.z = start_z
        self.z_from_tof = False        # "tof" 모드: 한 번이라도 ToF로 높이를 잡았는지
        self.tof_odd_since = None      # "tof" 모드: ToF가 예측과 달라지기 시작한 시각
        self.tof_odd_h0 = None         #   그때의 Tello h와 높이 추정 (가까워진 ToF를 믿을지 h 변화로 가린다)
        self.tof_odd_z0 = 0.0
        self.h_off = None              # "tof" 모드: 바닥 위에서 본 (ToF 높이 − Tello h)의 평소 값 (h_guard_gap)
        self.h_guarding = False        #   지금 밑에 가구가 있다고 보고 h로 본 높이를 쓰는 중인지
        self.h_guard_count = 0         #   그렇게 들어간 횟수 (기록용)
        self.h_guard_from = None       #   들어간 자리 (x, y)와 그때의 어긋남
        self.under = None              # "tof" 모드: 카메라 지도가 아는 기체 밑 가구 꼭대기 (받은 시각, m 또는 None=바닥) — on_under
        self.under_guarding = False    #   지금 'ToF + 꼭대기'를 높이로 쓰는 중인지
        self.under_guard_count = 0     #   그렇게 들어간 횟수 (기록용)
        self.under_ref = None          #   밑에 가구가 있는 동안 붙드는 기준 높이 (들어올 때의 높이, vz로만 이어 감)
        self.yaw_tello = 0.0
        self.use_map = self.p.use_map and occ2d is not None
        self.occ = occ2d
        self.solid = solid2d
        self.origin = np.asarray(origin, dtype=float)
        self.res = res
        self.free = None
        if occ2d is not None:
            self.free = ~_dilate(occ2d, int(math.ceil(self.p.drone_half / res)))
        n = self.p.n_particles if self.use_map else 1
        self.x = np.full(n, float(start_xy[0])) + (self.rng.normal(0, self.p.init_xy, n) if n > 1 else 0)
        self.y = np.full(n, float(start_xy[1])) + (self.rng.normal(0, self.p.init_xy, n) if n > 1 else 0)
        self.b = self.rng.normal(0, self.p.init_yaw, n) if n > 1 else np.zeros(1)     # 방향 오차
        self.s = 1 + self.rng.normal(0, self.p.scale_sigma, n) if n > 1 else np.ones(1)  # 속도 배율
        self.w = np.full(n, 1.0 / n)
        self.lost_resets = 0
        self.tof_ref = None            # 마지막으로 ToF 보정을 한 추정 자세 (x, y, yaw)
        self.yaw_hist: deque = deque()  # 최근 0.5초 (받은 시각, Tello yaw): 회전 속도 계산용
        self.yaw_rate = 0.0
        self.t_state = None            # 마지막 상태값을 받은 시각
        self.yaw_lead = 0.0            # ToF를 잰 순간까지 앞당긴 yaw (on_tof 안에서만 0이 아니다)

    # ------------------------------------------------------------ 예측: 상태값

    def map_yaw(self) -> float:
        if self.yaw_ref is None:
            return self.start_yaw
        return wrap(self.start_yaw + wrap(self.yaw_tello - self.yaw_ref))

    def on_state(self, si: dict, t: float) -> None:
        """tello_state.to_si 결과와 받은 시각으로 후보들을 움직인다."""
        if self.yaw_ref is None:
            self.yaw_ref = si["yaw"]
        self.yaw_tello = si["yaw"]
        self.t_state = t
        self.yaw_hist.append((t, si["yaw"]))
        while len(self.yaw_hist) > 2 and t - self.yaw_hist[0][0] > 0.5:
            self.yaw_hist.popleft()
        t0, y0 = self.yaw_hist[0]
        # 1° 단위 정수라 0.1초 간격으로 재면 1°만 바뀌어도 10°/s가 된다. 0.25초 넘는 간격으로 잰다
        self.yaw_rate = wrap(si["yaw"] - y0) / (t - t0) if t - t0 >= 0.25 else 0.0
        dt = 0.0 if self.t_last is None else min(max(t - self.t_last, 0.0), 0.5)
        self.z = self.height(si, dt, t)
        if self.t_last is None:
            self.t_last = t
            return
        self.t_last = t
        if dt <= 0:
            return
        n = len(self.x)
        vx0, vy0 = self._no_slip(si)
        vx = vx0 + (self.rng.normal(0, self.p.vel_noise, n) if n > 1 else 0)
        vy = vy0 + (self.rng.normal(0, self.p.vel_noise, n) if n > 1 else 0)
        if n > 1:
            self.b = self.b + self.rng.normal(0, self.p.yaw_walk * math.sqrt(dt), n)
            if self.p.scale_walk > 0:
                self.s = self.s + self.rng.normal(0, self.p.scale_walk * math.sqrt(dt), n)
        # takeoff: 속도값이 Tello yaw 0 방향 기준이다. 지도에서 그 방향 = 출발 방향 − 출발 때 Tello yaw
        heading = (self.map_yaw() if si["frame"] == "body" else wrap(self.start_yaw - (self.yaw_ref or 0.0))) + self.b
        c, s = np.cos(heading), np.sin(heading)
        nx = self.x + self.s * (c * vx - s * vy) * dt
        ny = self.y + self.s * (s * vx + c * vy) * dt
        if self.use_map:
            # 벽·가구 속으로 들어가는 후보는 그 자리에 멈춘다(벽에 붙은 셈)고 보고 점수를 깎는다.
            # 단 대부분이 막히면 멈추지 않는다 (block_stop_frac 설명)
            blocked = self.free_at(nx, ny) == 0
            if float(np.sum(self.w[blocked])) < self.p.block_stop_frac:
                nx[blocked] = self.x[blocked]
                ny[blocked] = self.y[blocked]
            self.w = self.w * np.where(blocked, 0.2, 1.0)
        self.x, self.y = nx, ny
        if self.use_map:
            self._normalize()

    def _no_slip(self, si: dict) -> tuple[float, float]:
        """옆 미끄럼 없음: 앞뒤로 날 때 속도의 방향을 기수(yaw)로 바로잡는다. 돌려주는 값은 si와 같은 기준.

        Tello 속도값은 축마다 0.1 m/s 단위 정수라, 축에서 비스듬히 천천히 날면 작은 쪽 성분이 0으로 잘린다.
        2026-11-05 17:17 실기체 복도: 기수 12–16°로 0.2–0.3 m/s 직진하는 7초 내내 vgx −2·−3, vgy 0이었다
        (16°면 옆 성분 0.055–0.083 m/s). 위치는 0°로 곧게, 카메라는 −16°로 보고 벽을 넣어 지도 속 벽이 경로 쪽으로
        기울었고, 복도 안에 가짜 문·벽 위 볼 자리가 생겨 67초에 6.7 m만 갔다. 쿼드콥터는 앞으로 명령하면 기수 쪽으로
        나는 데다 Tello 속도값과 yaw는 같은 추정에서 나오므로, 기수에 수직인 성분이 정수 자름 오차(slip_max) 안이면
        0으로 보고 기수 쪽 성분만 쓴다. Gazebo 기록 25개에서 이 규칙이 적용된 순간의 정답 옆 속도는 중앙 0.002 m/s
        (90% 0.015)였다. 그보다 크게 옆으로 가거나(돌기 전 속도가 남은 미끄럼), 빨리 돌거나,
        거의 멈춰 있으면 잰 값을 그대로 쓴다. 옆 성분 0.05 m/s 미만은 원래 축에 맞춰 날 때도 0으로 잘려 안 보이므로,
        이 규칙이 잃는 정보는 없고 기수가 축과 몇 도 틀어졌든 같은 오차가 된다.
        """
        vx, vy = si["vx"], si["vy"]
        p = self.p
        speed = math.hypot(vx, vy)
        if not p.no_slip or speed < p.slip_min_speed or abs(self.yaw_rate) > p.slip_max_yaw_rate:
            return vx, vy
        psi = 0.0 if si["frame"] == "body" else si["yaw"]   # takeoff: 앞으로 날면 (vx, vy)가 Tello yaw 쪽을 향한다
        c, s = math.cos(psi), math.sin(psi)
        fwd = c * vx + s * vy
        lat = -s * vx + c * vy
        if abs(lat) > p.slip_max or abs(fwd) < abs(lat):
            return vx, vy
        # 빠르기는 기수 쪽 성분만: 크기(hypot)를 쓰면 옆 잡음이 빠르기에 더해져 늘 길게 간다
        return c * fwd, s * fwd

    def height(self, si: dict, dt: float, t: float) -> float:
        """이번 상태값으로 본 높이 (start_z 기준 바닥 위)."""
        if self.p.height_source != "tof":
            return self.start_z + si["h"]
        pred = self.z + si["vz"] * dt
        tof = si.get("tof")
        if tof is None:                     # 바닥에 붙었거나(10 cm 이하) 너무 높음: 이륙 전이면 바닥, 아니면 vz로
            return pred if self.z_from_tof else self.start_z
        meas = self.start_z + tof
        h = si.get("h")
        h_ok = isinstance(h, (int, float)) and h >= self.p.h_guard_min
        top = self.under[1] if (self.under is not None and t - self.under[0] <= self.p.under_fresh and self.z_from_tof) else None
        if top is not None and top >= self.p.under_min_top:
            # 밑에 지도가 아는 가구 (11/09 17:07 조형물): 들어올 때의 높이를 기준으로 붙들고 ToF를 두 가지로만 읽는다
            if self.under_ref is None:
                self.under_ref = self.z
            self.under_ref += si["vz"] * dt
            over = meas + top   # 가구 위라면 ToF는 꼭대기까지: 바닥 위 높이 = ToF + 꼭대기
            if abs(meas - self.under_ref) < self.p.under_floor:
                self.under_guarding = False   # 가구 옆 바닥을 잰다: 아래 보통 규칙으로 (기준은 ToF에 다시 맞춘다)
                self.under_ref = meas
            elif abs(over - self.under_ref) < self.p.under_agree:
                if not self.under_guarding:
                    self.under_guarding, self.under_guard_count = True, self.under_guard_count + 1
                self.tof_odd_since = None
                self.tof_odd_h0 = h
                return over
            else:
                self.tof_odd_since = None     # 가장자리(조금씩 줄어드는 구간): 높이를 붙든다
                return pred
        else:
            self.under_guarding, self.under_ref = False, None
        if self.p.h_guard_gap > 0 and h_ok and self.h_off is not None and self.z_from_tof:
            ref = h + self.h_off
            gap = ref - meas
            if gap > self.p.h_guard_gap:
                # 밑에 가구: ToF가 조금씩 줄었어도 h는 그대로다. h는 0.1 m 단위라 들어간 뒤에는 부드럽게 따라간다
                xy = (float(np.average(self.x, weights=self.w)), float(np.average(self.y, weights=self.w)))
                if not self.h_guarding:
                    self.h_guarding, self.h_guard_count = True, self.h_guard_count + 1
                    self.h_guard_from = (xy, gap)
                    self.tof_odd_since = None
                    return ref
                (x0, y0), gap0 = self.h_guard_from
                if math.hypot(xy[0] - x0, xy[1] - y0) <= self.p.h_guard_span or abs(gap - gap0) > self.p.h_guard_track:
                    return self.z + 0.3 * (ref - self.z)
                # 멀리 옮겨 갔는데 어긋남이 그대로: 가구가 아니라 h가 어긋났다. 지금 차이를 평소 차이로 받아들인다
                self.h_off = meas - h
                self.h_guarding, self.h_guard_from = False, None
                self.tof_odd_since = None
                self.tof_odd_h0, self.tof_odd_z0 = h, meas
                return meas
        if self.h_guarding:
            self.h_guarding, self.h_guard_from = False, None
            pred = self.z   # 가구에서 벗어남: h로 이어 온 높이에서 ToF로 돌아간다
        if not self.z_from_tof or abs(meas - pred) < self.p.tof_jump:
            self.z_from_tof, self.tof_odd_since = True, None
            self.tof_odd_h0, self.tof_odd_z0 = h, meas   # 마지막으로 ToF를 믿은 때의 h와 높이
            if h_ok:   # 바닥 위에서 본 (ToF 높이 − h): 처음 값으로 잡고, 맞을 때만 천천히 따라간다
                d = meas - h
                if self.h_off is None:
                    self.h_off = d
                elif abs(d - self.h_off) < self.p.h_guard_track:
                    self.h_off += (d - self.h_off) * min(1.0, dt / self.p.h_guard_tau)
            return meas
        # 갑자기 가까워짐·멀어짐: 밑에 가구·턱. 높이는 vz로 이어 가다가, tof_settle 넘게 그대로면 ToF를 믿는다
        if self.tof_odd_since is None:
            self.tof_odd_since = t
        closer = meas < pred
        if closer and isinstance(h, (int, float)) and isinstance(self.tof_odd_h0, (int, float)):
            # 밑에 가구: ToF는 가구 높이만큼 줄었는데 h는 그대로다. 정말 내려간 것이면 h도 그만큼 내려간다
            if (self.tof_odd_h0 - h) < 0.5 * (self.tof_odd_z0 - meas):
                return pred
        if t - self.tof_odd_since >= (self.p.tof_settle_closer if closer else self.p.tof_settle):
            self.tof_odd_since = None
            return meas
        return pred

    def on_under(self, top, t: float) -> None:
        """카메라 지도가 아는 기체 바로 밑 가구 꼭대기 높이 (m, 없으면 None — 비행 노드가 /tt/under_map으로 보낸다). height()가 쓴다."""
        if top is None or (isinstance(top, (int, float)) and math.isfinite(top)):
            self.under = (t, None if top is None else float(top))

    # ------------------------------------------------------------ 위치 다시 잡기

    def spread(self, xy: float, yaw: float) -> None:
        """후보를 지금 믿음보다 넓게(1σ xy m, yaw rad) 흩는다.

        긴 비행에서 추정이 0.45 m 틀어진 채 σ 0.1 m로 굳으면, 진짜 자리 근처에 후보가 없어 ToF가 벽을
        재도 바로잡지 못한다. 흩은 뒤 제자리에서 천천히 한 바퀴 돌며 재면 맞는 후보만 살아남는다.
        벽 속으로 흩어진 후보는 다음 상태값에서 점수가 깎여 곧 사라진다.
        """
        n = len(self.x)
        if n <= 1:
            return
        self.x = self.x + self.rng.normal(0, xy, n)
        self.y = self.y + self.rng.normal(0, xy, n)
        self.b = self.b + self.rng.normal(0, yaw, n)
        self.w = np.full(n, 1.0 / n)
        self.tof_ref = None

    # ------------------------------------------------------------ 보정: 전방 ToF

    def on_tof(self, measured: float, fan: np.ndarray, t: float | None = None) -> None:
        """measured: 전방 ToF 최솟값(m, 못 재면 inf). fan: 기체 기준 수평 광선 각도들. t: 잰 시각(상태값과 같은 시계)."""
        if not self.use_map or abs(self.yaw_rate) > self.p.tof_max_yaw_rate:
            return
        lag = 0.0
        if t is not None and self.t_state is not None:
            lag = min(max(t - self.t_state + self.p.state_latency, 0.0), 0.3)
        self.yaw_lead = self.yaw_rate * lag
        try:
            self._tof_update(measured, fan)
        finally:
            self.yaw_lead = 0.0

    def _tof_update(self, measured: float, fan: np.ndarray) -> None:
        x, y, _, yaw, sigma = self.estimate()
        if self.tof_ref is not None:
            rx, ry, ryaw = self.tof_ref
            if math.hypot(x - rx, y - ry) < self.p.tof_min_move and abs(wrap(yaw - ryaw)) < self.p.tof_min_turn:
                return   # 제자리: 같은 물체를 다시 잰 것은 새 정보가 아니다
        expected = self.expected_ranges(fan)
        expected_solid = self.expected_ranges(fan, self.solid) if self.solid is not None else None
        if math.isfinite(measured) and measured <= self.p.tof_max:
            closer = measured < np.minimum(expected, self.p.tof_max + 1.0) - 3 * self.p.tof_sigma
            if float(np.sum(self.w[closer])) >= self.p.tof_outlier and not self._map_near_hit(x, y, yaw, measured,
                                                                                               fan, sigma):
                return   # 지도에 없는 물체를 잰 것. 지도의 다른 물체로 설명되는 엉뚱한 후보를 키우지 않는다
        self.tof_ref = (x, y, yaw)
        self.w = self.w * tof_likelihood(measured, expected, self.p.tof_max, self.p.tof_sigma, expected_solid)
        self._normalize()

    def _map_near_hit(self, x: float, y: float, yaw: float, measured: float, fan, sigma: float) -> bool:
        """평균 자리에서 잰 거리만큼 앞(원뿔 폭 전체) 근처에 지도의 벽·가구가 있는지.

        있으면 '지도보다 가까운' 측정이라도 지도에 없는 물체가 아니라 위치가 틀어진 채 아는 것을 잰 것일 수 있다.
        이것까지 버리면 위치가 0.5 m 틀어진 채 벽 앞에서 멈추기만 되풀이한다 (3단계 seed 2).
        """
        r = min(0.25 + 2 * (sigma if math.isfinite(sigma) else 0.0), 0.8)
        k = int(math.ceil(r / self.res))
        yaw = yaw + self.yaw_lead
        ox, oy = x + self.p.tof_offset * math.cos(yaw), y + self.p.tof_offset * math.sin(yaw)
        for a in fan:
            i = int(math.floor((ox + measured * math.cos(yaw + a) - self.origin[0]) / self.res))
            j = int(math.floor((oy + measured * math.sin(yaw + a) - self.origin[1]) / self.res))
            win = self.occ[max(i - k, 0):max(i + k + 1, 0), max(j - k, 0):max(j + k + 1, 0)]
            if win.size and win.any():
                return True
        return False

    def expected_ranges(self, fan: np.ndarray, grid: np.ndarray | None = None) -> np.ndarray:
        """후보마다 지도대로라면 원뿔 안 가장 가까운 거리 (없으면 inf). grid를 주면 그 격자로 잰다."""
        yaw = self.map_yaw() + self.yaw_lead + self.b
        ox = self.x + self.p.tof_offset * np.cos(yaw)
        oy = self.y + self.p.tof_offset * np.sin(yaw)
        ts = np.arange(0.04, self.p.tof_max + 0.06, 0.04)
        ang = yaw[:, None] + np.asarray(fan)[None, :]                         # (N, R)
        px = ox[:, None, None] + np.cos(ang)[:, :, None] * ts[None, None, :]  # (N, R, S)
        py = oy[:, None, None] + np.sin(ang)[:, :, None] * ts[None, None, :]
        hit = ~self.free_raw(px, py, grid)
        first = np.where(hit.any(axis=2), ts[np.argmax(hit, axis=2)], np.inf)  # (N, R)
        return first.min(axis=1)

    # ------------------------------------------------------------ 결과

    def estimate(self) -> tuple[float, float, float, float, float]:
        """(x, y, z, yaw, 위치 불확실성 σ[m])."""
        w = self.w
        x, y = float(np.sum(w * self.x)), float(np.sum(w * self.y))
        b = float(math.atan2(np.sum(w * np.sin(self.b)), np.sum(w * np.cos(self.b))))
        if len(self.x) > 1:
            dx, dy = self.x - x, self.y - y
            cov = np.array([[np.sum(w * dx * dx), np.sum(w * dx * dy)], [np.sum(w * dx * dy), np.sum(w * dy * dy)]])
            sigma = float(math.sqrt(max(np.linalg.eigvalsh(cov).max(), 0.0)))
        else:
            sigma = float("nan")
        return x, y, self.z, wrap(self.map_yaw() + b), sigma

    # ------------------------------------------------------------ 내부

    def free_at(self, x, y) -> np.ndarray:
        """기체 반폭만큼 부풀린 지도에서 비어 있으면 1, 아니면 0 (지도 밖도 0)."""
        i = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        j = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < self.free.shape[0]) & (j < self.free.shape[1])
        out = np.zeros(np.shape(x), dtype=float)
        out[ok] = self.free[i[ok], j[ok]]
        return out

    def free_raw(self, x, y, grid: np.ndarray | None = None) -> np.ndarray:
        """부풀리지 않은 지도(또는 grid)에서 비어 있으면 True (지도 밖은 막힘)."""
        grid = self.occ if grid is None else grid
        i = np.floor((x - self.origin[0]) / self.res).astype(int)
        j = np.floor((y - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < grid.shape[0]) & (j < grid.shape[1])
        out = np.zeros(x.shape, dtype=bool)
        out[ok] = ~grid[i[ok], j[ok]]
        return out

    def _normalize(self) -> None:
        total = float(self.w.sum())
        n = len(self.w)
        if not math.isfinite(total) or total <= 1e-300:
            # 모든 후보가 지도와 안 맞음: 지금 분포를 그대로 두고 점수만 되살린다
            self.w = np.full(n, 1.0 / n)
            self.lost_resets += 1
            return
        self.w = self.w / total
        if 1.0 / float(np.sum(self.w ** 2)) < n / 2:
            self._resample()

    def _resample(self) -> None:
        n = len(self.w)
        positions = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), positions), n - 1)
        self.x, self.y, self.b, self.s = self.x[idx], self.y[idx], self.b[idx], self.s[idx]
        # 같은 후보가 겹치지 않게 조금 흔든다
        self.x = self.x + self.rng.normal(0, 0.01, n)
        self.y = self.y + self.rng.normal(0, 0.01, n)
        self.b = self.b + self.rng.normal(0, math.radians(0.2), n)
        self.s = self.s + self.rng.normal(0, 0.003, n)
        self.w = np.full(n, 1.0 / n)


def tof_likelihood(measured: float, expected: np.ndarray, max_range: float, sigma: float,
                   expected_solid: np.ndarray | None = None) -> np.ndarray:
    """측정 ToF가 후보 자리의 기대 거리와 얼마나 맞는지 (0–1).

    expected_solid: 벽만 있다고 볼 때의 기대 거리. 주면 가구 상자보다 먼 측정이 벽 너머가 아니면
    (가구 빈틈으로 본 것일 수 있어) 약하게만 감점한다.
    """
    seen = math.isfinite(measured) and measured <= max_range
    exp_seen = expected <= max_range
    out = np.ones_like(expected, dtype=float)
    if seen:
        diff = measured - np.where(exp_seen, expected, max_range + 1.0)
        gauss = np.exp(-0.5 * (diff / sigma) ** 2)
        closer = diff < -3 * sigma          # 지도보다 확실히 가까움: 지도에 없는 장애물이거나 다른 후보가 맞다
        farther = diff > 3 * sigma          # 지도보다 확실히 멂: 벽을 뚫고 본 셈
        far_score = np.full_like(out, 0.05)
        if expected_solid is not None:      # 벽보다는 가까움: 가구 빈틈으로 본 셈
            far_score = np.where(measured <= expected_solid + 3 * sigma, 0.5, 0.05)
        out = np.where(closer, 0.3, np.where(farther, far_score, np.maximum(gauss, 0.05)))
    else:
        # 아무것도 안 보이는데 지도상으로는 확실히 보여야 할 벽이 있으면 감점 (가구뿐이면 약하게)
        out = np.where(expected < max_range - 0.15, 0.05, 1.0)
        if expected_solid is not None:
            out = np.where((expected < max_range - 0.15) & (expected_solid >= max_range - 0.15), 0.5, out)
    return out


def occupancy_slice(known_map: dict, height: float, res: float = 0.05):
    """받은 지도에서 비행 높이 단면의 2D 점유 격자 (occ, origin, res)."""
    vmap = VoxelMap.from_known_map(known_map, res)
    k = min(max(int(math.floor((height - vmap.origin[2]) / res)), 0), vmap.occ.shape[2] - 1)
    return vmap.occ[:, :, k].copy(), (float(vmap.origin[0]), float(vmap.origin[1])), res


def _dilate(occ: np.ndarray, r: int) -> np.ndarray:
    out = occ.copy()
    nx, ny = occ.shape
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy > r * r or (dx, dy) == (0, 0):
                continue
            xs = slice(max(dx, 0), nx + min(dx, 0)), slice(max(-dx, 0), nx + min(-dx, 0))
            ys = slice(max(dy, 0), ny + min(dy, 0)), slice(max(-dy, 0), ny + min(-dy, 0))
            out[xs[0], ys[0]] |= occ[xs[1], ys[1]]
    return out
