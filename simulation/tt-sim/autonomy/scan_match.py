"""카메라 지도로 위치 바로잡기 (4단계). ROS와 무관.

지도 없이 날 때 위치는 Tello 속도 적분뿐이라, 속도 배율이 몇 % 틀리면 3–5분 탐색에서 0.5 m 넘게 틀어졌다
(seed 1·2·4). 그러면 지도도 그만큼 어긋나게 그려지고, 돌아오는 길에 문틀에 닿거나 출발점에서 0.4 m 떨어져 내렸다.

새 깊이 영상에서 막힌 점(1–4 m, 높이 0.3–1.8 m)을 뽑아, 예전에 만든 평면 막힘 지도에 대어 본다. 위치를
±search_xy, 방향을 ±search_yaw 안에서 바꿔 가며 점이 지도의 막힌 칸에 얼마나 붙는지(거리 변환으로 점수)를
재고, 가장 잘 붙는 보정을 고른다.
- 예전 지도에 이미 있던 곳(점의 overlap 넘게가 막힌 칸 0.5 m 안)을 볼 때만 맞춘다. 새로 보는 곳은 맞출 것이 없다.
- 점수는 예전 지도에서 '본 칸'에 떨어진 점만으로 낸다. 못 본 칸에 떨어진 점까지 세면, 벽의 못 본 구간에 있던
  점들이 본 구간으로 옮겨 가는 쪽이 점수가 높아 복도를 따라 미끄러졌다.
- 최고 점수에 가까운 후보들의 평균을 보정으로 쓰되, 그 후보들이 길게 퍼진 방향(복도를 따라가는 쪽처럼 점수가
  판판해 알 수 없는 방향)으로는 고치지 않는다.
- 0에서 맞춘 점수보다 뚜렷이 나을 때만 보정을 낸다.
"""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass, field, replace

import numpy as np
from scipy import ndimage


@dataclass
class ScanMatchParams:
    search_xy: float = 0.3        # 위치 보정을 찾는 범위 (m). 가장 좋은 보정이 이 끝에 걸리면 믿지 않는다
    step_xy: float = 0.05
    search_yaw: float = math.radians(5.0)   # Tello yaw는 분당 약 1°씩 틀어진다 (5분 탐색이면 5°)
    step_yaw: float = math.radians(1.0)
    yaw_contrast: float = 0.004   # 방향별 최고 점수가 2° 옆보다 이만큼 높아야 방향을 고친다 (봉우리가 뚜렷할 때만)
    sigma: float = 0.1            # 점과 막힌 칸 사이 거리 점수 폭 (m)
    near: float = 0.5             # 점이 예전 지도와 겹친다고 볼 거리 (m)
    min_points: int = 80          # 겹치는 점이 이보다 적으면 맞추지 않는다
    overlap: float = 0.3          # 겹치는 점 비율이 이보다 적으면 맞추지 않는다
    min_gain: float = 0.02        # 0에서보다 점수가 이만큼 나아야 보정한다
    band: float = 0.05            # 최고 점수에서 이만큼 안의 후보를 '그럴듯한 보정'으로 본다
    max_spread: float = 0.08      # 그럴듯한 보정이 어느 방향으로 이보다 넓게(표준편차, m) 퍼지면 그 방향은 고치지 않는다
    max_points: int = 250         # 계산량을 줄이려고 겹치는 점을 이만큼까지 고르게 솎는다 (250개에 약 60 ms)
    scales: tuple = (1.0,)        # 깊이 배율도 함께 찾는다 (드론 둘레로 점을 늘이고 줄여 가장 잘 맞는 배율의 점수).
    #                               바닥으로 맞춘 뒤에도 장면 배율이 3–4% 흔들려, 2–3 m 벽이 7–10 cm 밀린 것을 위치로 고쳤다


class ScanMatcher:
    def __init__(self, params: ScanMatchParams | None = None):
        self.p = params or ScanMatchParams()
        p = self.p
        n = int(round(p.search_xy / p.step_xy))
        m = int(round(p.search_yaw / p.step_yaw))
        d = np.arange(-n, n + 1) * p.step_xy
        a = np.arange(-m, m + 1) * p.step_yaw
        dx, dy, da = np.meshgrid(d, d, a, indexing="ij")
        self.hyp = np.stack([dx.ravel(), dy.ravel(), da.ravel()], axis=1)    # (H, 3)
        self.grid = dx.shape                                                 # (x, y, 방향) 후보 수
        self.ref = None
        self.last: dict = {}

    def set_reference(self, occ2d: np.ndarray, origin, res: float, known2d: np.ndarray | None = None) -> None:
        """예전 평면 막힘 지도 (True = 막힘)와 본 칸 (None이면 전부 본 것으로). 거리 변환을 미리 구해 둔다."""
        self.origin, self.res = np.asarray(origin[:2], dtype=float), float(res)
        self.dist = ndimage.distance_transform_edt(~occ2d) * res if occ2d.any() else None
        if known2d is None:
            self.known = np.ones_like(occ2d)
        else:
            # 벽 바로 뒤는 늘 '못 본 칸'이다. 벽에서 조금 밀려난 점도 거리만큼 점수에 들어가도록 막힌 칸 둘레는 본 칸으로 친다
            r = int(round(self.p.search_xy / res))
            self.known = known2d | ndimage.binary_dilation(occ2d, iterations=r)
        self.ref = occ2d

    def match(self, points: np.ndarray, center) -> tuple[float, float, float, dict] | None:
        """points: (N, 2) 지금 위치로 지도 좌표에 놓은 막힌 점. center: 드론 위치 (x, y) — 방향 보정의 회전 중심.
        보정 (dx, dy, dyaw)와 정보, 또는 None (맞출 수 없음)."""
        if self.ref is None or self.dist is None or len(points) == 0:
            return None
        p = self.p
        d0 = self._lookup(points)
        near = d0 < p.near
        info = self.last = {"points": int(len(points)), "overlap": int(near.sum())}   # 마지막 시도 (기록용)
        if near.sum() < p.min_points or near.mean() < p.overlap:
            info["why"] = "겹침 부족"
            return None
        pts = points[near] - np.asarray(center, dtype=float)
        if len(pts) > p.max_points:
            pts = pts[np.linspace(0, len(pts) - 1, p.max_points).astype(int)]
        zero = int(np.argmin(np.abs(self.hyp).sum(axis=1)))
        score = score0 = self._scores(pts, center, zero)
        for k in p.scales:
            if k != 1.0:
                score = np.maximum(score, self._scores(pts * k, center, zero))
        info.update(score0=round(float(score0[zero]), 3), best=round(float(score.max()), 3))
        # 방향: 방향마다 가장 좋은 위치 점수를 모아, 봉우리가 뚜렷하면 포물선으로 다듬어 고른다. 1° 차이는 점을
        # 2–7 cm만 옮겨 위치 후보 전체에서 한꺼번에 고르면 묻혔다
        S = score.reshape(self.grid)
        prof = S.max(axis=(0, 1))
        na, ia, i0 = len(prof), int(np.argmax(prof)), len(prof) // 2
        if ia in (0, na - 1):
            info["why"] = f"방향이 범위 끝 ({math.degrees(self.hyp[ia, 2]):+.0f}°)"
            return None
        dyaw = 0.0
        contrast = prof[ia] - max(prof[max(ia - 2, 0)], prof[min(ia + 2, na - 1)])
        info["yaw_known"] = bool(contrast >= p.yaw_contrast)
        if info["yaw_known"]:
            y0, y1, y2 = prof[ia - 1], prof[ia], prof[ia + 1]
            curv = y0 - 2 * y1 + y2
            dyaw = (ia - i0 + (0.5 * (y0 - y2) / curv if curv < 0 else 0.0)) * p.step_yaw
        else:
            ia = i0                     # 방향을 알 수 없으면 지금 방향에서 위치만 고른다
        info["yaw_contrast"] = round(float(contrast), 4)
        # 위치: 그 방향에서 고른다
        Sxy = S[:, :, ia]
        best = float(Sxy.max())
        if best - score0[zero] < p.min_gain:
            info["why"] = "이미 맞음"
            return None
        # 가장 좋은 보정이 찾는 범위 끝에 걸리면 믿지 않는다. 벽 일부만 본 지도에서는 가까운(점이 빽빽한) 쪽을
        # 본 구간에 겹치는 쪽으로 끝까지 끌려간 것이었다. 위치는 천천히 틀어지므로 진짜 보정은 범위 안쪽에 있다
        bi, bj = np.unravel_index(int(np.argmax(Sxy)), Sxy.shape)
        if bi in (0, Sxy.shape[0] - 1) or bj in (0, Sxy.shape[1] - 1):
            info["why"] = "위치가 범위 끝"
            return None
        # 그럴듯한 보정들의 평균. 그것들이 길게 퍼진 방향(복도를 따라가는 쪽처럼 알 수 없는 방향)은 빼고 고친다
        xy = self.hyp.reshape(*self.grid, 3)[:, :, ia, :2]
        good = xy[Sxy >= best - p.band]
        dxy = good.mean(axis=0)
        info["weak"] = []               # 알 수 없는 방향 (단위 벡터). 칼만 거르개는 이 방향 측정을 믿지 않는다
        if len(good) > 2:
            vals, vecs = np.linalg.eigh(np.cov(good.T))
            for val, vec in zip(vals, vecs.T):
                if math.sqrt(max(val, 0.0)) > p.max_spread:
                    dxy -= float(dxy @ vec) * vec
                    info["weak"].append(vec.copy())
        info["candidates"] = int(len(good))
        return float(dxy[0]), float(dxy[1]), float(dyaw), info

    def _scores(self, pts: np.ndarray, center, zero: int) -> np.ndarray:
        """후보마다 점수 (H,). pts: 드론 기준 점 (N, 2)."""
        p = self.p
        c, s = np.cos(self.hyp[:, 2]), np.sin(self.hyp[:, 2])
        # (H, N): 회전 후 이동한 점
        xs = c[:, None] * pts[None, :, 0] - s[:, None] * pts[None, :, 1] + self.hyp[:, 0:1] + center[0]
        ys = s[:, None] * pts[None, :, 0] + c[:, None] * pts[None, :, 1] + self.hyp[:, 1:2] + center[1]
        dist, known = self._lookup(np.stack([xs, ys], axis=-1).reshape(-1, 2), with_known=True)
        dist, known = dist.reshape(xs.shape), known.reshape(xs.shape)
        n_known = known.sum(axis=1)
        score = np.where(known, np.exp(-0.5 * (dist / p.sigma) ** 2), 0.0).sum(axis=1) / np.maximum(n_known, 1)
        # 보정 없을 때보다 본 칸에 떨어지는 점이 절반도 안 되는 후보는 믿지 않는다 (몇 점만 남아 우연히 잘 맞는 것)
        score[n_known < 0.5 * max(n_known[zero], 1)] = 0.0
        return score

    def _lookup(self, pts: np.ndarray, with_known: bool = False):
        """막힌 칸까지 거리. 칸 가운데 값을 이어 보간한다 (칸 단위로만 재면 칸 안 어디든 0이라 점수가 1에 붙은
        넓은 평지가 생겨, 방향이 엉뚱한 쪽으로 골라졌다)."""
        g = (pts - self.origin) / self.res
        idx = np.floor(g).astype(int)
        nx, ny = self.dist.shape
        ok = (idx[:, 0] >= 0) & (idx[:, 0] < nx) & (idx[:, 1] >= 0) & (idx[:, 1] < ny)
        out = np.full(len(pts), np.inf)
        out[ok] = ndimage.map_coordinates(self.dist, (g[ok] - 0.5).T, order=1, mode="nearest")
        if not with_known:
            return out
        known = np.zeros(len(pts), dtype=bool)
        known[ok] = self.known[idx[ok, 0], idx[ok, 1]]
        return out, known


def depth_points(depth: np.ndarray, cam, yaw: float, hfov: float, stride: int = 4,
                 rng: tuple[float, float] = (1.0, 4.0), band: tuple[float, float] = (0.3, 1.8),
                 edge_ratio: float = 0.0, edge_window: int = 3) -> np.ndarray:
    """깊이 영상에서 맞추기에 쓸 막힌 점 (N, 2): 거리 rng 안, 높이 band 안. 가까운 곳은 짧게 추정해 빼고,
    먼 곳은 오차가 커서 뺀다. edge_ratio > 0이면 지도와 같은 식으로 깊이 경계의 중간 화소(헛 점)를 뺀다."""
    h, w = depth.shape
    vs, us = np.mgrid[stride // 2:h:stride, stride // 2:w:stride]
    d = depth[vs, us].astype(np.float64)
    f = (w / 2) / math.tan(hfov / 2)
    left, up = -(us - w / 2) / f, -(vs - h / 2) / f
    z = cam[2] + d * up
    ok = np.isfinite(d) & (d >= rng[0]) & (d <= rng[1]) & (z >= band[0]) & (z <= band[1])
    if edge_ratio > 0:
        clean = np.where(np.isfinite(depth) & (depth > 0), depth, 1e3)
        near = ndimage.minimum_filter(clean, size=edge_window, mode="nearest")[vs, us]
        far = ndimage.maximum_filter(clean, size=edge_window, mode="nearest")[vs, us]
        ok &= (far <= edge_ratio * near) | (d <= 1.05 * near)
    c, s = math.cos(yaw), math.sin(yaw)
    x = cam[0] + d * (c - s * left)
    y = cam[1] + d * (s + c * left)
    return np.stack([x[ok], y[ok]], axis=1)


@dataclass
class Correction:
    """추정 위치 → 지도 좌표 변환. 지도 좌표 = 기준점 + scale·R(yaw)·(추정 - 기준점) + (tx, ty), 방향 = 추정 방향 + yaw.
    기준점은 출발점이다. Tello 속도 적분 위치의 오차는 대부분 속도 배율(scale)과 쌓인 방향 오차(yaw)로, 출발점을
    중심으로 늘이고 돌린 꼴이다 (기록한 비행 6개에서 이 꼴로 맞추면 남는 오차가 평균 0.20–0.34 → 0.07–0.14 m).
    비행 노드에서는 맞추기 스레드가 고치고 조종 쪽이 읽으므로 잠금으로 묶는다."""
    tx: float = 0.0
    ty: float = 0.0
    yaw: float = 0.0
    scale: float = 1.0
    px: float = 0.0
    py: float = 0.0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def apply(self, pose):
        x, y, z, yaw = pose
        with self._lock:
            tx, ty, a, k, px, py = self.tx, self.ty, self.yaw, self.scale, self.px, self.py
        c, s = math.cos(a), math.sin(a)
        rx, ry = x - px, y - py
        return (px + k * (c * rx - s * ry) + tx, py + k * (s * rx + c * ry) + ty, z, yaw + a)

    def nudge(self, dx: float, dy: float, dyaw: float, center) -> None:
        """지도 좌표에서 center를 중심으로 dyaw 돌리고 (dx, dy) 옮기는 보정을 덧붙인다."""
        cx, cy = center
        c, s = math.cos(dyaw), math.sin(dyaw)
        with self._lock:
            tx, ty = self.px + self.tx - cx, self.py + self.ty - cy
            self.tx = c * tx - s * ty + cx + dx - self.px
            self.ty = s * tx + c * ty + cy + dy - self.py
            self.yaw += dyaw

    def add(self, dscale: float, dyaw: float, dtx: float, dty: float) -> None:
        with self._lock:
            self.scale += dscale
            self.yaw += dyaw
            self.tx += dtx
            self.ty += dty

    def set_pivot(self, px: float, py: float) -> None:
        """기준점을 옮긴다 (변환 결과는 그대로)."""
        with self._lock:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            ox, oy = self.px - px, self.py - py      # 옛 기준점을 새 기준점 기준으로
            self.tx += ox - self.scale * (c * ox - s * oy)
            self.ty += oy - self.scale * (s * ox + c * oy)
            self.px, self.py = px, py


@dataclass
class DriftParams:
    """속도 배율·방향·위치 보정 칼만 거르개 값 (LocalizerParams.model = 'drift')."""
    sigma_scale: float = 0.08          # 처음 속도 배율 불확실성 (Tello 속도 배율 오차를 ±8%까지 본다)
    sigma_yaw: float = math.radians(2.0)
    sigma_xy: float = 0.05             # 처음 위치 불확실성 (출발점은 안다)
    q_scale: float = 1e-7              # 초당 늘어나는 불확실성 (분산). 배율은 거의 그대로
    q_yaw: float = math.radians(1.0) ** 2 / 60   # 방향은 분당 약 1°씩 틀어진다
    q_xy: float = 0.05 ** 2 / 60
    meas_xy: float = 0.2               # 맞추기 한 번의 위치 잡음 (m). 정답 지도에 대어도 약 0.13 m 흔들렸다
    meas_yaw: float = math.radians(2.0)
    weak: float = 100.0                # 맞추기가 알 수 없다고 한 방향·방향각의 잡음 분산 (사실상 안 믿음)
    gate: float = 11.3                 # 마할라노비스 거리² 문턱 (자유도 3, 99%). 넘으면 엉뚱한 맞춤으로 버린다


@dataclass
class LocalizerParams:
    """기본값은 기록한 비행을 tools/replay_map.py로 다시 돌려 고른 것이다. 맞추기 한 번은 정답 지도에 대어도 약
    0.13 m 흔들리므로, 한 번에 다 고치지 않고 여러 번의 평균으로 고친다.
    model: 'offset' = 찾은 보정의 gain만큼 위치·방향을 옮긴다. 'drift' = 속도 배율·방향·위치를 칼만 거르개로 함께
    맞춘다 (속도 배율 오차가 있으면 고쳐야 할 양이 출발점에서 멀어질수록 커져, 옮기기만 해서는 따라가지 못했다)."""
    ref_age: float = 20.0         # 이만큼 지난 지도에 대어 본다 (s). 방금 그린 지도는 지금의 위치 오차를 그대로 담고 있다
    snapshot: float = 5.0         # 그 지도를 떠 두는 간격 (s)
    period: float = 0.5           # 맞추기 간격 (s). 한 번에 약 0.2 s (배율 3개 × 점 250개), 비행 노드는 따로 도는 스레드에서
    model: str = "offset"
    gain: float = 0.15            # offset: 찾은 보정의 이만큼만 한 번에 반영한다 (0.5 → 위치 오차 평균 0.27 m, 0.15 → 0.18 m)
    max_step: float = 0.1         # offset: 한 번에 옮기는 최대 거리 (m)
    deadband: float = 0.0         # offset: 이보다 작은 위치 보정은 버린다 (m). 0.05·0.1 m는 나아지지 않았다
    yaw_deadband: float = 0.0     # offset: 방향도 같은 식으로 (rad)
    band: tuple = (0.3, 1.8)      # 맞추기에 쓰는 높이 (바닥·천장 근처 제외)
    rng: tuple = (1.0, 4.0)       # 맞추기에 쓰는 깊이 거리 (m). 1–2.5 m로 줄이면 나빠졌다
    edge_ratio: float = 1.3       # 맞추기 점에서도 깊이 경계의 중간 화소(헛 점)를 뺀다 (지도와 같은 기준, 0이면 안 뺌)
    # 깊이 배율도 함께 찾는다: 빼면 seed 4에서 보정 전(0.27 m)보다 나빠졌다(0.31 m). 점을 120개로 줄여도 나빠졌다
    match: ScanMatchParams = field(default_factory=lambda: ScanMatchParams(scales=(0.96, 1.0, 1.04)))
    drift: DriftParams = field(default_factory=DriftParams)
    # 출발점 맞추기 (home_mode, 비행 노드가 출발점 위에서 한 바퀴 돌 때 켠다)
    home_ref_after: float = 30.0  # 이륙 뒤 이만큼 지난 지도(출발점 둘레를 둘러본 뒤)를 기준으로 떠 둔다 (s). 위치 오차가 거의 없을 때다
    home_search: float = 0.6      # 그때 찾는 범위 (m). 보통 범위(0.3 m)를 넘게 틀어진 채 돌아와, 범위 끝에 걸린 맞춤을 다 버렸다 (seed 4: 0.41 m)
    home_gain: float = 0.5        # 그때 반영하는 몫. 제자리에서 여러 방향을 고루 보므로 보통 때보다 크게


class MapLocalizer:
    """깊이 영상을 ref_age 넘게 지난 카메라 지도에 대어 '추정 → 지도 좌표' 보정(correction)을 조금씩 고친다.
    비행 노드(nodes/mission_node.py)와 재생 도구(tools/replay_map.py)가 같이 쓴다."""

    def __init__(self, params: LocalizerParams | None = None):
        self.p = params or LocalizerParams()
        self.correction = Correction()
        self.matcher = ScanMatcher(self.p.match)
        self.snapshots: deque = deque()   # (시각, 평면 막힘, 본 칸)
        self.ref_time = None
        self.match_at = -math.inf
        self.tries = 0
        self.fixes = 0
        self.rejected = 0
        self.moved = 0.0
        d = self.p.drift
        self.P = np.diag([d.sigma_scale ** 2, d.sigma_yaw ** 2, d.sigma_xy ** 2, d.sigma_xy ** 2])   # (배율, 방향, x, y)
        self.filter_t = None
        self.pivot_set = False
        # 출발점 맞추기: 넓게 찾는 맞추개와 이륙 직후 지도
        self.home_mode = False
        step = self.p.home_search / 6   # 계산량은 보통 맞추기와 같게 (후보 13×13). 칸이 성기니 퍼짐 기준도 넓힌다
        self.home_matcher = ScanMatcher(replace(self.p.match, search_xy=self.p.home_search, step_xy=step,
                                                max_spread=max(self.p.match.max_spread, 1.5 * step)))
        self.home_ref = None
        self.home_ref_set = False
        self.t_first = None
        self.home_fixes = 0

    def update(self, t: float, cmap, depth: np.ndarray, cam, pose, hfov: float, raw=None) -> bool:
        """t: 영상 시각, cmap: CameraMap, cam: 카메라 위치(지도 좌표), pose: 보정한 위치 (x, y, z, yaw),
        raw: 보정 전 위치 ('drift'에 필요). 고쳤으면 True."""
        p = self.p
        if raw is not None and not self.pivot_set:   # 첫 영상 자리(출발점)를 늘이고 돌리는 기준점으로
            self.correction.set_pivot(raw[0], raw[1])
            self.pivot_set = True
        if self.t_first is None:
            self.t_first = t
        r = cmap.p.res
        k0, k1 = (int(h / r) for h in p.band)
        if not self.snapshots or t - self.snapshots[-1][0] >= p.snapshot:
            self.snapshots.append((t, cmap.occupied()[:, :, k0:k1].any(axis=2), cmap.seen_band()[:, :, k0:k1].any(axis=2)))
            if self.home_ref is None and t - self.t_first >= p.home_ref_after:
                self.home_ref = self.snapshots[-1]
        while len(self.snapshots) >= 2 and t - self.snapshots[1][0] >= p.ref_age:
            self.snapshots.popleft()
        if t - self.match_at < p.period:
            return False
        if self.home_mode and self.home_ref is not None:
            if not self.home_ref_set:
                self.home_matcher.set_reference(self.home_ref[1], cmap.origin, r, self.home_ref[2])
                self.home_ref_set = True
            matcher, gain, max_step = self.home_matcher, p.home_gain, 2 * p.max_step
        else:
            ref = self.snapshots[0]
            if t - ref[0] < p.ref_age:
                return False
            if ref[0] != self.ref_time:
                self.matcher.set_reference(ref[1], cmap.origin, r, ref[2])
                self.ref_time = ref[0]
            matcher, gain, max_step = self.matcher, p.gain, p.max_step
        self.match_at = t
        x, y, _, yaw = pose
        self.tries += 1
        found = matcher.match(depth_points(depth, cam, yaw, hfov, rng=p.rng, band=p.band, edge_ratio=p.edge_ratio),
                              (x, y))
        if p.model == "drift" and matcher is self.matcher:
            return self._drift_update(t, found, raw if raw is not None else pose)
        if found is None:
            return False
        dx, dy, dyaw, _ = found
        if math.hypot(dx, dy) < p.deadband:
            dx = dy = 0.0
        if abs(dyaw) < p.yaw_deadband:
            dyaw = 0.0
        if dx == dy == dyaw == 0.0:
            return False
        step = math.hypot(dx, dy) * gain
        k = gain * min(1.0, max_step / step) if step > 0 else gain
        self.correction.nudge(k * dx, k * dy, gain * dyaw, (x, y))
        self.fixes += 1
        self.home_fixes += matcher is self.home_matcher
        self.moved += k * math.hypot(dx, dy)
        return True

    def _drift_update(self, t: float, found, raw) -> bool:
        """맞추기가 낸 보정 (dx, dy, dyaw)을 측정으로 (배율, 방향, tx, ty)를 칼만 거르개로 고친다."""
        d, c = self.p.drift, self.correction
        if self.filter_t is not None:
            self.P += np.diag([d.q_scale, d.q_yaw, d.q_xy, d.q_xy]) * max(t - self.filter_t, 0.0)
        self.filter_t = t
        if found is None:
            return False
        dx, dy, dyaw, info = found
        a, k = c.yaw, c.scale
        rx, ry = raw[0] - c.px, raw[1] - c.py
        gx, gy = math.cos(a) * rx - math.sin(a) * ry, math.sin(a) * rx + math.cos(a) * ry   # R(yaw)·(추정 - 기준점)
        H = np.array([[gx, -k * gy, 1.0, 0.0],      # 배율·방향·tx·ty를 조금 바꿀 때 드론 위치·방향이 바뀌는 양
                      [gy, k * gx, 0.0, 1.0],
                      [0.0, 1.0, 0.0, 0.0]])
        Rm = np.diag([d.meas_xy ** 2, d.meas_xy ** 2, d.meas_yaw ** 2])
        for v in info.get("weak", []):
            Rm[:2, :2] += d.weak * np.outer(v, v)
        if not info.get("yaw_known", True):
            Rm[2, 2] += d.weak
        z = np.array([dx, dy, dyaw])
        S = H @ self.P @ H.T + Rm
        Si = np.linalg.inv(S)
        if float(z @ Si @ z) > d.gate:
            self.rejected += 1
            return False
        K = self.P @ H.T @ Si
        delta = K @ z
        c.add(*delta)
        self.P = (np.eye(4) - K @ H) @ self.P
        self.fixes += 1
        self.moved += float(np.hypot(*(H[:2] @ delta)))
        return True
