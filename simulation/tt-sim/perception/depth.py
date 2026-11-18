"""단안 카메라 한 장으로 거리(깊이) 영상을 추정한다. ROS와 무관해서 녹화 영상·실기체 영상에도 그대로 쓴다.

Tello EDU에는 깊이 카메라가 없으므로, 카메라 영상만으로 장애물 지도를 만들려면 영상에서 거리를 추정해야 한다.
Depth Anything V2 (실내 미터 단위 모델, Small, Apache-2.0)를 PC GPU에서 돌린다. 처음 실행할 때 Hugging Face에서
모델(약 100 MB)을 내려받는다.

- DepthEstimator.estimate(rgb) → (H, W) float32, 카메라 광축 방향 거리(m)
- 이 모델은 화각 60° 카메라(Hypersim, 1024×768)로 학습해서, 화각이 다른 카메라에서는 물체 크기가 달라 보여 거리가
  틀린다. 깊이는 초점거리에 비례하므로 tan(30°)/tan(hfov/2)를 곱해 맞춘다 (Gazebo 카메라 82.6°면 0.66배 — 시뮬레이션에서
  잰 배율(정답/추정)의 중앙값도 0.67, tools/eval_depth.py). 실기체 Tello는 **수평 화각 54.7°**(팀 실측 보정 fx 927.5@960;
  11/03 비행 영상의 옆 흐름÷IMU 회전으로도 58–62°)라 공식으로는 1.12배지만, 실영상에서는 모델이 가까운 곳을 길게 읽어
  ToF와 맞는 배율이 약 0.85였다 → 실기체는 depth_scale로 직접 준다 (real.launch depth_scale, 11/03 저녁).
- 그래도 장면마다 전체 배율이 흔들린다 (시뮬레이션 150장에서 장면별 배율 0.98–1.24). 비행 높이를 알면 바닥이
  있어야 할 거리를 알므로, 영상 아래쪽 바닥 화소로 장면마다 배율을 맞춘다 (floor_scale). 장면별 배율을 맞추면
  화소 오차 중앙값이 8% → 3%로 준다. ToF처럼 정확한 한 점이 있으면 scale_to_anchor로도 맞출 수 있다.
- 거꾸로 바닥 화소로 '바닥 위 높이'를 재는 고도계는 못 된다 (11/09 실기체 가방 5개 재연: ToF가 바닥을 재는 동안 카메라 값이
  0.36–0.96 m로 흔들렸고 ToF와 0.2 m 안에 든 영상이 2–18%). 무늬 없는 카펫을 모델이 30%쯤 가깝게 읽고 장면마다 크게 흔들린다.
  밑에 가구가 있는지는 깊이로 쌓은 카메라 지도(기체 아래 기둥의 막힌 칸 꼭대기)로 본다 — nodes/mission_node.py under_top.
"""
from __future__ import annotations

import math

import numpy as np

MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
TRAIN_HFOV = math.radians(60.0)   # 학습 카메라 수평 화각 (Hypersim)


class DepthEstimator:
    def __init__(self, hfov: float | None = None, model_id: str = MODEL_ID, device: str | None = None,
                 depth_scale: float | None = None):
        """hfov: 이 카메라의 수평 화각(rad). 주면 학습 카메라와의 초점거리 차이만큼 깊이를 고친다.
        depth_scale: 주면(0보다 크면) 그 배율을 그대로 쓴다 — 실기체는 ToF로 맞춘 값(약 0.85)."""
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        self.torch = torch
        self.device = device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.half = self.device.startswith("cuda")
        self.processor = AutoImageProcessor.from_pretrained(model_id, use_fast=True)   # 가로세로 비율을 지켜 줄인다
        self.focal_scale = depth_scale if depth_scale else (math.tan(TRAIN_HFOV / 2) / math.tan(hfov / 2) if hfov else 1.0)
        model = AutoModelForDepthEstimation.from_pretrained(model_id).to(self.device).eval()
        self.model = model.half() if self.half else model

    def estimate(self, rgb: np.ndarray) -> np.ndarray:
        """rgb: (H, W, 3) uint8 → 깊이 (H, W) float32 미터."""
        torch = self.torch
        h, w = rgb.shape[:2]
        inputs = self.processor(images=rgb, return_tensors="pt")
        pixels = inputs["pixel_values"].to(self.device)
        if self.half:
            pixels = pixels.half()
        with torch.no_grad():
            pred = self.model(pixel_values=pixels).predicted_depth          # (1, h', w')
            pred = torch.nn.functional.interpolate(pred[:, None].float(), size=(h, w), mode="bilinear",
                                                   align_corners=False)[0, 0]
        return pred.cpu().numpy().astype(np.float32) * self.focal_scale


def floor_scale(depth: np.ndarray, hfov: float, cam_z: float, min_fraction: float = 0.05,
                limits: tuple[float, float] = (0.75, 1.33)) -> float | None:
    """바닥으로 맞춘 배율. depth: 광축 방향 거리 영상, cam_z: 카메라 높이(m, 바닥 기준). 카메라는 수평이라고 본다.

    수평선 아래(7° 넘게) 화소에서 바닥(높이 0)까지의 광축 거리는 cam_z / tan(내려다본 각)이다. 추정 깊이로 높이가
    바닥 근처(±0.35 m)인 화소를 바닥 후보로 잡아 (있어야 할 거리 / 추정 거리)의 중앙값을 구하고, 그 배율로 다시
    ±0.12 m 안 화소만 골라 한 번 더 구한다. 바닥 후보가 영상의 min_fraction보다 적거나 배율이 limits 밖이면 None.
    벽에 붙어 바닥이 거의 안 보일 때 벽 아래쪽을 바닥으로 잡아 1.3배로 튀었다 (seed 5).
    """
    h, w = depth.shape
    f = (w / 2) / math.tan(hfov / 2)
    up = -(np.arange(h, dtype=np.float64)[:, None] + 0.5 - h / 2) / f * np.ones((1, w))
    below = up < -0.12
    expect = np.where(below, cam_z / np.maximum(-up, 1e-3), np.inf)
    d = np.where(np.isfinite(depth) & (depth > 0), depth, np.inf)
    scale = 1.0
    for tol in (0.35, 0.12):
        floor = below & (np.abs(cam_z + scale * d * up) < tol) & (scale * d < 4.5)
        if floor.sum() < min_fraction * depth.size:
            return None
        scale = float(np.median(expect[floor] / d[floor]))
    return scale if limits[0] <= scale <= limits[1] else None


def scale_to_anchor(depth: np.ndarray, anchor_m: float, region: tuple[slice, slice]) -> float:
    """region(ToF 원뿔이 닿는 영상 부분)에서 가장 가까운 추정 거리가 anchor_m(ToF 측정)이 되게 하는 배율.

    ToF는 원뿔 안에서 가장 가까운 거리 하나를 주므로, 추정 영상에서도 그 영역의 아래쪽 5% 값(가장 가까운 쪽)과
    맞춘다. 맞출 수 없으면 1.0.
    """
    patch = depth[region]
    patch = patch[np.isfinite(patch) & (patch > 0.05)]
    if patch.size < 10 or not np.isfinite(anchor_m) or anchor_m <= 0:
        return 1.0
    near = float(np.percentile(patch, 5))
    return anchor_m / near if near > 0 else 1.0


class TofDepthScale:
    """정면 ToF로 맞추는 깊이 배율 (실기체, 11/04).

    11/04 12:57 실기체: 바닥으로 맞춘 배율(floor_scale)은 비행 중 590장 중 41장만 맞았고 그때마다 1.30–1.31(상한 1.33 근처)이
    나와 그대로 남아, 지도에 들어간 깊이가 ToF보다 30% 멀었다(중앙값 1.30배). 바닥 배율을 빼면 0.99배였다. 초록 카펫·비행 높이
    1 m·세로 화각 ±21°라 바닥이 화면 아래 조금만 보이는 탓으로 본다.
    대신 ToF가 1.2 m 안에서 면을 잴 때마다 같은 방향 깊이(원뿔 안 열의 가장 가까운 거리, 배율 곱하기 전)와의 비(ToF ÷ 깊이)를
    모아, 최근 window초의 중앙값을 limits 안으로 쓴다. 비가 accept 밖이면(유리를 꿰뚫어 봄, 다른 물체) 버린다. 표본이
    min_samples보다 적으면 지난 값을 그대로 쓴다(처음 1.0 — 깊이 노드의 depth_scale 0.85만).
    ToF는 초당 10번이라 한 면 앞에 2초만 있어도 20개가 쌓여 그 면 하나가 배율을 정하므로, 표본은 min_gap초에 하나만 받는다.

    **천천히, 1.0 가까이 (11/11)**: 실기체 6개 비행(11/05–11/11) 가방을 다시 돌려 보니 표본 하나하나가 ±22%씩 흩어지고(사분위 0.8–1.25),
    비가 거리에 따라서도 달랐다(0.5 m 안 0.7–0.8, 0.8–1.2 m 1.1–1.25 — ToF 원뿔과 깊이 열의 높이 띠가 보는 것이 다르다). 최근 60초
    중앙값(5개부터, 0.7–1.4)은 한 비행 안에서 0.70 ↔ 1.29로 튀었고, 정작 지도에 벽·문을 처음 넣는 1.2–3.5 m 거리에서는(ToF가 잰 점을
    그보다 앞선 깊이 장면에서 되짚어 잰 '실제 ÷ 깊이' 98개) 배율을 1.0으로 둔 것보다 나빴다: 오차 평균 24.5% 대 21.7%. 배율이 튀면
    같은 벽이 다른 자리에 또 찍혀 벽이 두꺼워지고 복도·문이 좁아진다. 그래서 창을 300초로 늘리고, 표본이 적을수록 1.0 쪽으로 당기며
    (ln 배율 × n/(n + shrink)), 범위를 0.8–1.25로 좁혔다 — 같은 가방에서 배율은 0.88–1.17 안에서 천천히 움직이고 먼 거리 오차는
    평균 20.9%·중앙 12.9%(전 16.7%). 11/06처럼 깊이가 한결같이 13% 멀던 비행은 4.9%까지 맞는다(전 9.1%, 1.0 고정 14.9%)."""

    def __init__(self, window: float = 300.0, min_samples: int = 5, limits: tuple[float, float] = (0.8, 1.25),
                 accept: tuple[float, float] = (0.6, 1.6), min_gap: float = 0.5, shrink: float = 10.0):
        self.window, self.min_samples, self.limits, self.accept = window, min_samples, limits, accept
        self.min_gap, self.shrink = min_gap, shrink
        self.samples: list[tuple[float, float]] = []   # (시각, ToF ÷ 깊이)
        self.scale = 1.0
        self.used = 0                                  # 받아들인 표본 수 (비행 전체)

    def add(self, tof_m: float, depth_m: float, t: float) -> bool:
        """같은 순간·같은 방향의 ToF 거리와 깊이(배율 곱하기 전). 받아들였으면 True."""
        if not (math.isfinite(tof_m) and math.isfinite(depth_m)) or tof_m <= 0.1 or depth_m <= 0.1:
            return False
        if self.samples and t - self.samples[-1][0] < self.min_gap:
            return False
        ratio = tof_m / depth_m
        if not self.accept[0] <= ratio <= self.accept[1]:
            return False
        self.samples.append((t, ratio))
        self.used += 1
        return True

    def value(self, t: float) -> float:
        self.samples = [(s, r) for s, r in self.samples if t - s <= self.window]
        n = len(self.samples)
        if n >= self.min_samples:
            med = float(np.median([r for _, r in self.samples]))
            if self.shrink > 0:   # 표본이 적을수록 1.0 쪽으로 (5개면 1/3만, 30개면 3/4)
                med = math.exp(n / (n + self.shrink) * math.log(med))
            self.scale = min(max(med, self.limits[0]), self.limits[1])
        return self.scale
