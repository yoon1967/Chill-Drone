"""오래된 입력으로 움직이지 않기 (11/04 외부 리뷰: 1초 전 위치, 5초 전 전방 ToF로도 전진 명령을 만들었다). ROS와 무관.

나이는 비행 노드가 메시지를 받은 시각에서 잰다(이 노드 시계).
- 위치가 POSE_STALE 넘게 안 왔으면 모든 움직임을 0으로(제자리), POSE_LOST 넘게면 내린다.
- 전방 ToF가 TOF_STALE 넘게 안 왔으면 앞으로만 가지 않는다(옆·뒤·오르내림·회전은 그대로), TOF_LOST 넘게면 내린다.
  ToF를 한 번도 못 받았으면(ToF 없는 시뮬레이션) 보지 않는다.
- 내리는 판단은 받은 뒤 제어 주기가 LOST_STEPS번 넘게 돌았을 때만 한다: 경로 계획처럼 긴 계산 동안 쌓인 메시지를 아직 못 읽은
  것을 '안 옴'으로 보지 않게.
"""
from __future__ import annotations

POSE_STALE = 0.5   # s. 위치는 상태 패킷마다(초당 약 10번) 온다
POSE_LOST = 2.0
TOF_STALE = 1.0    # s. 드라이버는 같은 값도 0.2초마다 보내고, 응답 한 번을 놓치면(0.5초 대기) 약 0.9초 빈다
TOF_LOST = 3.0
LOST_STEPS = 10    # 제어 주기 0.05 s × 10


def guard(cmd: tuple[float, float, float, float], pose_age: float,
          tof_age: float | None) -> tuple[tuple[float, float, float, float], str | None]:
    """(vx, vy, vz, yaw_rate) 명령을 입력 나이에 맞게 줄인다. (줄인 명령, 줄였으면 이유). tof_age None: ToF를 받은 적 없음."""
    if pose_age > POSE_STALE:
        return (0.0, 0.0, 0.0, 0.0), f"위치가 {pose_age:.1f}초째 안 옵니다. 제자리에 멈춥니다."
    vx, vy, vz, wz = cmd
    if vx > 0 and tof_age is not None and tof_age > TOF_STALE:
        return (0.0, vy, vz, wz), f"전방 ToF가 {tof_age:.1f}초째 안 옵니다. 앞으로 가지 않습니다."
    return (vx, vy, vz, wz), None


def lost(pose_age: float, pose_steps: int, tof_age: float | None, tof_steps: int) -> str | None:
    """내려야 할 만큼 오래 안 왔으면 이유."""
    if pose_age > POSE_LOST and pose_steps > LOST_STEPS:
        return f"위치가 {pose_age:.1f}초째 안 옵니다"
    if tof_age is not None and tof_age > TOF_LOST and tof_steps > LOST_STEPS:
        return f"전방 ToF가 {tof_age:.1f}초째 안 옵니다"
    return None
