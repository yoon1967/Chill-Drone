"""Tello SDK 2.0 상태 문자열과 우리 좌표계 사이의 변환. 실기체 드라이버와 시뮬레이션 흉내가 같이 쓴다.

Tello(EDU)는 UDP 8890으로 초당 약 10번 다음 같은 문자열을 보낸다.
  mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;roll:0;yaw:-12;vgx:3;vgy:0;vgz:0;templ:60;temph:63;
  tof:98;h:90;bat:87;baro:120.31;time:12;agx:-3.00;agy:1.00;agz:-999.00;
단위: 각도 도(정수), 속도 dm/s(정수), tof·h cm(정수), baro m, time s, 가속도 0.001 g.
미션 패드(mid, x, y, z)는 쓰지 않는다 (마커 없이 위치 추정).

우리 좌표계: x 앞, y 왼쪽, z 위, 각도는 반시계가 +(라디안).

CONVENTION은 공개 자료로 확실하지 않아 '가정'이다. 실기체가 오면 확인해 바꾼다.
  yaw_sign  Tello yaw가 시계 방향으로 돌 때 커지면 -1
  vel_frame vgx·vgy가 기체 기준이면 "body", 이륙(전원) 때 방향에 고정된 기준이면 "takeoff"
  vx_sign   vgx가 뒤가 +면(앞으로 날 때 음수면) -1
  vy_sign   vgy가 오른쪽이 +면 -1
확인 방법: 이륙 → `cw 90` 뒤 yaw 부호 확인 → `forward 50` 동안 vgx·vgy 중 어느 쪽이 변하는지 확인.
2026-10-28 실기체(임시 기체): rc 앞 20으로 약 17초 앞으로 2 m 가는 동안 vgx가 -1~-2였다 → vx_sign -1.
rc yaw +30(시계 방향)에 Tello yaw가 0 → 173으로 커졌다 → yaw_sign -1 맞음.
2026-10-29 실기체 충돌 비행: 앞으로 날 때 (vgx, vgy)의 방향이 Tello yaw와 같이 돌았다(yaw −176°·−18°·104°에서
늘 yaw + 180°). 속도값은 기체 기준이 아니라 전원 켤 때 방향(Tello yaw 0) 기준이다 → vel_frame "takeoff", vy_sign +1
(앞으로 날 때 vgx = −속도·cos yaw, vgy = −속도·sin yaw). 10/27 기록(yaw −72°에서 앞으로 갈 때 vgy만 +2)과도 맞다.
기체 기준(body)으로 읽어 돌아선 뒤 위치 추정이 거꾸로 갔고(이동 방향 − 기수 중앙 135°), 지도가 틀어진 채 모니터에 닿았다.
CONVENTION은 실기체 값이고, 시뮬레이션 흉내(tello_emu)도 이것으로 상태값을 만든다(시뮬레이션 = 실기체).
"""
from __future__ import annotations

import math

CONVENTION = {"yaw_sign": -1.0, "vel_frame": "takeoff", "vx_sign": -1.0, "vy_sign": 1.0}

FIELDS = ("mid", "x", "y", "z", "mpry", "pitch", "roll", "yaw", "vgx", "vgy", "vgz", "templ", "temph",
          "tof", "h", "bat", "baro", "time", "agx", "agy", "agz")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def parse(text: str) -> dict[str, float | str]:
    """상태 문자열 → {이름: 값}. 숫자가 아닌 값(mpry 등)은 문자열로 둔다."""
    out: dict[str, float | str] = {}
    for item in text.strip().strip(";").split(";"):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        try:
            out[key.strip()] = float(value)
        except ValueError:
            out[key.strip()] = value.strip()
    return out


def encode(fields: dict) -> str:
    """{이름: 값} → 상태 문자열 (Tello가 보내는 순서·형식)."""
    parts = []
    for key in FIELDS:
        value = fields.get(key, 0)
        if key in ("baro", "agx", "agy", "agz"):
            parts.append(f"{key}:{float(value):.2f}")
        elif key == "mpry":
            parts.append(f"{key}:{value if isinstance(value, str) else '0,0,0'}")
        else:
            parts.append(f"{key}:{int(round(float(value)))}")
    return ";".join(parts) + ";"


def to_si(raw: dict, conv: dict = CONVENTION) -> dict:
    """상태 값 → SI 단위·우리 좌표계.

    yaw   Tello 기준 0에서 잰 각도(rad, 반시계 +). 지도 방향과의 차이는 위치 추정이 맞춘다
    vx,vy 수평 속도(m/s, x 앞·y 왼쪽). frame이 "body"면 기체 기준, "takeoff"면 Tello yaw 0 방향 기준
    vz    m/s(위 +),  h  Tello가 내는 높이(m),  tof  하방 거리(m, 못 재면 None),  bat  %
    h는 믿지 않는다: 2026-10-27 실기체에서 0.78 m에 떠 있는 동안 h가 0–10 cm였다(하방 ToF 77–80 cm).
    10/23 수동 비행에서는 바닥 기준 높이와 맞았다. 실기체 높이는 state_estimator의 height_source="tof"로 잡는다
    """
    tof = float(raw.get("tof", 0)) / 100.0
    return {
        "yaw": wrap(conv["yaw_sign"] * math.radians(float(raw.get("yaw", 0)))),
        "vx": conv.get("vx_sign", 1.0) * float(raw.get("vgx", 0)) / 10.0,
        "vy": conv["vy_sign"] * float(raw.get("vgy", 0)) / 10.0,
        "vz": float(raw.get("vgz", 0)) / 10.0,    # Tello vgz는 위가 + (2026-10-27 실기체: 이륙 중 +6·+3, 착륙 중 -4)
        "frame": conv["vel_frame"],
        "h": float(raw.get("h", 0)) / 100.0,
        "tof": tof if 0.1 < tof < 6.0 else None,   # 10 cm 이하(바닥에 놓였을 때 10)·범위 밖은 '못 잼' (tello_link와 같음)
        "bat": float(raw.get("bat", 0)),
    }


def from_si(yaw: float, vx: float, vy: float, vz: float, h: float, tof: float | None, bat: float,
            conv: dict = CONVENTION, extra: dict | None = None) -> dict:
    """SI 값 → 상태 값(정수로 반올림). 시뮬레이션이 Tello 상태를 흉내 낼 때 쓴다 (to_si의 역)."""
    fields = {
        "mid": -1, "x": 0, "y": 0, "z": 0, "mpry": "0,0,0", "pitch": 0, "roll": 0,
        "yaw": round(math.degrees(wrap(yaw)) * conv["yaw_sign"]),
        "vgx": round(vx * 10 * conv.get("vx_sign", 1.0)), "vgy": round(vy * 10 * conv["vy_sign"]), "vgz": round(vz * 10),
        "templ": 60, "temph": 63,
        "tof": round(tof * 100) if tof is not None else 10,
        "h": round(h * 100), "bat": round(bat), "baro": 100.0, "time": 0,
        "agx": 0.0, "agy": 0.0, "agz": -1000.0,
    }
    if extra:
        fields.update(extra)
    return fields


# 드라이버(tello_link FlightController)가 스스로 착륙시킨 이유 → 사람이 읽는 말. 비행 노드 중단 메시지·기록 페이지에 쓴다
LANDING_REASONS = {
    "crash": "충돌로 보임 (30° 넘게 기운 뒤 바닥)",
    "flipped": "뒤집힘 (90° 넘게 기운 채 0.5초 — 모터를 껐습니다)",
    "grounded": "스스로 내려앉음 (기울지 않고 바닥에 닿음 — 충돌 아님)",
    "battery_low": "배터리 부족",
    "overheat": "과열",
    "state_lost": "상태값 끊김",
    "cmd_timeout": "속도 명령 끊김",
}


def describe_landing(reason) -> str:
    """착륙 이유 코드를 설명과 함께 돌려준다 (모르는 코드는 그대로)."""
    if not reason:
        return "알 수 없음"
    return f"{reason}: {LANDING_REASONS[reason]}" if reason in LANDING_REASONS else str(reason)


# 이륙 전 기수 값 확인 (11/16 10:18 실기체: takeoff를 두 번 보냈는데 Tello가 14 ms 만에 error로 거절). 지난 실기체 기록의 이륙 요청
# 52번을 견주니, 받아들인 29번은 직전 10초 동안 Tello yaw 값이 한 번도 안 바뀌었고(폭 0°), 거절한 23번은 모두 1° 이상 바뀌고 있었다
# (땅에 가만히 있는데 초당 0.05–3°로 흐름. 5초 창으로는 느린 흐름을 놓칠 수 있다). 온도·배터리로는 안 갈린다. 거절 뒤 성공한 비행은
# 기체를 들었다 다시 놓은 뒤였다. Tello가 왜 거절하는지는 알려 주지 않으므로 '기수 값이 흐르면 거절한다'는 기록에서 본 관계다
YAW_SETTLE_WINDOW = 10.0   # 이 시간(s) 동안
YAW_SETTLE_DEG = 2.0       #   기수 값이 이만큼(°) 이상 바뀌었으면 흐르는 중. 상태값이 1° 단위라 반올림 경계에서 1°씩 깜빡일 수 있어(Gazebo lab_f1)
#                            1°로는 안 잡는다: 거절 23번 중 22번이 2° 이상(나머지 하나는 20초에 1°), 받아들인 29번은 모두 0°
YAW_SETTLE_MIN = 3.0       # 상태값이 이만큼(s)도 안 쌓였으면 아직 모른다


def yaw_span(samples, now: float, window: float = YAW_SETTLE_WINDOW) -> float | None:
    """samples: (받은 시각 s, Tello yaw °)들 (시간순). 최근 window초 동안 기수 값이 바뀐 폭(°, ±180 넘김은 이어서 잰다).
    그 사이 상태값이 YAW_SETTLE_MIN초 미만이면 None (아직 모른다)."""
    recent = [(t, y) for t, y in samples if now - window <= t <= now]
    if len(recent) < 2 or recent[-1][0] - recent[0][0] < YAW_SETTLE_MIN:
        return None
    lo = hi = acc = 0.0
    for (_, a), (_, b) in zip(recent, recent[1:]):
        acc += (b - a + 180.0) % 360.0 - 180.0
        lo, hi = min(lo, acc), max(hi, acc)
    return hi - lo
