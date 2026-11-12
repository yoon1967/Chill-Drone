# Tello EDU 연결 모듈과 연결 점검 화면

Jira: [S15P31B205-156](https://ssafy.atlassian.net/browse/S15P31B205-156) 실제 Tello EDU 제어 연동.
PC에서 Wi-Fi로 Tello EDU에 SDK 명령을 보내고, 상태·영상을 받는 부분이다.
**이 단계는 이륙하지 않는다.** 연결만 확인하고, 이 모듈을 자율비행 쪽([S15P31B205-207](https://ssafy.atlassian.net/browse/S15P31B205-207) ROS 2 드라이버)에 넘긴다.

| 파일 | 역할 | 의존성 |
| --- | --- | --- |
| `tello_link.py` | 명령 송수신, 상태 파싱, 명령 차단, 회차 로그 | 표준 라이브러리 |
| `tello_video.py` | UDP 11111 H.264 수신·디코딩, 최신 1장 유지 | PyAV, OpenCV |
| `flight_control.py` | 수동 조작: 이륙 전 점검, 데드맨, 속도·높이 상한, 자동 착륙 | 표준 라이브러리 |
| `check_server.py` + `static/index.html` | 연결 점검·조작 웹 화면 | FastAPI, uvicorn |
| `fake_tello.py` | 드론 없이 시험하는 가짜 Tello(UDP) | 영상은 PyAV, numpy |
| `tests/test_tello_link.py` | 가짜 Tello로 명령·상태·차단·로그 시험 | 표준 라이브러리 |

## 실행

```powershell
cd modules/autonomy/tello_link
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt

.\.venv\Scripts\python check_server.py --fake   # 드론 없이 화면 확인
.\.venv\Scripts\python check_server.py          # 실제 Tello
# 브라우저: http://127.0.0.1:8010

python -B -m unittest discover -s tests -v       # 표준 라이브러리만 필요
```

YOLO 사람 윤곽선과 SegFormer 공간 마스크를 웹 카메라에 합성하려면 저장소 루트의
AI 의존성을 설치한 뒤 `--ai`로 실행한다. 카메라 제목 옆에서 **AI 합성/원본**을
바꿔 비교할 수 있다. 모델은 서버 시작 때 한 번 로드되며 첫 실행은 다운로드가 필요할 수 있다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/perception/inference/requirements.txt
.\.venv\Scripts\python.exe modules/autonomy/tello_link/check_server.py --ai
# 브라우저: http://127.0.0.1:8010 → 점검 시작 → 영상 켜기
```

가짜 모드는 값을 지어내므로 화면 위에 "가짜 Tello" 배지가 뜨고, 기체 SN이 `FAKE...`로 나온다. 실기체 결과로 쓰지 않는다.
`--fake-no-state`, `--fake-no-video`로 방화벽 차단 상황을 재현할 수 있다.

## 실제 Tello 점검 순서

1. **PC 인터넷 확보:** Tello Wi-Fi에 연결하면 PC 인터넷이 끊긴다. 유선 랜이 없으면 폰 USB 테더링을 먼저 켠다.
2. 폰 Tello 앱을 끄고, 폰의 Tello Wi-Fi 연결도 끊는다. 앱과 PC는 동시에 연결할 수 없다.
3. Tello 전원을 켜고 PC Wi-Fi를 `TELLO-xxxxxx`에 연결한다.
4. `check_server.py`를 실행한다. Windows 방화벽 허용 창이 뜨면 **개인·공용 네트워크 모두 허용**한다.
5. 화면에서 [점검 시작] → [영상 켜기]를 누르고 5단계 신호등을 확인한다.

| 단계 | 정상 기준 | 실패할 때 흔한 원인 |
| --- | --- | --- |
| 네트워크 | PC 주소가 192.168.10.x | Tello Wi-Fi에 연결되지 않음 |
| SDK 모드 | `command` → `ok` | 폰 앱이 연결돼 있음, 드론 전원 |
| 기체 정보 | battery?·sdk?·sn?·wifi? 응답 | 배터리 20% 미만이면 경고 |
| 상태 수신 | 약 10Hz, 2초 안에 갱신 | 방화벽이 UDP 8890 수신을 막음 |
| 영상 | 10fps 이상, 960×720 | 방화벽이 UDP 11111 수신을 막음 |

Tello는 팬이 없어 지상에서 켜 둔 채로 두면 과열로 꺼지고, 날면 바람으로 식는다. 점검 화면은 75°C 이상이면 경고하고, 85°C 이상이면 전원을 끄라고 안내한다(실측 전 가정값). 조작 기능은 온도로 이륙을 막지 않는다(아래 안전 장치).

## 안전 장치

`tello_link`는 명령을 다섯 종류로 나눠서 막는다. 점검 화면은 `--flight` 없이 실행하면 비행 명령을 보낼 수 없다.
`--flight`로 실행해도 비행 명령은 아래 "조작" 패널(`flight_control`)을 거쳐야 하고, 직접 입력 칸에서는 막힌다.

| 분류 | 예 | 기본 |
| --- | --- | --- |
| query | `battery?`, `sdk?`, `sn?`, `wifi?`, `tof?`, `EXT tof?`, `EXT version?` | 허용 |
| session | `command`, `streamon`, `streamoff` | 허용 |
| stop | `land`, `emergency`, `stop` | 항상 허용 |
| ext | `EXT led ...`, `EXT mled ...`(확장 키트 표시) | 허용 |
| flight | `takeoff`, `forward 50`, `rc ...`, `go ...`, `motoron` | `allow_flight=True`일 때만 |
| config | `ap ...`, `wifi ...`, `speed 50`, `mon`, `EXT mled sg/sc`, 그 밖의 `EXT` | `allow_config=True`일 때만 |

**확장 키트(RoboMaster TT):** 장착하면 Wi-Fi 이름이 `RMTT-xxxxxx`로 바뀌고 주소·포트는 같다. [점검 시작]이 `EXT version?`·`EXT tof?`도 묻고 "확장 키트 (선택)" 단계에 컨트롤러 버전과 전방 거리를 보여 준다.
전방 거리는 `parse_ext_tof()`가 `front_tof_m`(m)으로 바꾸고, 8190 이상(범위 밖, 실측 8190·8191)·`unknown command`·오류는 무효로 남긴다.
실측 요약: 0.6m까지 ±4mm, 1.2m까지 잡힘, **센서 갱신 약 5Hz(0.2초)**라 조회는 5~10Hz면 충분하다. 자세한 값과 명령 목록은 [입출력 문서 10장](../../../docs/drone/TELLO_EDU_IO.md#10-robomaster-tt-확장-키트)에 있다.

차단된 명령은 드론으로 보내지 않고, `blocked=true`로 기록만 남긴다.
`emergency`는 모터를 즉시 멈춘다(공중이면 추락). `land`와 다르다.

## 조작 (`--flight`)

```powershell
.\.venv\Scripts\python check_server.py --fake --flight   # 드론 없이 먼저 연습
.\.venv\Scripts\python check_server.py --flight          # 실제 Tello
```

화면 맨 위에 빨간 테두리의 "조작" 패널이 생긴다.

| 키 | 동작 |
| --- | --- |
| **Esc** / [비상 정지] | `emergency`. 모터 즉시 정지, **공중이면 떨어진다.** 이륙 응답을 기다리는 중에도 바로 나간다 |
| **T** / [이륙] | 확인 창 뒤 `takeoff`. 아래 점검을 모두 통과해야 켜진다 |
| **L** / [착륙] | `land` |
| W / S | 앞 / 뒤 |
| A / D | 왼쪽 / 오른쪽 |
| ↑ / ↓ | 상승 / 하강 |
| ← / → | 왼쪽 / 오른쪽 회전 |

이동 키는 누르고 있는 동안만 rc를 20Hz로 보낸다(WebSocket). 여러 키를 같이 누를 수 있다.

| 안전 장치 | 기본값 |
| --- | --- |
| 이륙 전 점검 | SDK 모드, 상태 1초 안에 갱신, 배터리 20% 이상, "주변 3m 비움·가드·안전 담당" 체크. 온도 75°C 이상은 **경고만**(이륙 허용) |
| 다시 이륙 | 착륙·비상 정지 뒤에는 체크가 풀린다. 매번 다시 확인한다 |
| rc 상한 | ±30 (±100 중) |
| 높이 상한 | 하방 tof 1.5m 이상이면 상승 입력을 0으로 바꾼다. 하방 tof가 무효면 높이를 모르는 것으로 보고 상승을 막는다. `h`가 1.5m 이상이어도 막는다. 하강은 허용 |
| 손 떼면 정지(데드맨) | 입력이 0.3초 없거나, 창을 벗어나거나, 연결이 끊기면 `rc 0 0 0 0`(제자리) |
| 자동 착륙 | 비행 중 상태 1초 끊김, 배터리 15% 미만. 고온 자동 착륙은 끔(기체 자체 보호에 맡김) |
| 기준을 완화한 이유(10/28) | Tello 앱은 온도로 이륙을 막지 않고, 위험 온도에서는 기체가 스스로 꺼진다. 지상에서 뜨거워진 기체는 띄워야 식는데, 80°C 이륙 차단이 오히려 이륙을 어렵게 했다. 배터리는 기체가 약 10%에서 스스로 착륙한다(사용자 보고) |
| 직접 입력 칸 | `takeoff`, `forward 50` 같은 비행 명령은 403으로 막는다 |

- 조작을 시작하면 `manifest.json`의 `operator_controlled`가 `true`가 된다. 사람이 조종한 회차는 자율 성공으로 세지 않는다.
- `commands.jsonl`의 `source`에 입력 출처가 남는다: `web`(버튼), `keyboard`, `deadman`, `+height_limit`, `+height_unknown`, `safety:<사유>`, `manual`(직접 입력).
- 비행 중에는 rc가 계속 나가므로 Tello의 "15초 무명령 자동 착륙"에 걸리지 않는다.
- rc 값은 조종 입력 비율이지 속도가 아니다. 실제 속도는 상태값·줄자 측정으로 따로 구한다.

### 첫 비행 순서

1. 사람 없는 3m × 3m 이상, 천장 2.5m 이상인 곳. 무늬가 있는 바닥이 하방 비전에 유리하다.
2. 프로펠러 가드를 끼우고, 조작하는 사람 말고 한 명이 옆에서 지켜본다.
3. 드론을 식힌 뒤 켜고, `--flight`로 서버를 실행한다. 지상 대기가 길면 뜨거워지므로 점검 후 바로 띄운다.
4. [점검 시작] → 체크박스 → **[이륙] → 10초 제자리 → [착륙]**. 첫 회차에는 이동 키를 쓰지 않는다.
5. 이상하면 망설이지 말고 **L**(착륙). 사람·물체와 부딪힐 것 같을 때만 **Esc**.

## 문제 해결

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| 상태·영상 단계만 ✕ | Windows 방화벽이 Python의 공용 네트워크 수신을 막음 | 관리자 PowerShell에서 아래 두 줄 |
| Tello에 붙으면 인터넷이 끊김 | Tello Wi-Fi에는 인터넷이 없음 | 폰 USB 테더링 + 테더링 우선순위 올리기(아래) |
| USB 테더링을 켰는데 IP가 안 잡힘 | 어댑터가 "Not Present"로 멈춤 | 장치 관리자 → 네트워크 어댑터 → Remote NDIS 장치 사용 안 함 → 사용 |
| `command`가 타임아웃 | 폰 앱이 연결돼 있거나 드론이 꺼짐 | 앱 끄기, 드론 다시 켜기 |
| 화면이 멈춤 | (수정됨) 영상 동시 변환 | 최신 코드 사용 |

방화벽(Python 경로는 각자 환경에 맞게 바꾼다):

```powershell
Get-NetFirewallApplicationFilter -Program "C:\Users\<사용자>\AppData\Local\Programs\Python\Python310\python.exe" | Get-NetFirewallRule | Where-Object Action -eq Block | Disable-NetFirewallRule
New-NetFirewallRule -DisplayName "Tello UDP (Python)" -Direction Inbound -Program "C:\Users\<사용자>\AppData\Local\Programs\Python\Python310\python.exe" -Protocol UDP -LocalPort 8890,11111 -RemoteAddress 192.168.10.0/24 -Action Allow -Profile Any
```

테더링 우선순위(번호는 `Get-NetAdapter`에서 Remote NDIS 장치의 `ifIndex`):

```powershell
Set-NetIPInterface -InterfaceIndex <번호> -InterfaceMetric 10      # 테더링을 인터넷 1순위로
Set-NetIPInterface -InterfaceIndex <번호> -AutomaticMetric Enabled # 되돌리기
```

## 자율비행 쪽에 넘기는 인터페이스

ROS 2 노드 안에서 그대로 import해 쓰는 것을 전제로, ROS에 의존하지 않게 만들었다.

```python
from tello_link import TelloLink

link = TelloLink(log_root="data/outputs/tello", allow_flight=True).start()
r = link.send("command")          # CommandResult: ok, response, rtt_ms, valid, reason, blocked
state = link.latest_state()       # dict: raw, si, valid, reasons, received_at, time_source, age_s
link.rc(0, 20, 0, 0)              # -100~100 클램프, 응답 없음
link.send("land")
link.close()                      # metrics.json 기록
```

- **명령:** 한 번에 하나씩 보내고 응답을 기다린다. 늦게 온 이전 응답은 버리고 `events.jsonl`에 남긴다. 응답이 없으면 `valid=false, reason="timeout"`이다.
- **상태:** `raw`는 SDK 원래 이름과 값, `si`는 단위가 확실한 값만 m·rad·°C·%·s로 바꾼다.
  - `height_m`(h), `tof_m`, `roll/pitch/yaw_rad`, `battery_pct`, `temp_low_c`, `temp_high_c`, `motor_time_s`
  - ToF가 10cm 이하이거나 6553이면 `tof_m=None`, `tof_valid=false`, `tof_reason`으로 표시하고 0으로 바꾸지 않는다. 기준값은 실측으로 확인해야 한다.
  - `vgx/vgy/vgz`, `agx/agy/agz`, `baro`는 문서마다 단위가 달라 실측 전까지 SI로 바꾸지 않는다(`si.unverified_unit_fields`).
- **시각:** 카메라·상태 패킷에 드론 시각이 없으므로 PC 수신 시각을 쓰고 `time_source="server_receive"`로 표시한다.
- **rc 타임아웃 정지:** 수동 조작은 `FlightController`(데드맨 0.3초)가 맡는다. -207 드라이버 노드는 `FlightController.set_input()`을 그대로 쓰거나, 같은 규칙(입력이 끊기면 `rc 0 0 0 0`)을 노드 안에 둔다.
- **비상 정지:** `TelloLink.emergency_now()`는 다른 명령의 응답을 기다리는 중에도 잠금 없이 바로 보낸다.

## 게이트웨이 조종 입력 변환 (서버 연결 전)

감독 화면·폰·자율 비행의 입력을 Tello rc로 바꾸는 부품이다. 계약은 `contracts/api/AUTONOMY_BRIDGE_V0_1.md` 3·5장을 따른다.

| 파일 | 하는 일 |
| --- | --- |
| `rc_mixer.py` | 자율 `cmd_vel`(m/s, rad/s, 기체 기준) → rc. 수평은 같은 비율로 줄여 대각선 방향을 지킨다. 사람 입력(-1~1)은 데드존 0.15를 빼고 끝까지 밀면 rc 30. 가감속 제한은 커질 때만 초당 rc 150(0 → 30이 0.2초), 줄이거나 멈출 때는 바로 |
| `tof_fresh.py` | 전방 ToF 새 값 판정. 바뀌었거나 같은 값이 0.2초 이상 이어질 때만 `seq`를 올린다(자율의 "두 번 연속" 필터용) |

- `K_LIN` 100 rc/(m/s), `K_YAW` 57 rc/(rad/s)는 가정값이다. rc 보정 실험(-274)으로 확정한다.
- 시험: `python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_rc_mixer.py"`

## 게이트웨이 모드 판정 (서버 연결 전)

감독 화면·폰·자율 비행이 한 기체를 함께 쓸 때 "지금 누구의 입력을 보낼지"를 정하는 순수 로직(`mode_arbiter.py`)이다.
계약은 `contracts/api/DRONE_CONTROL_V0_1.md`(화면 ↔ 게이트웨이), `AUTONOMY_BRIDGE_V0_1.md`(게이트웨이 ↔ 자율)를 따른다.
아직 서버(`/ws/drone`, `/ws/autonomy`)에는 연결하지 않았다.

- 모드 7개: `disconnected`·`grounded`·`manual`·`auto`·`paused`·`landing`·`emergency`. 전이, 우선순위, `allowed` 목록, 경고, 개입 횟수를 맡는다.

```python
from mode_arbiter import ModeArbiter

arb = ModeArbiter(source="tello")       # tello / fake / replay(모든 명령 거절)
arb.on_link(True)                       # 상태 패킷 수신 중 → grounded
arb.on_autonomy_connected()             # 지상에서 연결해야 auto_start 가능
result = arb.handle_cmd("takeoff", client="monitor", cmd_id="c-1")   # result 메시지
for action in arb.drain():              # flight: FlightController, autonomy: control 메시지, event: 화면
    ...
out = arb.tick()                        # 20Hz. out.rc가 None이 아니면 FlightController.set_input(*(v / 100 for v in out.rc))
```

- 실행(이륙·착륙·비상 정지, 20Hz rc 전송, 안전 착륙, 높이 상한, 데드맨)은 `FlightController`가 그대로 맡는다.
  안전 착륙이 나면 서버가 `arb.on_safety_landing(reason)`, 착륙이 끝나면 `arb.on_landed()`를 부른다.
- 자율 속도 명령은 `auto`일 때만 쓴다. 0.5초 끊기면 제자리(`auto` 유지), 2초면 `paused`, 연결이 끊기면 바로 `paused`.
- `auto`·`paused`에서 사람 조이스틱이 데드존 밖이면 넘겨받기(`manual`)다. 자율에 `stop`을 보내고 개입 횟수를 올린다.
- 자율의 `takeoff`·`emergency` 요청은 거절한다. `land`는 `auto`·`paused`에서만 받는다.
- 시험: `python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_mode_arbiter.py"` (실제 시계·네트워크 없이 끝난다)

## 드론 게이트웨이 서버 (`gateway_server.py`)

관제 화면(`apps/controller`)과 상태·명령을 주고받는다. 계약은 `contracts/api/DRONE_CONTROL_V0_1.md`를 따른다.

```powershell
python modules/autonomy/tello_link/gateway_server.py --fake      # 드론 없이 가짜 Tello
python modules/autonomy/tello_link/gateway_server.py             # 실제 Tello (드론 Wi-Fi에 연결된 PC)
```

브라우저에서 `http://127.0.0.1:8010`을 연다(실시간 감독 `#/live`, 현장 조작 `#/control`).
`apps/controller`가 있으면 서버가 그 화면을 함께 내준다. 폰에서 열려면 `--host 0.0.0.0`(같은 망 안에서만).

| 경로 | 내용 |
| --- | --- |
| `GET /api/drone/state` | 최신 `drone_state` 1개 |
| `WS /ws/drone` | 서버 → 화면: `drone_state` 약 10Hz, `event`, `result`. 화면 → 서버: `cmd`, `rc` |
| `GET /video.mjpg` | 영상. 최신 프레임만 |
| `GET /snapshot.jpg` | 최신 프레임 1장. 헤더 `X-Input-Id`(영상 세션, 켤 때마다 새 값)·`X-Frame-Id`·`X-Frame-Time`(게이트웨이 디코딩 시각, UTC)·`X-Frame-Age-Ms`·`X-Image-Size`. 영상이 꺼졌거나 끊겼으면 503(`video_off`·`video_stale`·`no_frame`), `--no-video`면 404 |
| `POST /api/detections` | 사람 탐지 이벤트 `person-detection-event/1.0` 1개. 201 저장, 200 중복(`event_id`), 400 검증 실패, 403 다른 PC, 500 저장 실패 |
| `GET /api/detections` | 발견 목록(최신순)과 탐지 상태 |
| `GET`·`PUT /api/detections/<event_id>.jpg` | 발견 사진. PUT은 게이트웨이 프레임이 아닌 이벤트의 사진 올리기(JPEG) |
| `POST /api/detections/<finding_id>/review` | 확인·오탐 `{"review": "confirmed" \| "false_positive" \| "unreviewed"}`. 200 저장(같은 값이면 `changed: false`), 400 모르는 값, 404 모르는 발견, 413 너무 큼, 500 저장 실패(되돌림), 기록 재생은 409 |
| `WS /ws/autonomy` | 자율 비행 하나(`AUTONOMY_BRIDGE_V0_1`). 게이트웨이 → `hello`·`tello_state`(원문 약 10Hz)·`flight_status`(바뀔 때 + 1Hz)·`control`, 자율 → `hello`·`cmd_vel`·`mission_state`·`ack`·`request`·`pose`·`path`·`people`. 기본은 같은 PC만, 두 번째 연결은 `busy` |
| `GET /api/autonomy` | 자율 위치·계획 경로·찾은 사람 전체. 상태(`drone_state.autonomy`)에는 버전만 싣는다 |

- `gateway.py`가 화면 메시지 → 모드 판정기 → `FlightController`를 잇는다. 20Hz 루프에서 연결·안전 착륙·착륙 완료·비상 정지 뒤 지상을
  판정기에 알리고, 판정기가 고른 rc를 `FlightController.set_input()`에 넣는다.
- 이륙은 이륙 전 점검(주변 확인 포함)을 통과해야만 판정기로 넘긴다. 실패하면 `preflight_failed`와 실패 항목을 돌려준다.
- 화면 연결이 끊기면 그 화면의 rc는 바로 0이다. `height_m`은 하방 tof 값이며 무효면 `null`이다.
- 사람 탐지 연결(`detection_inbox.py`): 탐지 프로그램은 `/snapshot.jpg`를 받아 처리하고, 헤더의 `X-Input-Id`·`X-Frame-Id`를 이벤트
  `input_id`·`frame_id`에, `X-Frame-Time`을 `detected_at_utc`에 넣어 `POST /api/detections`로 보낸다. 사람이 없어도 보낸다(동작 중 근거).
  - 게이트웨이는 같은 프레임의 사진·수신 시각·그때 드론 상태(모드·높이·배터리, 자율 위치가 새 값이면 `pose`(frame map))를 붙여 회차 폴더 `detections.jsonl`·`detections/<event_id>.jpg`에 남기고,
    사람이 있으면 화면에 `detection` 메시지를 보낸다. `drone_state.detection`에 동작 여부·초당 처리 수·발견 수가 있다.
  - 발견: 사람이 3초 넘게 안 보이다 다시 보이면 새 발견(사람 구분은 하지 않음). 대표 사진은 신뢰도가 가장 높은 장면, 사진은 발견마다 초당 1장까지.
  - 기본은 같은 PC에서 온 결과만 받는다(인증이 없음). 다른 PC에서 탐지를 돌리면 `--allow-remote-detections`(같은 망 시험에서만).
  - 확인·오탐: 화면이 발견마다 고른 판단을 게이트웨이가 저장한다(발견의 `review`·`review_at`, 처음은 `unreviewed`).
    바뀌면 `detections.jsonl`에 `type: review` 줄을 남기고 모든 화면에 `detection_review`(발견 번호·값·이전 값)를 보낸다.
    조종 화면처럼 어느 화면이든 보낼 수 있다(인증 없음). 기록에 못 남기면 되돌리고 500을 돌려준다.
  - 드론을 자동으로 멈추지 않는다. 시험·시연용 가짜 탐지: `python modules/autonomy/tello_link/fake_detector.py`(영상을 켠 뒤)
- 프레임 번호: `/snapshot.jpg`로 내준 프레임은 최근 64장(최대 10초)을 보관한다(`frame_store.py`). 탐지처럼 프레임을 가져가 처리하는 쪽은
  결과에 `input_id`·`frame_id`만 담으면 게이트웨이가 같은 사진과 시각을 다시 찾는다. 영상을 켜기 전에 남아 있던 프레임은 내주지 않는다.
- 자율 비행 연결(`autonomy_link.py`): 판정(속도 명령을 기체로 보낼지, 끊김 0.5초 제자리·2초 일시정지, 넘겨받기, 도착하면 일시정지)은 모드 판정기가 한다.
  - 자율이 지상에서 붙어야 자율 시작이 된다(위치 원점 = 출발점). 이륙 뒤 붙으면 `autonomy_joined_in_air`로 거절.
  - `pose`·`path`·`people`은 `frame: "map"`(켠 자리 원점, x 앞·y 왼쪽, m)만 받고 좌표를 바꾸지 않는다. `drone_state.pose`에 위치, `drone_state.autonomy`에 연결·명령 빈도·찾은 사람 수.
  - `people`은 0.1.0 초안에 없던 것으로, 자율 비행 쪽 `people.json`(확정 `people`·미확정 `unconfirmed`, 각 `x`·`y`·`conf`·`hits`·`gone`·`t_first`·`top`) 모양 그대로.
  - 회차 폴더 `autonomy/`: `events.jsonl`(연결·control·ack·request·임무 상태 변화·찾은 사람 변화), `pose.jsonl`(1초에 한 번), `latest.json`(경로·찾은 사람).
  - 시험용 가짜 자율: `python modules/autonomy/tello_link/fake_autonomy.py --gateway http://127.0.0.1:8010` (2 m 네모를 돌며 사람 2명을 찾는다. `--stall-at 5`는 끊김 시험, `--reject-start`는 시작 거절)
  - 다리 노드(ROS 2)와 WSL2 접속(`--allow-remote-autonomy`), 영상 UDP 복사는 다음 단계.
- 아직 없는 것: 전방 ToF 조회(`front_tof`는 `not_polled`, 자율에도 보내지 않음), 통신 품질(`link`), 기록 재생(`--replay`).
- `check_server.py`와 같은 기본 포트(8010)를 쓴다. Tello는 처음 `command`를 보낸 쪽에만 답하므로 둘을 동시에 켜지 않는다.
- 시험: `python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_gateway.py"` (가짜 Tello, 약 6초), 자율 연결은 `-p "test_autonomy.py"`

## 기록 재생 (`--replay`)

게이트웨이 회차 폴더를 기체 대신 시간에 맞춰 다시 내보낸다(`replay.py`). 드론 없이 화면이 잘 도는지 확인하거나, 같은 비행으로 화면·탐지를 비교할 때 쓴다.

```powershell
python modules/autonomy/tello_link/gateway_server.py --replay data/outputs/tello/<회차> --replay-loop      # http://127.0.0.1:8010
python modules/autonomy/tello_link/gateway_server.py --replay <회차> --replay-speed 2 --replay-start 60    # 2배속, 60초부터
```

- 화면은 실시간과 똑같이 붙는다(위쪽 표시 "기록 재생"). 명령은 모두 `replay_source`로 거절하고, 자율 연결도 받지 않는다. 원래 회차 폴더에는 쓰지 않는다.
- 다시 내보내는 것: 드론 상태(`sensor.jsonl`), 모드 변화·영상 켜고 끔(`events.jsonl`), 영상(`video/`, 받은 시각대로 같은 디코더에),
  사람 탐지·발견·사진(`detections.jsonl`·`detections/`, 발견은 기록된 `finding_id`로 묶음), 확인·오탐(그 시각에 `detection_review`, 재생 중에는 못 바꿈), 3D 지도(`map/snapshots/`, 없으면 `map/latest.json`),
  자율 위치·경로·찾은 사람·임무 상태(`autonomy/`). 시각은 원래 회차의 UTC 그대로. `drone_state.replay`에 재생 위치·길이·속도.
- 영상 원본을 남기기 전(11/05 이전) 회차는 영상 없이 재생된다. `/snapshot.jpg`도 재생 영상으로 내주므로 탐지 연결기를 다시 돌려 볼 수 있다(결과 수신은 아직 거절).
- 재생 시험: `python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_replay.py"`

## 회차 로그

`log_root/<run_id>/`에 남는다. 기본 위치 `data/outputs/tello/`는 Git에서 제외된다.

| 파일 | 내용 |
| --- | --- |
| `manifest.json` | 회차 ID, 출처(tello/fake), 포트, 차단 설정, 단위 |
| `commands.jsonl` | 보낸 명령·응답·왕복 시간, 차단된 명령 포함 |
| `sensor.jsonl` | 상태 패킷 전체(약 10Hz), 원시값과 SI 값 |
| `events.jsonl` | 연결 시작·종료, 점검 실행, 영상 켜기·끄기, 버린 응답 |
| `metrics.json` | 종료 시 명령 수, 차단 수, 타임아웃 수, 왕복 시간 중앙값·최대값, 상태 패킷 수 |
| `video/part_NNNN.h264`·`.jsonl`, `video/parts.jsonl` | 게이트웨이(`gateway_server.py`)만. 영상을 켤 때마다 받은 H.264 그대로 한 조각 + 패킷마다 `offset`·`size`·`timestamp_s`(monotonic)·`wall`(UTC epoch). 조각 시작·끝(영상 세션 `input_id`, 상태 `captured`/`partial`). 분당 약 10 MB, `--no-record-video`로 끈다. 지도 파트 녹화(`modules/mapping/data/live_recording.py`)와 같은 칸 |
| `detections.jsonl`·`detections/` | 게이트웨이만. 사람 탐지 결과와 발견 사진(실제 사람이 찍히므로 저장소·공유에 올리지 않는다) |
| `map/latest.json`, `map/snapshots/NNNN_rev<버전>.json.gz`, `map_updates.jsonl` | 게이트웨이만. 3D 지도 최신본, 5초마다 + 형식·세션·구간이 바뀔 때 한 버전, 버전마다 요약(`saved` = 남긴 파일) |
| `autonomy/` | 게이트웨이만. `events.jsonl`, `pose.jsonl`(받는 대로 전부), `path.jsonl`·`people.jsonl`(바뀔 때마다), `latest.json` |

pose는 만들지 않는다(`manifest.files.pose = null`). 자율 비행이 보낸 위치는 `autonomy/pose.jsonl`에 있다.
영상·지도 버전·자율 기록은 기록 재생과 화면 시험용 목업 데이터를 만들려고 남긴다. 실제 사람이 찍힌 영상·사진은 저장소에 넣지 않는다.

## 실기체 확인 결과 (2026-10-23, 이륙 없음)

기체: Tello EDU, SN `0TQZG9VED0052R`. PC: Windows 11, Wi-Fi로 Tello 연결, 인터넷은 폰 USB 테더링.

| 항목 | 결과 |
| --- | --- |
| SDK 모드 | `command` → `ok`, 왕복 16~47ms |
| `sdk?` | **`30`** (SDK 3.0으로 응답) |
| 상태 수신 | 9.9~10.0Hz |
| 영상 | 960×720, 약 24fps, 프레임 나이 0.03초 안팎, 디코딩 실패 0 |
| 영상 1분 연속 | 점검 서버 스레드 17~35개 유지, 상태 화면 응답 약 0.2초, 영상 연결 60회 붙였다 끊어도 정상 |

실측으로 알게 된 것:
- **방화벽:** 예전에 허용 창을 취소해서 생긴 Python 3.10 "공용 차단" 규칙 때문에 8890 수신이 막혔다. Tello Wi-Fi는 "공용 네트워크"로 잡힌다.
  차단 규칙을 끄고 192.168.10.0/24에서 오는 UDP 8890·11111만 허용하는 규칙을 추가해 해결했다.
- **가속도 단위:** 바닥에 둔 상태에서 `agz`가 약 -1004였다. 0.001g 단위로 보인다. 축 방향과 다른 축은 비행 중 확인 전까지 원시값으로만 둔다.
- **ToF:** 바닥에서는 `tof=10`으로 나오고 무효(`below_min_or_on_ground`)로 처리된다.
- **온도:** 식힌 뒤 켜면 46~48°C에서 시작해, 영상을 켜면 1분 반 만에 64°C까지 오른다.
  잠깐 껐다가 바로 다시 켜면 금방 **91°C**까지 올랐다. 점검은 짧게 하고, 사이사이 10분 이상 식힌다.
- **영상 변환 버그(수정됨):** 여러 연결이 같은 프레임을 동시에 JPEG로 바꾸면 PyAV가 프로세스를 깨뜨린다.
  처음 실기체 점검 때 스레드가 5,000개 넘게 불어나 서버가 멈췄다. `TelloVideo.jpeg()`에 잠금을 걸고 디코더 자동 스레드를 껐다.

## 첫 수동 비행 (2026-10-23, `--flight`)

회차 `tello_20261023_173915_3b7b19`. 실내, 프로펠러 가드, 운용자 조작(자율 아님). 이륙 전 온도 55°C, 배터리 89%.
두 번 이륙해 키보드로 짧게 움직인 뒤 둘 다 정상 착륙했다. 자동 착륙·비상 정지는 일어나지 않았다.
서버가 정상 종료되지 않아 이 회차에는 `metrics.json`이 없다. 아래 값은 `commands.jsonl`·`sensor.jsonl`에서 계산했다.

| 항목 | 비행 1 | 비행 2 |
| --- | --- | --- |
| `takeoff` 응답(이륙 완료까지) | 8.7초 | 8.6초 |
| 공중 시간(이륙 응답 ~ 착륙 명령) | 23.9초 | 32.3초 |
| `land` 응답 | 3.2초 | 4.6초 |
| 높이 `h` 최소 / 중앙 / 최대 | 80 / 130 / **150**cm | 70 / 130 / **150**cm |
| 하방 ToF 중앙 | 136.5cm | 134cm |
| 자세 pitch / roll 범위 | -3~6° / -2~0° | -2~4° / -1~0° |
| 배터리 | 85 → 74% | 73 → 69% |
| 온도(temph) | 59 → 60°C | 61 → 68°C |
| 상태 수신 | 9.9Hz, 최대 간격 0.11초 | 9.9Hz, 최대 간격 0.12초 |
| 실제 rc 전송 | **16.3Hz**, 간격 62ms(최대 65ms) | 16.3Hz, 간격 62ms(최대 68ms) |
| rc 입력 | 키보드 322, 데드맨 68 | 키보드 438, 데드맨 83 |

확인한 것:
- 이륙 전 점검 게이트, 이륙·제자리·착륙, 키보드 rc(±30), 데드맨, 상태 연속 수신이 실기체에서 동작했다.
- 상승 키를 계속 누르면 `h`가 150cm에서 멈췄다. 높이 상한(1.5m)이 작동했다.
- rc 앞뒤 ±30을 누르는 동안 `vgx`는 -2~3이었다. rc 30이 0.2~0.3m/s 정도라면 `vgx`는 dm/s 단위로 보인다. **줄자로 재지 않은 추정**이다.
- 이륙 명령은 응답이 오기까지 약 8.6초 걸린다. 이 동안 다른 명령은 잠금 때문에 기다리고, 비상 정지만 바로 나간다.

### 조작감: 폰 앱보다 매끄럽지 않음

운용자 소감: 폰 앱으로 조종할 때만큼 매끄럽지 않다. 로그로 보이는 원인은 아래와 같다.

| 원인 | 근거 | 개선 후보 |
| --- | --- | --- |
| 입력이 0 아니면 ±30뿐이다(켜짐·꺼짐) | 0이 아닌 rc가 모두 ±30 | 게임패드(브라우저 Gamepad API)로 아날로그 입력, 키보드는 가속·감속 램프 |
| 상한이 ±30으로 낮다 | 첫 비행용 제한 | 응답 측정 후 ±50 등으로 단계적으로 올림 |
| rc가 목표 20Hz가 아니라 16.3Hz로 나간다 | 간격 62ms ≈ Windows 기본 타이머 15.6ms × 4 | 고해상도 대기(Python 3.11+ 또는 타이머 보정), rc 로그 쓰기를 별도 스레드로 |
| 키를 떼면 바로 0으로 멈춘다 | 데드맨 rc가 비행마다 68·83회 | 떼면 짧게 감속한 뒤 0 |

이 개선은 다음 작업으로 남긴다. 비행 중 조작 반응(키 입력 → 상태 변화)을 재는 방법은 아래 "아직 확인하지 않은 것"에 포함했다.

## 아직 확인하지 않은 것

- 속도(vgx·vgy·vgz) 단위 확정(줄자 비교)과 기압(baro) 단위.
- 키 입력부터 기체 움직임(`vgx`·`h` 변화)까지의 지연.
- ToF 무효값 기준(10, 6553)이 비행 중에도 맞는지, 과열 경고(75°C)가 적절한지, 기체가 스스로 꺼지는 온도.
- 15초 무명령 자동 착륙(조작 중에는 rc가 계속 나가서 확인하지 않았다).
- WSL2(ROS 2)에서 Tello UDP 수신 여부. 기본 NAT 모드에서는 8890·11111 수신이 안 될 수 있다.

## 라이선스

FastAPI(MIT), uvicorn(BSD-3), PyAV(BSD-3), Pillow(MIT-CMU), OpenCV(Apache-2.0), numpy(BSD-3).
PyAV 휠에는 FFmpeg 바이너리가 들어 있고, 빌드 구성에 따라 GPL 구성 요소(libx264 등)가 포함될 수 있다.
이 저장소는 휠을 재배포하지 않고 의존성으로만 선언한다. 바이너리를 묶어 배포할 때는 다시 확인한다.
