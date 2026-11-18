# Tello EDU ROS 2 드라이버 (자율비행용)

자율비행 노드(`simulation/tt-sim/nodes/mission_node.py`)의 속도 명령과 이륙·착륙 요청을 Tello SDK 명령으로 바꾸고,
Tello 상태를 위치 추정 노드에 넘긴다. **실기체와 Gazebo 시뮬레이션이 같은 드라이버를 쓴다.**
시뮬레이션에서는 `simulation/tt-sim/nodes/gz_tello.py`가 Gazebo 드론을 UDP로 말하는 Tello처럼 흉내 내고,
드라이버는 주소·포트만 바꿔 그쪽에 붙는다.

```text
mission_node ──/tt/cmd_vel(m/s)──▶ tello_driver_node ──rc a b c d (20Hz)·takeoff·land──▶ Tello EDU (192.168.10.1:8889)
             ──/tt/flight_cmd───▶   (TelloLink +                                          또는 gz_tello.py (127.0.0.1:28889)
             ◀─/tt/flight_status─    FlightController) ◀──상태 패킷 10Hz (UDP 8890 / 28890)──┘
pose_node    ◀─/tt/state─────────
```

연결·안전 장치는 [tello_link](../tello_link/README.md)의 `TelloLink`와 `FlightController`를 그대로 쓴다.

| 파일 | 역할 |
| --- | --- |
| `driver_core.py` | ROS 없는 부분: 속도 → rc 변환, 이륙·착륙 요청 스레드, 명령 끊김 착륙, 상태 문자열 전달 |
| `tello_driver_node.py` | ROS 2 노드 (토픽·파라미터·영상) |
| `hover_check.py` | 실기체 첫 자율 명령 시험: 이륙 → 제자리 → (선택) 앞으로 짧게 → 착륙, 상태값 요약 |
| `tello_relay.py` | (Windows에서 실행) WSL2 NAT 네트워크용 Tello UDP 중계 |
| `start_driver.sh` | (WSL) 실기체 드라이버 실행: Windows 주소를 찾아 중계에 붙는다. `--fly`로 운용자 확인 |
| `tests/test_relay.py` | 중계를 사이에 두고 명령·응답·상태가 오가는지 (표준 라이브러리만) |
| `tests/test_driver_core.py` | 가짜 Tello(`tello_link/fake_tello.py`)로 변환·이륙 점검·rc·끊김 착륙·전방 ToF 시험 (표준 라이브러리만) |

## 토픽

| 방향 | 토픽 | 형식 |
| --- | --- | --- |
| 입력 | `/tt/cmd_vel` | `geometry_msgs/Twist`, 기체 기준 x 앞·y 왼쪽·z 위(m/s), angular.z 반시계 +(rad/s) |
| 입력 | `/tt/flight_cmd` | `std_msgs/String`: `takeoff`, `land`, `emergency`, `confirm`·`unconfirm`(운용자 확인 켜기·끄기, 자율비행 웹 화면의 체크박스) |
| 출력 | `/tt/state` | `std_msgs/String`, JSON `{"stamp": 받은 시각, "raw": "pitch:0;roll:0;...;"}` (시뮬레이션 흉내와 같은 형식) |
| 출력 | `/tt/flight_status` | `std_msgs/String`, JSON: `connected`, `flying`, `busy`, `can_takeoff`, `preflight`, `last_rc`, `last_result`, `last_auto_land`(이유 `crash`·`grounded`·`battery_low`·`overheat`·`state_lost`), `landing_reason`, `last_warning`(`sink_against_climb`·`climb_stalled`), `ceiling_cap_m` |
| 출력 | `/tt/camera/image_raw` | `sensor_msgs/Image` rgb8, `video:=true`일 때만 (PyAV 필요). `camera_width`(실기체 480)로 줄여 낸다: 960 영상(2 MB)을 초당 15장 내면 ROS가 큰 메시지를 버려 깊이 노드가 10초에 2–10장밖에 못 받았다 (2026-10-29) |
| 출력 | `/tt/tof_front` | `sensor_msgs/LaserScan` 한 줄(m), `front_tof:=true`일 때만. 아래 "전방 ToF" |

## 전방 ToF (RoboMaster TT 확장 키트)

`front_tof:=true`(실기체 `real.launch.py`는 기본으로 켬)면 떠 있는 동안 0.1초마다 `EXT tof?`를 묻고
([입출력 문서 10.3](../../../docs/drone/TELLO_EDU_IO.md)), 새 값만 `/tt/tof_front`로 낸다.

- 센서 값은 약 0.2초마다 바뀐다. 같은 값은 0.2초에 한 번만 낸다(`tello_link/tof_fresh.py`). 비행 노드는 두 번 연속
  가까워야 장애물로 보므로, 같은 응답을 두 번 내면 한 번 튄 값도 장애물이 된다. 멈춘 장애물은 0.2초마다 다시 낸다.
- 범위(1.2 m) 밖(`tof 8190`·`8191`)은 `inf`, 오류·무응답은 내지 않는다. 시각은 왕복의 가운데로 적는다.
- 이륙·착륙 중에는 묻지 않는다. 늦게 온 `tof` 응답을 takeoff·land의 응답으로 읽으면 떠 있는지 잘못 안다.
  같은 이유로 응답을 0.5초(실측 왕복 최대 507 ms)까지 기다린다. rc는 응답이 없어 이 대기를 기다리지 않고 바로 나간다
  (11/04 전에는 같은 잠금을 기다려 멈춤 rc까지 최대 약 0.5초 밀렸다 — 외부 리뷰, 실비행 명령 공백 523–571 ms). takeoff·land처럼
  응답을 읽는 명령은 여전히 이 조회가 끝나기를 기다린다(최대 0.5초).
- 연결 직후 한 번 물어 키트가 답하는지 로그에 찍는다(`전방 ToF(확장 키트): 0.481 m`).
- 시뮬레이션: `mission.launch.py tof:=ext`면 gz_tello.py가 Gazebo 전방 ToF로 `EXT tof?`에 답하고 이 경로를 그대로 쓴다.

## 속도 → rc

`rc a b c d`는 조종 입력 비율(±100)이다. 드라이버는 아래 가정으로 바꾸고, `rc_limit`(기본 ±30) 안으로 줄인다.
수평은 방향을 지킨 채 함께 줄인다(축마다 자르면 비스듬히 갈 때 방향이 틀어진다).

| 파라미터 | 기본 | 뜻 | 근거 |
| --- | --- | --- | --- |
| `v_full` | 1.0 | rc 100일 때 수평 속도 (m/s) | 실기체(`real.launch.py`)는 **1.03**: 2026-10-29 rc 20에 약 0.12 m/s, rc 40–42에 0.3–0.4 m/s를 이은 기울기 |
| `v_dead` | 0 | 수평 rc 무시 구간 (rc 100 = 1). 작은 rc를 Tello가 무시하므로 이만큼 건너뛰어 보낸다 | 실기체는 **0.095** (rc 9.5 아래는 안 움직임, 10/29) |
| `vz_full` | 1.0 | rc 100일 때 상하 속도 (m/s) | 가정 |
| `yaw_full_deg` | 100 | rc 100일 때 회전 속도 (°/s) | 가정 |
| `rc_limit` | 30 | rc 상한. 기본 ±30이면 수평 최대 약 0.3 m/s | 첫 비행 상한과 같음. 실기체 `real.launch.py`는 80 |
| `vz_dead` | 0 | 상하 rc 무시 구간 (rc 100 = 1). 높이 차 4 cm 넘는 상하 명령은 이만큼 건너뛰어 보낸다 | 실기체는 **0.1**: rc 상하 3–7을 무시해 비행 높이 1.0 m에 8 cm 모자란 0.92 m에 머물렀다 (10/27, 11/02) |
| `rc_slew` | 1.5 | rc 크기를 **키울 때만** 초당 이만큼(rc 100 = 1)씩 올린다: 0 → 45에 0.3 s. 줄이기·제동·반대 방향은 바로. 0이면 한 번에 바꿈 | 2026-10-30: 0 → 27, 회전 0 → 80을 한 번에 보내 기체가 덜컥거렸다 |

부호: Tello rc는 좌우 오른쪽 +, 회전 시계 방향 +이므로 우리 y(왼쪽 +)와 yaw(반시계 +)는 부호를 뒤집는다.
rc가 속도 명령과 맞지 않아도 비행 노드는 위치를 보며 고치므로 날 수는 있다. 다만 오차가 크면 경로 추종이 출렁이니
`hover_check.py`로 재서 맞춘다.

## 안전 장치

| 장치 | 동작 | 맡는 곳 |
| --- | --- | --- |
| 이륙 전 점검 | SDK 모드, 상태 1초 안, 배터리 10% 초과(이륙 기준 없음, `battery_min_takeoff` 기본 0 — 2026-11-02 결정), 온도 80°C 미만(`temp_max_takeoff`), **운용자 확인**(`operator_confirmed:=true`) | FlightController |
| 데드맨 | 속도 명령이 0.3초 없으면 `rc 0 0 0 0`(제자리) | FlightController |
| 높이 상한 | `height_limit` 이상이면 상승 입력 무시 (기본 1.5 m. 실기체 launch는 천장 − 0.5 m를 준다: 빈 지도 2.0 m, 11/03) | FlightController |
| 자동 착륙 | 상태 1초 끊김, 배터리 **10% 이하**(`battery_land`, 그 자리에 착륙. 그 밖에는 배터리로 출력을 줄이거나 내리지 않음), 온도 85°C 이상(`temp_land`) | FlightController |
| 착륙 전 스틱 0 | `land` 앞에 `rc 0 0 0 0`을 세 번 보낸다(요청 착륙·자동 착륙·종료 착륙 모두). Tello는 마지막에 받은 rc를 내려오는 동안에도 따른다. 전에는 입력을 0으로 정해 두기만 하고 rc 보내기를 멈춰, 움직이는 중에 착륙하면 그 방향으로 가며 내려왔다 (2026-11-12 13:58: 뒤쪽 제동 `rc 0 -36 0 0` 중에 착륙 요청 → 3.5초에 0.83 m 뒤로) | FlightController |
| 충돌 판정 `crash` | 떠 있다고 믿는 동안 30° 넘게 기운 뒤 0.5초 안에 하방 ToF가 바닥이면 부딪혀 떨어진 것: 착륙 처리로 명령을 끊는다 (2026-10-29 모니터 충돌 뒤 2초 넘게 회전 명령을 보냈던 것) | FlightController |
| 뒤집힘 판정 `flipped` | 떠 있다고 믿는 동안 90° 넘게 기운 채 0.5초면 바닥을 기다리지 않고 **비상 정지**(모터 끔). 뒤집히면 하방 ToF가 위를 봐 바닥 판정이 안 나고 land는 error로 끝난다 (2026-11-03 14:32 선반에 닿아 roll −173°) | FlightController |
| 내려앉음 판정 `grounded` | 기울지 않았는데 바닥이면 스스로 내려앉은 것(충돌 아님). 하방 ToF와 `h`가 둘 다 바닥이면 0.8초, ToF만이면 1.5초에 판정. 바닥이 0.3초 이어지면 판정 전이라도 입력을 0으로(밀지 않기) (2026-10-30 복도: 배터리가 약해 상승 입력을 무시한 채 내려앉았고, 1.5초 동안 카펫 위에서 밀렸다) | FlightController |
| 상승 중 하강 경고 | 상승 입력(rc 15 이상) 중 0.6초에 하방 ToF가 0.2 m 넘게 떨어지고 Tello `h`도 그 절반 넘게 같이 내려가면 `sink_against_climb` 사건과 로그 `기체가 상승 입력과 반대로 … 내려갔습니다 (하방 ToF, Tello 높이도 … m 같이 내려감)`. 착륙시키지는 않는다. ToF만 줄면 밑에 가구·바닥 턱이 들어온 것이라 남기지 않는다 (2026-11-05–11-09 실기체 경고 10번 모두 `h`는 그대로였다) | FlightController |
| 천장 눌림 `climb_stalled` | 상승 입력(rc 25 이상)을 2초 보내도 Tello `h`가 0.2 m도 안 오르면(`h` 1.2 m 위) 천장·매달린 것에 눌린 것으로 보고, 이번 비행의 상한(`ceiling_cap_m`)을 그 `h` − 0.3 m로 낮추고 1초 동안 rc −20으로 내려온다. 로그 `상승 입력(rc …)을 2초 보냈는데 Tello 높이가 … m에서 오르지 않습니다`. 착륙·이륙 때 지운다 (2026-11-09 17:07: 높이 상한 2.0 m 아래 `h` 1.9–2.0 m에서 40초 넘게 천장에 눌림) | FlightController |
| 착륙 응답 error | `land`에 Tello가 error로 답해도 하방 ToF·`h`가 둘 다 바닥이면 내려앉은 것으로 본다 (전에는 떠 있다고 믿고 land를 끝없이 보냈다) | FlightController |
| 전방 ToF 오류값 | `EXT tof?`가 20 mm 미만(`tof 0`)이면 센서 오류로 버린다 (2026-10-30: 떠 있는데 0이 4번) | tello_link |
| 명령 끊김 착륙 | 비행 중 속도 명령이 `cmd_timeout`(2초) 넘게 없으면 `land` (비행 노드가 죽은 경우) | driver_core |
| 지상 유지 신호 | 떠 있지 않으면 5초마다 `command`. 확장 키트 기체가 명령 없이 85–100초 지나면 상태를 멈췄다(2026-10-27) | driver_core |
| 노드 종료 | Ctrl+C로 드라이버를 끄면 떠 있을 때 `land` | FlightController.close |
| 비행 노드 쪽 | 드라이버가 스스로 내리면 ABORT, ABORT면 그 자리에 착륙 요청 | mission_node |

온도·배터리 기준 기본값은 `tello_link/flight_control.py`와 같고, 실기체 실행에서는 `real.launch.py`의 `temp_max`·`battery_min`으로 바꾼다.

자율비행 회차는 `manifest.json`의 `operator_controlled`가 `false`, `autonomy`가 `true`로 남는다.
기록은 `log_root`(기본 저장소 `data/outputs/tello/`, Git 제외)에 tello_link 형식으로 쌓인다.

## 시뮬레이션에서 쓰기

`simulation/tt-sim`의 `run_mission.sh`·`mission.launch.py`가 기본(`link:=tello`)으로 이 드라이버를 띄운다.
드라이버는 `127.0.0.1:28889`의 `gz_tello.py`에 붙고, 운용자 확인은 시뮬레이션이라 자동으로 켠다.

```bash
cd simulation/tt-sim
bash scripts/run_mission.sh worlds/s7_stage1             # 드라이버 → UDP → Gazebo
bash scripts/run_mission.sh worlds/s7_stage1 --direct    # 비교: 속도를 Gazebo에 바로
```

## 실기체에서 쓰기 (아직 실기체로 해 보지 않음)

ROS 2는 WSL2(Ubuntu 24.04)에서 돈다. Tello는 상태(UDP 8890)·영상(11111)을 명령을 보낸 PC로 보내는데,
WSL2 기본(NAT) 네트워크에서는 그 패킷이 Windows에서 멈춰 WSL까지 오지 않는다. 그래서 Windows에서 `tello_relay.py`가
Tello와 이야기하고 WSL 드라이버와 패킷을 주고받는다(표준 라이브러리만, 시스템 설정 변경 없음).

```text
WSL: tello_driver_node ──명령──▶ Windows:9889 tello_relay.py ──▶ Tello 192.168.10.1:8889
WSL: :8890 상태 / :11111 영상 ◀──────────────── Windows:8890 / :11111 ◀── Tello
```

가짜 Tello로 이 경로(WSL 드라이버 → Windows 중계 → 가짜 Tello)를 끝까지 확인했고, Gazebo에서 `hover_check.py`
(이륙 → 제자리 → 앞으로 → 제자리 → 착륙)를 같은 드라이버로 확인했다. 실기체 쪽 Windows 방화벽(공용 네트워크인 Tello Wi-Fi에서
Python으로 들어오는 UDP)은 아직 확인하지 못했다.

1. **준비:** 드론을 식히고(온도 80°C 미만) 완충 배터리(90% 이상 권장, 이륙 기준은 10% 초과뿐), 프로펠러 가드를 끼운다. 사람 없는 3 m × 3 m 이상, 천장 2.5 m 이상.
   조작하는 사람 말고 한 명이 옆에서 지켜본다. 폰 Tello 앱과 점검 웹 화면(`check_server.py`)은 같은 포트를 쓰므로 끈다.
2. **Wi-Fi:** 드론 전원을 켜고 PC Wi-Fi를 `TELLO-xxxxxx`(확장 키트를 달면 `RMTT-xxxxxx`)에 연결한다
   (인터넷이 끊기면 폰 USB 테더링, tello_link README).
3. **Windows (PowerShell):** 중계를 켠다. 처음에 방화벽이 Python 허용을 물으면 **개인·공용 모두 허용**한다.
   5초마다 명령·응답·상태 수가 찍힌다.

   ```powershell
   python modules\autonomy\tello_driver\tello_relay.py
   ```

4. **WSL 터미널 1:** 드라이버를 운용자 확인 없이 먼저 띄워 연결만 본다(이륙 요청은 모두 거절).
   `SDK 모드 연결`이 찍히고, 다른 터미널에서 상태가 보이면 된다. 확인 뒤 Ctrl+C.

   ```bash
   bash modules/autonomy/tello_driver/start_driver.sh
   ros2 topic echo --once /tt/state            # WSL 터미널 2
   ```

5. **제자리 시험:** 준비(1)를 다시 확인하고 운용자 확인을 켜서 드라이버를 띄운 뒤, 다른 터미널에서 시험을 돌린다.
   이륙(응답까지 약 8.6초) → 5초 제자리 → 착륙 뒤 저절로 끝난다.

   ```bash
   bash modules/autonomy/tello_driver/start_driver.sh --fly                     # WSL 터미널 1
   python3 modules/autonomy/tello_driver/hover_check.py --ros-args -p hold:=5.0  # WSL 터미널 2
   ```

6. **보정:** 같은 방법으로 `-p step_speed:=0.2 -p step_time:=2.0`(0.2 m/s로 2초, 약 0.4 m 앞으로)을 돌리고 줄자로 잰다.
   잰 거리 ÷ 0.4가 `v_full`의 배율이다(0.3 m면 `v_full`을 0.75로). 끝에 나오는 vgx 중앙값(dm/s 가정)도 함께 적는다.
   그다음 `simulation/tt-sim/autonomy/tello_state.py`의 CONVENTION(yaw 부호, 속도 기준)을 확인한다.

7. **자율비행 (웹 화면):** 드라이버·위치 추정·깊이 추정·YOLO·비행 노드·웹 화면을 한 번에 띄운다. 실기체는 탐색(카메라로 지도를 만들며
   사람 찾기) 하나만 한다(2026-11-03). 지도 파일은 쓰지 않고 켠 자리가 원점, 켠 방향이 +x다. 브라우저 http://localhost:8080 에서 점검 신호등과
   YOLO 준비를 보고 운용자 확인을 체크한 뒤 [자율비행 시작]. 영상·깊이·YOLO에 `~/venvs/tt`(PyAV·torch)와 YOLO 가중치가 필요하다.

   ```bash
   cd ~/ttrepo/simulation/tt-sim && ros2 launch launch/real.launch.py
   ```

멈추기: 드라이버 터미널에서 Ctrl+C를 누르면 착륙한다. 시험 노드만 끄면 2초 뒤 드라이버가 착륙시킨다.
위급하면 `ros2 topic pub --once /tt/flight_cmd std_msgs/String "data: emergency"`(모터 즉시 정지, 떨어진다).

WSL 미러 네트워크(`%UserProfile%\.wslconfig`에 `[wsl2]` `networkingMode=mirrored`)를 켰다면 중계 없이
`start_driver.sh --mirrored`로 Tello에 바로 붙을 수 있다(확인 안 함, Hyper-V 방화벽이 들어오는 UDP를 막을 수 있음).

## 실기체와 시뮬레이션이 아직 다른 것

- 전방 ToF: 실기체는 확장 키트 센서 한 줄(약 5Hz, 원뿔 약 25°), 시뮬레이션 기본(`tof:=direct`)은 세 줄(위·가운데·아래)을
  더 자주 낸다. 실기체와 같게 보려면 시뮬레이션을 `tof:=ext`로 돌린다. 키트 없는 Tello EDU는 `front_tof:=false`로 둔다.
- 영상은 실기체만 이 드라이버가 낸다(`video:=true`). 시뮬레이션은 Gazebo 카메라 토픽을 그대로 쓴다.
- 이륙·착륙 시간, rc → 속도 배율·지연(시뮬레이션 시상수 0.3 s)은 가정값이다. `hover_check.py` 결과로 맞춘다.
- 시간: 드라이버의 데드맨·안전 검사는 실제 시계로 돈다. 시뮬레이션이 실시간보다 아주 느려지면(RTF 0.15 미만)
  속도 명령 사이가 0.3초를 넘어 제자리 멈춤이 끼어들 수 있다.

## 시험

```bash
python3 -B -m unittest discover -s modules/autonomy/tello_driver/tests -v   # 표준 라이브러리만
```
