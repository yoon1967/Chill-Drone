# Tello EDU 입출력 정리

PC와 Tello EDU 사이에 오가는 데이터를 한곳에 정리한다. 자율비행·지도·탐지·백엔드·웹이 같은 이름과 단위를 쓰도록 하는 것이 목적이다.

- 기준 기체: Tello EDU 본체(확장 키트·외부 센서 없음), `sdk?` 응답 `30`
- 실측: 2026-10-23, Windows 11 PC. 연결 점검은 지상, 수동 비행 2회(실내, 운용자 조작, rc ±30)
- 구현: `modules/autonomy/tello_link/` ([MR !23](https://lab.ssafy.com/s15-final/S15P31B205/-/merge_requests/23), S15P31B205-156)
- 표기: **실측**은 우리 기체로 확인한 값, **문서**는 Tello SDK 문서·널리 쓰는 라이브러리 기준이며 아직 확인하지 않은 값이다.

## 1. 한눈에 보기

~~~mermaid
flowchart LR
  subgraph PC["PC (192.168.10.x)"]
    LINK["tello_link<br/>명령·상태·로그"]
    VID["tello_video<br/>H.264 디코딩"]
  end
  subgraph DRONE["Tello EDU (192.168.10.1)"]
    SDK["SDK"]
  end
  LINK -- "명령 텍스트 UDP → 8889" --> SDK
  SDK -- "응답 텍스트 (같은 소켓)" --> LINK
  SDK -- "상태 텍스트 약 10Hz → PC:8890" --> LINK
  SDK -- "H.264 → PC:11111 (streamon 이후)" --> VID
  LINK --> OUT["회차 로그<br/>manifest · commands · sensor · events · metrics"]
  LINK --> AUTO["자율비행 드라이버 (-207)"]
  VID --> PERC["탐지 영상 입력 (-170)"]
  VID --> MAP["지도 입력 (-228)"]
  LINK --> WEB["점검·모니터링 화면"]
~~~

| 채널 | 방향 | 포트 | 형식 | 주기 | 실측 |
| --- | --- | --- | --- | --- | --- |
| 명령 | PC → 드론 | 드론 UDP 8889 | ASCII 텍스트 한 줄 | 요청할 때 | 왕복 16~47ms |
| 응답 | 드론 → PC | 명령을 보낸 PC 소켓 | `ok`, `error...`, 값 | 명령마다 1개(rc 제외) | 확인 |
| 상태 | 드론 → PC | PC UDP 8890 | `키:값;` 텍스트 | 약 10Hz | 9.9~10.0Hz |
| 영상 | 드론 → PC | PC UDP 11111 | H.264(Annex B) 조각 | 약 24~30fps | 960×720, 약 24fps |

## 2. 연결

| 항목 | 내용 |
| --- | --- |
| 기본 모드 | 드론이 AP가 된다. SSID `TELLO-xxxxxx`(우리 기체 `TELLO-59FA6D`), 비밀번호 없음, 2.4GHz |
| 주소 | 드론 `192.168.10.1`, PC는 DHCP로 `192.168.10.x`를 받는다(실측 `.2`) |
| 동시 연결 | 폰 앱과 PC를 동시에 쓰지 않는다. 앱을 끄고 폰의 Tello Wi-Fi 연결도 끊는다 |
| 인터넷 | Tello Wi-Fi에는 인터넷이 없다. 유선 랜이나 폰 USB 테더링을 같이 쓰고, 테더링 쪽 인터페이스 우선순위를 높인다 |
| 방화벽 | Tello Wi-Fi는 Windows에서 "공용 네트워크"로 잡힌다. Python의 UDP 8890·11111 수신을 허용해야 상태·영상이 들어온다 |
| 스테이션 모드 | `ap <ssid> <pw>`로 드론을 공유기에 붙일 수 있다(문서). 기체에 저장되므로 기본 차단하고, 되돌리려면 전원 버튼 5초 초기화가 필요하다 |

## 3. 명령 채널 (UDP 8889)

### 규칙

- 첫 명령은 반드시 `command`다. 이후 SDK 모드로 동작하고 상태 패킷을 보내기 시작한다(실측).
- 한 번에 한 명령만 보내고 응답을 기다린다. 응답 대기 시간은 기본 7초로 둔다(조회는 3초).
- 늦게 온 이전 응답이 다음 명령의 응답으로 섞이지 않게, 보내기 전에 수신 버퍼를 비운다.
- `rc`는 응답이 없다. 연속으로 보내는 조종 입력이다.
- 비행 중 15초 동안 명령이 없으면 드론이 스스로 착륙한다(문서). 비행 중에는 rc나 조회로 계속 신호를 보낸다.
- 응답이 없으면 실패로 보고 `valid=false, reason="timeout"`으로 기록한다. 0이나 빈 값으로 바꾸지 않는다.

### 명령 목록과 우리 모듈의 분류

`tello_link`는 명령을 다섯 종류로 나누고, flight·config는 기본으로 보내지 않는다.

| 분류 | 명령 | 인자 범위(문서) | 응답 | 기본 | 실측 |
| --- | --- | --- | --- | --- | --- |
| session | `command` | - | `ok` | 허용 | ✅ |
| session | `streamon` / `streamoff` | - | `ok` | 허용 | ✅ |
| query | `battery?` | - | 0~100 (%) | 허용 | ✅ `100`, `90` |
| query | `sdk?` | - | SDK 버전 | 허용 | ✅ `30` |
| query | `sn?` | - | 시리얼 번호 | 허용 | ✅ |
| query | `wifi?` | - | 신호 세기(SNR) | 허용 | ✅ `90` |
| query | `speed?`, `time?` | - | 설정 속도(cm/s), 비행 시간 | 허용 | 미확인 |
| query | `height?`, `temp?`, `attitude?`, `baro?`, `acceleration?`, `tof?` | - | 각 값(SDK 1.3 문서) | 허용 | 미확인 |
| stop | `land` | - | `ok` | 항상 허용 | ✅ 응답까지 3.2~4.6초(착륙 완료 후 `ok`) |
| stop | `emergency` | - | `ok` | 항상 허용 | 미확인. **모터 즉시 정지, 공중이면 추락** |
| stop | `stop` | - | `ok` | 항상 허용 | 미확인. 제자리 정지 |
| flight | `takeoff` | - | `ok` | 차단(`--flight`에서 점검 후 허용) | ✅ 응답까지 8.6~8.7초, 약 80cm에서 정지 |
| flight | `up/down/left/right/forward/back x` | x: 20~500 (cm) | `ok` | 차단 | - |
| flight | `cw/ccw x` | x: 1~360 (°) | `ok` | 차단 | - |
| flight | `go x y z speed` | x·y·z: -500~500 (cm), speed: 10~100 (cm/s) | `ok` | 차단 | - |
| flight | `curve ...`, `flip x`, `jump ...` | 문서 참고 | `ok` | 차단 | - |
| flight | `rc a b c d` | 각 -100~100 (좌우·앞뒤·상하·회전) | **없음** | 차단 | 차단 확인 |
| config | `speed x` | 10~100 (cm/s) | `ok` | 차단 | - |
| config | `ap`, `wifi`, `multiwifi`, `port`, `mon`, `moff`, `mdirection` | 문서 참고 | `ok` | 차단 | - |
| config | `setfps`, `setbitrate`, `setresolution`, `downvision` | SDK 3.0 문서 | `ok` | 차단 | 우리 기체 지원 여부 미확인 |

`rc` 값은 조종기 입력 비율이지 속도가 아니다. 실제 속도와의 관계는 비행 응답 측정으로 따로 구한다.
실측: rc ±30을 누르는 동안 `vgx`·`vgy`는 -2~3 범위였다(줄자 미측정).

`takeoff`·`land`는 동작이 끝난 뒤 `ok`가 온다. 이 동안 다른 명령은 응답을 기다리느라 늦어지므로, `tello_link`는 `emergency`만 잠금 없이 바로 보낸다.

## 4. 상태 채널 (UDP 8890)

드론은 `command` 이후 PC의 8890 포트로 상태 한 줄을 약 10Hz로 보낸다. 실측 원문(지상, 전원 켠 직후):

```text
mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:-8;roll:1;yaw:54;vgx:0;vgy:0;vgz:0;templ:62;temph:65;tof:10;h:0;bat:100;baro:34.09;time:0;agx:-99.00;agy:-28.00;agz:-1004.00;
```

| 필드 | 뜻 | 원시 단위 | 지상 실측 | `tello_link` SI 출력 | 상태 |
| --- | --- | --- | --- | --- | --- |
| `pitch`, `roll`, `yaw` | 자세 | ° (정수) | -8, 1, 54 | `pitch_rad`, `roll_rad`, `yaw_rad` | 단위 확정 |
| `h` | 이륙 지점 기준 높이 | cm | 0 (비행 중 70~150) | `height_m` | 확정. 이륙 직후 약 80, 높이 상한 1.5m에서 150으로 멈춤 |
| `tof` | 하방 ToF 거리 | cm | 10 | `tof_m`, `tof_valid`, `tof_reason` | 10 이하·6553 이상은 무효로 처리(기준은 비행 중 확인 필요) |
| `bat` | 배터리 | % | 100 | `battery_pct` | 확정 |
| `templ`, `temph` | 기체 온도 최저·최고 | °C | 46~91 | `temp_low_c`, `temp_high_c` | 확정 |
| `time` | 모터 동작 시간 | s | 0 | `motor_time_s` | 지상만 확인 |
| `vgx`, `vgy`, `vgz` | 속도 | 문서마다 다름(dm/s 또는 cm/s) | 0 (rc ±30 비행 중 -2~3) | 원시값만 | **dm/s로 추정**, 줄자 비교 전까지 미확정 |
| `agx`, `agy`, `agz` | 가속도 | 0.001g로 추정 | -99, -28, -1004 | 원시값만 | 지상에서 `agz` ≈ -1g. 축 방향은 미확정 |
| `baro` | 기압 고도 | m로 추정(djitellopy는 ×100으로 cm 변환) | 12.5~34.09 | 원시값만 | **미확정**, 절대값보다 변화량으로 써야 함 |
| `mid` | Mission Pad ID | - | -1 (인식 안 됨) | - | Pad 미사용 |
| `x`, `y`, `z`, `mpry` | Pad 기준 위치·자세 | cm, ° | -100, 0,0,0 | - | Pad 인식 시에만 의미 있음 |

처리 규칙:

- 형식이 깨지거나 필수 필드(`pitch roll yaw h bat tof templ temph`)가 없으면 `valid=false`와 `reasons`를 남긴다.
- 상태에는 드론 시각이 없다. PC 수신 시각을 `received_at`으로 쓰고 `time_source="server_receive"`로 표시한다.
- 패킷이 2초 이상 끊기면 연결 이상으로 본다(점검 화면 기준).

### 온도 실측

| 상황 | 온도(temph) |
| --- | --- |
| 10분 이상 식힌 뒤 켬 | 48°C |
| 위 상태에서 영상 켜고 약 1분 반 | 64°C |
| 잠깐 끈 뒤 바로 다시 켬 | 88~91°C |

지상에서 켜 두기만 해도 온도가 오른다. 점검·대기 시간을 짧게 하고, 75°C 이상이면 경고, 85°C 이상이면 끄도록 안내한다(가정 임계값).

## 5. 영상 채널 (UDP 11111)

| 항목 | 내용 |
| --- | --- |
| 시작·종료 | `streamon` / `streamoff` 명령. 응답은 `ok` |
| 형식 | H.264 Annex B 원시 스트림. 컨테이너·타임스탬프 없음 |
| 패킷 | UDP 조각으로 온다(문서상 1460바이트 단위) |
| 해상도·속도 | 960×720, 약 24fps(실측 1분 평균) |
| 디코딩 | PyAV(`h264`)로 바이트를 직접 파싱·디코딩. 첫 키프레임 전에는 프레임이 없을 수 있다 |
| 시각 | 드론 시각이 없으므로 PC 수신 시각을 쓴다(`time_source="server_receive"`) |
| 보관 | 최신 1장만 유지한다. 프레임을 쌓지 않는다 |

주의: 같은 프레임을 여러 스레드에서 동시에 변환하면 PyAV가 프로세스를 깨뜨린다(실측·재현). `TelloVideo.jpeg()`는 한 번에 하나만 변환한다.

## 6. 우리 쪽 출력 (`tello_link`)

### 명령 결과 `CommandResult`

| 필드 | 타입 | 뜻 |
| --- | --- | --- |
| `cmd` | str | 보낸 명령 |
| `category` | str | query / session / stop / flight / config |
| `sent_at` | ISO 8601 UTC | 보낸 시각(차단되면 null) |
| `response` | str 또는 null | 드론 응답 원문 |
| `ok` | bool | 성공 여부(query는 값이 오고 `error`가 아니면 성공) |
| `valid` | bool | 응답을 받았는지 |
| `reason` | str 또는 null | `timeout`, `connection_reset`, `bad_packet`, `error_response`, `flight_blocked`, `config_blocked` |
| `rtt_ms` | float 또는 null | 왕복 시간 |
| `blocked` | bool | 차단돼서 보내지 않았는지 |

### 상태 `latest_state()`

| 필드 | 뜻 |
| --- | --- |
| `received_at`, `time_source`, `age_s` | 수신 시각, 시각 출처, 지금까지 지난 시간 |
| `raw` | SDK 원래 이름과 값(4장 표) |
| `si` | 단위가 확실한 값만 m·rad·°C·%·s로 변환, `unverified_unit_fields` 포함 |
| `valid`, `reasons` | 패킷 유효성 |

### 회차 로그 `data/outputs/tello/<run_id>/`

| 파일 | 한 줄 단위 | 내용 |
| --- | --- | --- |
| `manifest.json` | - | 회차 ID, 출처(tello/fake), 포트, 차단 설정, 단위, Jira 키 |
| `commands.jsonl` | 명령 1개 | `CommandResult` + `recorded_at`. 차단된 명령도 남긴다 |
| `sensor.jsonl` | 상태 패킷 1개 | `raw`, `si`, `valid`, `received_at` |
| `events.jsonl` | 사건 1개 | 연결 시작·종료, 점검, 영상 켜기·끄기, 버린 응답 |
| `metrics.json` | - | 종료 시 명령 수·차단 수·타임아웃 수·왕복 시간·상태 패킷 수 |

pose는 아직 만들지 않는다. 회차 로그는 Git에 올리지 않는다(`/data/outputs/` 제외).

## 7. 파트별 사용처

| 파트 | 받는 것 | 관련 Jira | 확인할 것 |
| --- | --- | --- | --- |
| 자율비행 | `TelloLink.send()`, `rc()`, `latest_state()` | -207, -208, -163 | -208에서 쓰는 상태 필드 이름·단위, WSL2에서의 UDP 수신 방식, rc 타임아웃 정지 위치(드라이버 노드) |
| 지도 | 영상 프레임 + 상태(자세·높이) | -228 | 영상과 상태 모두 PC 수신 시각이므로 둘 사이 지연을 따로 재야 함 |
| 탐지 | 영상 프레임(`/snapshot.jpg`, `/video.mjpg` 형태) | -170 | 최신 1장 방식으로 충분한지 |
| 백엔드 | 상태 요약(배터리·높이·온도·유효성) | -179 | 서버로 보낼 주기와 필드 |
| 웹 | 점검 화면(로컬), 이후 모니터링 화면 | - | 드론 상태를 팀 웹에 띄울 경로 |

## 8. 지켜야 할 규칙

- 지도·평가는 m·rad를 쓴다. cm·°는 Tello SDK 경계(`tello_link`)에서만 다룬다.
- 범위 밖·타임아웃·불량 패킷은 `valid=false`와 이유로 남기고 0으로 바꾸지 않는다.
- 단위를 모르는 값은 원시값만 두고 지어내지 않는다.
- `rc` 입력값을 실측 속도로 쓰지 않는다.
- `emergency`(모터 즉시 정지)와 `land`(착륙)를 구분한다. 운용자가 개입해 끝난 회차는 자율 성공으로 세지 않는다.
- 시뮬레이터용 클라이언트의 IP만 바꿔 실기체에 쓰지 않는다.

## 9. 아직 확인하지 않은 것

| 항목 | 확인 방법 |
| --- | --- |
| `vgx·vgy·vgz` 단위와 부호 | 비행 중 줄자로 잰 이동 거리·시간과 비교 |
| `baro` 단위와 잡음 | 정해진 높이로 상승·하강하며 `h`·`tof`와 비교 |
| `agx·agy` 축 방향 | 기체를 기울여 부호 확인 |
| ToF 무효값 기준(10, 6553) | 비행 중 다양한 높이에서 기록 |
| 15초 자동 착륙 | 이륙 후 명령 없이 대기(안전 담당 배치 후) |
| SDK 3.0 영상 설정 명령 | `setfps` 등 응답 확인. 기체 설정이 바뀌므로 승인 후 |
| 영상·상태 사이 지연 | 기체를 빠르게 회전시키며 화면과 `yaw` 변화 시각 비교 |

## 출처

- Tello SDK 2.0 User Guide, Tello SDK 3.0 User Guide(RoboMaster TT), Ryze Robotics
- [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) `tello.py`의 포트·대기 시간·상태 필드 설명(MIT)
- 우리 기체 실측: `modules/autonomy/tello_link/README.md`의 "실기체 확인 결과"
