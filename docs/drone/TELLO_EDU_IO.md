# Tello EDU 입출력 정리

PC와 Tello EDU 사이에 오가는 데이터를 한곳에 정리한다. 자율비행·지도·탐지·백엔드·웹이 같은 이름과 단위를 쓰도록 하는 것이 목적이다.

- 기준 기체: Tello EDU 본체, `sdk?` 응답 `30`. **2026-10-27부터 RoboMaster TT 확장 키트 장착**(10장)
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
| `h` | 이륙 지점 기준 높이 | cm | 0 (비행 중 70~150) | `height_m` | **믿기 어려움.** 첫 비행에서는 이륙 직후 약 80, 1.5m에서 150으로 멈췄지만, 허병우 10/27 비행에서는 0.78m에 떠 있는데 0~10, 다른 비행에서는 60이었다. 높이 판단은 하방 `tof`를 쓴다 |
| `tof` | **하방** ToF 거리(기체 본체 센서) | cm | 10 | `tof_m`, `tof_valid`, `tof_reason` | 10 이하·6553 이상은 무효로 처리(기준은 비행 중 확인 필요). 확장 키트의 전방 ToF와 다르다(10장) |
| `bat` | 배터리 | % | 100 | `battery_pct` | 확정 |
| `templ`, `temph` | 기체 온도 최저·최고 | °C | 46~91 | `temp_low_c`, `temp_high_c` | 확정 |
| `time` | 모터 동작 시간 | s | 0 | `motor_time_s` | 지상만 확인 |
| `vgx`, `vgy`, `vgz` | 속도 | **dm/s** (SDK 3.0 문서) | 0 (rc ±30 비행 중 -2~3) | 원시값만(코드 갱신 예정) | 문서와 비행 실측(0.2~0.3m/s)이 맞는다. 부호·축은 줄자 비교로 확인 |
| `agx`, `agy`, `agz` | 가속도 | **cm/s²** (SDK 3.0 문서) | -99, -28, -1004 | 원시값만(코드 갱신 예정) | 지상 `agz` ≈ -1004 ≈ -1g(980cm/s²)로 문서와 맞는다. 축 방향은 미확정 |
| `baro` | 기압 고도 | **m** (SDK 3.0 문서) | 12.5~34.09 | 원시값만(코드 갱신 예정) | 절대값이 날마다 달라 변화량으로만 쓴다. 잡음 미측정 |
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
| `vgx·vgy·vgz` 부호·축(단위는 문서상 dm/s) | 비행 중 줄자로 잰 이동 거리·시간과 비교 |
| `baro` 잡음(단위는 문서상 m) | 정해진 높이로 상승·하강하며 `h`·`tof`와 비교 |
| `agx·agy` 축 방향 | 기체를 기울여 부호 확인 |
| ToF 무효값 기준(10, 6553) | 비행 중 다양한 높이에서 기록 |
| 15초 자동 착륙 | 이륙 후 명령 없이 대기(안전 담당 배치 후) |
| SDK 3.0 영상 설정 명령 | `setfps` 등 응답 확인. 기체 설정이 바뀌므로 승인 후 |
| 영상·상태 사이 지연 | 기체를 빠르게 회전시키며 화면과 `yaw` 변화 시각 비교 |
| 확장 키트 항목 | 10.6 참고 |

## 10. RoboMaster TT 확장 키트

2026-10-27 도착·장착. **기체·컨트롤러 펌웨어를 갱신하지 않은 상태로 바로 동작했다.** 표의 "실측"은 이날 지상에서 확인한 값이다.
(폰 Tello 앱이 최신 Android에서 지원되지 않아 기체 펌웨어는 갱신하지 못했고, 버전도 확인하지 못했다.)
DJI는 이 키트를 "TELLO EDU 또는 RoboMaster TT와 함께 쓰는 키트"로, Tello EDU를 RoboMaster TT로 바꾸는 용도로 설명한다.

### 10.1 구성과 새로 생기는 기능

| 부품 | 내용 | 새 데이터 |
| --- | --- | --- |
| 오픈소스 컨트롤러 | ESP32-D2WD(160MHz), 2.4/5.8GHz Wi-Fi, Bluetooth, 상단 컬러 LED, 14핀 확장 포트(I2C·UART·SPI·GPIO·PWM·전원) | `EXT version?`, 상단 LED |
| 도트 매트릭스 + 거리 센서 모듈 | 8×8 빨강·파랑 LED, **ToF 거리 센서**(최대 1.2m, 흰 벽 실내 기준) | **`EXT tof?`**, 매트릭스 표시 |
| 확장 보드 | 2×7 2.54mm 헤더. 외부 센서 직접 연결용 | (직접 배선) |
| Micro USB 케이블 | 컨트롤러 펌웨어 갱신·프로그래밍 | - |

**ToF가 생기는가:** 키트만 달면 **전방(앞쪽) 단일 거리값**이 하나 생긴다. 기체에 원래 있던 하방 ToF(상태 패킷 `tof`, cm)와는 다른 센서다.
따로 주문한 VL53L5CX(8×8 멀티존)나 TFmini Plus LiDAR와도 다르다. 그 센서들은 확장 보드 헤더에 직접 배선해야 하고, 이 문서의 SDK 명령으로는 읽을 수 없다.

### 10.2 연결이 어떻게 바뀌는가

| 항목 | 키트 없음 | 키트 장착 |
| --- | --- | --- |
| Wi-Fi 이름 | `TELLO-59FA6D` | **`RMTT-9B10B2`**(실측). `TELLO-` 이름은 더 이상 보이지 않는다. 비밀번호 없음 |
| 주소·포트 | `192.168.10.1`, 8889/8890/11111 | **같다**(실측: PC `192.168.10.2`, 명령·상태 정상, 상태 9.9~10Hz) |
| `wifi?` 응답 | `90` | **`0`**(실측). 컨트롤러가 AP를 맡아 기체 쪽 신호 세기가 오지 않는 것으로 보인다 |
| 모드 스위치 | - | 키트 옆 스위치: 아래 = AP(직접 연결), 위 = 스테이션(공유기). 문서·사용자 보고 기준 |
| 명령 경로 | PC → 기체 | PC → 기체 → 컨트롤러(`EXT ...` 명령만 컨트롤러로 전달) |

`tello_link`의 네트워크 점검은 SSID가 아니라 PC 주소(192.168.10.x)로 판단하므로, 이름이 바뀌어도 그대로 동작해야 한다.

### 10.3 컨트롤러 명령 (UDP 8889, 응답은 같은 소켓)

`EXT` 명령은 **공장 펌웨어**에서만 지원된다(SDK 3.0 문서). 컨트롤러에 직접 만든 펌웨어를 올리면 이 명령은 동작하지 않는다.

| 명령 | 내용 | 응답 | `tello_link` 분류 |
| --- | --- | --- | --- |
| `EXT tof?` | 전방 ToF 거리 | `tof xxxx`, **단위 mm**. 범위 밖은 문서상 `tof 8192`지만 **실측은 `tof 8190`·`tof 8191`**. 드물게 `unknown command: tof` | query |
| `EXT version?` | 컨트롤러 펌웨어 버전 | 문서 `esp32vx.x.x.x`, **실측 `version esp32v1.0.0.12`**(앞에 `version `이 붙는다) | query |
| `EXT led r g b` | 상단 LED 색(각 0~255) | `led ok` / `error` | ext(표시용, 허용) |
| `EXT led br t r g b` | LED 숨쉬기 효과, t 0.1~2.5Hz | `led ok` / `error` | ext |
| `EXT led bl t r1 g1 b1 r2 g2 b2` | 두 색 깜빡임, t 0.1~10Hz | `led ok` / `error` | ext |
| `EXT mled g xxxx` | 매트릭스에 패턴 표시(`r`·`b`·`p`·`0` 최대 64자) | `mled ok` / `error` | ext |
| `EXT mled l/r/u/d r/b/p t 문자열` | 문자열 흘리기(70자 이내) | `mled ok` / `error` | ext |
| `EXT mled s r/b/p 문자` | 고정 문자 또는 `heart` | `mled ok` / `error` | ext |
| `EXT mled sl n` | 매트릭스 밝기 0~255 | `mled ok` / `error` | ext |
| `EXT mled sg xxxx` / `EXT mled sc` | 부팅 애니메이션 설정 / 지우기(**기체에 저장됨**) | `mled ok` / `error` | config(기본 차단) |

전방 ToF는 **상태 패킷에 들어오지 않는다.** 필요할 때마다 `EXT tof?`로 물어봐야 하고, 이 명령도 다른 명령처럼 한 번에 하나씩 응답을 기다린다.

#### 거리별 정확도 (2026-10-27, 지상, 거리마다 10Hz로 30회, 회차 `tello_20261027_100350_103037`)

거리는 확장 키트 앞쪽 센서 면에서 평평한 물체까지 줄자로 잰 값이다.

| 실제 거리 | 측정 중앙값 | 오차 | 표준편차 | 유효 |
| --- | --- | --- | --- | --- |
| 3cm | 28mm | -2mm | 1.5mm | 30/30 |
| 10cm | 96mm | -4mm | 0.5mm | 30/30 |
| 30cm | 303mm | +3mm | 0.7mm | 30/30 |
| 60cm | 597mm | -3mm | 1.8mm | 30/30 |
| 1.0m | 939mm | **-61mm** | 4.4mm | 28/30 (`unknown command: tof` 2회) |
| 1.2m | 1139mm | **-61mm** | 5.4mm | 30/30 |
| 1.5m | `8190` | 범위 밖 | - | - |
| 앞 비움(2m 이상) | `8190`·`8191` | 범위 밖 | - | - |

- **0.6m까지 ±4mm**로 정확하고, 3cm까지 가까워도 값이 나온다.
- 1.0m·1.2m가 똑같이 약 6cm 짧게 나왔다. 센서 특성인지 측정 배치(줄자 기준점, 물체 기울기) 때문인지는 **재확인이 필요**하다.
- 1.2m는 잡히고 1.5m는 범위 밖이다. 문서의 최대 1.2m와 맞는다.
- 이 단계의 왕복 시간은 중앙 8.9ms, 최대 160ms였다.

#### 실시간 반영 (2026-10-27, 15초 동안 가능한 한 빠르게 조회하며 손·상자를 앞뒤로 움직임, `tof_live_20261027.csv`)

| 항목 | 값 |
| --- | --- |
| 조회 속도 | 1,016회 / 15초 = 초당 68회 |
| 왕복 시간 | 중앙 9.9ms, 95% 39.9ms, 최대 507ms |
| **값이 바뀌는 간격** | **중앙 약 195ms(190~200ms에 몰림) → 센서 갱신 약 5Hz** |
| 같은 값이 연속으로 온 횟수 | 대부분 10회 이상 |
| 무효 | 범위 밖 741회(물체를 1.2m 밖으로 뺀 구간), `unknown command` 1회, 응답 없음 1회 |

- **센서 값은 약 0.2초마다 새로 바뀐다.** 5Hz보다 빨리 물어도 같은 값이 반복된다. 0.2초 동안 0.3m/s로 날면 6cm를 간다.
- 따라서 전방 거리가 실제 상황을 반영하기까지 **최대 약 0.2초 + 왕복 시간**이 걸린다고 보고, 장애물 정지 거리에 이만큼 여유를 둔다.
- 조회는 **5~10Hz면 충분하다.** 더 자주 물으면 rc(20Hz)와 같은 소켓을 나눠 쓰는 부담만 커진다.
- 이전 정적 시험(0.39m, 50회 연속 조회, 회차 `tello_20261027_093400_1d8d23`)은 중앙 390mm, 표준편차 0.5mm, 무효 0회였다.

`tello_link` 처리 규칙(구현됨):
- **8190 이상**(실측 8190·8191, 문서 8192)은 0이 아니라 `valid=false, reason="out_of_range"`로 남긴다.
- `unknown command: tof`는 `reason="unknown_command"`, `error`는 `error_response`, 응답 없음은 `no_response`로 남긴다.
- SI 출력 이름은 `front_tof_m`으로 하고, 하방 `tof_m`과 섞지 않는다(`parse_ext_tof()`).
- 점검 화면의 "확장 키트 (선택)" 단계가 `EXT version?`·`EXT tof?`를 묻고 결과를 보여 준다.

### 10.4 SDK 3.0에 새로 있는 기체 명령(참고)

| 명령 | 내용 | 우리 분류 |
| --- | --- | --- |
| `motoron` / `motoroff` | 프로펠러를 저속으로 돌리는 대기 상태. 문서상 **지상 대기 중 과열로 꺼지는 것을 막는 용도** | flight(차단). 과열 대책으로 검토 가치 있음 |
| `throwfly` | 5초 안에 던져서 이륙 | flight(차단) |
| `reboot` | 기체 재부팅 | flight(차단) |
| `port info video`, `setfps`, `setbitrate`, `setresolution`, `downvision`, `multiwifi` | 포트·영상 설정, 하방 카메라 전환, 컨트롤러 Wi-Fi 설정 | config(차단) |

### 10.5 펌웨어

| 대상 | 방법 | 기록할 것 |
| --- | --- | --- |
| 기체(Tello EDU) | 폰 Tello 앱으로 연결 → 펌웨어 업데이트 | **갱신 못 함.** 앱이 최신 Android에서 지원되지 않고, 상자 QR은 Play 스토어 밖 설치 경고가 떠서 쓰지 않았다. `sdk?`=30이고 `EXT` 명령이 동작하므로 현재 펌웨어로 충분하다 |
| 오픈소스 컨트롤러(ESP32) | PC의 RoboMaster Assistant + Micro USB | 갱신하지 않음. 현재 `esp32v1.0.0.12`(공장 펌웨어로 `EXT` 명령 동작) |

### 10.6 장착 후 확인할 것

| 항목 | 확인 방법 |
| --- | --- |
| ~~전방 ToF 범위·잡음~~ | 확인함(10.3 거리별 정확도). 1m 이상 약 6cm 짧은 것은 재확인 |
| ~~최소 거리와 범위 밖 값~~ | 확인함: 3cm에서도 유효, 범위 밖은 8190·8191 |
| ~~조회 가능 주기~~ | 확인함: 센서 갱신 약 5Hz, 조회는 5~10Hz 권장 |
| 비행 중 전방 ToF | rc 20Hz와 함께 5~10Hz로 물을 때 응답 지연·무효 비율 |
| 물체 색·재질 영향 | 어두운 천, 유리, 좁은 기둥(B-05 시나리오)에서 범위·무효 비율 |
| 무게 증가에 따른 비행 변화 | 이륙 응답 시간, 제자리 높이 흔들림, 배터리 소모를 키트 전후 비교 |
| `motoron` 대기 시 온도 | 지상 대기 5분 동안 `temph` 비교(안전 담당 배치 후) |

## 출처

- Tello SDK 2.0 User Guide, [Tello SDK 3.0 User Guide(RoboMaster TT, 2021)](https://xcopter.com/download/Tello_SDK_3.0_User_Guide_en.pdf), Ryze Robotics. 상태 필드 단위, `EXT` 명령, `motoron`
- [DJI Store: RoboMaster TT Expansion Kit](https://store.dji.com/product/robomaster-tt-expansion-kit) 구성, ToF 최대 1.2m, ESP32 사양, Tello EDU 호환
- TelloPilots 사용자 보고: [EDU + 키트 ToF 응답 `tof 481`, 펌웨어 v02.05.01.19](https://tellopilots.com/threads/rm-tt-extension-kit-on-edu-tof-good-value-received-but-djitellopy-api-doesnt-take-the-answer-into-account.6704), [RMTT SSID와 모드 스위치](https://tellopilots.com/threads/guide-connecting-multiple-robomaster-tts-to-a-wi-fi-router-with-auto-connect.6580/)
- [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) `tello.py`의 포트·대기 시간·상태 필드 설명(MIT)
- 우리 기체 실측: `modules/autonomy/tello_link/README.md`의 "실기체 확인 결과"
