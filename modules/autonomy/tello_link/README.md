# Tello EDU 연결 모듈과 연결 점검 화면

Jira: [S15P31B205-156](https://ssafy.atlassian.net/browse/S15P31B205-156) 실제 Tello EDU 제어 연동.
PC에서 Wi-Fi로 Tello EDU에 SDK 명령을 보내고, 상태·영상을 받는 부분이다.
**이 단계는 이륙하지 않는다.** 연결만 확인하고, 이 모듈을 자율비행 쪽([S15P31B205-207](https://ssafy.atlassian.net/browse/S15P31B205-207) ROS 2 드라이버)에 넘긴다.

| 파일 | 역할 | 의존성 |
| --- | --- | --- |
| `tello_link.py` | 명령 송수신, 상태 파싱, 명령 차단, 회차 로그 | 표준 라이브러리 |
| `tello_video.py` | UDP 11111 H.264 수신·디코딩, 최신 1장 유지 | PyAV, Pillow |
| `check_server.py` + `static/index.html` | 연결 점검 웹 화면 | FastAPI, uvicorn |
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

Tello는 지상에서 켜 둔 채로 두면 과열로 꺼진다. 기체 온도가 75°C 이상이면 경고하고, 85°C 이상이면 전원을 끄라고 안내한다(실측 전 가정값).

## 안전 장치

`tello_link`는 명령을 다섯 종류로 나눠서 막는다. 점검 화면에는 차단을 푸는 옵션이 없다.

| 분류 | 예 | 기본 |
| --- | --- | --- |
| query | `battery?`, `sdk?`, `sn?`, `wifi?`, `tof?` | 허용 |
| session | `command`, `streamon`, `streamoff` | 허용 |
| stop | `land`, `emergency`, `stop` | 항상 허용 |
| flight | `takeoff`, `forward 50`, `rc ...`, `go ...` | `allow_flight=True`일 때만 |
| config | `ap ...`, `wifi ...`, `speed 50`, `mon` | `allow_config=True`일 때만 |

차단된 명령은 드론으로 보내지 않고, `blocked=true`로 기록만 남긴다.
`emergency`는 모터를 즉시 멈춘다(공중이면 추락). `land`와 다르다.

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
- **rc 타임아웃 정지**(명령이 끊기면 `rc 0 0 0 0`)는 -207 드라이버 노드에서 구현한다. 이 모듈은 보내기만 한다.

## 회차 로그

`log_root/<run_id>/`에 남는다. 기본 위치 `data/outputs/tello/`는 Git에서 제외된다.

| 파일 | 내용 |
| --- | --- |
| `manifest.json` | 회차 ID, 출처(tello/fake), 포트, 차단 설정, 단위 |
| `commands.jsonl` | 보낸 명령·응답·왕복 시간, 차단된 명령 포함 |
| `sensor.jsonl` | 상태 패킷 전체(약 10Hz), 원시값과 SI 값 |
| `events.jsonl` | 연결 시작·종료, 점검 실행, 영상 켜기·끄기, 버린 응답 |
| `metrics.json` | 종료 시 명령 수, 차단 수, 타임아웃 수, 왕복 시간 중앙값·최대값, 상태 패킷 수 |

pose는 만들지 않는다(`manifest.files.pose = null`).

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

## 아직 확인하지 않은 것

- 속도(vgx·vgy·vgz)와 기압(baro) 단위. 비행해야 확인할 수 있다.
- ToF 무효값 기준(10, 6553)이 비행 중에도 맞는지, 과열 임계값(75·85°C)이 적절한지.
- WSL2(ROS 2)에서 Tello UDP 수신 여부. 기본 NAT 모드에서는 8890·11111 수신이 안 될 수 있다.

## 라이선스

FastAPI(MIT), uvicorn(BSD-3), PyAV(BSD-3), Pillow(MIT-CMU), numpy(BSD-3).
PyAV 휠에는 FFmpeg 바이너리가 들어 있고, 빌드 구성에 따라 GPL 구성 요소(libx264 등)가 포함될 수 있다.
이 저장소는 휠을 재배포하지 않고 의존성으로만 선언한다. 바이너리를 묶어 배포할 때는 다시 확인한다.
