# 저장 지도·실시간 세션 조회 1.0

생산자: `apps/console/serve.py`, `modules/mapping/localization/orbslam3_live.py`.
소비자: `apps/console/`, `tests/integration/replay_live_slam.py`.
기존 `phone-sfm-result/1.2`, `orb-map-timeline/1.0` 파일 형식은 유지한다.
아래 HTTP 경로와 응답 필드는 두 입력을 동시에 읽는 소비자가 사용한다.

## 고정 조회 경로

| 저장 결과 | 실시간 세션 | 응답 |
| --- | --- | --- |
| `/api/saved/state` | `/api/live/state` | 입력 종류·세션·지도 버전·좌표계·단위·처리 상태 |
| `/api/saved/run` | `/api/live/run` | 선택된 지도 전체. `source=saved` 또는 `live` |
| `/api/saved/timeline` | `/api/live/timeline` | 저장 결과는 빈 목록, 실시간은 실제 지도 생성 이력 |
| `/api/saved/frame/NAME` | `/api/live/frame/NAME` | 해당 지도에 등록된 촬영 프레임만 제공 |
| `/api/saved/object-texture/NAME` | `/api/live/object-texture/NAME` | 해당 지도에 결합된 평면 객체 질감만 제공 |
| `/api/saved/points.ply`, `poses.json`, `trajectory.csv` | 같은 이름의 `/api/live/…` | 선택 결과에 실제 파일이 있을 때만 제공 |
| 해당 없음 | `/api/live/slam-preview` | 현재 실시간 세션의 특징점 미리보기 |

실시간 입력을 아직 시작하지 않았으면 live state는 `status=idle`,
`session_status=idle`, `session_id=null`, `map_available=false`, `revision=0`이다.
이때 live run은 503이며 저장 지도를 대신 반환하지 않는다. 해당 프레임·미리보기는 404다.
저장 결과가 설정되지 않았으면 saved state의 `map_available`도 false이며 saved run은 503이다.
파일을 읽지 못하면 503 `source_unavailable`과 해당 `source`를 반환한다. Windows의 일시적
파일 공유 위반은 짧게 재시도하고, 지속되면 다른 입력의 지도나 상태로 대체하지 않는다.

저장 결과는 `source=saved`, `status=offline`, `session_status=null`,
`session_id=null`, `revision=0`이다. 점군 응답에 `stream`을 붙이지 않는다.
저장 지도의 실측 보정 단위와 실시간 단안 지도의 임의 단위를 서로 복사하지 않는다.

## 상태와 지도 버전

state의 `schema_version`은 `mapping-source-state/1.0`이다. 주요 필드:

| 필드 | 의미 |
| --- | --- |
| `source` | `saved` 또는 `live` |
| `session_id` | 현재 선택한 온라인 세션 ID. 시작할 때마다 새 값 |
| `revision` | 해당 세션 내부의 마지막 유효 지도 버전. 다른 세션과 비교하지 않음 |
| `segment_id` | 해당 지도 안의 좌표계 구간. 값이 바뀌면 이전 점군에 덧붙이지 않고 교체 |
| `coordinate_system`, `scale` | 해당 지도의 원본/표시 좌표계와 단위. 아직 알 수 없으면 null |
| `map_available` | 읽을 수 있는 유효 지도 존재 여부 |
| `session_status` | 브라우저 입력의 `idle/starting/receiving/stopping/complete/failed` |
| `mapping_state`, `tracking_state` | 지도 계산 및 ORB 추적 상태. 초기화·추적 상실·정상 추적 구분 |
| `last_input_sequence`, `last_input_timestamp_s` | 마지막으로 받은 입력 순번과 media 시각. 수신 전은 null |

마지막 입력 순번·시각은 새 브라우저 세션에서 제공하며 이전 기록에는 없을 수 있다.

ORB 세션은 지도 초기화 전에도 `scale.unit=arbitrary`와 표시축 변환을 제공한다.
실측 기준 없는 단안 지도는 미터 단위로 바꾸지 않는다. 다른 온라인 백엔드에서 만들어진
기존 세션은 원래 status 값을 유지하며 해당 scene의 좌표계·단위를 따른다.

소비자는 state를 읽은 뒤 다음과 같이 **세션과 revision을 고정**해 지도를 요청한다.

```text
GET /api/live/run?session=<state.session_id>&revision=<6자리 state.revision>
GET /api/live/frame/<frame_id>?session=<같은 session_id>&revision=<같은 revision>
GET /api/live/timeline?session=<같은 session_id>
```

새 세션으로 바뀌었으면 이전 session ID 요청은 409 `session_changed`다. 소비자는 최신
state부터 다시 읽는다. 한 HTTP 요청은 하나의 세션 참조를 고정해 파일과 전송 통계를 읽는다.
이전 revision을 요청하면 scene의 `revision`은 요청한 버전이고, `stream.revision`은 현재
마지막 버전이다. 현재 stream 메타데이터와 과거 scene 메타데이터를 서로 덮어쓰지 않는다.
revision은 6자리 숫자만 허용하며 저장 지도에 revision을 지정하면 400이다.

## 시작·중단·재시작

기존 `GET /api/live`는 지원 여부·시연 설정을 반환한다. 기존 POST 경로와 입력은
[ORB 계약](ORB_SLAM3_V1.md#브라우저-카메라-전송)을 사용한다.

- start 응답(201)은 새 `session_id`, `source=live`, `session_status`와 전송 통계를 반환한다.
- frame 응답(202)은 같은 ID와 마지막 입력 순번·시각을 포함한다. 순번/시각은 단조 증가다.
- stop 응답(202)은 요청한 ID와 현재 `session_status`를 반환한다. 실제 종료는 state로 확인한다.
- 실행 중에는 다른 start를 409로 거부한다. 종료 후 재시작은 새 디렉터리·ID·좌표계를 사용한다.
- 이전 ID의 frame/stop 요청은 409 `camera_session_mismatch`이며 새 세션에 적용되지 않는다.
- 종료된 세션의 마지막 유효 지도는 새 시작 전까지 조회할 수 있고 파일도 보존한다.
- 정상 수신 후 사용자가 중단했거나 입력 시간 초과로 종료됐어도 지도 초기화가 안 됐으면
  `session_status=complete`, `map_available=false`, `revision=0`일 수 있다. `complete`는 입력 처리의
  종료이며 3D 생성 성공을 뜻하지 않는다. 추적기 오류·예상치 못한 종료는 `failed`다.
- 새 시작 이후 이전 ID를 임의 디렉터리 조회에 사용하지 않는다. 기록을 다시 열려면 서버의
  `--session`에 명시적으로 지정한다.

## 실행과 이전 경로

### Tello 카메라 입력 (추가)

`GET /api/live`에 `tello_map_enabled`와 `tello_map_mode=direct|gateway|null`을 추가한다.
기존 브라우저 start/frame/stop 및 saved/live 조회 계약은 유지한다.

| 경로 | 입력·응답 |
| --- | --- |
| `POST /api/live/tello/start` | JSON `{fov, camera?}`. 기존 보정 검증을 거쳐 960×720·24fps의 새 SLAM 세션을 만들고 201로 반환한다. 연결·초기화 완료는 state로 확인한다. |
| `POST /api/live/tello/heartbeat` | JSON `{session_id}`. 현재 Tello 입력의 15초 수명을 갱신한다. 다른 ID는 409. 종료된 입력을 재개하지 않는다. |
| `GET /api/live/tello-preview.jpg?session=ID` | 해당 입력의 2초 이내 최신 원본 JPEG. 다른 세션은 409, 최신 프레임 없으면 404. |
| `POST /api/live/stop` | 기존과 같은 session_id로 종료한다. |

state/settings에는 `input=tello_camera`, `input_timestamp_basis`, `input_origin`을 기록한다.
직접 수신의 시각 기준은 `pc_decode_receive_monotonic`, 게이트웨이는
`mapping_bridge_receive_monotonic`이다. 실제 센서 촬영 시각이 아니다. 순번은 어댑터가 읽어
제출한 입력의 순서이며 UDP 디코딩 전 손실량은 포함하지 않는다.
원본 4:3 비율을 유지하고 다른 크기의 JPEG를 거부한다. input_origin은
`tello_sdk_camera`, `gateway_reported_fake`, `gateway_reported_tello`, `gateway_unspecified`로 출처를 구분한다.

Tello 세션의 frame 업로드는 서버 어댑터만 소유한다. 브라우저가 기존 frame API에 같은 ID를
사용해 덮어쓰려 하면 409다. 직접 Tello 사람 탐지 입력과 지도 입력의 동시 시작도 거부한다.
직접 모드는 수신기를 소유하고 camera session 명령만 보낸다. gateway 모드는 설정된 loopback
origin에 GET만 보내며 source 변경·오래된 상태·영상 종료를 감지하면 세션을 마친다.
영상 중단은 실패 또는 입력 종료 상태로 표시하고 마지막 유효 지도를 계속 조회할 수 있다.
새 시작은 새 ID·좌표계이며 이전 heartbeat/stop/preview가 새 입력에 적용되지 않는다.

`--result SAVED --session RECORDED`를 함께 지정해 저장 결과와 기존 온라인 기록을 제공할 수
있다. `--enable-camera`는 새 입력을 허용한다. `--objects`는 `--result`에만 결합되며 실시간
scene에는 그 결과에 결합된 객체만 사용한다. 두 출력 디렉터리를 합치거나 파일을 수정하지 않는다.

이전 `/api/state`, `/api/run`, `/api/timeline`, 프레임·질감·내려받기 경로도 호환용으로 유지한다.
이 경로는 현재 실시간 세션, 초기 `--session`, 저장 결과 순서로 선택한다. 두 지도를 각각
조회하는 화면은 위의 saved/live 경로를 사용해야 한다.
