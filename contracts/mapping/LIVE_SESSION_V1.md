# 실시간 녹화·세션 저장 1.0

생산자: `modules/mapping/data/live_recording.py`, 실시간 입력 어댑터.
소비자: `apps/console/`, 후속 세션 불러오기 구현.
영상·프레임·지도·전체 로그는 로컬 `output/phone_mapping/live_<id>/`에만 저장한다.

## 세션 묶음

루트의 `session.json`은 `schema_version=mapping-session/1.0`이며 다음 필드를 제공한다.

| 필드 | 의미 |
| --- | --- |
| `session_id`, `started_at`, `ended_at` | 회차 ID·UTC 시작/종료 시각. 종료 전 ended_at은 null |
| `input` | `tello_camera` 또는 `browser_camera` |
| `mapping_timestamp_zero_monotonic` | Tello 매핑 PTS 0에 대응하는 PC 단조 시각. 브라우저 입력은 null |
| `recording` | 저장 상태·큐 크기·바이트 수·구간 수·원본 H.264 여부·재생 파일 생성 오류 수 |
| `recordings` | 녹화 구간 목록. 재연결로 수신기가 다시 열리면 새 구간을 만든다 |
| `mapping` | 매핑 종료 상태·최초 종료 사유·오류·최신 지도 revision·보존된 지도 구간 |
| `assets` | 보정값·설정·최종 상태·native 지도·revision·키프레임·추적·진단의 상대 경로 |
| `asset_sha256` | 정상적으로 닫힌 뒤 읽을 수 있는 camera.json·settings.json·state.json의 SHA-256 |
| `scale`, `privacy` | 최종 지도의 `m`(Tello 추정) 또는 `arbitrary`, `local_only` |

`assets`의 지도 관련 경로는 지도 초기화 전 종료 시 존재하지 않을 수 있다.
세션 정보만 저장된 정상 종료와 지도 생성 성공은 별개다.
manifest의 파일 경로는 세션 루트 기준이다. 기존 `local_assets.json` 및 state의 로컬 경로 힌트는
이전 PC의 절대 경로일 수 있으므로 후속 불러오기에서는 manifest의 키프레임 루트로 다시 연결해야 한다.
다른 map ID는 [ORB 계약](ORB_SLAM3_V1.md)에 따라 독립 좌표계로 유지한다.

## Tello 실시간 자세와 지도

- 직접 연결은 TelloLink의 UDP 상태 수신 시각과 영상 디코딩 시각을 같은 PC 시계에서 연결한다.
- 게이트웨이는 `GET /video.mjpg?mapping=1` 응답에 `X-Tello-Sensor-Pose: 1`을 제공한다.
  자세를 붙일 수 있는 JPEG 파트에는 `X-Tello-Pose`가 있고, 값은 JSON 배열
  `[state_epoch, x, y, z, yaw, pitch, roll]`이다. 위치는 전원 켤 때 방향 기준 앞/왼쪽/위의 m,
  각도는 라디안이다. 헤더가 없는 구형 게이트웨이는 영상과 깊이로 첫 지도 기준을 잡는다.
- Tello 지도는 세션당 한 좌표계를 사용한다. 센서 상태가 끊기면 누적 프레임의 영상 특징과 깊이에서
  자세를 복구하고, 센서 좌표가 초기화되면 복구한 자세에 새 센서 좌표를 정렬한다.
  위치를 맞출 근거가 부족하면 최근 확인한 두 자세의 위치·회전 속도를 현재 영상 시각까지
  적용해 같은 지도에 누적한다. 확인된 자세가 하나뿐이면 그 자세를 유지한다.
- 이력과 저장 카메라는 센서 추정 `tello_sensor_odometry`, 영상 복구 `tello_visual_recovery`,
  선형 예측 `tello_linear_prediction`을 구분한다. 선형 예측 프레임의 추적 상태는 `pose_predicted`다.
  `m`은 추정값의 단위이며 기체 경로·지도 치수의 독립 실측 정확도를 나타내지 않는다.

## 원본과 수신 영상

- Tello 직접 연결: UDP 소켓에서 받은 H.264 payload를 디코딩·JPEG 변환 전에
  `recordings/part_0000.h264`에 순서대로 보존한다. 소켓에 도착하기 전의 무선 패킷 손실까지 복구한다는 뜻은 아니다.
- 게이트웨이·브라우저: 서버가 받은 JPEG를 `part_0000.mjpeg`에 그대로 이어 쓴다.
  이 입력은 `mode=received_jpeg`, `original_drone_bitstream=false`이며 원래 장치의 H.264 원본이 아니다.
- 구간마다 `.jsonl`에 `offset`, `size`, `timestamp_s`를 기록한다.
  `time_basis`는 PC UDP 수신 단조 시각, 게이트웨이 수신 단조 시각 또는 브라우저 media PTS다.
  원본은 SLAM 대기 큐에서 프레임을 생략하기 전에 저장한다.
- 구간 메타데이터는 원본 경로·인덱스 경로·실제 저장 바이트·SHA-256·첫/마지막 수신 시각을 담는다.
  직접 수신에서는 `receiver_bytes`, `receiver_packets`, `capture_complete`로 소켓 수신량과 저장량을 대조한다.
- 종료 시 인코딩된 패킷을 다시 인코딩하지 않고 H.264는 MP4, JPEG는 Matroska(MKV)로 묶는다.
  `playback`에는 경로·SHA-256·프레임 수·해상도·시각 기준·재인코딩 여부가 있다.
  H.264에는 원래 촬영 PTS가 없으므로 MP4 시각은 PC 패킷 수신 시각의 근사치다.
  키프레임 전 패킷은 재생 파일에서 생략할 수 있으며 원본에는 남는다.
  B-frame 순서에 필요한 촬영 PTS가 없거나 변환이 실패하면 원본을 보존하고 `playback_error`를 기록한다.

## 센서와 프레임별 매핑 기록 (추가 필드)

영상 촬영 시 같은 세션 루트에 `sensor.jsonl`, `frames.jsonl`을 자동 생성한다.
`session.json.assets.sensor`, `assets.frame_poses`로 찾으며 ZIP에도 포함한다.
기존 1.0 세션에 대한 호환 가능한 추가 항목이다. 기존 파일에 이 항목이 없으면 센서 재계산 가능한 기록으로 간주하지 않는다.
`settings.json`에는 보정 카메라·샘플링·매핑 옵션과 모델 ID, 확인 가능한 로컬 모델 revision, 패키지 버전을 남긴다.
모델 revision을 확인할 수 없으면 null이다. 저장 영상의 센서 활용 복원은 아래 소비 절차를 따른다.

### `sensor.jsonl` (`mapping-sensor/1.0`)

- `event=sample`: `source_run_id`, 원본 수신 순서 `sequence`, UTC `received_at`,
  원본 프로세스의 단조 수신 시각 `received_mono`(초), `raw`, `si`, `valid`, `reasons`.
- `raw`의 속도 `vgx/vgy/vgz`, 자세 `yaw/pitch/roll`, 거리 `tof`, 높이·기압 `h/baro`,
  가속도 `agx/agy/agz`, 배터리·온도·모터 동작 시간과 미션패드 필드를 원래 단위로 보존한다.
  `si`는 기존 수신기의 변환값이며 검증되지 않은 단위는 임의 변환하지 않는다.
- 직접 연결은 TelloLink의 수신 이력을 읽고, 게이트웨이는 로컬 전용
  `GET /api/drone/sensors?after=<sequence>`로 가져온다. 비행 명령은 보내지 않는다.
  응답은 `tello-sensor-batch/1.0`, `run_id`, `next_sequence`, `lost`, `samples`다.
  `/api/drone/state`도 같은 `run_id`를 제공한다. 구형 상태 응답에 run ID가 없어도
  센서 이력 응답에서 확인해 수집하며, ID 누락을 이유로 조용히 건너뛰지 않는다.
  이력은 최근 512개로 제한하며 처음 연결할 때 그 범위의 촬영 전 자료가 포함될 수 있다.
  동일 source/sequence는 중복 저장하지 않는다. 소스가 바뀌면 새 run ID의 순서로 구분한다.
- 다음 조회 전에 수신 이력이 밀려나면 `event=gap`, `after_sequence`, `lost`를 남기고
  녹화 상태를 partial로 표시한다. 최초 조회 이전의 이력은 전체 비행 기록이라고 주장하지 않는다.
  구형 게이트웨이·기록 재생 모드가 센서 이력을 제공하지 않으면 로그 오류로 표시하고 영상·매핑은 계속한다.

### `frames.jsonl` (`mapping-frames/1.0`)

이 파일은 이벤트 로그다. `frame_sequence`로 묶고 파일 순서대로 적용한다.

| 이벤트 | 주요 값 |
| --- | --- |
| `input` | 프레임 번호, 매핑 PTS `timestamp_s`, `admission`(submitted/model_wait/rate_limited), `source`, `sensor_epoch`, `sensor_pose`, `allow_visual_origin`, `front_tof`, 초기 `mapping_applied=false` |
| `mapping` | 같은 프레임 번호, 처리 이미지 `frame_id`, revision·map ID, tracking, pose_source, 단위·깊이 배율, 최종 `world_from_camera`(4×4), 실제 `mapping_applied` |

`input`만 있고 `mapping`이 없으면 지도에 반영되지 않은 프레임이다. submitted는 대기열에 들어갔다는 뜻이며
실제 반영을 보장하지 않는다(최신 프레임 교체·추적/깊이 표본 생략·종료 등이 가능).
처리된 프레임도 추정 실패 등으로 `mapping_applied=false`일 수 있다.
카메라 자세는 NPZ 관측의 `world_from_camera`와 같은 매핑 좌표계이며, 최종 화면 표시 좌표와 구분한다.
`pose_source`는 센서/영상 복구/선형 예측 또는 `orb_slam3_monocular`이다. 유효 자세가 없으면 null이다.
`sensor_pose`는 `[segment,x,y,z,yaw,pitch,roll]`이며 위치 m, 각도 rad다. 센서 단절 시 null이며
`sensor_epoch`와 연결 번호로 재연결 전후를 구분한다. 전방 ToF는 `[거리_m, 입력_단조시각_s]` 또는 null이다.

`source`는 연결 번호, 지도 입력 수신 시각과 녹화 조각 번호를 담는다.
게이트웨이 JPEG는 `recording_frame_index`(조각 내 0부터), `video_recorded`로 저장 영상 인덱스에 정확히 대응한다.
MJPEG의 `X-Tello-Frame` 헤더는 원래 게이트웨이 `run_id`, 디코딩 `timestamp_s`, UTC epoch `wall`을 전달한다.
그 시각은 센서의 `received_mono`와 같은 소스 시계다. 매핑 PTS는 별도 수신 시계에서
`mapping_timestamp_zero_monotonic`을 뺀 값이며, 모델 준비 전 프레임은 아직 기준이 없어 null이다.
직접 H.264 입력은 디코딩 시각으로 연결하므로 패킷 수신 시각과의 대응은 근사치다.
브라우저 입력은 media PTS이며 드론 원시 센서는 없다.

로그는 기존 영상 저장 작업자에 큐로 전달한다. 큐 초과·로그 파일 오류·센서 조회 실패는
`recording.log_error`, `log_dropped`, `sensor_gaps`와 partial 상태로 보고한다.
영상 파일과 센서/프레임 로그는 독립적으로 실패할 수 있으며, 영상·비행·매핑을 중단하지 않는다.
종료 후 로그의 SHA-256을 `asset_sha256`에 넣는다. 센서 원시값만으로 과거 실시간 결과의 완전 동일성을 보장하지 않는다.
드론 입력을 연결하면 `recording.sensor_required=true`이며, 실제 기록한 원시 센서 표본 수는
`recording.sensor_samples`다. 영상이 저장됐는데 표본이 0개이면 종료 상태는 complete가 아니라
partial이며 `recording_sensor_samples_missing` 오류를 남긴다.

이전 수집 누락으로 센서 파일이 비어 있으면 `sensor_recovery.recover_empty_sensor_log`로
종료된 세션만 복구할 수 있다. 프레임에 기록된 원본 run ID·단조 시각과 게이트웨이 manifest를
대조하고, 해당 구간의 실제 원시 패킷만 보존한다. 기존 센서 내용이 있거나 출처·시각·순서가
맞지 않으면 덮어쓰지 않는다. manifest의 `capture_metadata.sensor_recovery`에 원본 해시와
시각 범위·복구 시각을 남기며, 영상·프레임·기체 로그 원본은 변경하지 않는다.

## 독립적인 영상·센서 시각 대응

드론 영상과 센서를 공유할 때 `frames.jsonl`, 이미지, 매핑 결과를 제외할 수 있도록
세션 루트에 `video_sensor_sync.json`(`video-sensor-sync/1.0`)을 자동 저장한다.
`session.json.assets.video_sensor_sync`, `asset_sha256`, `recording.video_sensor_sync`에도 등록하며 ZIP에 포함한다.
새 촬영은 매핑 선택 전에 녹화 입력에서 원본 시각을 보존하므로 매핑 로그가 없어도 생성된다.
영상 조각별 내부 인덱스에 `sensor_clock={run_id,timestamp_s}`를 추가한다.

| 필드 | 의미 |
| --- | --- |
| `status` | complete / approximate / partial / unavailable / not_applicable |
| `sensor_path`, `sensor_sha256` | 함께 사용할 센서 파일과 SHA-256 |
| `sample_fields` | samples의 두 값: video_time_s, sensor_received_mono_s |
| `parts[].part_id`, `video_path`, `video_sha256` | 해당 영상 조각과 파일 식별자 |
| `parts[].source_run_id` | 대응할 sensor.jsonl의 source_run_id |
| `parts[].samples` | 실제 재생 파일의 PTS(초)와 센서 수신 시계(초)의 대응 쌍. 시각 미상은 null |
| `parts[].coverage` | 재생 시각 수, 원본 시각 누락 수, 중복 재생 시각과 역행 시각 수 |
| `parts[].sensor_available_range_s` | 해당 센서 원본에 기록된 단조 시각 범위 |
| `parts[].accuracy` | gateway_decode_receive_alignment 또는 packet_receive_approximation |

사용자는 원하는 간격으로 영상을 처리하고, 그 재생 시각을 대응 쌍 사이에서 선형 보간해
같은 source_run_id의 센서 `received_mono`에 연결한다. 범위 밖 외삽, null 구간 연결,
중복된 영상 시각의 임의 선택은 하지 않는다. `video_sensor_sync.sensor_time_at`은 이런 구간에 None을 반환한다.
필요한 공유 입력은 영상·sensor.jsonl·video_sensor_sync.json과 카메라/설정/세션 메타데이터다.
매핑에서 사용했던 사진·선택 번호·자세는 이 파일에 포함하지 않는다.

네트워크 지연 변화 때문에 대응을 고정 시간차 하나로 줄이지 않는다. 컨테이너가 PTS를
반올림하므로 실제 저장 영상에서 다시 읽은 시각을 사용한다. 중복·누락은 partial로 표시한다.
H.264는 촬영 PTS가 없으므로 패킷 수신 시각의 근사이며 approximate로 표시한다.
어느 경로도 카메라 노출 순간을 실측한 시각이라고 주장하지 않는다.
센서가 필요한 녹화의 대응 자료가 불완전하면 녹화 상태도 partial과 오류를 표시한다.

기존 종료 회차는 `video_sensor_sync.export_existing_sync(회차_경로)`로 파일을 생성할 수 있다.
새 내부 인덱스 또는 과거 입력 기록에서 시각만 추출하며 영상·센서·사진·기존 로그는 변경하지 않는다.
기존 대응 파일은 덮어쓰지 않는다. 영상 파일을 변환/편집하면 PTS·해시가 달라질 수 있으므로
원래 파일의 대응 자료가 그대로 유효한지 다시 검증해야 한다.

## 드론 측정 및 센서 활용 복원

기존 기록 형식을 그대로 소비한다. `GET /api/import/drone-recordings`는 실측 영상 입력 폴더와
현재 녹화 루트의 `live_*` 중 종료된 Tello 세션을 반환한다. `POST /api/import/drone-recording`에
`session_id`를 보내면 센서 기반 복원 초안이 생성된다. 깊이 모델을 선택한 뒤 기존 import start API로 실행한다.
요약의 `kind`와 결과의 `depth_mapping.input_kind`는 `drone_recording`이다.

JPEG 녹화는 `recording_part`, `recording_frame_index`, `video_recorded=true`로 입력 이벤트를 찾고
영상 PTS와 `receive_timestamp_s`가 2ms 이내인지 확인한다. 저장된 구간별 `sensor_pose`를 우선하고,
저장 자세가 없는 구간은 같은 `source.run_id`의 원시 센서를 적분한다. 원시 적분의 시작 위치와
저장 자세의 적분 시작 위치는 서로 다른 좌표 원점으로 취급한다. H.264는 재생 메타데이터의
`first_timestamp_s`에 PTS를 더한 패킷 수신 시각으로 원시 센서를 대응한다.

복원은 실시간 감독과 같은 센서 좌표 변환 및 점군 통합을 사용한다. 저장 지도에 반영되지 않았던 입력
프레임도 복원 대상이며, 이전 `mapping` 이벤트의 결과 자세를 새 복원의 입력 자세로 사용하지 않는다.
새 결과는 별도 import 회차에 저장한다. 영상·센서 원본과 생산자인 실시간 녹화 형식은 유지한다.

## 부하·오류·종료

수신 스레드는 파일을 쓰지 않고 전용 작업 큐에 넣는다. 8MiB 또는 대기 작업 8,192개에 도달하면
영상 데이터 수락을 중단한다. 구간 닫기·종료를 위한 소수의 제어 작업은 계속 수락한다.
세션당 녹화 구간 한도는 64개다. 큐 초과 시 이후 녹화를 중단하고 이미 받은 연속된 앞부분을 보존한다.
저장 장치 오류도 매핑 중단 사유로 전달하지 않으며 웹의 녹화 상태에 별도로 표시한다.
녹화 중단 뒤의 전체 수신량은 `bytes_received`에 계속 누적되지 않는다.
직접 연결의 구간별 `receiver_bytes`가 닫힌 소켓의 최종 수신량이다.

`recording.status`는 `waiting`, `recording`, `finalizing`, `complete`, `partial`이다.
오류가 있으면 정리 중에도 partial일 수 있으므로 내려받기 준비 여부는 `finalized`도 확인한다.
Tello 입력 종료와 매핑 종료가 모두 확인된 후 파일을 닫고 재생 파일·최종 manifest를 만든다.
두 단계의 종료 순서가 바뀌어도 아직 도착하는 원본을 미리 닫지 않는다.
강제 프로세스 종료 시에는 남은 원본 앞부분과 `finalized=false` manifest가 보존되며,
완전하게 저장된 회차로 간주하지 않는다. 주기적인 버퍼 flush는 전원 장애에 대한 완전한 보장을 뜻하지 않는다.

## 로컬 HTTP

### 영상 복원 화면의 촬영 자료 선택

촬영 원본 선택과 기존 복원 결과 열기는 별도 API로 제공한다. 촬영 루트는 지도 서버의 `--live-root`를 사용하며,
클라이언트가 임의 절대 경로를 입력하지 않는다. 모든 조회는 읽기 전용이다.

| 경로 | 동작 |
| --- | --- |
| `GET /api/import/reconstruction-library` | 기존 library 중 실시간 드론 촬영 원본·그 별칭 및 결과 없는 로컬 폴더를 제외한 복원 기록 |
| `GET /api/import/capture-library` | `root_path`, 최신 촬영순 `items`(최대 100개). 영상이 없는 종료 회차는 제외 |
| `GET /api/import/capture-bundle?session_id=ID` | 저장 완료·필수 파일 존재 여부를 다시 확인한 선택 묶음. 미완료는 400 |
| `GET /api/import/capture-file?session_id=ID&path=RELATIVE` | 선택 묶음에 실제 포함된 파일만 제공. 영상은 HTTP Range 지원 |
| `POST /api/import/capture-folder` | 서버의 실제 촬영 루트를 Windows 파일 탐색기에서 열기. 사용자 경로를 받지 않음 |

목록 항목과 선택 응답은 `schema_version=mapping-capture-selection/1.0`이며 다음 필드를 제공한다.

| 필드 | 의미 |
| --- | --- |
| `session_id`, `name`, `started_at`, `ended_at` | 서버가 관리하는 회차 폴더명과 촬영 시각 |
| `saved_at` | 수신 중 누적 저장본의 저장 요청 시각. 일반 촬영은 null이며, 값이 있으면 이 시각순으로 정렬 |
| `status`, `selectable`, `missing` | `recording`(미종료/미확정), `incomplete`(필수 파일 부족), `ready`. ready만 선택 가능 |
| `sync_status` | `video-sensor-sync/1.0` 파일의 complete/partial/approximate/unavailable 상태. partial·approximate는 명시하고 선택 허용 |
| `parts` | 구간별 `part_id`, `video_path`, `index_path`, `duration_s` |
| `files` | 실제 존재하는 파일의 `path`, `role`, `bytes`, 같은 서버의 `url` |
| `reconstruction_supported` | 이번 선택 전용 구현에서는 false. 선택 API는 import 생성·모델 실행·센서 자세 복원을 하지 않음 |

허용 파일은 `session.json`, 비어 있지 않은 `sensor.jsonl`, `video_sensor_sync.json`, 존재하는 `camera.json`·`settings.json`,
그리고 manifest의 각 구간에 선언된 `recordings/*.mkv`/`*.mp4`와 그 `index_path`의 `recordings/*.jsonl`이다.
모든 구간의 영상과 대응 JSONL, 센서·시각 대응 파일이 있어야 선택 가능하다. 원본 manifest는 그대로 제공하지만
그 `assets`에 적힌 파일을 재귀적으로 연결하지 않는다. `frames/`, `frames.jsonl`, 임의 JSONL, 원시 비트스트림,
지도 결과·진단 및 심볼릭 링크/정션은 선택 파일에 포함하지 않는다. 시각 대응 파일은 기존 별도 저장 기능의 산출물을 읽는다.

브라우저 연결점: `capture-library.js`의 `selectedCaptureBundle()`은 선택 묶음의 복사본 또는 null을 반환한다.
document의 `mapping-capture-selected` 이벤트도 같은 묶음을 `detail`로 제공하며 선택 해제 시 null이다.
여러 구간을 누락 없이 소비하고 실제 복원 시작 직전에 `capture-bundle`로 재검증하는 것은 후속 복원 소비자의 책임이다.
수신 시각의 대응은 동일한 3D 결과나 노출 시각의 정밀 계측을 보장하지 않는다.

### 기존 전체 세션 다운로드

| 경로 | 동작 |
| --- | --- |
| `GET /api/live/session?session=ID` | 현재 또는 명시적으로 서버에 연결한 세션의 manifest |
| `GET /api/live/session-download?session=ID` | 매핑 종료·저장 finalized 후 세션 전체 ZIP 스트리밍 |

세션 ID 불일치는 409, 세션/녹화 정보 없음은 404, 저장 진행 중 ZIP 요청은 409다.
입력의 임의 파일 경로를 받지 않는다. ZIP은 프레임·영상·지도·진단을 포함하고
미완료 `.tmp` 파일과 심볼릭 링크/정션을 제외한다. 별도 ZIP 사본을 디스크에 만들지 않는다.
manifest 파일 쓰기가 실패해도 살아 있는 서버는 메모리의 최신 진단 manifest를 ZIP에 넣을 수 있다.
반출 ZIP도 촬영 자료를 포함하는 비공개 자료이며 자동 업로드하지 않는다.

이번 버전은 저장·내려받기만 추가한다. ZIP/폴더 불러오기 화면·보정 프로필·초기화 튜토리얼은 후속 단계다.

### 수신 중 누적 저장

`POST /api/live/checkpoint`의 JSON은 현재 `session_id`를 받는다. 요청을 수락한 지점에 녹화 큐의 경계를 넣고,
그 앞의 영상·센서·입력 로그를 flush한 뒤 파일별 바이트 길이를 고정한다. 원본 수신기·지도 처리·조종은 종료하지 않는다.
새 `saved_YYYYMMDD_HHMMSS_<id>/` 회차에 그 길이까지만 복사하고 재생 영상과 `video_sensor_sync.json`을 생성한다.
매번 원본 회차의 처음부터 새 요청 지점까지 저장하며, 이전 저장본은 덮어쓰지 않는다.

지도는 요청 시점에 공개 완료된 revision을 고정한다. 깊이 지도는 해당 revision까지의 관측 전체를 별도로 합성한다.
처리 중인 장면을 기다리지 않으므로 영상의 마지막 시각과 지도에 반영된 마지막 시각이 다를 수 있다.
기존 native 지도는 그 시점에 공개된 scene을 저장한다. 관측·카메라 이미지 등 지도 재열기에 필요한 파일도 보존한다.
저장 회차의 `checkpoint`에는 `source_session_id`, `cutoff_at`, `cutoff_monotonic`,
`scope=session_start_to_request`, `map_revision`, `map_scope=committed_observations_at_request`를 기록한다.
`started_at`은 원본 시작 시각, `ended_at`은 저장 요청 시각이다. 저장 실패는 원본 녹화를 중단하지 않는다.

응답 202는 `{id,status:saving,cutoff_at,source_session_id,...}`이며, 처리 완료 후
`GET /api/live/checkpoint-status`의 `latest`에 `status=complete`, 저장 `path`, `capture_status`, `sync_status`가 나온다.
실패는 `status=failed,error`로 표시하고 다시 저장할 수 있다. 이미 저장 중이면 `checkpoint_busy`,
지도가 준비되지 않았으면 `checkpoint_map_not_ready`로 400을 반환한다. 수신 종료·마무리 중에는 요청을 받지 않는다.
부분 녹화·시각 누락은 `recording.status=partial`로 유지하며 정상 완료로 바꾸지 않는다.

관제 UI에서 이 두 API를 호출할 때만 설정된 로컬 gateway의 Origin(동일 포트의 localhost/127.0.0.1)을 허용한다.
POST의 `X-Mapping-Client: 1`과 기존 Host 검증을 유지하며 비행·영상 종료 API의 교차 Origin 권한을 넓히지 않는다.
