# PoC 연결 초안 0.1.0

2026-10-22 사용자 요청으로 먼저 작성한 기본 설계. 관련 아키텍처 작업:
[S15P31B205-9](https://ssafy.atlassian.net/browse/S15P31B205-9).
생산자·소비자는 이 버전과 작은 합성 예제로 병렬 개발한다. 앱·API·검증기가 구현됐다는 뜻은 아니다.

## 1. 공통 규칙

- JSON은 UTF-8, 필드명은 snake_case. `schema_version`은 문자열 `0.1.0`.
- `run_id`는 소문자 UUID 문자열. 재실행·축척 변경 결과는 새 run_id, 원래 회차는 `parent_run_id`로 참조한다.
- `frame_id`는 `frame-000001` 같은 회차 내 고유 문자열이며 COLMAP image_id와 별도다.
- `pts_us`는 원본 영상 PTS를 마이크로초로 환산한 0 이상의 정수. 첫 프레임을 0으로 재기준화하지 않는다.
  원본 PTS가 음수이거나 시간 기준 변환이 필요하면 원본 time_base·PTS와 적용 offset을 manifest에 함께 기록한다.
  단순 프레임 번호/FPS로 대체하지 않으며 같은 PTS의 프레임은 frame_id로 구분한다.
- 촬영 절대 시각 `captured_at_utc`는 ISO 8601 UTC 문자열 또는 null. 알 수 없을 때 서버 수신 시각을 촬영 시각으로 쓰지 않는다.
- 파일 참조는 묶음 내부 POSIX 상대 경로. 절대 경로·상위 이동·심볼릭 링크·외부 URL은 받지 않는다.
- 필수 필드 누락·다른 run_id·중복 frame_id·비유한 수·미지원 버전은 오류다. 선택값 미상은 null.
  호환되는 선택 필드는 추가 가능하며 알 수 없는 필수 의미나 major 변경은 새 버전으로 처리한다.

## 2. 영상·2D 관측: 지도 데이터 → 복원

입력 묶음은 `manifest.json`, `frames/`, 선택적 `observations.csv`, `matches.csv`, `measurements.json`으로 구성한다.
원본 영상과 프레임은 비공개 로컬에 둔다. JSON/CSV는 교환 원본, COLMAP SQLite DB는 실행용 파생물이다.

| 필드·파일 | 형식·규칙 |
| --- | --- |
| manifest 공통 | schema_version, run_id, input_kind=`rgb_video`, source_sha256(원본 파일 SHA-256), captured_at_utc, producer |
| frames[] | frame_id, pts_us, frame_path, original_width/height, rotation_deg(0/90/180/270), camera_id |
| 처리 이미지 정보 | processed_width/height, original_to_processed(행 우선 3×3). 회전·크롭·리사이즈를 적용한 변환을 기록 |
| cameras[] | camera_id, model(초기 경로 SIMPLE_RADIAL), params 또는 null, parameter_source=`measured/estimated/unknown` |
| observations.csv | run_id, frame_id, point_idx(프레임 내 0 기반), x_px, y_px. 원본 프레임 좌표, 왼쪽 위 원점·오른쪽 x·아래 y |
| matches.csv | run_id, frame_id_a, point_idx_a, frame_id_b, point_idx_b, geometric_inlier(boolean) |
| measurements.json | run_id와 measurements[]의 measurement_id, 두 끝점의 frame_id/x_px/y_px, measured_length_m(양수), 측정 방법·불확실성 |

점 좌표는 원본 해상도 기준이다. 1920×1080 원본의 관측을 축소 프레임 좌표로 몰래 교체하지 않는다.
복원 입력으로 바꿀 때 `original_to_processed`를 적용하고 intrinsics도 같은 이미지 기준으로 갱신한다.
왜곡 보정은 단순 3×3으로 표현하지 않고 카메라 모델·왜곡 계수·보정 전후 정보를 별도 기록한다.
camera params가 null인 RGB 입력도 허용하며 복원에서 추정한 파라미터와 출처를 결과에 기록한다.
깊이·IMU·정답 pose·ROS 입력은 요구하지 않는다.

## 3. 복원 결과: 복원 → 저장·미리보기·향후 BE

`result.json`과 `points.ply`가 최소 묶음이다. 원본 영상·실내 사진·DB·가중치는 웹 전달 묶음에서 제외한다.
`points.ply`는 희소 점군의 XYZ float + RGB uchar, 초기 웹 전달은 ASCII PLY·최대 100,000점으로 한다.
전체 해상도 원본 결과는 로컬에 남기고 축소 결과는 선택 방법·원래 점 수를 producer 설정에 기록한다.

| result.json 필드 | 필수 형식·의미 |
| --- | --- |
| schema_version/run_id/parent_run_id | 공통 버전·회차, parent는 없으면 null |
| input_kind/source_sha256 | `rgb_video`는 원본 SHA-256 필수. `synthetic` 예제만 null 허용 |
| producer | name, version, settings 객체. 실제 실행 도구/버전·명령 옵션을 기록 |
| status | `complete/partial/failed`. complete는 기대 프레임 전체 pose와 유효 점군이 있는 경우 |
| pose_source | `sfm/synthetic`. 시뮬레이션 정답 데이터는 다른 계약에서 `ground_truth`로 구분 |
| coordinate_frame | 초기 `colmap_world`, 임의 원점·방향의 오른손 좌표계. ENU/지도 북쪽으로 가정하지 않음 |
| scale | status=`arbitrary/metric`, unit=`arbitrary/m`, factor_from_sfm(null 또는 양수), reference(null 또는 실측 근거) |
| frames[] | frame_id, pts_us, pose_status=`registered/unregistered`, T_camera_from_world(4×4 행 우선 배열 또는 null) |
| quality | expected_frames, registered_frames, point_count, reprojection_error_px(미측정 null) |
| artifacts[] | artifact_id, path, media_type, sha256, bytes. 초기 artifact_id=`pointcloud`, path=`points.ply` |

COLMAP 관례로 카메라 x는 오른쪽, y는 아래, z는 앞이다. 변환은 `X_camera = R X_world + t`이며
카메라 중심은 `-R^T t`다. frame_id와 COLMAP image_id/name 매핑을 생산자가 유지한다.
복원 실패 프레임의 행렬은 null이며 단위 행렬·0 위치로 정상처럼 채우지 않는다.
보정 전에는 arbitrary/null을 사용한다. metric 결과는 factor_from_sfm과 실측 근거가 있어야 하며
점군 XYZ·카메라 중심·변환의 t에 같은 축척을 적용한다. 회전은 바꾸지 않는다.
화면은 단위를 표시하고 arbitrary 상태에 m 눈금이나 실거리 측정 기능을 켜지 않는다.
측정한 축척은 별도 길이로 검증하기 전까지 정확도 보증을 뜻하지 않는다.

정상 합성 예제: [작은 결과 묶음](../data/samples/poc-v0.1/README.md).
누락/오류의 기대 동작:

| 입력 변형 | 소비자 처리 |
| --- | --- |
| 촬영 절대 시각 없음 | null을 수용, 촬영 시각 미상 표시 |
| 일부 pose 없음 | status=partial, 해당 pose_status=unregistered/T=null, 화면에 부분 결과 표시 |
| arbitrary인데 unit=m | 형식 오류, 자동 보정하지 않음 |
| manifest/artifact 해시 불일치·다른 회차 | 수신 거부·기존 회차 보존 |
| pose는 있는데 pointcloud 누락 | complete로 수용하지 않음. failed/partial도 잘못된 파일 참조는 허용하지 않음 |

## 4. 탐지 이벤트: 추론 → 파일·향후 BE

회차별 `events.jsonl`, 한 줄에 한 객체. 공통 버전/run_id와 다음 필드를 둔다.

| 필드 | 기본값·규칙 |
| --- | --- |
| event_id/frame_id/pts_us | event_id UUID, 동일 run_id/frame_id와 원본 PTS로 지도 입력 연결 |
| class_name/confidence | 초기 `person`, 0~1 실수 |
| bbox_xyxy_px | 원본 영상 픽셀 `[x_min,y_min,x_max,y_max]`, 범위 안의 양의 면적 |
| image_width/image_height | bbox 기준 원본 크기 |
| model | name, version, weights_sha256. 가중치 파일은 포함하지 않음 |
| object_position | 초기 null. 2D bbox나 카메라 위치를 사람의 3D 위치로 대입하지 않음 |
| captured_at_utc | 모르면 null |

중복은 run_id+event_id로 식별하며 같은 값은 재수신해도 추가하지 않는다. 다른 내용의 같은 ID는 충돌이다.
초안은 사진을 필수로 요구하지 않는다. 실제 사진·사람 위치 공개는 별도의 자료별 검토 대상으로 둔다.

## 5. 후속 웹 API 기본안

이번 주 로컬 PoC의 선행 조건이 아니다. Python/FastAPI의 단일 API 프로세스와 SQLite 메타데이터,
파일 볼륨으로 시작한다. 브라우저는 같은 HTTPS origin의 `/console/`과 `/api/v1/`을 사용한다.
기존 `/api/status`, `/healthz`는 현행 상태 서비스 경로로 유지한다. 아래 경로는 아직 구현하지 않았다.

| 메서드·경로 | 요청·응답 기본안 |
| --- | --- |
| POST /api/v1/runs | multipart의 manifest(result.json) + pointcloud(points.ply). 검증·원자적 저장 후 201 |
| GET /api/v1/runs?limit=20&cursor=... | 최신 수신 순 목록, limit 1~100, 불투명 커서와 next_cursor(null 가능) |
| GET /api/v1/runs/{run_id} | 검증된 result 메타데이터와 artifact_id별 다운로드 href |
| GET /api/v1/runs/{run_id}/artifacts/pointcloud | 승인된 파일만 스트리밍. 클라이언트 파일 경로를 열지 않음 |
| POST /api/v1/runs/{run_id}/events | events 배열, 최대 100건. 중복 event_id는 동일값만 멱등 수용 |
| GET /api/v1/runs/{run_id}/events | limit/cursor로 회차의 탐지 이벤트 조회 |

- 팀 전용 API/console은 HTTPS의 개인별 Basic 인증을 reverse proxy에 두는 설계다.
  실제 계정·비밀값·권한 설정은 구현 때 관리자가 공급한다. 프런트 코드/저장소에 비밀값을 넣지 않는다.
  인증 전에는 원본·점군·이벤트·메타데이터를 공개하지 않는다. 현재 서버 접근 범위를 변경하지 않는다.
- 초기 제한: 요청 합계 50 MiB, JSON 5 MiB, 점군 100,000점, 회차 저장 총량 1 GiB.
  초과 시 413/507로 거부한다. 실패한 임시 파일을 정리하고 기존 결과를 보존한다.
- ZIP 해제·원본 영상 업로드·서버 복원 실행·실시간 스트리밍·웹소켓·공개 사용자 가입은 초기 API에서 제공하지 않는다.
- 같은 run_id와 같은 manifest 바이트/파일 해시는 200 재수신, 다른 내용은 409. 저장된 결과를 덮어쓰지 않는다.
- 400은 multipart 형식, 401은 인증, 404는 회차/파일, 409는 ID 충돌, 413은 요청 크기,
  422는 계약/해시/좌표 오류, 507은 저장 용량 부족. 오류 본문은 `{ "error": { "code": "...", "message": "...", "request_id": "..." } }`.
- 추가 파일 경로·미디어 유형·정점 수·유한 좌표를 검증한다. 요청명/JSON의 path를 직접 저장 경로로 사용하지 않는다.
  파일은 서버가 만든 회차별 디렉터리에 임시 저장하고 검증 후 확정한다. SQLite는 메타데이터만 저장한다.
- 화면은 회차 목록 → 상세 점군/카메라 경로/단위/품질/이벤트를 표시하고 사용자가 갱신하거나 5초 간격으로 조회한다.
  부분/실패·단위 미정·pose 미제공을 각각 표시한다. 새로고침은 재복원 실행을 뜻하지 않는다.

## 6. 검증 인계·변경

이 초안에서는 JSON 문법·합성 묶음의 해시/경로·행렬·단위 일관성을 확인한다.
실제 영상 전달·복원 품질·API 인증/저장/충돌·화면 표시는 해당 구현에서 검증한다.
그 결과가 나오기 전에는 전체 파이프라인 검증이나 분야별 최종 합의를 받았다고 기록하지 않는다.

선택 근거: [COLMAP 출력·좌표 관례](https://colmap.github.io/format.html),
[SQLite 사용 범위](https://www.sqlite.org/whentouse.html),
[FastAPI 배포](https://fastapi.tiangolo.com/deployment/).
