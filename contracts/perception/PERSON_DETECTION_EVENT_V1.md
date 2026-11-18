# 사람 탐지 이벤트 1.0

사람 인스턴스 세그멘테이션 결과를 웹과 자율주행 모듈이 동일하게 해석하기 위한 프레임 단위 계약이다.
한 이벤트의 `detections`에는 같은 프레임에서 탐지한 모든 사람이 들어가며, 사람이 없으면 빈 배열이다.

- `schema_version`: `person-detection-event/1.0` 고정
- `event_id`: 이벤트 UUID
- `input_id`: 입력 세션 식별자. 재연결 또는 프로세스 재시작 시 새 값 사용
- `frame_id`: 입력 세션 안에서 유일한 프레임 식별자
- `detected_at_utc`: 서버가 프레임을 받은 Unix 시각을 UTC RFC 3339로 변환한 값
- `timestamp_source`: 현재는 `server_receive` 고정. 카메라 촬영 시각으로 해석하지 않는다.
- `image`: 원본 이미지의 픽셀 너비와 높이
- `model`: 모델 이름, 런타임 버전, 가중치 SHA-256. 알 수 없는 값은 `null`
- `detections`: 사람별 결과 배열

각 탐지는 호환·검색용 `bbox_xyxy_px`와 실제 표시용 `polygons_px`를 함께 가진다. bbox 순서는
`[x_min, y_min, x_max, y_max]`이며 이미지 경계 안의 양의 면적이어야 한다. polygon은 픽셀 좌표점
3개 이상으로 이루어지며, 한 사람 마스크가 분리되어 있으면 `polygons_px`에 여러 polygon을 보존한다.

JSON Schema는 `person_detection_event_v1.schema.json`, 공개 합성 예시는
`samples/person_detection_event_v1/`에 있다. 예시는 실제 사람 이미지나 개인정보를 포함하지 않는다.

## 생산자와 소비자 영향

- 추론 생산자: `TelloPersonStream.stats().detection_event`로 최신 이벤트를 제공한다. 기존 화면의
  `detections`는 호환을 위해 유지한다.
- 웹 전송: 후속 Jira 작업은 이 이벤트와 해당 프레임 이미지를 함께 전송하며 필드 이름을 바꾸지 않는다.
- 웹·자율주행 소비자: `schema_version`을 먼저 확인하고 bbox는 검색/호환, polygon은 윤곽 표시와
  공간 계산에 사용한다. 모르는 버전은 추측해서 처리하지 않는다.

