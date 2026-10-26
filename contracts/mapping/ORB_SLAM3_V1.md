# ORB-SLAM3 지도 연결 1.0

생산자: `modules/mapping/localization/orbslam3/bridge.cpp`, `orbslam3.py`.
소비자: `apps/console/`. 실제 영상·프레임·지도·로그는 로컬 ignore 산출물이다.

## 입력과 추적

입력은 로컬 RGB 동영상 또는 브라우저 카메라 JPEG와 카메라 보정값이다. 각 프레임의 원본 PTS를 전달하며,
디코더가 유효 시각을 제공하지 않으면 nominal FPS 대체와 횟수를 명시한다.
카메라 내부 파라미터는 회전/크롭이 일치하는 자료만 받아 처리 해상도로 함께 변환한다.
이전 COLMAP 결과에서는 내부 파라미터만 읽는다. 자세·점·매칭은 재사용하지 않는다.

`orb-slam3/1.0` native snapshot에는 `revision`, `map_id`, `map_change_index`,
`big_change_index`, `final`, `keyframes`, `points`가 있다.
각 키프레임은 native ID, 원본 프레임 참조·시각, `T_cw`, 2D 관측과 native point ID를 갖는다.
`T_cw`는 world→camera의 4×4 강체 변환이다. 원본 카메라 축은 x-right/y-down/z-forward다.
점은 native ID·3D 좌표·관측 수를 갖는다. ID는 같은 지도 안의 landmark 식별자이며
병합·삭제에 따라 바뀔 수 있다. 프레임의 코너 번호나 사물 종류로 해석하지 않는다.

## 웹 발행

`phone-sfm-result/1.2` 형식과 기존 revision 읽기 경로를 재사용한다.
`pose_source=orb_slam3`, `segment_id=map_id`, `slam`에 원본 지도 버전과 보정 상태를 추가한다.
display 좌표는 native world에 diag(1,-1,-1)을 적용하며 카메라 중심·방향·점 모두 동일하게 변환한다.
지도 수정 후 이전 좌표의 점/경로를 누적하지 않고 완성된 한 snapshot으로 교체한다.
다른 map ID로 전환하면 웹도 별도 좌표계로 교체한다. 추적 상실 중에는 마지막 유효 지도를 유지한다.

새 scene은 [결과 계약의 관측 연결](PHONE_SFM_RESULT_V1.md#선택적-관측-프레임-연결)을 함께 발행한다.
`point_observations.basis=keyframes`이며 전체 수신 프레임 수가 아닌 선택된 키프레임의 고유 관측이다.
과거 scene은 native snapshot의 revision·map ID·카메라 ID/프레임 목록이 모두 일치할 때만
웹 응답 메모리에서 보완한다. 저장 파일은 수정하지 않는다. 최신 native로 이전 재생 기록을
보완하지 않으며 연결 정보가 없는 기록은 명시적인 하한 계산으로 표시한다.

`state.json`은 입력/정상 추적 수, 추적 상태, 반복 관측 수, 처리 시간 P95,
영상 시각 대비 재생 지연과 마지막 유효 revision을 노출한다.
`initializing_frames`와 `lost_tracking_frames`를 구분하며 초기화 전 프레임을 추적 상실로 세지 않는다.
`/api/slam-preview`는 왜곡 보정된 현재 프레임과 지도 점 ID 대응을 보여주는 로컬 이미지다.
노랑은 직전 처리 프레임과 같은 native ID, 초록은 이번에 연결된 ID다.
서로 다른 좌표계의 추적 상태를 합치지 않으며 ID 대응은 의미적 사물 인식이 아니다.

## 성능·범위

파일 재생은 순차 처리하며 계산이 느려지면 지연을 기록한다. 프레임을 버리지 않았다는 사실을
실시간 성능으로 취급하지 않는다. `performance_target_met`는 입력 30장 초과, 원래 속도,
정상 추적 비율 90% 이상, tracking P95가 원본 프레임 간격 이하, 재생 지연 P95 2초 이하의
동시 충족 여부다. 이는 희소 추적에만 해당하며 전체 공간 복원·비행 정확도 인증이 아니다.
키프레임 지도는 약 1초 간격으로 발행하고 웹은 상태를 250ms 간격으로 확인한다.
추적과 지도 파일 발행은 별도 스레드다. 프레임 RGB는 최근 180장만 메모리에 유지하고
실제 선택된 키프레임만 저장한다. 미리보기는 최대 5Hz·최근 8장으로 제한한다.
웹은 최근 600개 visible 렌더 프레임의 FPS·프레임 간격 P95를 별도 표시한다.
단안 단위는 arbitrary다. IMU·깊이·실측 없이 미터로 승격하지 않는다.
밀집 깊이·메시·텍스처와 실기체 RTSP/UDP 입력은 후속 단계다.

## 브라우저 카메라 전송

`serve.py --enable-camera`일 때만 POST를 제공한다. loopback Host와 동일 Origin을 확인하며
모든 POST는 `X-Mapping-Client: 1`이 필요하다. 외부 CORS는 열지 않는다.

| 경로 | 내용 |
| --- | --- |
| `GET /api/live` | 지원 여부, 현재 세션 상태, 명시적으로 지정된 시연 영상 여부 |
| `POST /api/live/start` | JSON `width,height,fps,camera?` 또는 `fov`; 새 고유 `session_id` 반환 |
| `POST /api/live/frame?session=ID` | JPEG ≤2MiB, `X-Frame-Timestamp`(0부터 단조 증가 media 초), `X-Frame-Sequence`(0부터 단조 증가) |
| `POST /api/live/stop` | JSON `session_id`; 마지막 전송 프레임 처리 후 정상 종료 요청 |

전송 이미지는 보정값과 같은 해상도·크롭이어야 한다. 내부 카메라 전송은 음성 없이 최대
960px이며 보정 JSON이 없으면 화각 추정 상태를 노출한다. 서버 대기는 최신 1장,
Docker stdin은 처리 ACK 전까지 1장으로 제한한다. stdin 형식은 ASCII `JPEG바이트수 PTS\n`
뒤에 정확히 그 수의 JPEG 바이트다. native `ORB_ACK PTS`는 추적 함수가 반환된 시점이며
웹 렌더 완료 ACK가 아니다. EOF는 정상 종료다. 10초 입력 중단은 세션을 마무리한다.

상태의 `transport_received`, `transport_submitted`, `queue_depth/capacity`, `frame_drops`를
함께 읽는다. drops는 입력 시퀀스 누락 + 아직 처리하지 않은 대기 프레임 교체이며,
처리된 `frames_received`·초기화·추적 상실 수와 구별한다. 브라우저는 영상 콜백의
presentedFrames를 시퀀스 기준으로 사용하고, 미지원 환경에서는 콜백 수를 기준으로 사용한다.
파일 입력과 브라우저 입력은 같은 snapshot 발행기를 사용한다.

## 지도 시계열 재생

새 native snapshot은 `timestamp_s`(복사한 입력의 최신 PTS)를 추가하고 소비자는
`slam.timestamp_s`와 `slam.timestamp_basis=snapshot_input_pts`를 유지한다.
`orb-map-timeline/1.0`은 `session_id`, `entries` 배열이며 각 항목에 `revision,map_id,
timestamp_s,timestamp_basis,points3D,keyframes,final`이 있다.

`GET /api/timeline`은 이 인덱스, `GET /api/run?revision=000001&session=ID`는 고정된 지도다.
session 불일치는 409이며 키프레임·미리보기 요청도 session을 확인한다. 재생은 시간 이하의
가장 최근 revision 전체를 표시하고, 첫 revision 이전은 0점이다. 버전마다 점의 보정·삭제를
그대로 반영하며 서로 다른 map ID를 합치지 않는다. 새 기록은 약 1Hz 지도 발행 시점 기준이며
30Hz의 모든 내부 상태 이력이 아니다. 구 기록은 마지막 생존 키프레임 시각을 추정으로 표시한다.
재생과 카메라 전송은 별개 시계이며 **최신 지도**로 수신 결과 추종에 복귀한다.
