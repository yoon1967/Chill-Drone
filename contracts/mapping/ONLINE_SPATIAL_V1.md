# 온라인 지도 계약 1.x

생산자: `modules/mapping/localization/online.py`, `planar.py`, `streaming_planes.py`.
소비자: `apps/console/serve.py`, Three.js 공간 화면. 기존 오프라인 1.0 결과와 수치 snapshot은 변경하지 않는다.

## 지도 스냅샷

온라인 scene은 `schema_version=phone-sfm-result/1.2`, pose_source=sfm이다.
선택 필드 `planar_objects`는 [평면 객체 계약](PLANAR_OBJECTS_V1.md)을 따른다.
기존 points/cameras/quality/scale과 revision, segment_id, gauge_aligned를 유지한다.
1.2에서는 점군 인접 관계로 만들던 `surface`를 제거했다. 생산자는 선·삼각형 메시를 생성하지 않는다.
소비자는 오프라인 1.0과 기존 온라인 1.1도 읽되, 과거 `surface`와 상태의 표면 통계는 제공하지 않는다.
카메라의 position과 world_from_camera_rotation은 표시 좌표 기준이다. 모든 객체·점·경로는 같은 좌표를 사용한다.

- `source_pts_s`: 실제 소스 PTS 초. 알 수 없으면 null.
- `timestamp_s`: 선택한 시계의 초. 카메라는 이 값으로 정렬한다.
- `timestamp_source`: source_pts / nominal_fps_fallback / receiver_monotonic. 대체 시계를 촬영 시각으로 취급하지 않는다.
- `scale.status=arbitrary`, unit=arbitrary. gauge 정렬의 factor는 미터 보정값이 아니다.
- `segment_id`: 이전 지도와 좌표 정렬이 불가능하면 증가한다. 다른 segment를 연속 경로로 잇지 않는다.
- point_id는 해당 revision의 융합 셀 ID이며 native SIFT/영구 landmark ID가 아니다.
- `quality.mean_reprojection_error_px`는 현재 SfM 창의 평균이다. 누적 voxel 지도의 절대 정확도나 전체 재투영 오차가 아니다.
- `quality.current_window_points`와 `points3D`는 현재 SfM 점 수와 누적 융합 대표점 수를 구분한다.
- 카메라/미리보기 이미지는 현재 창과 유지 중인 면의 원본 시점을 포함한다. 창 밖의 원본 시점에는
  `retained_object_source=true`가 붙는다. `quality.registered_frames`는 현재 SfM 창의 등록 수다.
  과거 융합 점은 다음 창에도 남을 수 있다. 최대 융합 셀은 50,000개다.

`session/state.json`이 최신 공개 revision 경로의 유일한 포인터다. 각 revision 폴더의 scene·images·완료 상태를
먼저 완성한 후 포인터를 atomic 교체한다. 실패한 결과는 최신 포인터로 승격하지 않는다.
서버는 `revisions/6자리숫자` 경로만 허용한다. 원본 DB·로컬 경로 설정·실제 자료 폴더는 HTTP로 열지 않는다.
원본 입력 URI나 비밀 인증값은 상태/설정에 저장하지 않는다.

## 점군과 객체

누적 점군의 voxel 집계는 메모리 제한용으로만 유지한다. 공간 셀이나 점 사이의 추정 면을 표시하지 않는다.
`planar_objects`의 사각 앞면은 영상 경계와 native 2D↔3D 관측, 여러 시점의 기하 일치로 복원한다.
이는 점군의 최근접점 연결과 별개이며 [평면 객체 계약](PLANAR_OBJECTS_V1.md)을 따른다.
관측되지 않은 면·내부 부피·통과 가능한 공간은 추정하지 않는다.

## 상태와 읽기

`/api/state`: capture_state, mapping_state, status, map_available, revision, FPS·드롭·현재 창·오류/갱신 이력.
선택 `frame_tracking`은 처리한 수신 프레임 수/이어받은 특징점/새 코너 수/분석 해상도,
`frame_proposals`는 마지막 키프레임의 경계 수와 검출 시간이다. LK의 2D 트랙 수를 3D 점 수로 해석하지 않는다.
프레임 단위 계수는 메모리에서 갱신하고 파일 발행은 최대 10Hz로 합친다. 상태 전환·새 지도 발행은 즉시 저장한다.
`objects.update_mode=incremental_keyframe`은 새 관측 확정·기존 면/텍스처 유지 경로를 나타낸다.
`/api/run`: 점군 + 평면 객체 + 해당 포인터의 stream 상태. 초기 지도가 없으면 503과 상태 JSON.
웹은 상태를 먼저 확인하고 revision이 바뀌었을 때만 전체 지도를 읽는다.
`/api/frame/<frame_id>?revision=000001`은 보존된 해당 revision의 whitelist 프레임만 제공한다.

수신 실패/끊김과 EOF를 구분한다. 기존 지도가 있으면 실패 후에도 마지막 지도를 유지하되 상태를 정상으로 꾸미지 않는다.
파일 재생은 실제 드론·네트워크 지연 검증과 다르다. publish_lag_seconds는 로컬 수신→발행 지연이며 촬영→수신 지연은 제외한다.
실시간 SLAM의 루프 폐쇄/재위치, 실제 미터 스케일·장애물·비행 판단은 후속 실기체 검증 대상이다.
