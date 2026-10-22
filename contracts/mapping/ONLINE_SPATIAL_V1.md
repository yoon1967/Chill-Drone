# 온라인 지도·관측 표면 계약 1.0

생산자: `modules/mapping/localization/online.py`, `surface.py`.
소비자: `apps/console/serve.py`, Three.js 공간 화면. 기존 오프라인 1.0 결과와 수치 snapshot은 변경하지 않는다.

## 지도 스냅샷

온라인 scene은 `schema_version=phone-sfm-result/1.1`, pose_source=sfm이다.
기존 points/cameras/quality/scale을 유지하며 surface, revision, segment_id, gauge_aligned를 추가한다.
카메라의 position과 world_from_camera_rotation은 표시 좌표 기준이다. 모든 표면·셀·점·경로는 같은 좌표를 사용한다.

- `source_pts_s`: 실제 소스 PTS 초. 알 수 없으면 null.
- `timestamp_s`: 선택한 시계의 초. 카메라는 이 값으로 정렬한다.
- `timestamp_source`: source_pts / nominal_fps_fallback / receiver_monotonic. 대체 시계를 촬영 시각으로 취급하지 않는다.
- `scale.status=arbitrary`, unit=arbitrary. gauge 정렬의 factor는 미터 보정값이 아니다.
- `segment_id`: 이전 지도와 좌표 정렬이 불가능하면 증가한다. 다른 segment를 연속 경로로 잇지 않는다.
- point_id는 해당 revision의 융합 셀 ID이며 native SIFT/영구 landmark ID가 아니다.
- `quality.mean_reprojection_error_px`는 현재 SfM 창의 평균이다. 누적 voxel 지도의 절대 정확도나 전체 재투영 오차가 아니다.
- `quality.current_window_points`와 `points3D`는 현재 SfM 점 수와 누적 융합 대표점 수를 구분한다.
- 카메라/미리보기 이미지는 현재 창 범위다. 과거 융합 점은 다음 창에도 남을 수 있다. 최대 융합 셀은 50,000개다.

`session/state.json`이 최신 공개 revision 경로의 유일한 포인터다. 각 revision 폴더의 scene·images·완료 상태를
먼저 완성한 후 포인터를 atomic 교체한다. 실패한 결과는 최신 포인터로 승격하지 않는다.
서버는 `revisions/6자리숫자` 경로만 허용한다. 원본 DB·로컬 경로 설정·실제 자료 폴더는 HTTP로 열지 않는다.
원본 입력 URI나 비밀 인증값은 상태/설정에 저장하지 않는다.

## surface

`schema_version=observed-surface/1.0`, method=voxel_centroid+normal_graph+open3d_ball_pivoting.

| 필드 | 의미 |
| --- | --- |
| scene_binding_sha256 | points/cameras/scale의 정렬된 canonical JSON 해시. 다른 지도 표면 연결 거부 |
| unit, voxel_size | 지도 단위와 고정 공간 셀 크기 |
| positions, colors, normals | 대표점 XYZ, RGB 0~255, 단위 법선 |
| cells | floor(XYZ/voxel_size)로 얻은 정수 셀 좌표 |
| sample_counts | 셀에 묶인 입력 점 수. 독립 사진/광선 관측 수가 아님 |
| edges | 대표점 인덱스 쌍. 가까운 점, 법선 차이, 차수 상한을 충족 |
| triangles | 대표점 인덱스 세 개. Ball Pivoting 후 긴 변·퇴화·법선 불일치를 제거 |
| max_edge_length | 4×max(voxel_size, 최근접 거리 중앙값) |
| statistics | 원본/지원점/대표점/선/면 수와 구조화 시간 |

원본 점군이 희소하면 표면도 부분적이다. 표면은 관측점 사이의 추정 연결이며 충돌 검증 메시가 아니다.
Poisson/생성 모델로 관측되지 않은 면을 채우지 않는다. 모든 미관측 셀은 unknown이며 free가 아니다.
가려진 구조·반사·연기·동적 객체·얇은 물체의 안전한 형상 복원을 보장하지 않는다.

## 상태와 읽기

`/api/state`: capture_state, mapping_state, status, map_available, revision, FPS·드롭·현재 창·오류/갱신 이력.
`/api/run`: 지도 + surface + 해당 포인터의 stream 상태. 초기 지도가 없으면 503과 상태 JSON.
웹은 상태를 먼저 확인하고 revision이 바뀌었을 때만 전체 지도를 읽는다.
`/api/frame/<frame_id>?revision=000001`은 보존된 해당 revision의 whitelist 프레임만 제공한다.

수신 실패/끊김과 EOF를 구분한다. 기존 지도가 있으면 실패 후에도 마지막 지도를 유지하되 상태를 정상으로 꾸미지 않는다.
파일 재생은 실제 드론·네트워크 지연 검증과 다르다. publish_lag_seconds는 로컬 수신→발행 지연이며 촬영→수신 지연은 제외한다.
실시간 SLAM의 루프 폐쇄/재위치, 실제 미터 스케일·장애물·비행 판단은 후속 실기체 검증 대상이다.
