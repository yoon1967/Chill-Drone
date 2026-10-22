# 휴대폰 SfM 복원 결과 1.0

생산자: `modules/mapping/localization/reconstruct.py`.
소비자: `apps/console/`, 후속 지도 저장·미리보기 도구.
입력은 `PHONE_INPUT_SNAPSHOT_V1.md`의 기존 회차/프레임 ID를 보존한다.
공통 PoC 교환 형식으로 자동 변환하지 않는다.

## 파일과 상태

| 파일 | 내용 |
| --- | --- |
| `status.json` | running / failed / complete. complete 결과만 웹에서 연다 |
| `scene.json` | `schema_version=phone-sfm-result/1.0`, 회차·시각·점·카메라·스케일·품질 |
| `points.ply` | ASCII PLY. 표시 좌표의 XYZ, 원본 영상에서 평균한 RGB 0~255 |
| `poses.json` | scene의 points/point_fields를 제외한 메타데이터와 카메라 |
| `trajectory.csv` | frame_id, source_pts_s, x/y/z, unit, pose_source |
| `scale.json`, `quality.json` | scene과 같은 스케일/품질 필드 |
| `settings.json`, `provenance.json` | 고정 도구·설정·검증 결과·입출력 해시·실행 환경 |
| `raw/`, `sparse/` | native COLMAP 모델. 임의 SfM 좌표/단위이며 표시 좌표와 구분 |
| `local_assets.json` | 원본 프레임의 로컬 연결. 절대 경로 포함, 공유/HTTP 공개 제외 |

실제 촬영에서 파생된 위 파일은 모두 로컬 `output/` 안에 보관한다. Git에는 생성 코드·명세·집계 기록만 올린다.
`points` 한 행은 `[point_id,x,y,z,r,g,b,error_px,track_length]`다.
point_id는 native COLMAP ID이고 SIFT 후보 트랙 ID와 다르다.
등록된 카메라는 원본 영상 `source_pts_s`의 초 단위 PTS 순서로 정렬한다. 절대 촬영 시각은 생성하지 않는다.

## 카메라와 좌표

`pose_source=sfm`. 카메라 로컬 축은 COLMAP의 x 오른쪽/y 아래/z 전방이다.
카메라마다 frame_id, image_id, source_pts_s, camera_id, model, width/height, params를 기록한다.
SIMPLE_RADIAL의 params는 `[f,cx,cy,k]`이며 실측 내부 보정이 아닌 영상 기반 추정이다.

native 자세의 방향은 다음과 같다.

```text
camera_xyz = cam_from_world_rotation_raw @ raw_world_xyz + cam_from_world_translation_raw
raw_camera_center = -transpose(cam_from_world_rotation_raw) @ cam_from_world_translation_raw
display_xyz = factor * scale.rotation @ raw_world_xyz + scale.translation
display_camera_center = factor * scale.rotation @ raw_camera_center + scale.translation
world_from_camera_rotation = scale.rotation @ transpose(cam_from_world_rotation_raw)
```

`position`은 표시 좌표의 카메라 중심이다. `world_from_camera_rotation`의 열은 표시 좌표에서 표현한
카메라 로컬 x/y/z 축이다. 점과 카메라에 같은 similarity 변환을 적용한다. 회전 행렬은 우수 좌표계의
직교 행렬이며 determinant=+1이다. raw 카메라 translation을 표시 좌표 중심으로 잘못 읽지 않는다.

미터 표시 좌표는 A가 원점, x는 A→B, y는 평균 카메라 화면의 위 방향에서 x 성분을 제거한 축,
z는 x×y다. y는 실측 중력 방향이 아니며 표시 격자는 바닥 추정 결과가 아니다.

## 스케일과 품질

- `scale.status=reference_scaled`, `unit=m`: 등록된 3시점 이상 A/B 관측을 왜곡 보정 후 삼각측량했다.
  모든 사용 관측은 양의 깊이, 최대 시차 1.5° 이상, 최대 재투영 오차 6px 이하를 충족해야 한다.
  `factor=0.44/raw_reference_length`. 이 6px는 처리 허용 기준이며 실제 치수 오차 허용치가 아니다.
- 조건을 충족하지 않으면 `status=arbitrary`, `unit=arbitrary`, factor=1·항등 변환과 이유를 기록한다.
- 관측별 오차·사용 시점 수·시차를 `anchor_quality`에 남긴다. C는 입력 단계부터 비활성 상태를 검사한다.
- `independent_length_validation=not_performed_unconfirmed_floor_C`.
  44cm 보정 수치를 독립 치수 정확도로 재사용하지 않는다. 실측 오차는 미상이므로 null이다.
- `input_frames`는 원본 snapshot 수, `submitted_frames`는 제외 후 실행 입력 수,
  `registered_frames`는 선택한 모델의 카메라 수다. 등록률의 분모는 전체 원본 수다.
- `excluded_frames`는 실행자가 명시한 제외, `unregistered_frames`는 전체 원본에서 선택한 모델에 없는 프레임이다.
- 모든 모델의 카메라/점 수와 선택 여부를 `components`에 기록한다. 다른 모델의 좌표는 합쳐 쓰지 않는다.
- `mean_point_reprojection_error_px`는 COLMAP 점별 평균 픽셀 오차의 산술평균이며 독립 위치 오차·관측 RMSE가 아니다.
  median/p95 역시 점별 평균 오차의 분포다. `mean_reprojection_error_px`는 같은 COLMAP 집계다.
- `processing_seconds`는 입력 대조 이후 SfM·색상 추출까지이며 설치/입력 검사/웹 표시 시간은 포함하지 않는다.
- `kind=sparse_sfm`, dense_reconstruction=not_performed, absolute_pose_accuracy=unknown_no_ground_truth.

수신자는 `validate_result.py`로 해시, PLY/JSON, 원본 프레임·PTS, CSV, 중심·방향의 변환을 독립 확인한다.
계약 생산자·소비자를 함께 변경할 때는 이 버전의 필드와 단위를 보존하거나 새 버전을 만든다.
