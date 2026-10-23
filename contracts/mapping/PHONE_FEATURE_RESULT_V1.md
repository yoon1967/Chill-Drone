# 파일 입력의 2D 특징점·매칭 결과 1.0

생산자: `scripts/phone_mapping/full_features.py`의 `run_features()`.
검사기: `scripts/phone_mapping/validate_features.py`의 `validate_features()`.
생성 버전은 PyCOLMAP 4.2.1이다. 새 결과 metadata의 schema_version은 `phone-feature-result/1.0`이다.

## 입력과 좌표

`--input-dir`은 [새 영상 입력 1.0](PHONE_VIDEO_INPUT_V1.md)의 manifest·frames·frame_decisions를 받는다.
기존 첫 실험의 runId/frames_first_test 형식도 읽는다. `feature_input.py`가 두 형식을 검증·정규화한다.
최소 3장, 중복 없는 파일 이름, 실제 JPEG 크기/해시, 유한하고 증가하는 원본 PTS를 검사한다.
같은 SIMPLE_RADIAL 카메라를 쓰므로 한 입력 안의 처리 이미지 크기는 같아야 한다.

특징점 좌표는 저장된 이미지 픽셀 기준이다. 새 입력에서는 회전/축소가 이미 적용된 `processed_image`,
기존 입력에서는 `original_image`이다. 첫 픽셀 중심은 `(0.5,0.5)`, x 오른쪽/y 아래다.
원본 coded 좌표 변환은 새 입력 manifest 및 feature_metadata의 input_transforms를 참조한다.
카메라 크기는 실제 처리 이미지 크기이며, 원본 영상 크기로 잘못 재해석하지 않는다.
초기 내부 파라미터는 추정값이다. 실측 보정이나 미터 스케일을 생성하지 않는다.

## 출력

| 파일 | 내용 |
| --- | --- |
| `database.db` | SIFT 특징점·기술자·매칭·두 시점 기하·카메라/이미지의 native DB |
| `keypoints.csv` | run_id,image_id,frame_id,source_pts_s,keypoint_id,x_px,y_px |
| `matches.csv` | frame_id_a,keypoint_id_a,frame_id_b,keypoint_id_b,geometric_inlier |
| `candidate_tracks.csv` | 같은 프레임의 점이 중복되지 않는 2D 대응 연결 묶음 |
| `frames_manifest.json` | 프레임 이름·DB ID·PTS·크기·체크섬·점 수 |
| `feature_metadata.json` | 입력 형식/manifest 해시·설정·카메라·좌표 규칙·DB 해시·변환 출처 |
| `quality_report.json` | 프레임/쌍 통계·연결망·약한 구간·한계 |
| `validation_report.json` | DB/CSV/프레임/후보 트랙 전체 대조 결과 |
| `pairs.txt`, `match_previews/` | 시각 순서에 따른 시도 쌍과 인접 쌍 확인 그림 |
| `status.json`, `timing.json`, `execution.log` | 이번 단계의 상태·시간 범위·로그 |

기존 CSV 열과 DB 인덱스 의미를 유지한다. `keypoint_id`는 이미지별 0 기반 DB 행 인덱스이고,
후보 트랙 ID는 3D 점 ID가 아니다. CSV에는 DB의 SIFT 기술자/두 시점 기하 전체가 없으므로
SfM에는 DB도 함께 제공한다. 매칭 창 기본값은 선택 프레임 4장 앞까지이며 31장/114쌍 고정 조건은 없다.

## 보호와 상태

새 실행의 출력 폴더는 존재하지 않아야 한다. 기존 출력은 명시적인 `--export-only`에서만 다시 내보낸다.
이 옵션은 기존 설정과 feature_run_id를 유지하고, 새 입력 형식은 manifest가 달라지면 거부한다.
metadata에 DB 해시가 있으면 재내보내기 전에 일치하는지 검사한다.
추출/매칭 후 DB를 standalone 파일로 마무리하고, 내보내기·검사는 임시 DB 사본에서 수행한다.
원본 영상은 2D 단계에 필수 파일이 아니며 준비한 프레임/manifest만으로 실행할 수 있다.

모든 이미지의 특징점이 존재하고 DB/CSV/입력 대조가 통과하면 `status=features_ready`다.
한 이미지라도 특징점이 없거나 참조/해시/좌표/DB 검사가 실패하면 `failed`로 남긴다.
프레임 연결망이 나뉘거나 기하가 평면에 가까운 경우는 quality_report에 기록한다.
features_ready는 3D 복원 성공을 보장하지 않는다. 새 입력과 reconstruct.py의 연결은 다음 단계다.

시간은 입력 검증부터 전체 DB/CSV 대조까지이며 마지막 timing/log 쓰기는 제외한다.
feature_extraction_seconds, matching_and_geometry_seconds, export_seconds, validation_seconds를 구분한다.
export-only에서는 특징 추출·매칭 시간은 null이며 과거 처리 시간을 이번 실행 시간으로 재사용하지 않는다.
