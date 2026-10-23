# 좌표·위치 추정·3D 복원

휴대폰 프레임의 SfM/COLMAP 복원, 카메라 pose, 좌표·스케일·SLAM 코드를 둔다. 문의: 김수호. 실제 입력 계약은 contracts/mapping에서 관리한다.

## 실행 환경

검증 환경: Windows 11, Python 3.12.14, PyCOLMAP 4.2.1, NumPy 2.3.5, SciPy 1.17.0,
Pillow 12.3.0. CPU SfM을 사용한다. GPU·ROS·DB 서버·외부 업로드는 필요하지 않다.
저장소 루트에서 환경을 준비한다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/localization/requirements.txt
```

## 입력과 실제 복원

[확정 입력 snapshot](../../../contracts/mapping/PHONE_INPUT_SNAPSHOT_V1.md)을 검증한 후,
`--assets-root` 아래에서 DB와 31장 프레임을 SHA-256으로 찾는다. 현재 Drive의
`01_captures/`, `02_runs/` 구조와 기존 인계 패키지 구조를 모두 지원한다.
동일 자료 사본이 여러 개 있으면 더 좁은 루트를 지정한다. 원본 DB는 읽기 전용으로 대조하고
새 출력 폴더에 복사한 DB에서만 작업한다. 기존 출력 폴더를 덮어쓰지 않는다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/localization/reconstruct.py `
  --assets-root data/real_data `
  --output output/phone_mapping/2f1_sfm_review `
  --exclude-frame frame_0038.jpg
```

현재 촬영본에서는 첫 실행의 `frame_0038.jpg` 카메라 경로 급변과 A/B 기준점의 큰 재투영 불일치를
확인하고 위와 같이 한 장을 명시적으로 제외했다. 이 옵션은 모든 영상에 적용할 기본값이 아니다.
다른 입력은 `--run-dir`로 지정하고 제외 없이 먼저 검사한다. `--threads` 기본값은 4다.

출력은 native `sparse/`, 원본 임의 좌표의 `raw/`, 표시 좌표의 `points.ply`, `scene.json`,
`poses.json`, `trajectory.csv`, `scale.json`, `quality.json`, 설정·출처·완료 상태다.
소비자는 [복원 결과 계약](../../../contracts/mapping/PHONE_SFM_RESULT_V1.md)을 사용한다.
`local_assets.json`의 절대 경로는 로컬 웹 연결 전용이며 공유 계약에 포함하지 않는다.

44cm A/B를 등록된 3개 이상 시점에서 왜곡을 고려해 삼각측량한다. 충분한 시차·전방 깊이·6px 이하
재투영 오차를 충족하면 같은 similarity 변환을 모든 점·카메라 중심·방향에 적용한다.
충족하지 않으면 오류 이유를 보존하고 `arbitrary` 단위로 내보낸다. 미확인 바닥 C는 사용하지 않는다.
44cm에 맞췄다는 사실은 독립 치수 정확도를 검증했다는 뜻이 아니다.

## 웹 화면

웹 실행은 [console 안내](../../../apps/console/README.md)를 따른다.

원본·실제 점군·로그는 ignore된 `data/real_data/`, `output/`에 둔다.
팀 작업 절차는 저장소 루트의 `docs/team/START_HERE.md`를 따른다.

## 연속 영상·공간 구조화

[온라인 처리 안내](ONLINE.md)는 파일/RTSP/UDP/카메라 수신, 제한된 키프레임 SfM 갱신,
voxel 대표점·연결선·삼각형 표면과 웹 탐색을 다룬다. 기존 오프라인 수신/보정 경로는 유지한다.
실시간 위치 추적·루프 폐쇄가 필요한 드론 SLAM의 완료를 의미하지 않는다.

## 새 영상 입력과 연결 (파일 파이프라인 4단계)

2단계의 [영상 입력](../../../contracts/mapping/PHONE_VIDEO_INPUT_V1.md)과 3단계의
[특징점 결과](../../../contracts/mapping/PHONE_FEATURE_RESULT_V1.md)를 받는 별도 실행 경로다.
위의 기존 snapshot/44cm 보정 실행은 유지한다. 새 입력 경로에는 실측 파일이나 기존 실험의 제외 프레임을 적용하지 않는다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/localization/requirements.txt
.\.venv\Scripts\python.exe -B modules/mapping/localization/reconstruct.py `
  --input-dir output/phone_mapping/hallway_20261023_run01/input `
  --features-dir output/phone_mapping/hallway_20261023_run01/features `
  --output output/phone_mapping/hallway_20261023_run01/reconstruction
```

세 폴더는 서로 분리하고 출력은 존재하지 않는 새 경로를 사용한다. 새 입력 옵션 두 개는 함께 지정하며
--run-dir/--assets-root와 혼용하지 않는다. Python에서는 reconstruct(output, input_dir=..., features_dir=...)를 호출한다.
전체 이미지 해시·PTS·DB/CSV 참조·카메라 크기를 검사하고 입력 DB 사본으로만 CPU SfM을 실행한다.
카메라 내부 보정은 3단계 초기값에서 추정하며, 원래 카메라의 교정 정확도를 보장하지 않는다.

- scale.json: unit=arbitrary, reason=no_metric_reference. 좌표·경로에 같은 임의 좌표계를 사용한다.
  m·cm, 실제 이동 속도, 절대 방향이나 실제 높이로 해석할 수 없다.
- status.json: complete는 제외 후 제출한 모든 프레임이 선택 모델에 등록된 상태,
  partial은 선택 모델이 있지만 미등록 제출 프레임이 있는 상태다. 빈 복원은 failed다.
  complete도 정확한 지도나 치수 검증 완료를 뜻하지 않는다.
- quality.json: 등록 수/비율, 미등록/명시적 제외 목록, 모델별 프레임·점 수와 선택 모델을 확인한다.
  가장 많은 카메라(동률이면 가장 많은 점)의 모델을 선택한다. 다른 모델은 sparse/<model_id>/에 별도로 남기며 합치지 않는다.
- points.ply / scene.json: 선택 모델의 희소 3D 점. poses.json / trajectory.csv: 같은 좌표계의 카메라 경로와 원본 PTS.
- input_validation.json / provenance.json: 생산 입력 전체 대조, 입력/DB/메타데이터 해시, 회전·축소 변환 출처.
- timing.json: 입력 검사·DB 복사/SfM·색상/내보내기 및 합계. 설치·영상 추출·특징점 생성·웹 표시와 마지막 timing 쓰기는 제외한다.
  quality.processing_seconds는 기존 소비자 호환용으로 입력 검사 후 DB 복사부터 색상 추출까지의 값이다.
- 뷰어는 complete/partial 결과를 열고 부분 복원·임의 단위를 표시한다. local_assets.json은 로컬 이미지 연결용이다.
  프레임을 옮겼으면 images_root를 현재 경로로 바꿔야 하며, native 결과/좌표는 이 연결과 독립적이다.

검사 (합성 3D 모델의 좌표/자세·기존 실측 경로·부분/실패·출력 보호·뷰어 HTTP):

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s modules/mapping/localization/tests -v
```

2026-10-22 제작자 로컬 검사: 기존 2층 영상에서 다시 추출한 56장 중 선택 모델에 45장(80.4%),
3D 점 6,508개가 등록됐다. 11장 미등록으로 partial이며 별도 작은 모델도 보존했다.
이 수치는 4단계 당시 실측 없는 입력 연결 검사다. 이후 전체 단일 명령/PNG 종단 검증은
[검증 요약](../../../docs/phone_mapping/validation_summary.md)을 따른다. 새 장소 영상은 아직 검사하지 않았다.
재투영 오차는 입력 관측에 대한 일치 정도이며 실제 공간 정확도의 검증값이 아니다.
