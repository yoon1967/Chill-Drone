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
