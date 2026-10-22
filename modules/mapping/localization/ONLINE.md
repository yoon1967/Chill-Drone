# 연속 영상에서 탐색 가능한 3D 구조까지

Python 3.12, PyCOLMAP 4.2.1, NumPy 2.3.5, OpenCV headless 4.13.0.92,
Open3D 0.19.0, SciPy 1.17.0을 사용한다. Windows 11 CPU에서 검증했다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/localization/requirements-online.txt
```

## 1. 기존 미터 점군을 공간 셀·선·면으로 만들기

기존 `scene.json`을 변경하지 않고 새 결과에 저장한다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/localization/surface.py `
  --scene output/phone_mapping/2f1_sfm_v3/scene.json `
  --output output/phone_mapping/my_surface.json `
  --voxel-size 0.03 `
  --mesh-output output/phone_mapping/my_surface.ply
```

기존 미터 보정 결과에서 0.03은 3cm다. 미보정 단안 지도에서는 **임의 단위 0.03**이며 cm가 아니다.
`--voxel-size`를 생략하면 미터 지도는 3cm, 임의 단위는 관측점 바운딩 박스 대각선의 1/200을 사용한다.
메시 PLY는 Blender/MeshLab에서 불러올 수 있다. Blender MCP는 처리 서버의 필수 의존이 아니다.

알고리즘은 다음과 같다.

1. 점의 평균 재투영 오차 1.5px 이하, 트랙 길이 3 이상인 관측점을 선택한다.
2. 각 축의 `floor(x / cell_size)`를 키로 같은 셀의 점·색상을 평균한다. 빈 셀에 점을 생성하지 않는다.
3. 주변 점의 PCA 법선을 Open3D로 추정하고 가까운 관측 카메라 방향으로 맞춘다.
4. 셀 크기와 최근접점 간격 중 큰 값을 base로 정한다. 거리 4×base 이내, 법선 차이 50° 이하인 점만
   가까운 순서로 연결한다. 한 점의 연결 수는 최대 8개다.
5. Open3D Ball Pivoting(반경 1.5×base, 2×base)으로 면을 만든다. 긴 변, 퇴화 면, 법선 차이가 큰 면은 제거한다.
6. 셀별 관측 수, 원본 지도의 바인딩 해시, 거리 한계와 단위를 기록한다. 표면은 측정값 사이의 **추정 연결**이다.

이 과정은 완전한 방/벽을 생성하는 장면 완성 모델이 아니다. 희소 관측만 있으면 표면도 부분적이며,
가구 내부·유리 반사·무늬 없는 면의 복원은 제한된다. 빈 영역을 통과 가능한 공간으로 해석하지 않는다.

## 2. 동영상 수신과 온라인 갱신

원본 동영상 경로를 지정한다. 기존 snapshot의 프레임 이름이나 수동 제외 목록은 사용하지 않는다.

```powershell
.\.venv\Scripts\python.exe -u -B modules/mapping/localization/online.py `
  --source "data/real_data/내_촬영_폴더/source_video.mp4" `
  --session output/phone_mapping/my_online_run
```

`--source`는 존재하는 영상 파일, RTSP/UDP URL, 카메라 번호 문자열을 받는다. URL의 토큰·암호는
공유 문서나 커밋에 넣지 않는다. Windows 명령 기록에도 민감한 URI를 직접 남기지 않도록 운영 환경에서 주입한다.
실제 드론 URI·수신 포트·코덱은 장비 제공 후 확정해야 한다. 지금은 파일 재생만 실제 검증했다.

| 설정 | 기본값 | 의미 |
| --- | --- | --- |
| `--sample-fps` | 2 | 키프레임 후보의 최대 샘플 속도 |
| `--replay-rate` | 1 | 파일은 원래 재생 속도. 0은 가능한 빠르게 읽는 부하 시험 |
| `--width` | 960 | 처리 이미지의 최대 가로 픽셀. 비율 유지 |
| `--min-sharpness` | 20 | Laplacian 분산의 최소값. 기기/해상도별 재검증 필요 |
| `--max-keyframes` | 24 | 계산 창과 임시 키프레임 파일의 상한 |
| `--batch-size` | 8 | 새 키프레임 8장마다 최신 지도 계산 요청 |
| `--threads` | 4 | COLMAP CPU 스레드 |
| `--keep-revisions` | 3 | 최근 실행 폴더 수. 현재 유효 지도는 추가 보존 가능 |
| `--max-seconds` | 0 | 0은 입력 종료/사용자 중단까지 실행 |
| `--voxel-size` | 자동 | 같은 segment 안에서 고정되는 공간 셀 크기 |

수신 스레드 → 최신 프레임 슬롯 1개 → 시간/흐림/중복 검사 → 키프레임 창 → 최신 작업 슬롯 1개 →
COLMAP SIFT·순차 매칭·RANSAC·SfM → gauge 정렬 → voxel 지도 융합 → 선/면 생성 → atomic 스냅샷 순서다.
처리가 밀리면 오래된 프레임/계산 요청을 대체한다. 융합 지도는 최대 50,000개 셀 대표점으로 제한한다.
겹치는 계산 창을 독립된 추가 관측으로 세지 않으며 트랙 길이를 합산하지 않는다.

카메라의 국소 급이탈은 전체 이동 간격의 중앙값과 이웃 경로의 우회 비율로 검사한다.
프레임 이름에 따른 예외는 없다. 고속 왕복 비행을 오탐할 수 있어 이 규칙은 실기체 평가가 필요하다.
공통 카메라 3개 이상의 방향·중심으로 지도 좌표계를 맞춘다. 정렬 불일치 때는 새 segment로 시작하고
이전 지도를 같은 좌표로 합치지 않는다. 루프 폐쇄·재위치·절대 스케일 보정은 제공하지 않는다.

EOF, 단절, 초기화/갱신 실패, 사용자 중단을 상태로 구분한다. 유효 지도가 없는 입력 실패는 종료 코드 2다.
네트워크 단절 때 마지막 지도를 보존하고 연결을 끝낸다. 자동 재접속/재위치는 후속 SLAM 연동 범위다.
`state.json`에는 FPS·드롭·갱신 시간·수신 후 발행 지연·최근 32건의 갱신 기록을 남긴다.
source PTS를 얻지 못하면 null과 대체 시계 종류를 기록한다. 수신 시각을 촬영 PTS로 꾸미지 않는다.

## 3. 웹 연결

```powershell
# 저장된 미터 지도 + 표면
.\.venv\Scripts\python.exe apps/console/serve.py `
  --result output/phone_mapping/2f1_sfm_v3 --surface output/phone_mapping/my_surface.json --port 8790

# 온라인 지도. 수신 전에도 시작할 수 있다.
.\.venv\Scripts\python.exe apps/console/serve.py --session output/phone_mapping/my_online_run --port 8791
```

[온라인 공간 계약](../../../contracts/mapping/ONLINE_SPATIAL_V1.md)을 참고한다.
영상 수신 약 30fps와 SfM 수초 갱신은 서로 다른 성능이다. 드론 비행용 실시간 SLAM이라고 보고하지 않는다.
