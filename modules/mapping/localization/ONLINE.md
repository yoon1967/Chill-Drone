# 연속 영상에서 탐색 가능한 3D 구조까지

Python 3.12, PyCOLMAP 4.2.1, NumPy 2.3.5, OpenCV headless 4.13.0.92,
SciPy 1.17.0을 사용한다. Windows 11 CPU에서 검증했다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/localization/requirements-online.txt
```

## 1. 동영상 수신과 온라인 갱신

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
| `--batch-size` | 8 | 초기 자세 계산을 시작할 최소 키프레임 수. 이후 새 키프레임마다 최신 계산 요청 |
| `--threads` | 4 | COLMAP CPU 스레드 |
| `--keep-revisions` | 3 | 최근 실행 폴더 수. 현재 유효 지도는 추가 보존 가능 |
| `--max-seconds` | 0 | 0은 입력 종료/사용자 중단까지 실행 |
| `--voxel-size` | 자동 | 누적 점군의 메모리 제한에 쓰는 크기. 선·면 생성이나 셀 표시에 사용하지 않음 |
| `--plane-profile` | balanced | 평면 검출 범위: balanced / coverage / strict |
| `--plane-error-px` | profile 값 | 관측 거리·초점거리로 환산하는 평면 허용 거리의 픽셀 계수, 1~12 |

수신 스레드 → 최신 프레임 슬롯 1개 → 프레임별 LK 특징점 추적 → 시간/흐림/중복 검사 →
별도 최신 요청 슬롯에서 새 키프레임의 경계 후보 1회 검출 → 키프레임 창 → 최신 작업 슬롯 1개 →
COLMAP SIFT·순차 매칭·RANSAC·SfM → gauge 정렬 → 새 관측의 증분 평면 확정과 텍스처 추가 →
누적 점군 융합 → atomic 스냅샷 순서다. 점군을 인접 관계만으로 연결하는 선·삼각형 생성은 제거했다.
처리가 밀리면 오래된 프레임/계산 요청을 대체한다. 융합 지도는 최대 50,000개 대표점으로 제한한다.
`--voxel-size`를 생략하면 같은 segment에서 처음 관측한 점군 바운딩 박스 대각선의 1/200을 고정한다.
점군 융합에는 이 값의 절반 간격을 사용한다. 단안 지도의 값은 미터가 아닌 임의 단위다.
겹치는 계산 창을 독립된 추가 관측으로 세지 않으며 트랙 길이를 합산하지 않는다.

카메라의 국소 급이탈은 전체 이동 간격의 중앙값과 이웃 경로의 우회 비율로 검사한다.
프레임 이름에 따른 예외는 없다. 고속 왕복 비행을 오탐할 수 있어 이 규칙은 실기체 평가가 필요하다.
공통 카메라 3개 이상의 방향·중심으로 지도 좌표계를 맞춘다. 정렬 불일치 때는 새 segment로 시작하고
이전 지도를 같은 좌표로 합치지 않는다. 루프 폐쇄·재위치·절대 스케일 보정은 제공하지 않는다.

EOF, 단절, 초기화/갱신 실패, 사용자 중단을 상태로 구분한다. 유효 지도가 없는 입력 실패는 종료 코드 2다.
네트워크 단절 때 마지막 지도를 보존하고 연결을 끝낸다. 자동 재접속/재위치는 후속 SLAM 연동 범위다.
`state.json`에는 FPS·드롭·갱신 시간·수신 후 발행 지연·최근 32건의 갱신 기록을 남긴다.
source PTS를 얻지 못하면 null과 대체 시계 종류를 기록한다. 수신 시각을 촬영 PTS로 꾸미지 않는다.

## 2. 웹 연결

```powershell
# 저장된 미터 점군
.\.venv\Scripts\python.exe apps/console/serve.py `
  --result output/phone_mapping/2f1_sfm_v3 --port 8790

# 온라인 지도. 수신 전에도 시작할 수 있다.
.\.venv\Scripts\python.exe apps/console/serve.py --session output/phone_mapping/my_online_run --port 8791
```

[온라인 공간 계약](../../../contracts/mapping/ONLINE_SPATIAL_V1.md)을 참고한다.
영상 수신 약 30fps와 SfM 수초 갱신은 서로 다른 성능이다. 드론 비행용 실시간 SLAM이라고 보고하지 않는다.

## 3. 영상 경계를 가진 평면 객체

기존 결과의 native `sparse/` 모델과 `local_assets.json`이 가리키는 원본 프레임을 사용한다.
새 결과 폴더에 객체 JSON과 정사 보정 JPEG 표면을 생성하며 원본 지도·사진을 수정하지 않는다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/localization/planar.py `
  --result output/phone_mapping/2f1_sfm_v3 `
  --output output/phone_mapping/my_planar_objects --max-frames 30

.\.venv\Scripts\python.exe -B apps/console/serve.py `
  --result output/phone_mapping/2f1_sfm_v3 `
  --objects output/phone_mapping/my_planar_objects --port 8790
```

`--max-frames`는 3~60, 기본 12다. 분석 영상은 최대 폭 960px로 왜곡 보정한다.
native landmark의 2D 관측과 변환된 3D 좌표를 연결하고 RANSAC/SVD 평면에 맞는 지지점을 고른다.
닫힌 사각 윤곽과 이어진 유한 직선으로 긴 문·작은 칸·얇은 가로 면을 각각 제안한다.
사각 영역을 정사 보정하고 내부를 가로지르는 관측 이음새가 있을 때만 칸을 분리한다. 정해진 간격으로 임의 분할하지 않는다.
직선 결합은 영상에서 대체로 수평·수직인 경계를 가정한다. 임의 회전은 닫힌 윤곽 경로만 적용될 수 있다.
경계 근처 지지점을 우선하며 전역 평면이 설명하지 못하는 영역에는 국소 평면을 추가한다.
카메라 보정값과 사각형의 두 소실 방향으로 평면 법선이 일치하는지 검사한다. 경계의 네 변을 각각 검증하고,
3개 이상 시점의 경계 일치와 카메라 이동을 요구한다. 지지 없는 경계나 열린 곡면은 채우지 않는다.
최대 3개의 관측 평면에 여러 영상의 경계를 정렬하고, 최소 3시점·45% 이상에서 반복되는 선으로
추가 사각형을 제안한다. 평면당 최대 24개를 선택하고 지지가 가장 많은 원본 시점에서 재검증한다.
경계 띠에 무늬가 충분하면 정사 보정 영상의 밝기 상관도 함께 확인해 다른 깊이의 선·반사를 줄인다.
큰 유효 면을 우선하고 동일 위치의 평행한 중복 조각은 제거한다. 인접한 면을 공통 특징점만으로 합치지 않는다.
평면 허용 거리는 카메라 초점거리·관측 거리·픽셀 오차로 추정하며 실측 정확도 보증값이 아니다.
원본 영상의 해당 사각형을 평면에 정사 보정한 뒤 JPEG 텍스처로 저장한다. 투명 영역의 외관도 포함되므로
사진에 보이는 내부·반사상이 3D 부피나 통과 가능한 공간으로 측정됐다는 뜻은 아니다.

온라인은 `streaming_planes.py`의 증분 지도를 사용한다. 수신 슬롯에서 꺼낸 각 프레임을 폭 480px 이하에서
LK 정방향·역방향 오차 1px 미만으로 추적한다. 기존 점 주변 7px는 새 코너 검출에서 제외하고 최대 500점을 유지한다.
흐림·시간·영상 변화 검사로 선택된 키프레임만 별도 작업 스레드에서 최대 폭 640px로 경계 검출을 한 번 수행한다.
추적을 기다리게 하지 않으며 검출이 밀리면 오래된 요청을 최신 요청으로 대체하고 횟수를 기록한다. 따라서 모든 입력 프레임마다
3D 면을 계산하는 방식이 아니다. 밀린 수신 프레임은 기존 latest-slot 정책에 따라 건너뛰며 드롭 수를 기록한다.

자세 계산 후 새로 등록된 프레임과 갱신당 최대 2개의 미확정 프레임만 면 후보를 확정한다.
미확정 후보는 같은 창에서 최대 3회 시도한다. 첫 프레임의 2D 후보는 깊이가 확정된 3D 면으로 취급하지 않는다.
원본 경계 후보는 현재 카메라 보정값으로 왜곡 보정해 재사용하고, 기존 프레임의 사각형 검출·전체 영상 합의 atlas를
반복하지 않는다. 확인된 유한 면 내부의 3D 지지점은 전역 RANSAC 입력에서 제외하며 기존 평면 방정식은
새 영역의 깊이 가설로 재사용한다. 경계 90% 포함, 법선 차이 8° 이내, 같은 깊이 지지 70% 이상일 때
기존 면으로 건너뛴다. 영상 좌표 또는 native landmark ID만으로 다른 면을 합치지 않는다.

확정한 면의 ID·좌표·텍스처는 유지하고 새 면에만 JPEG를 만든다. 최대 32면이며 상한 도달 여부를 기록한다.
이전 면의 텍스처와 원본 시점 이미지를 새 revision으로 복사해 계산 창·이전 revision 정리 후에도 선택할 수 있다.
원본 시점 카메라에는 `retained_object_source=true`를 붙여 현재 자세 계산 창과 구분한다.
같은 segment에서만 유지하며 정렬 실패로 새 segment가 되면 이전 면을 합치지 않는다. 새 출력이 실패하면
마지막 유효 지도·면을 보존한다. 지도 준비 중에는 아직 면이 없을 수 있다.

`scene.planar_objects`와 revision의 `object_textures/`를 함께 발행한다. 프레임 추적/경계 검출은 수신 단계지만
3D 확정·웹 발행은 비동기 키프레임 SfM 완료 시점이다. 새 요청은 매 키프레임마다 넣어도 CPU 계산과 웹 2초
확인 간격만큼 지연된다. COLMAP 자세 계산 자체는 여전히 창을 재구성하므로 고주파 실시간 드론 운용에는
후속 SLAM 자세 공급·루프 폐쇄·동적 객체 처리가 필요하다. 추가 계산 시간은 전체 지도 갱신 지연에 포함된다.
카메라 모델은 SIMPLE_PINHOLE, PINHOLE, SIMPLE_RADIAL, RADIAL, OPENCV를 지원한다.
세부 필드와 표현 한계는 [평면 객체 계약](../../../contracts/mapping/PLANAR_OBJECTS_V1.md)을 따른다.

### 검출 범위 조정

| profile | 평면 계수(px) | 면 내부/경계 최소 지지점 | 최소 면적(영상 비율) | 짧은 변(px) | 경계 평균/각 변 일치율 | 법선 차이 한계 |
| --- | --- | --- | --- | --- | --- | --- |
| strict | 2.5 | 8 / 5 | 0.004 | 20 | 0.75 / 0.50 | 10° |
| balanced | 4 | 5 / 3 | 0.0015 | 10 | 0.68 / 0.40 | 15° |
| coverage | 6 | 3 / 2 | 0.0004 | 5 | 0.62 / 0.35 | 20° |

위 픽셀 값은 폭 960px 분석 영상의 경계 검사 기준이다. 평면 거리만 원본 카메라의 거리/초점거리와
`max(plane_error_px, 4 × 관측점 오차 중앙값)`으로 환산한다. 따라서 px 계수 자체가 mm 정확도는 아니다.
모든 profile은 최소 3시점과 카메라 이동 검사를 유지한다. 지지점 hull 면적 비율은 strict 0.12,
balanced 0.03, coverage 0.015다. 작은 가로 면이 빠지면 먼저 coverage로 확인하고,
서로 다른 깊이의 면이 합쳐지면 계수를 낮추거나 strict로 돌아간다.
영상에서 기울어져 얇게 보이는 면을 위해 2D 변 비율 상한은 balanced 30, coverage 40, strict 16이다.
복원된 3D 면의 변 비율 상한 12는 그대로 적용한다. 경계 외관 상관도의 하한은 각각 0.55 / 0.45 / 0.65다.

```powershell
# 더 작은/얇은 면까지 제안. 새로운 결과 폴더를 사용한다.
.\.venv\Scripts\python.exe -B modules/mapping/localization/planar.py `
  --result output/phone_mapping/2f1_sfm_v3 `
  --output output/phone_mapping/my_wide_objects --max-frames 30 `
  --profile coverage --plane-error-px 6
```

온라인에서도 `--plane-profile coverage --plane-error-px 6`으로 같은 기준을 적용한다.
`objects.json`의 detection에 실제 기준, statistics.rejected에 단계별 탈락 수를 기록한다.
가로형 앞면과 실제 윗면은 다르다. shape는 비율, plane_orientation은 대표 평면과 법선의 차이이며,
중력/의미 분류가 아니다. 윗면이 영상에 보이지 않거나 3D 지지가 없으면 threshold를 올려도 복원할 근거가 없다.

평면 투영과 보정의 기반은 [OpenCV homography 안내](https://docs.opencv.org/4.x/d7/dff/tutorial_feature_homography.html)를 참고한다.
