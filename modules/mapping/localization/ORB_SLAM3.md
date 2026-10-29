# Docker ORB-SLAM3 → 프레임 대응점·카메라 경로·웹 지도

영상 → 원래 시각 순서의 프레임 → ORB 검출/매칭 → 자세 추적 → 키프레임 삼각측량·
국소 최적화·재위치/루프 폐쇄 → native 지도 snapshot → Three.js 순서다.
기존 초당 2장 COLMAP 창 재계산 경로와 분리했다. ORB-SLAM3가 키프레임을 선택한다.
기록 RGB 영상과 브라우저 카메라의 단안 희소 지도를 연결한다. 밀집 표면·사물 이름 인식은 포함하지 않는다.

## 준비

Docker Desktop의 Linux 엔진과 Python 3.12를 사용한다. Windows에 Ubuntu/C++ 개발 도구를
별도로 설치할 필요는 없다. 저장소 루트에서 각 작업 공간의 환경을 준비한다.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/localization/requirements-online.txt
pnpm --dir apps/console install --frozen-lockfile
docker build -t ssafy-orbslam3:local modules/mapping/localization/orbslam3
```

Docker는 Ubuntu 22.04, 공식 ORB-SLAM3 revision
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, Pangolin 0.6을 빌드한다.
외부 소스·vocabulary는 이미지 안에만 두며 저장소에 복사하지 않는다.
라이브러리 수정은 예제 target 생략과 read-only Atlas getter다. native 카메라 추적·국소
최적화·루프 폐쇄는 원래 ORB-SLAM3를 사용한다. 추가한 특징점 통합은 추적기와 분리해
**자세 추정 → snapshot 자세 고정 → 관측 검증·같은 점 통합 → 웹 발행** 순서로 수행한다.
점 통합 결과를 추적기에 보내거나 점을 맞추려고 카메라 위치·방향을 바꾸지 않는다.
단안 영상 자세 자체는 특징 기반 추정이며, 후속 native snapshot에서는 SLAM 최적화로
달라질 수 있다. 별도 센서로 측정한 절대 이동 경로나 영구 고정된 자세라고 해석하지 않는다.
연결 C++ 코드는 GPL-3.0-or-later이며 ORB-SLAM3 GPLv3 라이선스를 따른다.
Upstream: https://github.com/UZ-SLAMLab/ORB_SLAM3

## 실행

같은 카메라/촬영 방향의 기존 결과가 있으면 자체 추정 내부 파라미터만 가져올 수 있다.
아래 경로는 각자의 실제 로컬 자료 경로로 지정한다. 출력은 존재하지 않는 새 폴더를 쓴다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/localization/orbslam3.py `
  --source data/real_data/my_capture/source_video.mp4 `
  --calibration-result output/phone_mapping/my_prior_result `
  --session output/phone_mapping/my_orb_run
.\.venv\Scripts\python.exe apps/console/serve.py `
  --session output/phone_mapping/my_orb_run --port 8793
```

실측 보정값이 있으면 `--camera camera.json`을 대신 사용한다. JSON에는
`width,height,fx,fy,cx,cy,distortion:[k1,k2,p1,p2]`를 넣는다.
기본 폭 960px, ORB 최대 1800개, 원래 속도 재생, CPU 6개·컨테이너 메모리 8GB 제한이다.
`--width`, `--features`, `--cpus`, `--max-seconds`로 바꿀 수 있다.
`--replay-rate 0`은 대기 없는 오프라인 처리이며 실시간 결과로 보고하지 않는다.

원본 영상 폴더는 컨테이너에 읽기 전용, 새 세션 폴더만 쓰기 가능으로 연결한다.
실행 중 외부 네트워크·호스트 포트·특권은 필요하지 않다. Ctrl+C는 이 실행의 컨테이너만 중단한다.
실행용 프레임·지도·재생 시간 기록은 세션에 보관하므로 장시간 영상의 디스크 사용량을 고려한다.

웹의 노란 점은 직전 프레임과 같은 native 지도 점이다. 추적 상실·다른 지도 전환 때는
연속된 공간으로 꾸미지 않는다. [결과 계약](../../../contracts/mapping/ORB_SLAM3_V1.md)을 따른다.
실기체의 RTSP/UDP 입력, 독립 치수, 루프 폐쇄가 실제 발생하는 재방문 영상은 별도 확인 범위다.

### 불안정 특징점 확정 방지

`landmarks.py`가 native ID별로 현재 snapshot의 실제 관측을 다시 검사한다. 지도에는
서로 다른 키프레임 3개 이상에서 카메라 앞쪽·영상 내부이고 `error² / sigma² ≤ 5.991`인
관측이 있으며, 그 시점들 사이 최대 시차각이 1° 이상인 점만 발행한다. `sigma²`는 ORB
검출 피라미드 레벨의 분산이다. 이전 native 기록에는 필드가 없어 1px²를 사용한다.
관측 수는 native 누적 숫자를 가져오지 않고 유효한 고유 키프레임을 다시 센다.
잘못된 관측은 신뢰도 계산에서도 제외하며 가까운 좌표·같은 색의 다른 점을 합치지 않는다.

아직 확정점이 없으면 `mapping_state=confirming_geometry`로 기다리고 마지막 유효 지도를
보존한다. 새 세션에서 끝까지 확정점이 없으면 파일 실행은 종료 코드 3을 반환한다.
세부 제외 수는 state/scene의 `landmark_confirmation`에 기록한다. 고정된 위치에서 회전만
하거나 무늬가 부족한 영상은 표시할 점이 적을 수 있고, 잘못된 보정값·움직이는 물체까지
완전히 해결하는 조건은 아니다. 서로 다른 지도/세션의 ID는 합치지 않는다.

`fixed_pose_association.py`는 확정된 점의 ORB 기술자와 고정된 자세의 상호 재투영을
사용한다. 양방향으로 유일한 후보(Hamming ≤40, 차선 후보 대비 비율 <0.75)이며
모든 상대 시점의 관측에 맞아야 같은 ID로 표시한다. 한 프레임에서 별도로 관측된 점은
합치지 않는다. 모호한 반복 무늬·같은 픽셀의 다른 깊이·다른 지도는 유지한다.
최대 12,000점의 기술자를 대상으로 회차당 512개 순환 질의와 역방향 확인을 수행해
계산을 제한한다. 검사하지 않은 점은 기존 ID를 유지한다. 매 revision에서 대응을 다시
검사하고 대표 ID와 `point_aliases`를 보존한다. 좌표는 더 많이 관측된 기존 점을 사용한다.
새 native 기술자 내보내기는 위 Docker 빌드로 적용한다. 구 snapshot은 기술자가 없어
기존 ID와 관측 검증만 사용한다. 색상·표시 격자는 동일 지점 판단에 사용하지 않는다.

### 입력부터 웹 표시까지 지연

실시간 탭에서 시작한 입력은 같은 브라우저의 `performance.now()`로 프레임 콜백 수신,
JPEG 완료, 업로드 응답을 기록한다. 지도의 유효 관측에 포함된 키프레임이 처음 그려진 뒤
다음 requestAnimationFrame까지의 전체 지연을 평균·P95·최대로 표시한다. 초기화와
3시점 관측 대기, 서버 대기/추적/검증, polling, 지도 수신·그리기까지 포함한다.
저장 기록 재생·다른 세션·숨김 화면은 새 측정에서 제외한다. 일반 프레임이나 버려진
프레임이 모두 3D 지도 키프레임이 되는 것은 아니다. 센서 노출부터 모니터 광출력까지의
실측 또는 모든 프레임에 대한 지연 보장이 아니며, 실시간성 판정에는 실제 결과가 필요하다.
**지연 기록 저장**은 최근 600개 키프레임의 원본 측정과 부호화·업로드·지도 응답·적용·
렌더 제출 시간, 표시 지연 통계와 제외된 오래된 입력 표식 수를 JSON으로 저장한다.
입력 표식은 최근 1,800장만 보관하며 임의로 빠진 기록을 보완하지 않는다.

합성 시점·왜곡·깊이·시차·관측 이상과 발행 대기/복귀 검사:

```powershell
.\.venv\Scripts\python.exe -B -m unittest modules.mapping.localization.tests.test_landmarks -v
```

## 카메라에서 웹까지 실시간 연결

```powershell
.\.venv\Scripts\python.exe -B apps/console/serve.py `
  --session output/phone_mapping/my_orb_run --port 8793 --enable-camera
```

Docker Linux 엔진을 켜고 `http://127.0.0.1:8793/`에서 **카메라 연결 · 시작**을 누른다.
사용자가 허용한 카메라 영상만 사용하며 오디오는 요청하지 않는다. USB 카메라나 드론의
가상 카메라 장치는 브라우저 입력으로 연결할 수 있다. 드론 RTSP/UDP를 브라우저에 직접
넣는 기능은 아니다. 카메라 JSON을 선택하지 않으면 수평 화각에서 내부 파라미터를 추정하고
`fov_estimated; not_calibrated`로 표시한다. 렌즈·크롭·회전이 다른 보정값을 재사용하지 않는다.

브라우저는 실제 영상 콜백의 media PTS에 맞춰 최대 960px JPEG를 전달한다. 렌더링 FPS와
추적 입력 FPS는 별개다. HTTP 전송은 한 번에 하나, 서버 대기는 최신 한 장, Docker 전송은
추적 ACK를 기다리는 한 장으로 제한한다. 지연 시 오래된 대기를 교체하고 입력 시퀀스 누락을
합산해 표시한다. 시차가 충분해야 초기화되며 제자리 회전만으로 지도가 생기지 않을 수 있다.
10초 동안 입력이 없으면 세션을 종료하고 마지막 지도를 보존한다. 수신 종료·페이지 이탈도
해당 세션만 종료한다. 새 연결은 새 폴더·좌표계이며 이전 세션에 합치지 않는다.

물리 카메라 없이 같은 전송 경로를 시연하려면 다음 옵션을 추가한다.

```powershell
  --demo-source data/real_data/my_capture/source_video.mp4
```

이 옵션은 지정한 MP4 하나만 로컬 브라우저에 제공한다. 초기 `--session`의 `camera.json`이
같은 영상의 보정값이어야 한다. **현재 녹화 영상 · 스트리밍 시연 → 스트리밍 시연 시작**을
선택하면 브라우저에서 디코딩·JPEG 전송하고 실제 Docker 추적을 새로 실행한다. 저장 결과
재생과 달리 이 기능에는 Docker가 필요하다. 실제 카메라·기체 검증 결과로 취급하지 않는다.

### 저장 결과에서 녹화 영상 전송 재현

기존 온라인 세션이 없어도 같은 영상의 저장 복원에서 **카메라 내부 파라미터만** 가져와
시연할 수 있다. 원래 카메라 경로와 3D 점은 새 SLAM 계산에 넣지 않는다.

```powershell
.\.venv\Scripts\python.exe -B apps/console/serve.py `
  --result output/phone_mapping/my_prior_result --port 8791 --enable-camera `
  --demo-source data/raw/my_capture/source_video.mp4 `
  --demo-calibration-result output/phone_mapping/my_prior_result
```

실측 카메라 JSON은 `--demo-camera camera.json`으로 대신 지정한다. 두 보정 옵션은 동시에
사용하지 않는다. 옵션을 생략하면 기존 `--session`의 `camera.json`을 사용한다. 보정값이
없거나 영상의 가로세로 비율이 다르면 서버 시작 단계에서 오류를 알린다. 보정값은 같은
렌즈·크롭·방향의 영상에만 사용하며, 저장 결과의 자체 추정값은 실측 보정으로 표시하지 않는다.

브라우저에서 **현재 녹화 영상 · 스트리밍 시연 → 스트리밍 시연 시작**을 선택한다.
서버는 시작 시 브라우저와 같은 최대 960px 크기로 보정값을 맞춘다.

반복 가능한 HTTP 종단 검증은 별도 터미널에서 실행한다.

```powershell
.\.venv\Scripts\python.exe -B tests/integration/replay_live_slam.py `
  --url http://127.0.0.1:8791 --source data/raw/my_capture/source_video.mp4 `
  --report output/phone_mapping/my_live_check/replay_report.json
```

브라우저 시연과 HTTP 검증은 한 번에 하나씩 실행한다. 이 검증은 OpenCV로 영상을 원래
속도로 읽어 브라우저와 동일한 start/frame/stop API로 JPEG를 보낸다. 점군 응답을 실행 중
두 번 이상 읽고 추적된 프레임이 있는지 검사한다. 보고서에는 원본 해시·입력 시각·수신 및
추적 상태·관찰한 지도 revision을 남긴다. 실패 시 입력, SLAM 시작, 프레임/지도 전송,
SLAM 처리 단계를 표시하며 0이 아닌 종료 코드를 반환한다. 보고서 경로는 새 파일이어야 한다.
로컬 HTTP에 직접 연결하며 지연된 원본 프레임은 생략해 재생 속도를 유지한다. 생략 수와
전송 수를 따로 기록하고 종료 중에만 나온 지도는 실시간 갱신 횟수에 포함하지 않는다.
이는 녹화 파일의 HTTP 연결 검증이며 브라우저 디코딩·렌더링과 실제 카메라는 별도로 확인한다.

저장 결과와 온라인 세션은 `--result`와 `--session`을 함께 지정해 같은 서버에서 제공할 수 있다.
각각의 고정 조회 주소와 세션 ID·revision을 고정하는 읽기 순서는
[저장 지도·실시간 세션 조회 계약](../../../contracts/mapping/VIEWER_SOURCES_V1.md)을 따른다.

## 지도 생성 과정 재생

**재생**은 입력 영상 시각에 맞춰 `timeline.json`의 실제 지도 revision을 차례로 읽는다.
첫 유효 지도 전에는 0점, 이후에는 그때 존재했던 점·카메라·경로만 표시한다. BA 보정·점 삭제·
지도 ID 전환도 해당 revision 전체 교체로 재현하며 최종 점군에 임의 등장 시간을 붙이지 않는다.
이력은 지도 발행 snapshot 기준이며 개별 프레임의 모든 내부 최적화 이벤트까지 재생하지는 않는다.
일시정지, 처음, 시각 탐색, 0.5/1/2배속, 최신 지도 복귀를 지원한다. 최종 보기의 조작 시점을
유지하며 지도 ID가 바뀌면 새 좌표계에 맞춘다. 웹 캐시는 최근 3개 지도만 유지한다.

예전 세션에 timeline이 없으면 저장된 revisions에서 인덱스를 읽되 시각을 마지막 생존
키프레임 기준 **추정**으로 명시한다. 새 기록은 native snapshot의 입력 시각을 보존한다.
모든 JPEG·지도·로그·이력은 ignore된 `output/phone_mapping/live_*`에 저장된다. 장시간 수신은
지도 이력이 계속 커지므로 디스크 용량을 관리해야 한다. 저장 기록 재생에는 Docker가 필요 없다.
