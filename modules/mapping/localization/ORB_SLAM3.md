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
수정은 예제 target 생략과 read-only Atlas getter 추가다. 추적 알고리즘을 대체하지 않는다.
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

## 지도 생성 과정 재생

**재생**은 입력 영상 시각에 맞춰 `timeline.json`의 실제 지도 revision을 차례로 읽는다.
첫 유효 지도 전에는 0점, 이후에는 그때 존재했던 점·카메라·경로만 표시한다. BA 보정·점 삭제·
지도 ID 전환도 해당 revision 전체 교체로 재현하며 최종 점군에 임의 등장 시간을 붙이지 않는다.
이력은 약 1Hz이므로 개별 프레임의 모든 내부 최적화 이벤트까지 재생하지는 않는다.
일시정지, 처음, 시각 탐색, 0.5/1/2배속, 최신 지도 복귀를 지원한다. 최종 보기의 조작 시점을
유지하며 지도 ID가 바뀌면 새 좌표계에 맞춘다. 웹 캐시는 최근 3개 지도만 유지한다.

예전 세션에 timeline이 없으면 저장된 revisions에서 인덱스를 읽되 시각을 마지막 생존
키프레임 기준 **추정**으로 명시한다. 새 기록은 native snapshot의 입력 시각을 보존한다.
모든 JPEG·지도·로그·이력은 ignore된 `output/phone_mapping/live_*`에 저장된다. 장시간 수신은
지도 이력이 계속 커지므로 디스크 용량을 관리해야 한다. 저장 기록 재생에는 Docker가 필요 없다.
