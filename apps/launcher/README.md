# Chill-Drone 실행기 (`scripts/start.cmd`)

프로젝트를 켜는 실행 파일은 `scripts\start.cmd` 하나다. 더블클릭하면 체험과 실기체 관제 메뉴가 뜬다.
명령줄에서는 `scripts\start.cmd fake` 또는 `scripts\start.cmd real`. 내용은 이 폴더의 Python 모듈(`python -m apps.launcher`)이다.
매핑 계측 도구는 [modules/mapping/benchmarks](../../modules/mapping/benchmarks/)에 있다.

| 메뉴 | 명령 | 켜는 것 |
| --- | --- | --- |
| 1 체험 | `start.cmd fake` | 가짜 드론 게이트웨이(8030) + 사람 탐지 + 가짜 지도(8795)·지도 연결기 + 영상 복원용 지도 콘솔(8796) → 관제 화면 `#/live`. 드론 없이 화면을 볼 때 |
| 2 실기체 관제 | `start.cmd real` | 실기체 게이트웨이(8010) + 사람 탐지 + 지도 콘솔(8790)·지도 연결기 + **지도 자동 시작** → 관제 화면 |
| (메뉴 없음) | `start.cmd sim` | **자율 비행 시뮬레이터 시연(WSL)**: 자율 비행 시뮬레이터 + 보기 노드 + 게이트웨이 `--ros-view`(8020) + 사람 탐지 + 지도 콘솔(8797)·지도 연결기 + 지도 자동 시작 → 관제 화면(자율 탐색 프리셋). 아래 절 |
| (메뉴 없음) | `start.cmd autonomy` | **자율 비행 실기체(WSL + Windows 중계)**: Windows 중계 `tello_relay.py` + 자율 비행 `real.launch.py` + 보기 노드 + 게이트웨이 `--ros-view`(8020) + 사람 탐지 + 지도 콘솔(8797)·지도 연결기 + 지도 자동 시작 → 관제 화면. `--fake-tello`면 드론 없이 같은 길을 시험. 아래 절 |

## 자율 비행 시뮬레이터 시연(`sim`)

실기체 없이 자율 비행을 관제 화면에서 보이고 조작한다. 자율 비행 쪽 코드·launch·기준은 그대로다(자율 비행 README대로 켠다).

```powershell
scripts\start.cmd sim
```

1. WSL `Ubuntu-24.04`에 ROS 2 Jazzy·자율 비행 환경이 있는지 본다. 없으면 설치 명령을 알려 주고 멈춘다.
   - 처음 한 번(약 10 GB, 인터넷 필요): 관리자 PowerShell에서 `wsl --install -d Ubuntu-24.04`로 Ubuntu와 사용자를 만든다.
   - 그다음 실행기가 알려 주는 설치 명령(`wsl -d Ubuntu-24.04 -u root -- bash …/apps/launcher/wsl_sim_setup.sh <사용자> …`)을 돌린다
     (ROS 2·Gazebo, YOLO 환경, PyAV·transformers, YOLO·깊이 모델, 세계 만들기). 비밀번호는 묻지 않는다.
   - 다른 배포판 이름이면 환경 변수 `CHILL_WSL_DISTRO`.
2. WSL 사본 `~/ttrepo`를 이 코드 폴더의 지금 커밋으로 맞춘다.
   - 커밋하지 않은 변경은 사본에 들어가지 않는다(게이트웨이·화면은 지금 파일로 켠다).
   - 이 코드 폴더에 시뮬레이터가 없으면(자율 비행 코드가 안 합쳐진 브랜치) 사본을 그대로 쓴다.
   - 시뮬레이터가 이미 켜져 있으면 사본·세계·찾은 사람 파일은 건드리지 않고 빠진 것만 켠다.
3. 세계 `s7_stage3`(사람 찾기 단계)이 없으면 만든다. 지난 시연의 찾은 사람·장애물 파일은 시각을 붙여 옮겨 둔다(시뮬레이션은 세계 폴더의 같은 파일을 다시 쓴다).
4. 자율 비행 시뮬레이터(`mission.launch.py`: Gazebo 화면 없이, 자율 비행 웹 8080, YOLO)와 보기 노드(WSL `127.0.0.1:8094`)를 숨은 창에서 켠다. 로그 `data/outputs/launcher/sim-*.log`.
5. 게이트웨이 `--ros-view`(8020), 사람 탐지, 지도 콘솔(8797)·지도 연결기, 지도 자동 시작을 켜고 관제 화면을 연다(`?preset=autonomy#/live`).
   - 시작: 조종 줄 "이륙" → 주변 확인 체크 → "이륙"을 4초 안에 두 번(자율 비행 웹과 같다).
   - 2D 지도를 클릭하면 보낼 자리가 찍히고, "여기로 보내기"(G)로 비행 노드에 보낸다.
   - 착륙(L)·비상 정지(0.6초 길게·Esc)도 자율 비행 웹과 같은 명령이다. 같은 비행을 자율 비행 웹(8080)에서도 본다.
6. 실행기 창에서 Enter를 누르면 모두 끈다. 시뮬레이터는 이 실행기가 켠 `mission.launch.py`·`gz sim`·보기 노드만 끈다(실기체용 `real.launch.py`는 건드리지 않는다).

- 자율 비행은 한 번 끝나면(끝·중단) 다시 시작하지 않는다. 다시 날리려면 모두 끄고 다시 켠다.
- 이 PC 시뮬레이터는 실시간보다 느리다(Tello 상태 약 4 Hz). GPU 하나를 자율 비행 YOLO·사람 탐지·지도가 나눠 쓴다.
- 지도는 시뮬레이터 영상만으로 만든다(센서 자세 없음). 이 PC 시뮬레이터는 실제보다 느려 Tello 상태 속도를 벽시계로 적분하면
  거리가 맞지 않고, 상태 사이가 길어 자세 없는 장이 많다.

## 자율 비행 실기체(`autonomy`)

자율 비행(ROS)이 드론을 조종하고, 관제 화면은 보기·같은 명령의 버튼·사람 탐지·3D 지도를 붙인다. 자율 비행 쪽은
[자율 비행 README의 실기체 순서](../../modules/autonomy/tello_driver/README.md) 그대로다(Windows 중계 → WSL `real.launch.py`).
우리 실기체 게이트웨이(`start.cmd real`)와는 같이 켤 수 없다(Tello는 처음 붙은 쪽에만 답한다).

```powershell
scripts\start.cmd autonomy               # 드론을 켜고 PC를 드론 Wi-Fi에 연결한 뒤
scripts\start.cmd autonomy --fake-tello  # 드론 없이 가짜 Tello로 같은 길을 끝까지 시험(날지 않음)
```

1. WSL 확인·WSL 사본 맞추기는 `sim`과 같다(처음 한 번 `wsl_sim_setup.sh`).
2. 드론 Wi-Fi 이름으로 확장 키트를 정한다: `RMTT-…`면 전방 ToF를 쓰고, `TELLO-…`(기본 Tello)면 `front_tof:=false`(자율 비행 README).
3. Windows 중계 `modules/autonomy/tello_driver/tello_relay.py`를 켠다(WSL 기본 NAT 네트워크는 Tello 상태·영상 UDP가 WSL까지 오지 않는다).
   - 이 Python이 드론 상태·영상(UDP 8890·11111)을 받아야 한다. 막혀 있으면 실행기가 Windows 관리자 확인 창을 띄워 맞춘다(아래 "방화벽").
   - WSL 드라이버 → 중계(UDP 9889)는 규칙 없이 들어온다(11/16 이 PC 확인).
4. WSL `real.launch.py`(드라이버·위치·깊이·YOLO·비행 노드·자율 비행 웹 8080)와 보기 노드를 켠다. 보기 노드는 가장 새 비행 폴더
   (`~/ttrepo/data/outputs/flights/<시각>/people.json`)의 찾은 사람을 본다. 로그 `data/outputs/launcher/autonomy-*.log`·`tello-relay.log`.
5. 게이트웨이 `--ros-view`(8020)·사람 탐지·지도 콘솔(8797)·지도 연결기·지도 자동 시작·관제 화면은 `sim`과 같다.
   - 자율 비행 드라이버는 실기체 영상을 480×360으로 줄여 낸다(깊이·YOLO 부하). 지도 파트는 Tello 원본 960×720만 받아서,
     게이트웨이가 지도용 영상(`/video.mjpg?mapping=1`)만 960×720으로 다시 키운다(자르지 않고 줄인 것이라 내장 보정값이 맞다).
   - 3D 지도 위치는 드론 센서 자세로 잡는다(`sim`과 다른 점, 게이트웨이 `--ros-view-sensor-pose`). 드라이버가 받은 Tello 상태를
     게이트웨이가 손 조종 모드(`real`)와 같은 적분으로 장마다 붙인다. 없으면 지도가 영상으로만 위치를 맞추다 놓쳐 날면서 번진다.
6. 실행기 창에서 Enter: 드론이 땅에 있을 때만 끈다(공중이면 아무것도 끄지 않는다). 관제 쪽 → WSL `real.launch.py`·보기 노드(Ctrl+C면 드라이버가
   착륙하고 비행 기록 페이지 `viewer.html`을 만든다) → Windows 중계 순서.

- 자율 비행은 기본 120초 탐색 뒤 출발점으로 돌아와 내린다(`search_time`). 한 번 끝나면 다시 시작하지 않는다.
- 자율 비행 launch 인자를 바꿔야 하면 실행기 뒤에 `이름:=값`으로 붙인다. `real.launch.py`(`sim`이면 `mission.launch.py`)에 그대로 더 넘긴다.
  기본값은 자율 비행 파트가 정한 그대로라, 그 파트와 맞춘 값만 쓴다. 이미 켜진 자율 비행에는 적용되지 않는다(실행기가 알린다).
  WSL 명령줄에 붙으므로 `이름:=값` 꼴과 영문·숫자·`_ . , / + -`만 받는다.

  ```powershell
  scripts\start.cmd autonomy search_time:=150
  scripts\start.cmd autonomy --fake-tello search_time:=150   # 드론 없이 넘어가는지 확인
  ```
- **폰에서도 열기(`--phone`, `sim`도 같음):** 게이트웨이(8020)를 같은 망에도 열고 폰 주소를 네트워크 이름·방화벽 상태와 함께 알려 준다.
  PC는 드론 Wi-Fi와 폰 핫스팟(두 번째 Wi-Fi 어댑터나 USB 테더링)에 같이 붙인다. 폰 화면(`#/control`)은 시작(이륙 → 주변 확인 → 두 번)·착륙·
  비상 정지와 상태·영상이고 손 조종은 없다(자율 비행이 조종). 주소 끝을 `/?preset=autonomy#/live`로 바꾸면 전체 감독 화면이다.
  이미 켜진 게이트웨이가 이 PC에서만 열려 있으면 바꾸지 못한다고 알린다(땅에서 끄고 다시 켠다). 인증이 없으니 우리 기기만 있는 망에서만 쓴다.

  ```powershell
  scripts\start.cmd autonomy --phone
  ```
- 비행 기록: 자율 비행은 WSL `~/ttrepo/data/outputs/flights/<시각>/`, 관제 게이트웨이는 `data/outputs/tello/ros_…`.

## 새 Windows PC에서 시작하기

1. 인터넷에 연결하고 `scripts\start.cmd`를 더블클릭한다. Python이 준비되어 있으면 체험 또는 실기체 관제를 선택한다.
2. **실행 전 확인**에 표시된 항목을 준비한 뒤 같은 파일을 다시 실행한다.
   - [Python 3.12(64비트)](https://www.python.org/downloads/windows/)와 [Node.js LTS](https://nodejs.org/)를 설치한다.
     이미 지도 콘솔 웹 의존성이 준비됐거나 pnpm을 쓸 수 있으면 Node.js 설치 안내는 생략한다.
   - ORB-SLAM3를 쓰려면 [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/)을 열어
     Linux 컨테이너 엔진을 시작한다.
3. 일반 실행은 프로젝트 Python 가상환경과 Python·웹 패키지만 준비한다. 영상 복원 모델은 **지도 콘솔**에서 영상을 열고 직접 준비한다.
   실시간 지도·사람 탐지를 처음 쓰기 전에는 인터넷에 연결해 `scripts\start.cmd prepare-flight`를 실행한다.
   이 명령이 기존 비행용 모델과 Docker 이미지를 준비한다.
   Dockerfile 또는 빌드에 포함된 파일이 바뀌었을 때도 이 명령으로 이미지를 갱신할 수 있다.
4. 준비가 끝나면 관제 화면이 열린다. 실기체를 쓸 때는 준비 완료 안내가 나온 뒤 드론을 켜고 PC를 드론 Wi-Fi에 연결한다.

메뉴 1·2의 **지도 콘솔**(관제 화면 `#/map`)에서 드론 녹화·일반 영상 복원과 저장 결과의 3D 지도 편집을 쓴다.
드론 없이 이 흐름을 보려면 1(체험)을 켠다(영상 복원용 콘솔 8796). [지도 콘솔 안내](../console/README.md#지도-콘솔과-영상-폴더)

## 실기체 관제(`real`)

1. 실행기의 준비 단계가 끝나면 드론을 켜고 PC를 드론 Wi-Fi(`TELLO-…`/`RMTT-…`)에 연결한다. 게이트웨이는 드론 연결을 기다린다.
2. 8010 포트나 드론 UDP 포트(8889·8890·11111)를 다른 프로그램(점검 화면 `check_server.py`, 다른 게이트웨이, 직접 연결 지도)이
   쓰고 있으면 켜지 않고 그 프로그램을 보여 준다. Tello는 처음 `command`를 보낸 쪽에만 답하기 때문이다.
3. 게이트웨이, 사람 탐지, 지도 콘솔·지도 연결기를 백그라운드에서 켠다. 서비스 로그는 `data/outputs/launcher/`, 드론 회차 기록은 `data/outputs/tello/`에 있다.
4. **지도:** 영상이 켜지면(화면의 "영상 켜기" 또는 이륙하면 자동) 실행기가 지도 콘솔에 Tello 지도 세션을 시작하고 2초마다 유지한다
   (`mapkeeper.py`). 기본 설정은 인명 구조 지도, 깊이 384, 32,000점, 초당 5장, Tello 내장 보정값이다.
   현재 점군은 실시간 감독 지도 칸의 **3D 점군**에 보인다. 지도 콘솔(`#/map`)에서는 촬영 영상을 복원하거나 저장한 점군을 편집한다.
   - 영상이 30초 안에 돌아오면 같은 지도가 이어진다. 더 길게 끊기면(배터리 교체·재부팅) 콘솔이 세션을 끝내고(지도는 콘솔
     기록에 남는다) 영상이 다시 켜질 때 새 지도를 시작한다. 관제 화면 3D 칸도 새 지도로 바뀐다.
   - 지도 시작이 연달아 세 번 실패하면(Docker 꺼짐 등) 더 시도하지 않는다. 다른 지도가 돌거나 정리 중인 것은 실패로 세지 않는다.
   - 지도 콘솔의 "1. 드론 연결" 칸은 관제 게이트웨이의 드론 연결·배터리·영상을 보여 주고, 직접 연결 버튼(드론 검색·Wi-Fi 연결 등)은
     숨긴다(드론은 게이트웨이가 쥐고 있어서). 콘솔에서 지도를 끝내는 버튼은 **지도 마무리**다.

`--phone`을 붙이면 게이트웨이를 같은 망에 열고(`--host 0.0.0.0`) 폰 주소를 네트워크 이름과 함께 알려 준다. 인증이 없으니 같은 망 시험에서만 쓴다.
폰 접속용 방화벽 규칙(TCP)은 같은 망 시험용이라 실행기가 만들지 않는다. 이 Python이 그 망 종류(공용·개인)에서 받기 허용돼 있거나
그 포트 허용 규칙이 있으면 그대로 열리고, 아니면 "방화벽에 막힐 수 있음"과 넣을 명령을 보여 준다(관리자 PowerShell에서 직접 추가·삭제).

## 끄기 (실행기 창에서 Enter)

실행기 창을 열어 두면 지도를 유지한다. **Enter 또는 Ctrl+C**로 종료한다.

- 켜져 있는 게이트웨이의 상태를 먼저 읽어, 실기체가 이륙·비행·착륙 중이면 아무것도 끄지 않고 다시 기다린다.
- 땅이면 지도 세션을 끝내 저장하고, 사람 탐지 → 지도 연결기 → 가짜 지도 → 지도 콘솔 → 게이트웨이 순으로 종료한다.
  게이트웨이는 회차 `metrics.json`과 영상 조각을 마무리한다. 정해진 시간 안에 응답하지 않는 서비스는 강제 종료한다.
- 실행기 창을 그냥 닫으면 백그라운드 서비스는 남을 수 있다. 다시 실행하면 기존 서비스를 감지하며, Enter로 종료할 수 있다.

## Python 환경

`scripts\start.cmd`는 이 checkout의 `.venv`, main worktree의 `.venv`, 환경 변수 `CHILL_DRONE_PYTHON` 순으로
사용 가능한 Python 3.12(64비트)를 찾는다. 설치된 기본 Python을 찾으면 실행 전 확인을 통과한 뒤 이 checkout에 `.venv`를 만든다.

### 방화벽(실기체)

실기체(`real`·`autonomy`)는 이 Python(게이트웨이·자율 비행 중계)이 드론 상태·영상(UDP 8890·11111)을 받아야 한다.
실행기가 켤 때 확인한다:

- 드론 Wi-Fi(`TELLO-…`/`RMTT-…`)의 네트워크 종류(공용·개인)에서 이 Python에 허용 규칙이 있고 차단 규칙이 없으면 그대로 켠다.
  드론 Wi-Fi에 연결하기 전이면 공용·개인 둘 다 허용돼야 통과로 본다.
- 모자라면 **Windows 관리자 확인 창**을 띄운다. "예"를 누르면 이 Python의 UDP 8890·11111만 모든 네트워크 종류에서 여는 규칙
  `Tello UDP (Chill-Drone)`을 넣고, 이 Python의 UDP 수신 차단 규칙(처음 뜨는 Windows 허용 창에서 취소하면 생김)은 끈다(지우지 않음).
- "아니요"를 누르거나 규칙을 읽지 못하면 관리자 PowerShell에서 넣을 명령을 보여 준다. 지울 때는
  `Remove-NetFirewallRule -DisplayName "Tello UDP (Chill-Drone)"`.
- 처음 켤 때 Windows 허용 창이 따로 뜨면 **허용**을 누른다(취소하면 차단 규칙이 생겨 위 확인 창이 다시 필요해진다).
모델 파일만 따로 받으려면 `.\.venv\Scripts\python.exe -B -m modules.mapping.data.prepare_models --download`.
