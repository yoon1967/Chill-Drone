# 포팅 매뉴얼 (Windows, 통합 실행)

다른 Windows PC에서 처음부터 관제 화면을 켜기까지의 순서다.
기준은 통합 브랜치 `feat/final-integration`(2026-11-17)이고, 켜는 파일은 `scripts\start.cmd` 하나다.
실행기 자세한 동작은 [apps/launcher/README.md](../apps/launcher/README.md)에 있다.

## 1. 무엇이 어디서 도나

| 부분 | 어디서 | 하는 일 |
| --- | --- | --- |
| 관제 게이트웨이 | Windows, Python 3.12 | 드론 상태·영상·명령의 창구, 관제 화면 제공, 회차 기록 |
| 관제 화면 | 브라우저 | 실시간 감독 `#/live`, 탐색 결과 `#/review`, 지도 콘솔 `#/map`, 폰 조작 `#/control` |
| 사람 탐지 | Windows, GPU | 게이트웨이 영상으로 사람을 찾아 사진·윤곽을 게이트웨이에 보냄 |
| 지도 콘솔·지도 연결기 | Windows(Python + 웹 의존성) | 영상으로 3D 점군을 만들고 관제 화면 3D 칸에 보냄 |
| 자율 비행 | WSL Ubuntu-24.04, ROS 2 Jazzy | 드라이버·위치·깊이·YOLO·비행 노드·자율 비행 웹(8080) |
| 보기 노드 | WSL | 자율 비행 토픽을 게이트웨이에 넘기고, 관제 화면 버튼을 자율 비행 웹과 같은 명령으로 냄 |
| Windows 중계 | Windows | 실기체 자율 비행 때 Tello ↔ WSL 드라이버 사이 UDP를 옮김(WSL 기본 NAT 네트워크용) |

```text
실기체 자율 비행(start.cmd autonomy)
Tello ──UDP──▶ Windows 중계(tello_relay.py) ──▶ WSL 자율 비행(real.launch.py) ──토픽──▶ 보기 노드(8094)
                                                                                       │
브라우저 관제 화면 ◀── 관제 게이트웨이(8020) ◀──────────────────────────────────────────┘
                         ▲            ▲
                    사람 탐지     지도 연결기 ◀── 지도 콘솔(8797)
```

Tello는 처음 `command`를 보낸 프로그램 하나에만 답한다. 그래서 실기체 손 조종(`real`)과 실기체 자율 비행(`autonomy`)은 같이 켤 수 없다.
폰 Tello 앱, 점검 화면(`check_server.py`)도 끈다.

## 2. 필요한 것과 버전

| 무엇 | 버전·조건 | 확인한 곳 |
| --- | --- | --- |
| OS | Windows 11(이 PC: Windows 11 Enterprise) | 이 PC |
| Python | **3.12, 64비트**(실행기가 검사, 다른 버전이면 멈춤) | `scripts/start.cmd`, `apps/launcher/prepare.py` |
| Node.js | LTS(버전 고정 없음). 지도 콘솔 웹 의존성을 `pnpm`(없으면 `npx pnpm@10`)으로 받음 | `apps/launcher/prepare.py` |
| Docker Desktop | Linux 컨테이너 엔진. `prepare-flight`가 지도 파트 ORB-SLAM3 이미지 `ssafy-orbslam3:local`을 만들 때 필요 | `apps/launcher/docker_assets.py` |
| WSL | Ubuntu-24.04(다른 이름이면 환경 변수 `CHILL_WSL_DISTRO`). 네트워크는 기본(NAT) | `apps/launcher/sim.py` |
| ROS 2 | Jazzy + Gazebo Harmonic(`ros-jazzy-ros-gz`), 자율 비행 `setup_jazzy.sh` | `simulation/tt-sim/scripts/setup_jazzy.sh` |
| GPU | NVIDIA GPU와 CUDA 12.8 판 PyTorch가 도는 드라이버. 이 PC는 RTX 4070 Laptop GPU, 드라이버 591.44 | 이 PC `nvidia-smi` |
| PyTorch(Windows) | `torch==2.11.0`, `torchvision==0.26.0`. NVIDIA GPU가 보이면 CUDA 12.8 판(cu128), 아니면 CPU 판 | `apps/launcher/prepare.py` |
| Python 패키지(Windows) | `modules/mapping/data/requirements-stream.txt`(ultralytics 8.4.189, transformers 4.57.6 등), `modules/autonomy/tello_link/requirements.txt`(fastapi·uvicorn·av·opencv 등) | 각 파일 |
| Python 환경(WSL) | `~/venvs/tt`(시스템 패키지 같이 씀, `numpy<2`, `opencv-python<4.11`, ultralytics, av, transformers). 버전 고정은 이 둘뿐이라 설치 때 최신이 들어옴(이 PC 11/13: torch 2.15.0+cu132, ultralytics 8.4.211) | `setup_yolo.sh`, `apps/launcher/wsl_sim_setup.sh` |
| 드론 | Tello EDU(`TELLO-…`) 또는 RoboMaster TT 확장 키트(`RMTT-…`, 전방 ToF) | 자율 비행 README |
| 노트북 전원 | **충전기 필수.** 배터리면 GPU가 묶여(이 PC 210 MHz) 자율 비행 깊이 추정이 21–28 ms에서 227 ms로 느려진다(11/16) | 실기체 시험 기록 |

## 3. 처음 한 번 준비 (인터넷 되는 곳에서)

드론 Wi-Fi에 붙으면 인터넷이 없다. 아래는 모두 드론에 붙기 전에 끝낸다.

### 3-1. 코드 받기

```powershell
git clone https://lab.ssafy.com/s15-final/S15P31B205.git
cd S15P31B205
git checkout feat/final-integration
```

### 3-2. Windows 준비

1. Python 3.12(64비트), Node.js LTS, Docker Desktop을 설치한다. Docker Desktop은 한 번 열어 Linux 엔진이 켜질 때까지 둔다.
2. `scripts\start.cmd`를 더블클릭한다. 빠진 것이 있으면 **[실행 전 확인]**에 나온다. 준비한 뒤 다시 실행한다.
   - 처음 실행 때 이 코드 폴더에 `.venv`를 만들고 패키지·지도 콘솔 웹 의존성을 받는다(이 PC 약 8분, 11/09).
   - Python을 찾는 차례: 이 checkout의 `.venv` → main worktree의 `.venv` → 환경 변수 `CHILL_DRONE_PYTHON` → 설치된 Python 3.12.
3. 모델과 이미지를 받는다.

   ```powershell
   scripts\start.cmd prepare-flight
   ```

   | 받는 것 | 어디에 |
   | --- | --- |
   | 깊이 모델 `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf` | Hugging Face 캐시 |
   | 장면 분할 모델 `nvidia/segformer-b0-finetuned-ade-512-512` | Hugging Face 캐시 |
   | 사람 탐지 `yolo26n-seg.pt`(Hugging Face `Ultralytics/YOLO26`, 판·SHA-256 고정) | torch hub `checkpoints` |
   | ORB-SLAM3 이미지 `ssafy-orbslam3:local` | Docker. 코드 해시 라벨이 다르면 다시 만든다 |

   - 사람 탐지 모델이 없으면 실행기는 사람 탐지만 건너뛰고 "`prepare-flight`를 실행해 주세요"라고 알린다.
   - 모델만 따로: `.\.venv\Scripts\python.exe -B -m modules.mapping.data.prepare_models --download`
   - ORB-SLAM3 이미지를 만들다 PC가 멈추면(이 PC 11/05에 겪음) `%UserProfile%\.wslconfig`에 아래를 넣고 `wsl --shutdown` 뒤 다시 한다.
     이 PC는 이 설정으로 15분에 만들었다.

     ```ini
     [wsl2]
     memory=16GB
     swap=16GB
     autoMemoryReclaim=disabled

     [experimental]
     autoMemoryReclaim=disabled
     ```

### 3-3. WSL 자율 비행 환경 (시뮬레이터·실기체 자율 비행에 필요)

1. 관리자 PowerShell에서 Ubuntu를 설치하고 Ubuntu 사용자를 만든다(이름·비밀번호는 직접 정함).

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. `scripts\start.cmd sim`을 실행하면 환경이 없을 때 설치 명령을 보여 주고 멈춘다. 그 명령을 그대로 돌린다(약 10 GB, 인터넷 1시간 안팎).
   root로 돌아 비밀번호를 묻지 않는다. 꼴은 아래와 같다(경로·브랜치는 실행기가 채운다).

   ```powershell
   wsl -d Ubuntu-24.04 -u root -- bash <wsl_sim_setup.sh의 WSL 경로> <Ubuntu 사용자> <코드 폴더의 WSL 경로> <git 폴더> <브랜치>
   ```

   | 단계 | 하는 일 |
   | --- | --- |
   | 1/7 | git·venv·scipy·opencv |
   | 2/7 | 자율 비행 사본 `~/ttrepo`(Windows 저장소의 지금 커밋을 얕게 복사) |
   | 3/7 | ROS 2 Jazzy + Gazebo Harmonic(자율 비행 `setup_jazzy.sh`) |
   | 4/7 | YOLO 환경 `~/venvs/tt`(자율 비행 `setup_yolo.sh`) |
   | 5/7 | 드라이버 영상(PyAV)·깊이 추정(transformers) |
   | 6/7 | 모델 미리 받기: `yolo26n-seg.pt`(`simulation/tt-sim/weights/`), 깊이 모델(Hugging Face 캐시) |
   | 7/7 | 시뮬레이터 세계 `s7_stage3`(`generate_world.py --seed 7 --stage 3`) |

3. 끝에 GPU 이름이 나오면 WSL에서 GPU를 쓴다(이 PC 확인).

- WSL 사본은 실행기가 켤 때마다 이 코드 폴더의 **지금 커밋**으로 맞춘다. 커밋하지 않은 변경은 WSL에 들어가지 않는다.
- WSL 미러 네트워크(`networkingMode=mirrored`)는 확인하지 않았다. 기본(NAT)으로 둔다.

## 4. 실행

| 명령 | 켜는 것 | 관제 화면 |
| --- | --- | --- |
| `scripts\start.cmd fake` (메뉴 1) | 체험: 가짜 드론 게이트웨이·사람 탐지·가짜 지도·지도 콘솔 | `http://127.0.0.1:8030/#/live` |
| `scripts\start.cmd real` (메뉴 2) | 실기체 손 조종: 게이트웨이·사람 탐지·지도 콘솔·지도 자동 시작. `--phone`이면 같은 망 폰에서도 열림(인증 없음) | `http://127.0.0.1:8010/#/live` |
| `scripts\start.cmd sim` | 자율 비행 시뮬레이터(WSL) + 보기 노드 + 게이트웨이 + 사람 탐지 + 3D 지도 | `http://127.0.0.1:8020/?preset=autonomy#/live` |
| `scripts\start.cmd autonomy` | 실기체 자율 비행: Windows 중계 + WSL `real.launch.py` + 보기 노드 + 게이트웨이 + 사람 탐지 + 3D 지도 | 위와 같음 |
| `scripts\start.cmd autonomy --fake-tello` | 위와 같은 길을 가짜 Tello로(날지 않음). 드론 없이 끝까지 시험할 때 | 위와 같음 |
| `scripts\start.cmd prepare-flight` | 모델·이미지 준비(3-2) | 없음 |

- `fake`·`real`·`sim`·`autonomy`에 `--no-browser`를 붙이면 브라우저를 열지 않는다.
- 모든 모드가 켜기 전에 빠진 Python·웹 패키지를 확인하고 받는다(모델은 받지 않음, 3-2).
- **실기체 자율 비행 순서:** 드론 켜기 → PC를 드론 Wi-Fi(`TELLO-…`/`RMTT-…`)에 연결 → `scripts\start.cmd autonomy`.
  Wi-Fi 이름으로 확장 키트를 정한다(`RMTT-…`면 전방 ToF 씀, `TELLO-…`면 `front_tof:=false`). 화면까지 약 3분.
- 자율 비행 웹 `http://127.0.0.1:8080`에서도 같은 비행을 본다.
- 자율 비행은 한 번 끝나면(끝·중단) 다시 시작하지 않는다. 다시 날리려면 모두 끄고 다시 켠다.
- 자율 비행 탐색 시간은 자율 비행 launch 기본값(`search_time` 120초, 끝나면 출발점으로 돌아와 착륙 `return_home`)이다.
  바꿀 때는 자율 비행 파트와 값을 맞춘 뒤 실행기 뒤에 `이름:=값`으로 붙인다. 실행기는 `real.launch.py`(`sim`이면 `mission.launch.py`)에 그대로 넘긴다.
  예: `scripts\start.cmd autonomy search_time:=150`. 이미 켜진 자율 비행에는 적용되지 않는다(실행기가 알린다).

## 5. 포트

| 포트 | 무엇 | 모드 |
| --- | --- | --- |
| TCP 8010 | 실기체 게이트웨이(기록 재생 기본도 8010) | real |
| TCP 8020 | 자율 비행 보기 게이트웨이 | sim, autonomy |
| TCP 8030 | 가짜 드론 게이트웨이 | fake |
| TCP 8790 | 지도 콘솔 | real |
| TCP 8795 | 가짜 지도 서버 | fake |
| TCP 8796 | 영상 복원용 지도 콘솔 | fake |
| TCP 8797 | 지도 콘솔 | sim, autonomy |
| TCP 8080 | 자율 비행 웹(WSL) | sim, autonomy |
| TCP 8094 | 보기 노드(WSL `127.0.0.1`, Windows `127.0.0.1`로 붙음) | sim, autonomy |
| UDP 8889 | Tello 명령(드론 `192.168.10.1:8889`) | real, autonomy |
| UDP 8890 | Tello 상태 받기 | real, autonomy |
| UDP 11111 | Tello 영상 받기 | real, autonomy |
| UDP 9889 | Windows 중계 ← WSL 드라이버 명령 | autonomy |
| UDP 18889·18890·11112 | 가짜 Tello 명령·상태·영상 | autonomy `--fake-tello` |

다른 프로그램이 이 포트를 쓰면 실행기가 켜지 않고 그 프로그램(PID·명령줄)을 보여 준다.

## 6. 방화벽 (실기체)

- 이 Python(게이트웨이·중계)이 드론 상태·영상(UDP 8890·11111)을 받아야 한다.
- 실행기가 켤 때 드론 Wi-Fi 네트워크 종류(공용·개인)에서 허용·차단 규칙을 본다. 드론 Wi-Fi에 붙기 전이면 둘 다 허용돼야 통과다.
- 모자라면 **Windows 관리자 확인 창**을 띄운다. "예"면 이 Python의 UDP 8890·11111만 모든 네트워크 종류에서 여는 규칙
  `Tello UDP (Chill-Drone)`을 넣고, 이 Python의 UDP 차단 규칙은 끈다(지우지 않음).
- "아니요"면 관리자 PowerShell에서 넣을 명령을 보여 준다. 지우기: `Remove-NetFirewallRule -DisplayName "Tello UDP (Chill-Drone)"`.
- 처음 켤 때 Windows 허용 창이 따로 뜨면 **허용**을 누른다.
- WSL 드라이버 → Windows 중계(UDP 9889)는 규칙 없이 들어왔다(이 PC 11/16).
- 폰 접속(`real --phone`)용 TCP 규칙은 실행기가 만들지 않고 명령만 보여 준다. 시험 뒤 지운다.

## 7. 끄기

- 실행기 창에서 **Enter** 또는 Ctrl+C.
- 실기체가 이륙·비행·착륙 중이면 아무것도 끄지 않고 다시 기다린다. **땅에서만 끈다.**
- 드론을 먼저 꺼서 땅인지 모를 때는 10초 안에 Enter를 한 번 더 누르면 끈다.
- 끄는 차례: 지도 저장 → 사람 탐지·지도 연결기·지도 콘솔 → 게이트웨이(회차 `metrics.json` 마무리) → WSL 자율 비행·보기 노드(Ctrl+C면 드라이버가 착륙하고 `viewer.html`을 만든다) → Windows 중계.
- 실행기 창을 그냥 닫으면 서비스가 남을 수 있다. 다시 실행하면 켜진 것을 알아보고, Enter로 끈다.

## 8. 기록과 다시 보기

| 무엇 | 어디 |
| --- | --- |
| 관제 게이트웨이 회차 | `data/outputs/tello/<회차>`(손 조종 `tello_…`, 자율 비행 보기 `ros_…`): manifest·commands·sensor·events·detections·지도 스냅샷·`autonomy/` |
| 실행기·서비스 로그 | `data/outputs/launcher/`(`gateway-8020.log`, `people-8020.log`, `map-bridge-8020.log`, `console-8797.log`, `autonomy-real.log`, `autonomy-view-node.log`, `tello-relay.log` 등) |
| 자율 비행 기록 | WSL `~/ttrepo/data/outputs/flights/<시각>/`(찾은 사람 `people.json`, 찾은 장애물 `discovered.json`, 3D 칸 지도 `camera_map.npz`, `viewer.html`). 시뮬레이터는 찾은 사람을 세계 폴더 `worlds/s7_stage3/people.json`에 씀 |
| 3D 지도 | `output/phone_mapping/live_<id>` |

기록 다시 보기(드론 없이 같은 화면, 명령은 모두 거절):

```powershell
.\.venv\Scripts\python.exe -B modules\autonomy\tello_link\gateway_server.py --replay data\outputs\tello\<회차> --replay-loop
# http://127.0.0.1:8010/#/live   (--replay-speed 2 두 배속, --replay-start 60 60초부터, --port로 포트 바꿈)
```

자율 비행 보기 회차(`ros_…`)에는 영상 원본이 없다(영상은 자율 비행 드라이버가 자기 폴더에 남긴다).

## 9. 시험 (선택)

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests
cd modules\autonomy\tello_link
..\..\..\.venv\Scripts\python.exe -B -m unittest discover -s tests
```

- 자율 비행 드라이버 시험(표준 라이브러리만, WSL): `python3 -B -m unittest discover -s modules/autonomy/tello_driver/tests -v`
- 게이트웨이 시험은 가짜 Tello 포트를 쓴다. 다른 시험·실행기와 동시에 돌리면 포트가 겹쳐 오류가 난다.

## 10. 문제가 생기면

| 증상 | 할 일 |
| --- | --- |
| [실행 전 확인] Python | Python 3.12 64비트를 설치하고 다시 실행 |
| 드론 상태·영상이 안 옴 | 6 방화벽, 폰 앱·점검 화면 끄기, 드론 껐다 켜기. `tello-relay.log`의 "5초 동안 명령 n, 응답 n, 상태 n, 영상 패킷 n"이 0이 아니어야 함 |
| "드론 Wi-Fi에 연결돼 있지 않아요" | PC를 `TELLO-…`/`RMTT-…`에 먼저 연결(드론 없이면 `--fake-tello`) |
| 다른 자율 비행이 이미 켜져 있다고 멈춤 | 그 실행기 창에서 Enter로 모두 끄고(실기체는 땅에서) 다시 실행 |
| 화면에 "이륙 거절" 알림 | 드론을 켠 직후 40–55초는 기수 값이 흘러 Tello가 이륙을 거절한다. 시작 칸 기수 값이 "멈춤"이 되면 다시 시작 |
| 자율 비행이 잘 안 움직임·인식이 느림 | 노트북 충전기 확인(배터리면 GPU가 묶임) |
| 드론이 뜨거움 | 땅에서 분당 약 10°C 오른다. 드론을 켜고 약 1분 뒤 기수 값이 멈추면 1–2분 안에 이륙. 자율 비행 드라이버는 80°C 이상 이륙을 막고 85°C면 그 자리에 착륙 |
| 뒤집힘·충돌 빨간 띠 | 프로펠러·가드·모터를 살핀 뒤 다시 켠다(띠는 다음 이륙까지 남음) |
| 3D 점이 안 쌓임 | 실행기 창의 "지도 세션 … 실패" 문구, `console-8797.log` |
| 사람 탐지 결과가 없음 | 발견 칸 상태, `people-8020.log`, 모델 준비(3-2) |

## 11. 라이선스 주의

오픈소스 공개 전 의존성 라이선스를 확인한다.

| 의존성 | 라이선스 |
| --- | --- |
| Ultralytics YOLO | AGPL-3.0 |
| ORB-SLAM3 | GPLv3 |
| COLMAP | BSD |
| 깊이 모델 `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf` | Hugging Face 모델 카드에 라이선스 표시가 없다(11/17 확인). 공개 전 원본 저장소에서 확인 |
| 장면 분할 모델 `nvidia/segformer-b0-finetuned-ade-512-512` | 모델 카드 "other": NVlabs/SegFormer 저장소의 LICENSE를 따른다(상업 이용 조건은 그 파일에서 확인) |
| 게이트웨이 패키지(fastapi MIT, uvicorn BSD-3-Clause, av BSD-3-Clause, opencv-python Apache-2.0 등) | `modules/autonomy/tello_link/requirements.txt` 주석 |
