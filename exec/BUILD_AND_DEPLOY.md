# Chill-Drone 포팅 매뉴얼

실내 정찰 관제의 최종 통합본을 새 Windows PC에서 설치하고 실행하는 절차입니다. 코드 기준은 [feat/final-integration의 411e36e8](https://lab.ssafy.com/s15-final/S15P31B205/-/commit/411e36e85cf6144ffab8573bd34f326d7a5de1fd)입니다.

- [외부 서비스와 모델 준비](EXTERNAL_SERVICES.md)
- [DB 덤프 제출 여부와 기록 복원](DATABASE.md)
- [4. 시연 시나리오](PRESENTATION_DEMO.md)

## 1. 설치 환경과 버전

| 구성 | 버전과 설정 | 저장소 기준 파일 |
| --- | --- | --- |
| 개발·실행 PC | Windows 11 x64 기준. Git과 PowerShell 사용. IDE는 선택 사항이며 프로젝트에 고정 버전이 없음 | `scripts/start.cmd` |
| JVM / WAS | JVM이나 별도 Java WAS 설치가 필요하지 않음. 드론 게이트웨이는 Python FastAPI/Uvicorn으로 구동 | `modules/autonomy/tello_link/requirements.txt` |
| Windows Python | **3.12 x64**. Git checkout별 `.venv`. FastAPI ≥0.110, Uvicorn ≥0.29, PyAV ≥12, OpenCV ≥4.10은 요구 범위 | `apps/launcher/prepare.py`, `modules/autonomy/tello_link/requirements.txt` |
| 관제·지도 화면 | 관제는 브라우저 정적 파일. 지도 콘솔은 Three.js **0.180.0**. Node.js LTS, pnpm 또는 `npx pnpm@10`. 웹 의존성은 lockfile 사용 | `apps/controller/`, `apps/console/package.json`, `apps/console/pnpm-lock.yaml` |
| Windows AI | PyTorch **2.11.0**, torchvision **0.26.0**. NVIDIA GPU를 인식하면 CUDA **12.8** 휠, 그 외 CPU 휠. Ultralytics **8.4.189** | `apps/launcher/prepare.py`, `modules/mapping/data/requirements-stream.txt` |
| 지도 추적기 | Docker Desktop Linux 엔진, 실시간 지도용 `ssafy-orbslam3:local`. 저장 영상의 DPVO·DPV-SLAM 계열 기본 이미지는 CUDA 컴퓨팅 기능 8.6 이상 대상 | `apps/launcher/docker_assets.py`, `modules/mapping/localization/` |
| 자율 비행·시뮬레이션 | WSL 2 Ubuntu **24.04**, 기본 NAT, ROS 2 **Jazzy**, Gazebo **Harmonic**. WSL 가상환경 `~/venvs/tt` | `apps/launcher/wsl_sim_setup.sh`, `simulation/tt-sim/scripts/setup_jazzy.sh` |
| EC2 상태 페이지 | Ubuntu **24.04 amd64**, Docker Compose, Caddy **2 Alpine**과 Python **3.13 Alpine**의 고정 이미지 digest | `infra/edge.compose.yml`, `infra/status/Dockerfile` |

## 2. 클론과 최초 빌드

GitLab 읽기 권한으로 다음 명령을 실행합니다. 고정 커밋에서 설치와 실행을 시작하면 이 문서의 동작을 재현할 수 있습니다.

```powershell
git clone https://lab.ssafy.com/s15-final/S15P31B205.git
Set-Location S15P31B205
git switch --detach 411e36e85cf6144ffab8573bd34f326d7a5de1fd
git rev-parse HEAD
```

1. 인터넷이 연결된 상태에서 Python 3.12 x64와 Node.js LTS를 설치합니다. 3D 지도까지 사용할 PC는 Docker Desktop을 실행해 Linux 컨테이너 엔진이 준비된 상태로 둡니다.
2. 저장소 루트에서 `.\scripts\start.cmd prepare-flight`를 실행합니다. 실행기가 Python·웹 의존성을 준비하고 비행용 모델을 받고 ORB-SLAM3 이미지를 빌드합니다. 가상환경과 캐시는 해당 PC에서 생성됩니다. **[실행 전 확인]**이 나오면 표시된 항목을 준비하고 다시 실행합니다.
3. 자율 비행이나 Gazebo를 사용한다면 관리자 PowerShell에서 `wsl --install -d Ubuntu-24.04`를 실행하고 Ubuntu 초기 사용자를 설정합니다. 이후 일반 PowerShell에서 `.\scripts\start.cmd sim`을 실행합니다. 환경이 비어 있으면 실행기가 표시하는 `wsl_sim_setup.sh` 명령을 실행해 ROS/Gazebo, 모델과 `s7_stage3` 월드를 준비한 다음 `sim`을 다시 시작합니다. WSL의 `~/ttrepo`에는 현재 Windows checkout의 **커밋된 코드**가 동기화됩니다.
4. 설치가 끝나면 사용할 모드를 실행합니다. 첫 실행 및 캐시 재빌드에는 인터넷과 디스크 여유가 필요합니다. 드론 Wi-Fi에 연결하기 전에 모델과 이미지를 준비합니다.

| PowerShell 실행 인자 | 관제 화면 | 용도 |
| --- | --- | --- |
| `.\scripts\start.cmd fake` | `http://127.0.0.1:8030/#/live` | 드론 없이 관제·가짜 지도·영상 복원 화면 확인 |
| `.\scripts\start.cmd real` | `http://127.0.0.1:8010/#/live` | Tello 실기체 수동 관제·영상·사람 탐지·실시간 지도 |
| `.\scripts\start.cmd sim` | `http://127.0.0.1:8020/?preset=autonomy#/live` | WSL Gazebo 자율 비행을 관제에서 확인 |
| `.\scripts\start.cmd autonomy` | `http://127.0.0.1:8020/?preset=autonomy#/live` | Windows UDP 중계와 WSL ROS를 사용하는 실기체 자율 비행 |
| `.\scripts\start.cmd autonomy --fake-tello` | `http://127.0.0.1:8020/?preset=autonomy#/live` | 실기체 없이 자율 비행 연결 경로 확인 |

각 모드에 `--no-browser`를 붙이면 주소를 직접 열 수 있습니다. `fake`의 관제에서 **지도 콘솔**로 들어가면 `#/map`에서 저장 영상 복원을 사용할 수 있습니다. 선택형 복원 모델은 인터넷 연결 상태에서 화면 안내에 따라 미리 설치합니다.

## 3. 환경 변수와 계정·설정 파일

| 이름 또는 설정 파일 | 기본값·입력 방법 | 적용 범위 |
| --- | --- | --- |
| `CHILL_DRONE_PYTHON` | 필요할 때 호스트의 Python 3.12 x64 실행 파일 절대 경로를 지정. checkout과 기본 worktree의 `.venv`를 먼저 탐색 | `scripts/start.cmd` |
| `CHILL_WSL_DISTRO` | `Ubuntu-24.04`. WSL 배포판 이름이 다른 경우 설정 | `apps/launcher/sim.py` |
| `ROOMFORM_CHECKPOINT` | 선택. 별도로 받은 Roomform 가중치 경로. 지도 서버 `--roomform-checkpoint` 옵션도 지원 | `apps/console/README.md` |
| `APP_IMAGE` | 배포 스크립트가 `release.env`에 기록. CI에서는 `s15p31b205/status:<CI_COMMIT_SHA>`를 사용 | `infra/app.compose.yml`, `infra/deploy.sh` |
| `REVISION` / `CI_COMMIT_SHA` | GitLab CI가 checkout 커밋을 제공. 빌드가 `REVISION` 인자로 넘겨 상태 페이지에 표시 | `.gitlab-ci.yml`, `infra/status/Dockerfile` |
| `DEPLOY_ROOT` / `HEALTH_URL` | 필요할 때만 지정. 기본 `/srv/s15p31b205` / `https://k15b205.p.ssafy.io/healthz` | `infra/deploy.sh` |
| EC2 운영 계정 / Runner 인증 | 운영 안내의 EC2 사용자 `ubuntu`와 운영자가 보관한 SSH 키 사용. Runner 인증은 서버 `/etc/gitlab-runner/config.toml`에서 관리 | `docs/EC2_CICD.md` |
| HTTPS·Compose·DB | `infra/Caddyfile`이 도메인·인증서·프록시 설정을, Compose가 이미지와 내부 포트를 정함. 통합 실행에 DB 계정이나 접속 문자열이 필요하지 않음 | `infra/Caddyfile`, `infra/app.compose.yml`, [DB 안내](DATABASE.md) |

기본 로컬 실행에서 별도 `.env`는 필요하지 않습니다. GitLab·EC2 관리자 인증 정보는 팀에서 지정한 방식으로 받습니다.

## 4. 배포와 네트워크

**Windows 관제**는 드론과 연결된 PC에서 실행합니다. `real`과 `autonomy`는 드론 Wi-Fi 연결과 Tello 명령 세션을 하나씩 사용하므로 동시에 실행할 수 없습니다. `TELLO-…`는 기본 기체, `RMTT-…`는 확장 키트 연결에 사용됩니다. 영상·상태가 들어오지 않으면 Windows 실행기의 방화벽 안내와 동일 기체에 붙어 있는 다른 조종 앱을 확인합니다. 실기체 착륙을 확인한 뒤 실행기에서 Enter로 기록 저장과 종료를 진행합니다.

| 포트 | 서비스 |
| --- | --- |
| TCP `8010 / 8020 / 8030` | 실기체 또는 기록 재생 / 자율 비행 관제 / 체험 게이트웨이 |
| TCP `8790 / 8795 / 8796 / 8797` | 지도 콘솔(real) / 가짜 지도 / 체험 모드 영상 복원 / 자율 비행 지도 콘솔 |
| TCP `8080 / 8094` | WSL 자율 비행 웹 / 관제용 보기 노드 |
| UDP `8889 / 8890 / 11111` | Tello 명령 / 상태 수신 / 영상 수신. `autonomy`는 WSL에서 Windows 중계로 `9889`도 사용 |

브라우저 주소는 기본적으로 루프백입니다. `--phone`은 인증 없이 같은 망에서 게이트웨이를 열기 때문에 통제된 시연 네트워크에서만 사용합니다.

**EC2 배포 대상은 상태 페이지입니다.** GitLab CI가 보호된 `develop`에서 `infra/status` 이미지를 빌드하고 `bash infra/deploy.sh <이미지> <커밋 SHA>`로 Compose 배포를 실행합니다. Caddy가 HTTPS `443`에서 호스트 `127.0.0.1:18065`로 전달하고, Compose가 이를 상태 컨테이너 `18066`에 연결합니다. 신규 EC2 구축, Runner 권한, 실패 시 이전 릴리스로의 복구는 [EC2 운영 안내](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/docs/EC2_CICD.md)를 따릅니다.

## 5. 설치 결과 확인

- 드론 없이 `.\scripts\start.cmd fake`를 실행해 관제의 연결 상태와 지도 콘솔 진입을 확인합니다.
- 자율 비행 환경을 준비한 경우 `sim`에서 관제 `8020`과 자율 비행 웹 `8080`을 각각 엽니다.
- 실행기와 백그라운드 서비스 로그는 `data/outputs/launcher/`에서 확인합니다. 모델 파일을 찾지 못하면 인터넷이 연결된 상태에서 `prepare-flight`를 다시 실행합니다.
- EC2 배포를 맡은 운영자는 [HTTPS 헬스 체크](https://k15b205.p.ssafy.io/healthz)의 응답과 GitLab 배포 잡 결과를 확인합니다.

[통합 실행기](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/apps/launcher/README.md) · [지도 콘솔](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/apps/console/README.md)
