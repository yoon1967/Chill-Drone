# 설치·실행 가이드

통합 관제는 Windows의 `scripts/start.cmd`에서 시작합니다. 자율 비행 모드는 WSL에서 ROS 2 노드를 함께 실행합니다. 아래 명령은 저장소 루트에서 PowerShell로 실행합니다.

## 모드 선택

| 목적 | 명령 | 관제 화면 | 준비 |
| --- | --- | --- | --- |
| 관제 화면과 흐름 체험 | `.\scripts\start.cmd fake` | [8030](http://127.0.0.1:8030/#/live) | Windows, Python 3.12 x64, Node.js LTS 또는 사용 가능한 pnpm |
| 실기체 수동 관제 | `.\scripts\start.cmd real` | [8010](http://127.0.0.1:8010/#/live) | Windows 환경, 비행용 자산, Tello EDU 또는 RoboMaster TT |
| 드론 없이 자율 탐색 확인 | `.\scripts\start.cmd sim` | [8020](http://127.0.0.1:8020/?preset=autonomy#/live) | Windows·WSL 환경, Gazebo, 비행용 자산 |
| 실기체 자율 탐색 | `.\scripts\start.cmd autonomy` | [8020](http://127.0.0.1:8020/?preset=autonomy#/live) | Windows·WSL 환경, 비행용 자산, Tello EDU 또는 RoboMaster TT |

`fake`는 가짜 드론과 예제 지도를 사용합니다. 실제 사람 탐지 모델까지 쓰려면 비행용 자산을 준비합니다. `autonomy --fake-tello`는 실기체 없이 자율 비행 연결 경로를 확인하는 옵션입니다. `sim`과 `autonomy`의 자율 비행 웹은 [8080](http://127.0.0.1:8080/)에서도 열립니다.

## 처음 한 번 준비

1. GitLab 접근 권한으로 저장소를 클론하고 Python 3.12 x64와 Node.js LTS를 설치합니다. 실행기가 `.venv`와 지도 콘솔의 웹 의존성을 준비합니다.
2. 사람 탐지와 실시간 매핑을 쓰려면 Docker Desktop의 Linux 엔진을 켜고 인터넷 연결 상태에서 `.\scripts\start.cmd prepare-flight`를 실행합니다. 비행용 모델과 ORB-SLAM3 Docker 이미지를 준비합니다.
3. 자율 비행 모드는 WSL 2의 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic을 사용합니다. Ubuntu를 준비하고 `.\scripts\start.cmd sim`을 실행한 뒤, 처음 표시되는 `wsl_sim_setup.sh` 설치 명령을 따릅니다. 자율 비행 코드의 WSL 사본은 Windows 저장소에 커밋된 버전으로 동기화됩니다.
4. 실기체를 사용하는 경우 설치와 모델 준비를 마친 뒤 드론 Wi-Fi에 연결합니다. `real`과 `autonomy`는 같은 기체의 명령 연결을 사용하므로 한 번에 하나만 실행합니다.

저장 영상의 자세·깊이 복원과 Roomform은 지도 콘솔에서 해당 모델을 처음 선택할 때 별도로 준비합니다. 사용할 모델을 미리 받아 두면 드론 Wi-Fi에 연결한 상태에서도 실행할 수 있습니다.

## 화면과 기록

| 화면 | 주소 경로 | 사용 |
| --- | --- | --- |
| 실시간 감독 | `#/live` | 영상, 요구조자 발견, 2D 지도와 3D 점군, 드론 조작 |
| 지도 콘솔 | `#/map` | 영상·드론 세션 ZIP 복원, 저장 점군 편집과 Roomform 구조 복원 |
| 탐색 결과 | `#/review` | 발견 카드·사진, 확인과 오탐, 지도와 비행 기록 검토 |
| 현장 조작 | `#/control` | 같은 망에 연결된 폰에서 상태·영상과 지원 조작 사용 |

실기체 착륙을 확인한 뒤 실행기 창에서 Enter를 눌러 종료합니다. 실행 로그는 `data/outputs/launcher/`, 관제 회차는 `data/outputs/tello/`, 지도 세션은 `output/phone_mapping/`에서 확인합니다.

| 증상 | 확인할 내용 |
| --- | --- |
| 실행 전 확인에서 멈춤 | Python 3.12 x64, Node.js 또는 pnpm, 요청한 모드의 WSL/Docker 상태와 실행기 출력 |
| 실기체의 상태·영상이 들어오지 않음 | 드론 Wi-Fi, 실행기에 표시된 Windows 방화벽 안내, 동일 기체를 쓰는 다른 조종 앱 |
| 모델 또는 추적기 이미지를 찾지 못함 | 인터넷에 연결해 `prepare-flight`를 다시 실행하거나 지도 콘솔에서 사용하려는 모델 준비 |
| 관제에 3D 점군이 표시되지 않음 | 영상 연결, 지도 칸의 **3D 점군** 및 지도 상태, 실행 로그의 지도 시작 메시지 |

- [실행기 상세 설정](../apps/launcher/README.md) · 포트, Python 검색 순서, Windows와 WSL 연결
- [포팅 매뉴얼](../exec/BUILD_AND_DEPLOY.md) · 재현 가능한 버전, 환경 변수, EC2 운영 설정
- [외부 서비스와 모델](../exec/EXTERNAL_SERVICES.md) · 최초 설치에 필요한 서비스와 가중치
- [시연 시나리오](../exec/PRESENTATION_DEMO.md) · 자율 탐색, 요구조자 탐지, 실시간 3D 지도
- [기록 재생과 데이터](../exec/DATABASE.md) · 회차 구조와 재생 방법
