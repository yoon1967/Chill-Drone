# 외부 서비스와 모델 준비

[빌드·배포 포팅 매뉴얼](BUILD_AND_DEPLOY.md)의 최종 통합 커밋 기준입니다. 드론 Wi-Fi에 연결하기 전에 소스, 의존성, 모델과 Docker 이미지를 준비합니다.

## 서비스 계정과 네트워크

| 서비스 | 프로젝트에서 하는 일 | 필요한 접속 권한 |
| --- | --- | --- |
| SSAFY GitLab | 개발 원본 클론, MR, CI·환경 확인 | 검수자는 프로젝트 읽기 권한. 배포·Runner 관리에는 별도 운영 권한 |
| SSAFY Jira Cloud | 이슈·작업 상태, GitLab 이벤트 연동 | 이슈를 열어볼 때 프로젝트 접근 권한. 관제 실행에는 계정 불필요 |
| AWS EC2 / Let's Encrypt | EC2에서 Docker 상태 페이지를 제공하고 Caddy가 TLS 인증서 발급·갱신 | 배포 관리자는 팀에서 승인한 EC2 SSH 정보 사용. HTTPS 상태 페이지 조회와 Windows 관제 실행은 별도 클라우드 계정 없이 사용 |
| PyPI·PyTorch 휠 저장소·npm Registry | Python, PyTorch CPU/cu128, pnpm·Three.js 등의 설치 | 기본 공개 배포 경로에는 별도 토큰 불필요. 첫 설치 때 인터넷 접속 |
| Docker Hub·Ubuntu/ROS 패키지 저장소·GitHub | 베이스 이미지, ROS 2/Gazebo, 추적·복원 의존 소스 | 기본 공개 리소스의 다운로드 권한. 사내 프록시 또는 Docker Hub 다운로드 제한에 걸리면 해당 환경의 접근 정책 확인 |
| Hugging Face | 공개 AI 가중치와 일부 추적 자산 배포 | 문서 기준 모델을 받을 때 별도 앱 API 키 불필요. 실행 중 추론은 Windows/WSL에서 수행 |
| Tello EDU / RoboMaster TT | Wi-Fi 직접 연결과 로컬 UDP 명령·영상·상태 | 기체 사용 권한과 드론 Wi-Fi 연결. 소셜 로그인, Photon Cloud, 원격 컴파일 계정은 실행에 필요하지 않음 |

[GitLab 프로젝트](https://lab.ssafy.com/s15-final/S15P31B205) · [Jira 프로젝트](https://ssafy.atlassian.net/jira/software/c/projects/S15P31B205/list) · [EC2 상태 페이지](https://k15b205.p.ssafy.io/)

## 모델 준비 방식

| 기능 | 자산 | 설치와 보관 위치 |
| --- | --- | --- |
| Windows 비행 중 지도, 의미 분할, 사람 탐지 | `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`, `nvidia/segformer-b0-finetuned-ade-512-512`, `Ultralytics/YOLO26`의 `yolo26n-seg.pt` | `.\scripts\start.cmd prepare-flight`. 깊이·분할은 Hugging Face 캐시, YOLO는 Torch Hub `checkpoints`. YOLO는 코드가 고정 리비전과 SHA-256 확인 |
| 자율 비행 WSL | 깊이 추정 모델, `yolo26n-seg.pt` | `wsl_sim_setup.sh`에서 설치. 가중치는 WSL Hugging Face 캐시 및 `~/ttrepo/simulation/tt-sim/weights/` |
| 저장 영상의 자세·깊이 복원 | ORB-SLAM3, DPVO, ORB-SLAM3 + DPVO, DPV-SLAM, DPV-SLAM++, ViSTA-SLAM. 깊이는 Depth Anything V2, Metric Video Depth Anything Small, MoGe-2 Small 등 | **지도 콘솔 → 지도 복원**에서 영상을 선택하고 필요한 모델을 준비. 모델 파일은 `data/models/reconstruction/`, 추적 실행 이미지는 Docker 로컬 저장소 |
| 저장 점군의 구조 복원 | Roomform `jchiu/roomform`, 리비전 `219272c20d8f8a71382405e54dbf1c6e7020f5ad` | **지도 콘솔 → 지도 편집 → 저장된 지도 → AI 복원모델 다운받기**. 준비 후 **AI 복원하기**로 벽·바닥·천장 면을 PC에서 추론. `data/models/reconstruction/roomform/`에 설치 |

`prepare-flight`는 실시간 지도·사람 탐지에 필요한 자산을 준비합니다. 저장 영상의 모델 선택 화면에서 요구하는 자산과 Roomform은 각각 처음 쓰기 전에 준비합니다. 모델별 리비전, SHA-256, 크기 및 Docker 빌드 대상은 아래 정의에서 확인할 수 있습니다.

- [실시간 비행 모델과 캐시 확인](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/modules/mapping/data/model_assets.py)
- [저장 영상·Roomform 모델 매니페스트](https://lab.ssafy.com/s15-final/S15P31B205/-/blob/411e36e85cf6144ffab8573bd34f326d7a5de1fd/modules/mapping/data/reconstruction_models.py)
- [공식 Roomform 가중치](https://huggingface.co/jchiu/roomform/tree/219272c20d8f8a71382405e54dbf1c6e7020f5ad)
