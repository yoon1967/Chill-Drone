# 개발·운영 환경 구성

작업: [S15P31B205-8](https://ssafy.atlassian.net/browse/S15P31B205-8).
기준: 2026-10-22 현재 저장소와 EC2. 파트별 앱·DB·운영 승격은 아직 구현되지 않았다.

## 환경 구분

| 환경 | 실행 위치·용도 | 기준과 차이 |
| --- | --- | --- |
| 팀 로컬 개발 | 작업별 clone/worktree, 자기 파트 구현·단위 검증 | Git, 지원 Python 등 파트별 필요 도구만 설치 |
| 상태 서비스 로컬 재현 | 로컬 Docker 또는 제공된 테스트 | Dockerfile의 Python 3.13 Alpine 이미지 digest 기준. 호스트 상태 없으면 degraded가 정상 |
| CI 검증 | EC2의 프로젝트 전용 Docker Runner | .gitlab-ci.yml의 Python 3.13 이미지 digest, privileged=false, Docker 소켓 미공유 |
| staging 배포 | 제공 EC2 Ubuntu 24.04 x86_64 | Docker·Compose, 보호된 develop만 빌드/배포. 현재는 상태 웹 |
| 향후 운영 | master 또는 별도 승격 환경 | 도메인·권한·DB·백업·승격 조건 미정. 현재 staging을 운영 HA로 부르지 않음 |
| 지도/탐지 실험 | 허가된 로컬 계산 자원 | 공개/합성 예제와 실제 촬영 분리, 데이터·모델 버전 기록 |
| Gazebo 시뮬레이션 | Jira에 기록된 WSL2/Ubuntu 개발 환경 | Ubuntu 24.04·ROS 2 Jazzy·Gazebo Harmonic. 해당 패키지의 이 저장소 재현은 미실행 |

현재 PC의 Python 3.14.6에서도 저장소 테스트를 실행했다. CI 기준은 3.13이며 각 파트의
Python/Node/ROS 버전과 lockfile은 실제 구현자가 해당 README에 명시한다. 한 전역 환경에 모두 설치하지 않는다.

## 최소 재현

최신 develop을 별도 작업 공간으로 가져온 뒤 저장소 루트에서 실행한다.

~~~text
python -B -m unittest discover -s tests -v
git diff --check
~~~

로컬 Docker가 있는 경우 상태 웹의 생존 경로만 아래처럼 재현할 수 있다.
아래 명령은 가이드이며 이번 문서 작업에서 팀원의 PC에 Docker를 설치하거나 실행하지 않았다.

~~~text
docker build -t s15p31b205/status:local infra/status
docker run --rm -p 127.0.0.1:18065:18066 s15p31b205/status:local
~~~

접속은 http://127.0.0.1:18065/healthz다. 로컬에 호스트 상태 snapshot을 제공하지 않았으므로
/api/status의 fresh=false·degraded는 예상 결과다. Docker 소켓이나 서버 토큰을 공유해 해결하지 않는다.
실제 EC2 재구성은 [운영 가이드](../EC2_CICD.md)의 관리자 절차를 따른다.

## 설정과 비밀정보

| 설정 | 현재 공급 위치·역할 |
| --- | --- |
| APP_IMAGE | 배포 스크립트가 전달하는 커밋 SHA 이미지 참조. 비밀값 아님 |
| REVISION | Docker build argument로 빌드한 배포 식별자 |
| 배포 release 상태 | 서버의 /srv/s15p31b205, current/previous와 release별 Compose·release.env |
| Runner 인증 | root만 읽는 /etc/gitlab-runner/config.toml |
| TLS 개인 키 | Caddy 전용 named volume, 권한 제한 |
| 향후 앱/DB/API 인증 | 필요한 이름을 먼저 명세하고 보호된 CI 변수 또는 서버 권한 제한 파일에 공급 |
| 개인 SSH 키 | 각 소유자의 제한된 로컬 저장소. Git·CI 변수·MR·Jira에 복사하지 않음 |

현재 설치 스크립트의 apt 패키지는 실행 시점 저장소 버전을 받는다. 조사된 버전과 이미지 digest는
운영 기록이며 모든 apt 패키지를 고정한 재현 lockfile은 아니다. 재구성 전 버전 차이를 확인하고
기존 서비스가 있는 서버에 설치 스크립트를 무심코 다시 실행하지 않는다.

## 권한·데이터·검증

- 검증 Runner와 배포 Runner를 분리하고 배포 Runner는 보호 브랜치의 신뢰된 코드에만 사용한다.
- 현재 GitLab 팀원 역할은 프로젝트 설정에 따르며 폴더 담당표가 서버 관리자 권한을 부여하지 않는다.
- 공개 가능한 작은 합성 예제는 data/samples, 원본·가중치·생성물은 ignore된 로컬/비공개 저장 경로를 쓴다.
- worktree마다 의존성·캐시·산출물과 run_id를 분리한다. 경로·데이터 해시·버전·설정 이름으로 인계한다.
- 미구현 BE/FE 실행, 실물 비행, GPU 학습, DB 복구는 통과했다고 표시하지 않는다.
- 문서 변경은 Jira 템플릿 검사·로컬 링크·diff 검증, MR은 기존 CI 검증으로 확인한다.

[팀 시작 안내](../team/START_HERE.md)와 [파트별 경계](../team/REPOSITORY_STRUCTURE.md)를 함께 사용한다.
