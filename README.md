# S15P31B205 · 실내 탐색·3D 지도·사람 탐지

휴대폰 영상 기반 3D 복원과 사람 탐지, ROS 2·Gazebo 자율비행을 개발하는 팀 저장소입니다.
각 기능의 구현·검증 수준은 연결된 Jira 작업과 MR의 실제 결과를 기준으로 확인합니다.

## 처음 참여한다면

1. [팀 시작 안내](docs/team/START_HERE.md): 계정 설정부터 첫 브랜치·MR까지.
2. [폴더 구조와 병렬 작업](docs/team/REPOSITORY_STRUCTURE.md): 코드 위치, 공용 계약, 작업 공간 분리.
3. [현재 Jira 담당·작업 지도](docs/team/JIRA_WORK_MAP.md): 담당자별 실제 이슈와 코드 경계.
4. [공통 협업 규칙](.gitlab/WORKFLOW_SETUP.md): 브랜치·커밋·MR 및 완료 기준.

현재 개발 원본은 [GitLab](https://lab.ssafy.com/s15-final/S15P31B205), 계획·담당·상태의 원본은
[Jira](https://ssafy.atlassian.net/jira/software/c/projects/S15P31B205/list)입니다.
GitHub는 공개 저장소가 정해진 뒤 [공개 전환 안내](docs/team/GITHUB_RELEASE.md)에 따라 연결합니다.

| 위치 | 역할 |
| --- | --- |
| `modules/` | 위치 추정·지도 데이터·객체 인식·자율비행 코드 |
| `simulation/` | ROS 2·Gazebo 모델·월드·실행 구성 |
| `services/api/`, `apps/console/` | API와 사용자 화면의 구현 공간 |
| `contracts/` | 파트 사이의 API·메시지·파일 형식 계약 |
| `data/` | 데이터 취급 안내와 공개 가능한 작은 예제 |
| `infra/` | EC2·Docker·HTTPS·배포와 상태 수집 |
| `tests/`, `docs/` | 공통 검증과 팀 문서 |

현재 폴더는 협업 경계를 준비한 것이며 각 파트의 앱·패키지가 실행 가능하다는 뜻은 아닙니다.
기존 다른 브랜치의 코드를 일괄 이동하지 않습니다.

## 서버와 배포

- [실제 서버·Docker 상태](https://k15b205.p.ssafy.io/)
- [배포 확인](https://k15b205.p.ssafy.io/healthz)
- EC2 운영 문서는 배포 기반 MR의 `docs/EC2_CICD.md`를 사용합니다. develop 반영 후 같은 경로에서 볼 수 있습니다.
- Jira 상태 연동은 기존 Webhook, 코드 검증·배포는 EC2 Runner가 담당합니다.
  자동 convention 검사·자동 리뷰 댓글·필수 동료 승인 대기는 두지 않습니다.

기여 방법은 [CONTRIBUTING.md](CONTRIBUTING.md), 에이전트 지침은 [AGENTS.md](AGENTS.md),
보안 제보는 [SECURITY.md](SECURITY.md)를 참고합니다.
오픈소스 라이선스와 GitHub 공개 대상은 아직 확정되지 않았습니다.
