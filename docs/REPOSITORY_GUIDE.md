# 저장소·개발·운영 안내

개발 원본은 [GitLab](https://lab.ssafy.com/s15-final/S15P31B205)이며, 작업 계획과 담당자는 [Jira 프로젝트](https://ssafy.atlassian.net/jira/software/c/projects/S15P31B205/list)에서 확인합니다.

## 저장소 구조

| 경로 | 역할 |
| --- | --- |
| [`scripts/`](../scripts/) | Windows 실행 진입점 `start.cmd` |
| [`apps/`](../apps/) | `launcher/` 통합 실행·준비, `controller/` 관제 웹, `console/` 지도 복원·편집 웹과 로컬 서버 |
| [`modules/autonomy/`](../modules/autonomy/) | Tello 통신·관제 게이트웨이, ROS 2 드라이버와 자율 비행 웹, 이전 실험 코드 |
| [`modules/mapping/`](../modules/mapping/) | 위치 추정과 SLAM, 영상 복원·점군·저장 파이프라인, 성능 측정 |
| [`modules/perception/`](../modules/perception/) | 영상 사람 탐지·의미 분할, 이벤트 출력, 학습 문서 |
| [`simulation/`](../simulation/) | `tt-sim/`의 Gazebo 월드·모델, 탐색·계획 노드, ROS 2 실행 설정 |
| [`contracts/`](../contracts/) | API, 탐지 이벤트, 좌표·영상·지도·저장 파일의 파트 간 계약과 예제 |
| [`data/`](../data/) | 지도 설정·검증 자료, 공개 가능한 샘플, 로컬 모델 디렉터리 자리 |
| [`services/api/`](../services/api/) | 별도 저장·조회 API 구현을 위한 예약 경로 |
| [`src/`](../src/) | 초기 휴대폰 IP 카메라·YOLO 확인 도구 |
| [`tests/`](../tests/) | 공통 회귀 검사와 통합 검증 안내. 기능별 검사도 해당 모듈에 위치 |
| [`infra/`](../infra/) | 상태 서비스, Caddy·Compose, EC2 배포·롤백과 자율 비행 웹 실행 도구 |
| [`automation/`](../automation/) | GitLab→GitHub 미러링, Jira MR 연동 |
| [`docs/`](./) · [`exec/`](../exec/) | 프로젝트·팀 기술 문서와 제출용 포팅·시연 문서 |
| [`.gitlab/`](../.gitlab/) · [`.github/`](../.github/) · [`.cursor/`](../.cursor/) | CI, MR·이슈 양식, 도구별 협업 설정 |

루트의 `.gitlab-ci.yml`은 검증·미러·빌드·배포 작업을 연결합니다. `CODEOWNERS`와 `AGENTS.md`에는 담당 경계와 작업 정책이 있습니다. 포맷 및 추적 설정은 `.editorconfig`, `.gitattributes`, `.gitignore`를 참고합니다.

## 변경과 검증

1. Jira 작업 범위를 정하고 최신 `develop`에서 작업 브랜치를 만듭니다. 계약이나 공용 파일이 바뀌면 소비하는 모듈과 검증 범위를 확인합니다.
2. 프로젝트 Python 환경에서 공통 회귀 검사와 diff 검사를 실행합니다.

   ```powershell
   .\.venv\Scripts\python.exe -B -m unittest discover -s tests -v
   git diff --check
   ```

3. 화면과 지도, 드론, 시뮬레이터는 모듈별 문서의 추가 검증을 따릅니다. 변경 내용과 실제 검증 결과를 `develop` 대상 MR에 남깁니다.

[기여 안내](../CONTRIBUTING.md) · [공통 협업 규칙](../.gitlab/WORKFLOW_SETUP.md) · [팀 시작 안내](team/START_HERE.md) · [통합 시험 안내](../tests/integration/README.md)

## 브랜치와 서비스 배포

| 대상 | 현재 동작 |
| --- | --- |
| MR | GitLab CI 검증 |
| `develop` | 팀 개발 통합, GitHub 미러, EC2 상태 페이지 이미지 빌드와 스테이징 배포 |
| `master` | 릴리스 기준, GitLab 검증과 GitHub 미러 |
| Windows·WSL | 실제 드론 관제, 지도 복원과 자율 비행 실행 |

EC2 상태와 상태 API는 [상태 페이지](https://k15b205.p.ssafy.io/) 및 [헬스 체크](https://k15b205.p.ssafy.io/healthz)에서 확인합니다. 서버 설정과 스테이징 복구는 [EC2 운영 가이드](EC2_CICD.md), GitHub 미러 운영은 [GitHub 연동 문서](team/GITHUB_RELEASE.md)를 참고합니다.

## 데이터와 보안

실행이 생성한 회차 기록, 원본 영상, 로컬 모델 캐시는 데이터 정책에 따라 관리합니다. 팀 공유용 지도 설정·수치와 예제 자료는 해당 하위 폴더에 버전·출처를 기록합니다. 보안 제보 절차는 [보안 정책](../SECURITY.md)을 따릅니다.

[데이터 관리](../data/README.md) · [파트 간 계약](../contracts/README.md) · [제출 문서](../exec/README.md)
