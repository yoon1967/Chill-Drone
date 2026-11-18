# 폴더 구조와 병렬 작업

현재 Jira의 휴대폰 3D 복원·지도 데이터·객체 인식·자율비행 작업을 기준으로 준비한 구조다.
언어·프레임워크·ROS 패키지 이름은 실제 구현자가 확정한다. 아래 경계가 실행 가능한 앱을 뜻하지는 않는다.

```text
modules/
  mapping/
    localization/       # 좌표·SfM·SLAM·카메라 pose 복원
    data/               # 프레임·2D 매칭·점군·저장·재생·전달
  perception/
    inference/          # 영상 입력·탐지·이벤트 출력
    training/           # 라벨·학습·평가 코드, 가중치와 원본 데이터 제외
  autonomy/             # 경로 계획·제어·기체 어댑터
simulation/             # Gazebo 모델·월드·ROS 실행/브리지 구성
services/api/           # DB·저장·조회·업로드 API
apps/console/           # 운영·결과 확인 화면, 아직 스택 미정
contracts/
  mapping/              # 입력·좌표·스케일·파일 묶음
  perception/           # 영상·탐지·객체 위치 이벤트
  api/                  # HTTP·실시간 메시지·오류 계약
data/samples/           # 재배포 가능한 작은 합성 예제
infra/                  # EC2·Docker·배포·상태 서비스
tests/integration/      # 파트 간 연결·오류·회차 격리 검증
docs/architecture/      # 설계 결정, 선택 근거, 적용 한계
docs/team/              # 시작·분담·협업·공개 전환 가이드
```

## 소유와 공용 경계

- 공통 자료 저장 위치·Drive 분류/등록 규칙은 [DRIVE_STRUCTURE.md](DRIVE_STRUCTURE.md), 휴대폰 실험의 실행/인계는
  [팀원 안내서](../phone_mapping/team_guide.md)에 둔다. 개별 실험 설명은 분야별 docs에 관리하고 모듈 실행 방법/계약과 연결한다.

- `modules/` 안의 파트는 자체 의존성·실행·테스트를 해당 README와 lockfile로 관리한다.
  최상위에 여러 Python/Node/ROS 환경을 한꺼번에 섞지 않는다. `.venv`, build/install/log는 worktree별로 분리한다.
- `simulation/`은 가상 센서·월드·실행 구성을 소유한다. 위치 추정 알고리즘과 비행 제어 구현은 각 module에 둔다.
  실제 패키지가 다른 배치를 요구하면 설계 근거와 이동 범위를 MR에 기록한다.
- `contracts/`는 명세와 공개 가능한 예제만 보관한다. 생산자·소비자가 같은 버전의 형식으로 독립 개발하도록 한다.
  이번 구조 변경은 Jira에 있는 계약의 필드나 단위 자체를 확정하거나 구현하지 않는다.
- 파트 단위 테스트는 파트 내부, 파트 간 종단 검증은 `tests/integration/`에 둔다.
- `CODEOWNERS`는 GitLab에서 확인한 문의 담당 안내다. 필수 승인 규칙을 추가하는 용도가 아니며,
  GitHub 계정명이 같다고 가정하지 않는다.

## 충돌을 줄이는 방법

1. 같은 Jira 작업·브랜치·worktree를 여러 사람이 동시에 편집하지 않는다.
2. 공용 형식은 먼저 작은 MR로 다루고 생산자·소비자 양쪽 실행 예제를 남긴다.
   호환되지 않는 변경은 버전을 올리고 이전 입력 처리·전환 시점을 명시한다.
3. API·메시지·결과 파일은 내부 폴더를 직접 참조하지 않고 `contracts/`의 계약으로 연결한다.
4. 공용 CI·배포·최상위 lockfile 변경은 기능 변경과 구분해 영향과 복구 방법을 적는다.
5. 장기 작업 중 develop의 변경은 자신의 작업 브랜치에서 merge해 맞춘다. 팀원의 브랜치에 force push하지 않는다.

## 실제 데이터·생성물

보관 여부는 [개인 로컬 환경과 산출물 관리](../../.gitlab/WORKFLOW_SETUP.md#개인-로컬-환경과-산출물-관리)를 따른다.
아래 로컬·외부 저장 경로는 프로젝트에 직접 필요한 자료를 위한 것이며, 불필요한 백업·검증·테스트·리뷰
산출물을 보관하는 예외가 아니다. ignore 처리만 하고 남기지 말고, 이번 작업의 임시 자료는 사용 후 정리한다.

Git에 올리는 예제는 `data/samples/`에 출처·이용 조건·생성 방법을 적은 작은 재배포 가능 자료로 제한한다.
원본 촬영·얼굴·실내 위치 정보·대용량 프레임·가중치·지도·로그는 승인된 비공개 저장소나 각자의 로컬 경로에 둔다.
로컬 기본 경로 `data/raw/`, `data/processed/`, `data/outputs/`, `output/`, `artifacts/`는 ignore한다.
회차별 결과는 `run_id`로 분리하고 manifest·입력 해시·버전·실행 설정을 함께 관리한다.

이미 다른 원격 브랜치에 올라간 `output/phone_mapping/` 자료는 이동·삭제하지 않았다.
ignore는 과거 커밋이나 이미 추적 중인 파일을 제거하지 않는다. GitHub 공개 전에는 해당 브랜치를 포함한 이력을 점검한다.

2026-10-22 사용자 승인으로 팀의 다음 작업에 필요한 확정 수치 입력·설정·검증 기록은
`data/mapping/<입력 종류>/<회차>/<버전>/`에서 Git으로 공유한다. 실제 영상·사진·DB·ZIP·전체 로그는
계속 외부에 둔다. 활성 파일과 외부 자료는 회차 manifest/해시로 연결하며 세부 범위는 `data/mapping/README.md`를 따른다.
