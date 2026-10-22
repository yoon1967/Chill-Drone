# 팀 시작 안내

2026-10-22 기준. 작성 규칙의 원본은 [공통 협업 규칙](../../.gitlab/WORKFLOW_SETUP.md)이며,
현재 작업 배분은 [Jira 작업 지도](JIRA_WORK_MAP.md)와 실제 Jira 이슈를 확인한다.

## 자료와 실행 안내 찾기

- [저장소 폴더 구조와 작업 경계](REPOSITORY_STRUCTURE.md)
- [팀 공통 Drive 구조·실제 공유 위치·등록 규칙](DRIVE_STRUCTURE.md)
- [휴대폰 2D 입력·3D 복원 팀원 안내](../phone_mapping/team_guide.md)

공통 구조/운영 규칙은 `docs/team/`, 개별 실험의 실행/인계 설명은 해당 `docs/<분야>/`,
현재 구현의 설치/실행 방법은 모듈 README에 관리한다. 입력/출력 계약은 `contracts/`를 따른다.

## 1. 처음 한 번

1. 각자 GitLab·Jira 계정으로 로그인하고 프로젝트 접근을 확인한다. 팀원 계정은 이미 GitLab 멤버에 등록돼 있다.
2. Git을 설치하고 저장소를 복제한다. 팀원 토큰·SSH 개인 키·EC2 PEM을 복사받아 Git 인증에 쓰지 않는다.
3. Git 작성자 이름·이메일을 본인 값으로 설정한다. 향후 GitHub 기여 표시가 필요하면 GitHub에서 검증한 주소나
   본인 GitHub의 noreply 주소를 사용한다. 커밋에 이름·이메일이 남으므로 공개 전 이력 점검에 포함한다.
4. 저장소 루트에서 [AGENTS.md](../../AGENTS.md)를 읽고 개발 도구를 연다.
5. 자기 파트 README에 따라 환경을 구성한다. Python·Node·ROS 등을 모든 팀원에게 일괄 설치하지 않는다.
   파트 최초 구현자는 버전·설치·실행·테스트·필수 설정 이름을 해당 README에 추가한다.

```text
git clone https://lab.ssafy.com/s15-final/S15P31B205.git
cd S15P31B205
git switch develop
git pull --ff-only origin develop
```

아직 이 가이드가 develop에 없다면 협업 규칙 MR 반영 후 가져온다.
진행 중인 작업 공간에서 위 명령을 실행하기 전 `git status`로 기존 변경을 확인한다.

## 2. Jira에서 작업 선택

- 본인 담당인 기존 작업을 사용한다. 같은 이름의 새 Jira 이슈나 GitLab 이슈를 중복 생성하지 않는다.
- 목적·완료 기준·선행 작업·변경 폴더를 확인한다. 맡은 사람이 없거나 예전 배분과 다르면 Jira 담당자를 먼저 확인한다.
- 기능 전체를 여러 사람이 구현하면 각자 독립 완료 가능한 Jira 작업과 MR로 나눈다. 하나의 작업을 여러 MR로
  나누면 첫 develop 머지 때 완료 처리될 수 있으므로 일반 작업의 1 MR : 1 Jira 규칙을 지킨다.
- 없는 키나 Epic 키로 구현 브랜치를 임의 생성하지 않는다. 작은 실제 작업 키를 사용한다.

## 3. 작업 공간 만들기

한 디렉터리에서 다른 사람이 브랜치를 바꾸지 않도록 작업별 worktree를 권장한다.
아래는 PowerShell 예시다. 첫 두 값은 Jira에서 확인한 자신의 작업 키와 짧은 기능명으로 입력한다.

```powershell
$issueKey = Read-Host '실제 Jira 작업 키'
$taskSlug = Read-Host '소문자-kebab-case 기능명'
git fetch origin
git worktree add "../S15P31B205-worktrees/$issueKey-$taskSlug" -b "feature/$issueKey-$taskSlug" origin/develop
git worktree list
```

브랜치가 이미 있으면 `-b`를 다시 쓰지 않고 기존 브랜치를 연결한다.
같은 작업을 이어받으면 새 worktree 생성보다 기존 작업 공간·브랜치·MR 재사용을 먼저 확인한다.
다른 사람의 변경을 stash·reset·삭제하거나 공유 브랜치의 이력을 재작성하지 않는다.

## 4. 구현·검증·push·MR

1. [폴더 구조](REPOSITORY_STRUCTURE.md)에서 자신의 영역을 수정한다. 공용 계약 변경은 생산자·소비자 영향을 적는다.
2. 파트별 검증, `git diff --check`를 실행한다. 문서·CI·템플릿 변경에는 아래 로컬 검사도 실행한다.
3. `git add <이번 작업 파일>`로 필요한 파일만 올린다. `git diff --cached`에서 키·비밀정보·대용량 파일을 확인한다.
4. 공통 형식으로 커밋하고 `git push -u origin HEAD`한다.
5. GitLab에서 develop 대상 MR을 만들고 같은 Jira 키·변경 내용·실제 검증을 기록한다. Squash를 켠다.
6. 새 브랜치 push 후 Jira 진행 중과 브랜치 링크, MR 생성 후 MR 링크가 있는지 확인한다.
   자동화 지연·실패 시 [운영 가이드](../../automation/jira-mr/README.md)를 보고 중복 이슈를 만들지 않는다.
7. Maintainer가 검증과 충돌을 확인해 머지한다. 실제 develop 머지 후 Jira 완료를 확인한다.

```text
python -B -m unittest discover -s tests -p test_workflow_policy.py -v
git diff --check
```

기존 계획 작업의 완료 여부는 임의로 되돌리지 않는다. 새 구현의 미완료 단계는 이슈와 MR에 명확히 기록한다.
동료 리뷰·승인은 선택이며 자동 convention 검사·자동 리뷰 댓글을 추가하지 않는다.

## 5. 병렬 개발과 인계

- 공용 계약을 초안+작은 합성 예제로 먼저 공유한다. 상대 파트의 전체 구현을 기다리지 않고 예제·대체 구현으로 진행한다.
- 모의 입력·Gazebo 정답 위치·실제 영상·실물 센서의 검증 결과를 구분한다.
- 공용 lockfile, CI, 최상위 README는 여러 기능 MR에서 동시에 크게 바꾸지 않는다.
- MR 인계에는 작업 키/브랜치/MR, 수정 폴더, 실행 방법·입출력 버전, 실제 검증 결과,
  미완료·외부 의존·다음 담당자를 적는다. 비밀값은 복사하지 않고 저장 위치·설정 이름만 안내한다.
- 사용이 끝난 worktree는 변경과 미푸시 커밋이 없는지 확인한 뒤 `git worktree remove <정확한 경로>`로 정리한다.
  원격 브랜치 삭제는 merge 후 확인하며 데이터·실험 결과를 먼저 보존한다.
