# Jira·GitLab 팀 협업 규칙

담당 작업: [S15P31B205-14](https://ssafy.atlassian.net/browse/S15P31B205-14).
2026-10-22 EC2 구성과 팀 병렬 개발 요청을 반영했다. 과거 서버 부재를 이유로 한 CI 중지 방침은
EC2 검증·배포 CI로 대체한다. 자동 convention 검사·자동 리뷰 댓글·필수 동료 승인은 도입하지 않는다.

## 원본과 역할

| 항목 | 현재 원본 | 변경 원칙 |
| --- | --- | --- |
| 요구사항·담당·우선순위·의존 관계 | Jira | 기존 이슈를 재사용하고 담당자를 확인한다 |
| 코드·검증 결과·MR | GitLab | 작업 브랜치에서 변경하고 develop MR로 반영한다 |
| 공개 코드·외부 제안 | 향후 GitHub | 공개 전환 계획에 따라 별도 연결한다 |
| 공용 API·데이터 형식 | `contracts/` | 생산자·소비자 영향을 함께 기록한다 |

Jira와 GitLab에 같은 작업의 이슈를 각각 만들지 않는다. GitLab Issues가 필요한 별도 논의는
Jira 링크를 명시한다. 제목·설명·담당의 무제한 양방향 복제를 하지 않는다.

## 팀 작업 순서

1. [담당 작업 지도](../docs/team/JIRA_WORK_MAP.md)에서 자신의 실제 Jira 작업을 열고 완료 조건과 선행 관계를 확인한다.
2. 최신 develop에서 작업 브랜치를 만든다. 기존 작업은 같은 브랜치·MR·이슈를 이어 쓴다.
3. [폴더 경계](../docs/team/REPOSITORY_STRUCTURE.md) 안에서 작업하고 파트별 검증을 실행한다.
4. 실제 단일 Jira 키로 커밋하고 원격 브랜치에 push한다. 새 브랜치 이벤트가 해당 해야 할 일을 진행 중으로 바꾼다.
5. develop 대상 MR을 [양식](merge_request_templates/Default.md)으로 작성하고 Squash를 켠다.
6. 작성자가 이름·키·본문·검증 결과를 확인한다. 동료 리뷰는 필요에 따라 요청하며 필수 대기는 아니다.
7. Maintainer가 변경 내용·검증·충돌을 확인하고 머지한다. develop의 실제 merge 이벤트가 Jira 완료를 처리한다.

로컬 브랜치 생성, 커밋, push, MR 작성 또는 승인만으로 완료라고 하지 않는다.
기존 Jira 담당자는 자동화로 덮어쓰지 않는다. 자동 생성 작업은 작성자가 실제 담당자를 지정한다.
Jira 이슈 작성에는 [작업 양식](../docs/conventions/JIRA_ISSUE_TEMPLATE.md)을 사용한다.

## 이름과 병합

- 브랜치: `<유형>/<실제 Jira 키>-<소문자-kebab-case>`.
  유형: feature, fix, hotfix, infra, docs, refactor, test, chore.
- 커밋·MR 제목: `<type>(<선택 scope>): [<실제 Jira 키>] <구체적인 변경>`.
  type: feat, fix, docs, style, refactor, perf, test, build, ci, chore, infra.
- 일반 작업은 1 MR : 1 Jira 작업이다. 브랜치·새 커밋에 사용한 키도 MR과 같아야 한다.
  개별 커밋의 키는 생략할 수 있다.
- MR 본문·주석·예시에도 다른 Jira 키를 넣지 않는다. 자동화가 원문 전체에서 키를 추출한다.
  선행 작업은 Jira의 연결 관계에서 관리하고 MR에는 현재 작업 링크와 기술적 의존 조건을 적는다.
- develop/master에 직접 push하지 않는다. 이미 공유한 이력을 force push·reset으로 소급 수정하지 않는다.
- 릴리스 develop→master MR은 릴리스 작업을 별도로 사용하고 포함된 원래 커밋 키를 유지한다.
  현재 상태 자동화의 완료 대상은 develop이며 릴리스 완료 전환은 별도다.

## 적용한 설정과 범위

| 항목 | 운영 상태 |
| --- | --- |
| develop/master 직접 push | 금지, force push 비활성화 |
| develop/master merge | Maintainer가 검증 후 MR로 수행 |
| Squash | 일반 작업에서 사용 |
| 동료 승인 | 선택 |
| Jira Webhook | 새 브랜치 시작·MR 연결·실제 develop 머지 완료 |
| Custom issue tracker | Jira 프로젝트 및 번호 링크, 2026-10-22 활성화 |
| GitLab CI/CD | EC2 기반 검증·배포 설정 사용, 배포 기반 MR에서 관리 |
| Jira 개발 패널 | GitLab for Jira Cloud 사이트·그룹 연결은 별도이며 아직 연결하지 않음 |

Custom issue tracker는 `S15P31B205-번호`를 Jira로 연결한다. 다른 프로젝트 접두사도 숫자만 추출하므로
이 저장소에서는 타 프로젝트 키를 작업 키로 사용하지 않는다. 이 기능 자체가 상태를 동기화하지는 않는다.
상태 변경은 기존 [Jira 자동화](../automation/jira-mr/README.md)가 담당한다.
이미 승인된 자동 생성 이슈의 MR 닫힘 삭제 조건은 해당 가이드를 따른다. 기존 직접 작성 이슈는 보존한다.

EC2 CI의 코드 검증·배포 성공 여부를 실제 결과로 보고한다. 서버 설정과 저장소 파일 반영은 별개이며
MR 미병합·미실행 항목은 분리해 적는다. 배포 전 검증과 실패 복구는 `docs/EC2_CICD.md`를 따른다.
이 협업 변경은 배포 기반 MR의 `.gitlab-ci.yml`을 CI 중지 설정으로 덮어쓰지 않는다.

## 로컬 검증·보안·인계

- 변경에 맞는 검증과 `git diff --check`를 실행한다.
- 협업 문서·CI·템플릿 변경에는 `python -B -m unittest discover -s tests -p test_workflow_policy.py -v`를 실행한다.
  이 검사는 Jira 템플릿을 확인하며 convention·리뷰 승인 강제 장치가 아니다.
- 비밀 변수·API 토큰·웹후크 암호·개인 키·실제 촬영 자료를 저장소·MR·Jira에 복사하지 않는다.
  값 없는 `.env.example`과 공개 가능한 합성 예제를 사용한다.
- [시작 안내](../docs/team/START_HERE.md)의 인계 항목과 실제 검증 결과를 남긴다.
- [GitHub 공개 전환](../docs/team/GITHUB_RELEASE.md)은 별도 공개 대상·라이선스·이력 점검을 확인한 뒤 수행한다.

## 참고

- [GitLab 보호 브랜치](https://docs.gitlab.com/user/project/repository/branches/protected/)
- [Custom issue tracker](https://docs.gitlab.com/user/project/integrations/custom_issue_tracker/)
- [Jira 개발 패널](https://docs.gitlab.com/integration/jira/development_panel/)
- [GitLab push mirror](https://docs.gitlab.com/user/project/repository/mirror/push/)
