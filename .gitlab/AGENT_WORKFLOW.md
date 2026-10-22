# 에이전트 협업 규칙 적용

공통 진입점은 [AGENTS.md](../AGENTS.md), 상세 작성 규칙은
[WORKFLOW_SETUP.md](WORKFLOW_SETUP.md)이다. 사용자별 에이전트가 작업 중간에 들어와도
현재 브랜치·기존 변경·Jira·MR을 확인한 후 같은 규칙으로 작업을 이어간다.

## 도구별 연결

| 도구 | 저장소 파일 | 적용 방식 |
| --- | --- | --- |
| Codex | 루트 AGENTS.md | 세션 시작 시 저장소 지침 탐색 |
| Claude Code | 루트 CLAUDE.md | @AGENTS.md import로 공통 지침 로딩 |
| Cursor | .cursor/rules/00-project-workflow.mdc | alwaysApply: true로 매 세션 포함, 상세 문서는 직접 읽도록 지시 |
| GitHub Copilot | .github/copilot-instructions.md | 지원되는 환경의 저장소 공통 지침, AGENTS.md와 공통 문서 참조 |

일반 대화창에 코드 일부만 붙여 넣거나 저장소를 열지 않은 도구까지 자동으로 적용되지는 않는다.
도구·버전·신뢰 설정·사용자 설정에 따라 지침을 제외하거나 덮어쓸 수 있으므로 실제 로딩을 확인한다.
파일을 읽고 따랐다는 자기 보고는 보조 확인이며 증명이 아니다.

## 팀원 최초 적용 순서

1. 이 변경이 develop에 반영되면 팀원은 최신 develop을 가져온다.
2. 저장소 루트를 프로젝트로 열고 사용하는 에이전트에 프로젝트 접근을 허용한다.
   사용자별 전역 설정을 공유하거나 다른 사람의 토큰을 복사하지 않는다.
3. 이미 진행 중인 세션에는 AGENTS.md와 WORKFLOW_SETUP.md를 지금 읽고
   현재 브랜치·Jira·MR을 확인해 이어가도록 지시한다. 새 세션에서는 도구의 규칙 자동 로딩을 확인한다.
4. Jira와 GitLab을 직접 수정할 에이전트에는 각 사용자가 자신의 계정으로 필요한 연결을 설정한다.
   파일 추가만으로 Jira·GitLab 인증이나 권한이 다른 사용자에게 전달되지는 않는다.
5. Jira는 [작업 양식](../docs/conventions/JIRA_ISSUE_TEMPLATE.md), MR은
   [공통 양식](merge_request_templates/Default.md)으로 작성하고 변경에 맞는 로컬 검증을 수행한다.

## 운영 범위

| 장치 | 역할 | 운영 방식 |
| --- | --- | --- |
| 에이전트 지침 | 작성 규칙·기존 변경 보존·검증·인계 절차 안내 | 작성자와 에이전트가 참조 |
| 보호 브랜치 | develop/master 직접 push 금지 | 기존 서버 설정 유지 |
| Jira Automation | 이슈 생성·기존 키 연결·브랜치 시작·develop 머지 완료 | 서버 없이 기존 Webhook 연동 유지 |
| GitLab CI | 코드 검증·배포·실패 복구 | EC2 Runner 및 배포 기반 MR의 CI 설정 사용 |
| 동료 리뷰·승인 | 필요한 의견 공유 | 선택, 필수 대기 단계 없음 |

EC2 제공에 따라 검증·배포 CI를 사용한다. 자동 convention 검사, 병합 충돌 보고서,
자동 리뷰 댓글과 댓글용 토큰·정기 실행은 추가하지 않는다.
커밋·MR 작성 규칙은 문서로 유지하고 작성자가 확인한다. 별도 형식 검사나 리뷰 승인을 기다리지 않는다.

변경에 맞는 로컬 검증과 실제 결과 기록은 계속 수행한다. GitLab 자체의 병합 가능 여부 확인과
보호 브랜치는 유지하며 Maintainer가 MR을 머지한다. 로컬 Git hook이나 사용자 전역 설정은 설치·변경하지 않는다.

파트별 폴더·병렬 작업·인계는 [팀 시작 안내](../docs/team/START_HERE.md)를 따른다.
공개 전환 전까지 GitLab을 개발 원본으로 쓰며 GitHub 외부 기여 정책은
[CONTRIBUTING.md](../CONTRIBUTING.md)에 따른다. 계정·토큰·다른 사용자의 작업 공간은 공유하지 않는다.

## 공식 동작 근거

- [Codex AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md)
- [Claude Code memory와 import](https://code.claude.com/docs/en/memory)
- [Cursor Project Rules](https://cursor.com/docs/rules)
- [Copilot custom instructions 지원 범위](https://docs.github.com/en/copilot/reference/custom-instructions-support)
