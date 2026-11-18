# 공통 저장소 규칙

변경 전에 저장소 루트의 [AGENTS.md](../AGENTS.md)와
[협업 규칙](../.gitlab/WORKFLOW_SETUP.md)을 읽고 따른다. 중간에 맡은 작업에도 동일하게 적용한다.
Jira는 [작업 양식](../docs/conventions/JIRA_ISSUE_TEMPLATE.md),
MR은 [공통 양식](../.gitlab/merge_request_templates/Default.md)으로 작성한다.

일반 작업은 작업 브랜치 → develop MR → 변경에 맞는 로컬 검증 → Maintainer 머지 순서이다.
동료 리뷰·승인은 선택이며 자동 convention 검사·자동 리뷰 댓글을 대기 단계로 추가하지 않는다. EC2 검증·배포 CI는 운영 가이드를 따른다.
develop/master에 직접 push하지 않는다. 실제 단일 Jira 키로 브랜치·커밋·MR을 연결한다.
커밋·MR 제목은 type(scope): [실제 Jira 키] 변경 형식이며 개별 커밋의 키만 생략 가능하다.
기존 사용자 변경을 보존하고, 규칙 경로·실제 검증 결과·미완료 단계를 보고한다.
Jira 작업 단위와 일반 develop MR의 소스 브랜치 자동 삭제는 .gitlab/WORKFLOW_SETUP.md의
「Jira 작업 단위와 완료 이슈 보존」 및 「일반 develop MR의 소스 브랜치 자동 삭제」를 따른다.
머지·완료된 이슈의 관련 후속 작업도 새 이슈로 분리하고, 일반 MR의 Delete source branch는 ON을 유지한다.
수동 브랜치 삭제·MR 삭제/닫기 등은 같은 문서의 「브랜치·MR 삭제·닫기 전 필수 확인」을 따른다.
개인 로컬 환경과 임시 산출물은 같은 문서의 「개인 로컬 환경과 산출물 관리」를 따른다.
별도 요청 없는 개인 환경 백업과 프로젝트에 직접 필요하지 않은 보존·검증·테스트·리뷰 산출물은
저장소에 올리거나 로컬에 남기지 않으며, 이번 작업의 임시 파일은 사용 후 정리한다.
현재 개발 원본은 GitLab이다. 폴더·병렬 작업은 docs/team/START_HERE.md를 따른다.
GitHub 공개·외부 기여는 CONTRIBUTING.md와 docs/team/GITHUB_RELEASE.md를 따르며, 임의로 원본을 바꾸거나 중복 PR을 만들지 않는다.
