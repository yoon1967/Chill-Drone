# GitLab 브랜치·MR과 Jira 작업 상태 연동

2026-10-15 기준으로 Jira Automation과 GitLab MR Webhook을 연결하고 실제 이벤트를 검증했습니다.
별도 서버나 GitLab Runner 없이 Jira 이슈 생성과 MR 제목·본문 연결이 동작합니다.
Jira에 먼저 만든 이슈도 이슈 키를 포함한 GitLab 브랜치를 생성하면 진행 중으로 전환합니다.
MR 머지 시 완료 전환과 머지 없이 닫힘 시 제한된 범위의 영구 삭제도 적용했습니다.
2026-10-15 사용자가 정확한 삭제 범위와 복구 제한을 확인한 뒤 활성화를 승인했습니다.

2026-10-22 업데이트: 이 Jira Webhook은 유지하며 EC2 코드 검증·배포 CI를 함께 사용합니다.
과거 서버 부재로 CI를 중지한 기록은 아래 검증 이력일 뿐 현재 운영 방침이 아닙니다.
GitLab Custom issue tracker도 Jira 프로젝트/번호 링크용으로 활성화했습니다.
[팀 시작 안내](../../docs/team/START_HERE.md)와 [현재 작업 지도](../../docs/team/JIRA_WORK_MAP.md)를 함께 사용합니다.

## 현재 적용 상태

- [Jira 자동화 흐름](https://ssafy.atlassian.net/jira/software/c/projects/S15P31B205/settings/automate#/rule/01a13d45-ffd2-7cb5-92e3-bd5d36a066e5): 활성화.
- GitLab 프로젝트 ID: 1457098, 저장소: s15-final/S15P31B205.
- Jira 프로젝트: S15P31B205, 프로젝트 ID: 23878, 작업 유형 ID: 10002.
- 실행 계정: Automation for Jira. 범위: S15P31B205 단일 프로젝트.
- GitLab Webhook: 1개, 훅 ID 840028, SSL verification 활성화.
  Push events는 All branches, Merge request events도 활성화했습니다.
- GitLab 토큰: 사용자가 GET·PUT 두 요청의 숨김 PRIVATE-TOKEN에 직접 입력했습니다.
- Jira 웹후크 암호: 사용자가 GitLab의 X-Automation-Webhook-Token 헤더에 직접 입력했습니다.
- 인증정보는 저장소에 포함하지 않으며, 실제 값은 열람하지 않았습니다.
- 최종 Check for errors에서 모든 단계의 구성 유효성을 통과했습니다.
- 흐름 이름: GitLab Jira 브랜치·MR 작업 연동.
- 새 작업 브랜치: 해당 프로젝트의 단일 Jira 키를 찾아 해야 할 일 → 진행 중 전환 및 브랜치 링크 추가.
- merge 이벤트 + 최신 MR 상태 merged: 연결된 이슈를 완료로 전환합니다.
- close 이벤트 + 최신 MR 상태 closed: 보호 조건을 통과한 자동 생성 작업을 영구 삭제합니다.

## 담당 작업과 검증 결과

담당 작업은 [S15P31B205-14 협업 저장소·개발 규칙 정의](https://ssafy.atlassian.net/browse/S15P31B205-14)입니다.
[팀 공통 협업 규칙](../../.gitlab/WORKFLOW_SETUP.md)과 함께 관리합니다.
검증을 위해 만든 임시 이슈·MR의 링크 대신 운영에 필요한 결과만 남깁니다.

| 경로 | 실제 확인 결과 |
| --- | --- |
| 키 없는 MR 생성 | Jira 작업 생성, 제목·본문 연결, Squash 활성화 |
| 순차 이벤트 재전송 | 기존 작업 재사용, 본문 링크 복구 |
| 따옴표·역슬래시·줄바꿈 | MR 갱신 후 원문 보존 |
| Jira 선등록 후 새 브랜치 | 진행 중 전환, 브랜치 웹 링크 추가 |
| 브랜치명에만 기존 키가 있는 MR | 기존 작업 재사용 및 MR 연결 |
| 직접 만든 작업의 MR 닫힘 | 기존 이슈와 진행 중 상태 보존 |
| MR 승인 후 실제 머지 | 승인만으로 상태 유지, 실제 merge 이벤트로 완료 |
| 자동 생성 작업의 MR 닫힘 | 해당 MR 라벨·미완료·하위 작업 없음 조건에서 영구 삭제 |
| 유지하는 Jira 연동 템플릿 | 로컬 검증 대상, 결과는 MR에 기록 |
| GitLab CI·자동 convention 검사·자동 리뷰 댓글 | 10/21 중지 기록 이후 10/22 EC2 검증·배포 CI 사용. 형식 검사·자동 리뷰 댓글은 제외 |

## 팀원이 사용할 순서

1. 계획된 작업은 Jira에 이슈와 완료 조건을 먼저 작성합니다.
2. 최신 develop에서 feature/, fix/, infra/ 등의 작업 브랜치를 만듭니다.
   Jira에 먼저 만든 이슈가 있으면 feature/S15P31B205-번호-기능명처럼 키를 넣습니다.
   GitLab에서 브랜치를 생성하거나 처음 원격에 push하면 해당 이슈가 진행 중으로 바뀝니다.
   이슈가 없으면 feature/offline-translation처럼 키 없이 시작할 수 있습니다.
3. feat: 사람 탐지 입력 추가, infra: 협업 규칙 정리 같은 convention으로 커밋합니다.
4. develop을 대상으로 MR을 만듭니다.
5. 키가 없으면 자동화가 Jira 작업을 만들고 MR 제목·본문에 키를 붙입니다.
   기존 Jira 키가 하나 있으면 실제 이슈를 조회해 재사용합니다.
6. 자동 생성된 작업의 담당자·완료 조건·검증 내용을 보완하고 변경에 맞는 로컬 검증을 수행합니다.
7. Maintainer가 변경 내용과 실제 검증 결과를 확인해 머지합니다. EC2 CI 운영 가이드를 따르며 동료 승인은 선택입니다.
8. develop에 머지되면 연결된 Jira 이슈가 완료로 전환됩니다.
   현재 규칙은 develop 반영을 완료 기준으로 사용합니다.

MR 예시: infra: [S15P31B205-14] 협업 규칙 정리

자동화는 계획을 미리 작성했는지나 코드가 요구사항을 해결했는지까지 판단하지 않습니다.
브랜치 이름이나 이미 푸시한 커밋을 Jira 키를 넣기 위해 소급 변경하지 않습니다.
담당자 매핑은 현재 자동화에 포함하지 않았습니다.

## 저장된 자동화의 처리 방식

### Jira 선등록 → 브랜치 생성 → 진행 중

기존 MR 필터보다 앞에 독립적인 IF 분기를 두었습니다. 비교 값:

```text
{{webhookData.object_kind}}|{{webhookData.project_id}}|{{webhookData.project.id}}|{{webhookData.before}}|{{webhookData.ref}}
```

다음 정규식에 정확히 일치하는 새 브랜치 Push만 처리합니다.

```regex
^push\|1457098\|1457098\|0{40}\|refs/heads/.+$
```

branchMatchText 변수에는 다음처럼 원본을 두 번 넣습니다.

```text
{{webhookData.ref}} | {{webhookData.ref}}
```

branchJiraKeys 변수:

```text
{{branchMatchText.toUpperCase.match("(?:^|[^A-Z0-9_])([A-Z][A-Z0-9_]*-[1-9][0-9]*)(?=$|[^A-Z0-9_])").distinct.join(",")}}
```

Jira match()는 단일 일치와 복수 일치의 반환 형식이 다르므로, 원본을 두 번 넣어
키가 하나만 등장해도 컬렉션의 distinct/join으로 처리합니다. 경계 검사는 백슬래시가
필요 없는 문자 클래스를 사용합니다. MR의 mrText에도 같은 반복·경계 검사 방식을 적용했습니다.

키가 `^S15P31B205-[1-9][0-9]*$`와 정확히 일치해야 다음 JQL 분기에 진입합니다.

```jql
project = S15P31B205 AND key = {{branchJiraKeys}} AND status = 10000
```

마지막 실행 이후 변경 여부 제한은 해제합니다. 분기 안에서 진행 중(3)으로 전환하고,
이슈의 웹 링크에 해당 GitLab 브랜치 주소를 추가합니다. 원본 브랜치명과 키는 감사 로그에 기록합니다.
일반 커밋 push·브랜치 삭제·태그 이벤트는 시작 조건에 포함하지 않습니다.
키 없음·다중 키·다른 프로젝트 키·존재하지 않는 이슈는 새 이슈 생성 없이 건너뜁니다.
진행 중 또는 완료 상태인 이슈를 브랜치 생성만으로 다시 전환하지 않습니다.
로컬에서만 만든 브랜치는 GitLab 이벤트가 없으므로 처음 원격 push한 시점부터 인식됩니다.

이 규칙은 Jira 이슈의 웹 링크를 추가합니다. Jira 개발 패널의 GitLab 브랜치 생성 버튼 및
커밋·MR 개발 이력 표시는 아래 GitLab for Jira Cloud 사이트·그룹 연결과 별개입니다.

### 이벤트와 최신 MR 확인

수신 웹후크의 업무 항목 기준은 '웹후크에서 제공된 업무 항목 없음'입니다.
다음 값을 연결한 문자열을 정규식으로 검사합니다.

```text
{{webhookData.object_kind}}|{{webhookData.project.id}}|{{webhookData.object_attributes.target_project_id}}|{{webhookData.object_attributes.action}}|{{webhookData.object_attributes.target_branch}}|{{webhookData.object_attributes.iid}}
```

```regex
^merge_request\|1457098\|1457098\|(open|reopen|update|merge|close)\|develop\|[1-9][0-9]*$
```

이후 고정 프로젝트의 MR을 GET으로 다시 조회합니다.

```text
https://lab.ssafy.com/api/v4/projects/1457098/merge_requests/{{webhookData.object_attributes.iid}}
```

PRIVATE-TOKEN은 숨김으로 저장하고, 응답을 받을 때까지 다음 단계를 지연합니다.
HTTP 200, project_id/target_project_id=1457098, state가 opened/merged/closed 중 하나이고
target_branch=develop인 경우만 진행합니다.
이후 제목·본문·브랜치에는 이 최신 응답을 사용합니다.

### 기존 키와 중복 조회

mrText는 최신 브랜치·제목·본문을 연결한 문자열을 ` | `로 구분해 두 번 반복한 값입니다.
원본 MR의 제목·본문 자체를 중복 저장하는 것은 아닙니다. candidateKeys는 다음 식입니다.

```text
{{mrText.toUpperCase.match("(?:^|[^A-Z0-9_])([A-Z][A-Z0-9_]*-[1-9][0-9]*)(?=$|[^A-Z0-9_])").distinct.join(",")}}
```

빈 값 또는 S15P31B205 프로젝트의 단일 키만 계속 처리합니다.
다른 프로젝트 키나 여러 키가 함께 있으면 새 이슈를 만들지 않습니다.
단일 키가 있으면 실제 이슈를 조회하고, 없으면 MR 번호 라벨로 이전 생성 결과를 찾습니다.

```text
project = S15P31B205 AND {{#if(equals(candidateKeys,""))}}labels = "gitlab-1457098-mr-{{webResponse.body.iid}}"{{/}}{{#if(not(equals(candidateKeys,"")))}}key = {{candidateKeys}}{{/}}
```

블록 사이에 {{else}}를 쓰지 않습니다. 조건과 반대 조건을 각각 닫아야 합니다.
조회에 실패하거나 결과가 모호하면 이슈를 무조건 새로 만들지 않습니다.

### 새 작업 생성과 키 전달

아래 완료·닫힘 분기를 평가한 뒤 state=opened 조건을 통과해야 생성 경로에 진입합니다.
lookupIssues.size=0이고 candidateKeys가 비어 있을 때만 작업을 만듭니다.
요약은 최신 MR 제목, 설명은 MR URL·작성자·브랜치·본문과 완료 조건 보완 안내입니다.
라벨은 created-from-mr와 gitlab-1457098-mr-번호 두 개를 넣습니다.
담당자는 미지정 상태일 수 있으므로 팀원이 확인합니다.

새 작업을 만든 **같은 IF 블록 안에서** newJiraKey 변수를 만듭니다.

```text
{{createdIssue.key}}
```

공통 경로의 jiraKey 변수는 다음 값을 사용합니다.

```text
{{newJiraKey}}{{#if(equals(lookupIssues.size,1))}}{{lookupIssues.first.key}}{{/}}
```

IF 블록 바깥에서 createdIssue를 직접 참조하면 키가 전달되지 않은 것이 실제 검증에서
확인됐으므로, newJiraKey에 보관하는 단계를 생략하지 않습니다.
확정한 키가 ^S15P31B205-[1-9][0-9]*$와 정확히 일치해야 MR을 수정합니다.

### MR 갱신과 반복 호출 방지

cleanTitle은 Draft:/WIP: 접두사를 제거한 최신 제목입니다.
updatedMrTitle은 기존 convention 접두사 뒤에 키를 넣고 Draft 상태를 보존합니다.
제목에 ': '가 없으면 chore: 접두사를 사용합니다. type 등 작성 규칙은 작성자가 확인합니다.
본문에는 키가 없을 때 Jira 링크를 앞에 붙이고 기존 내용을 보존합니다.
이미 제목·본문이 원하는 값이고 squash=true이면 PUT을 생략합니다.

PUT 주소는 GET과 같은 고정 프로젝트의 MR API입니다.
요청 본문은 [gitlab-update-body.json.template](gitlab-update-body.json.template)을 사용합니다.
jsonEncode는 문자열 내용을 이스케이프하므로 JSON 값 바깥의 따옴표를 유지합니다.
GET·PUT 두 요청의 토큰 헤더는 모두 숨김으로 보관합니다.

## 머지와 닫힘 처리

기존 이슈 조회 직후 다음 두 IF 분기를 독립적으로 평가합니다.
이후 주 경로의 state=opened 조건으로 종료 이벤트가 생성·MR 갱신 경로에 진입하는 것을 막습니다.

### 머지 → 완료

다음 값이 `merge|merged|1`일 때만 실행합니다.

```text
{{webhookData.object_attributes.action}}|{{webResponse.body.state}}|{{lookupIssues.size}}
```

관련 업무 항목 분기의 JQL:

```jql
project = S15P31B205 AND key = {{lookupIssues.first.key}} AND status != 10001
```

JQL 분기의 '마지막 실행 이후 변경된 업무 항목만 포함'은 해제합니다.
분기 안에서 '전환 업무 항목'의 상태를 완료(10001)로 선택합니다.
Automation for Jira 계정에 업무 항목 보기·전환 권한이 필요합니다.
기존에 직접 만든 이슈도 단일 키로 연결되어 있으면 완료 처리 대상입니다.
여러 MR을 같은 이슈에 연결하면 첫 MR 머지로 완료될 수 있으므로, 1 MR : 1 작업 원칙을 지킵니다.

### 머지 없이 닫힘 → 자동 생성 작업 영구 삭제

다음 값이 `close|closed|1|`일 때만 검사합니다.

```text
{{webhookData.object_attributes.action}}|{{webResponse.body.state}}|{{lookupIssues.size}}|{{webResponse.body.merged_at}}
```

삭제 후보 분기의 JQL:

```jql
project = S15P31B205 AND key = {{lookupIssues.first.key}} AND issuetype = 10002 AND labels = "created-from-mr" AND labels = "gitlab-1457098-mr-{{webhookData.object_attributes.iid}}" AND statusCategory != Done
```

이 분기도 마지막 실행 이후 변경 여부 제한을 해제합니다.
분기 안에서 `project = S15P31B205 AND parent = {{issue.key}}`로 하위 작업을 조회하고,
결과가 0개일 때만 MR_CLOSE_DELETE_CANDIDATE 로그를 남깁니다.
그다음 활성화 스위치 용도의 **리터럴 true = true** 조건을 통과한 뒤 영구 삭제합니다.
급히 자동 삭제를 멈추려면 이 조건의 첫 번째 값을 false로 바꾸고 저장합니다.
이때 자동 등록과 완료 전환은 계속 동작합니다.

초기에는 false = true 조건으로 차단한 상태에서 후보 기록만 검증했습니다.
사용자의 영구 삭제 활성화 승인 후 true = true로 바꾸고,
로그의 enabled 표시 및 흐름 이름·설명을 함께 수정했습니다.
삭제 계정에는 Delete work items 권한이 필요합니다.

직접 만든 이슈, 다른 MR이 생성한 이슈, 완료된 이슈, 하위 작업이 있는 이슈는 삭제하지 않습니다.
동일 키를 다른 MR에서 참조하는지까지는 조회하지 않으므로 자동 생성 이슈를 여러 MR에서 공유하지 않습니다.
Jira 이슈 삭제는 댓글·첨부를 포함한 영구 삭제이며 일반 휴지통 복원을 지원하지 않습니다.
삭제 후 MR을 다시 열면 기존 제목의 사라진 키 때문에 자동 재생성이 차단됩니다.
이때 MR 제목·본문에서 삭제된 키를 제거하거나 새 Jira 이슈 키로 수정해야 합니다.
과거에 닫힌 MR을 일괄 처리하지 않으며, 재전송된 close 이벤트도 현재 상태를 다시 검증합니다.

### 검증 범위

머지 검증은 별도의 검증 대상 브랜치에서 실제 이벤트로 수행했습니다. 검증 후
이벤트 필터와 API 응답 필터 모두 develop 전용으로 복원했습니다.
직접 만든 이슈의 닫힘 보존과 자동 생성 이슈의 영구 삭제도 실제 이벤트로 확인했습니다.
이 검증은 Runner 없이 동작하는 Jira·GitLab Webhook 연동에 대한 결과입니다.

GitLab이 HTTP 200을 받았더라도 Jira 실행 기록이 없었던 구간이 있었습니다.
흐름 중지·재활성화 후 이벤트 처리가 재개됐으므로 HTTP 응답만으로 성공을 판단하지 말고
Jira 감사 로그와 실제 이슈 상태를 확인합니다.

## EC2 CI와 Jira 자동화의 구분

2026-10-22 사용자 제공 EC2에 검증·배포 CI를 구성했습니다. 배포 기반 MR의 `.gitlab-ci.yml`과
`docs/EC2_CICD.md`를 사용하며 이 협업 MR에서는 CI 중지 파일을 추가하지 않습니다.
자동 convention 검사·병합 충돌 보고서·자동 리뷰 댓글·필수 동료 승인 대기는 추가하지 않습니다.
Jira의 이슈 생성·기존 키 연결·브랜치 진행 중·develop 머지 완료는 이 Webhook이 계속 담당합니다.
유지하는 템플릿은 로컬에서 다음 명령으로 확인합니다.

```text
python -B -m unittest discover -s tests -p test_workflow_policy.py -v
```

## 선택적으로 추가할 연동

- GitLab for Jira Cloud: Jira 개발 패널에 브랜치·커밋·MR을 표시하려면 사이트 관리자와
  GitLab 그룹 연결 작업이 별도로 필요합니다. 프로젝트 자동화 권한과는 다릅니다.
- Jira issues integration: 추가로 사용할 때만 Jira 이메일과 API 토큰을 별도로 입력합니다.
  현재 자동 등록 흐름의 필수 조건은 아닙니다.
- 담당자 자동 지정: 확인된 GitLab 사용자 ID와 Jira accountId 매핑을 준비합니다.
- 완료 기준 변경: 현재 develop 머지를 완료 기준으로 적용했으며, 추가 검수가 필요하면 변경합니다.
- 동료 리뷰·승인: 선택입니다. 자동 convention 검사나 필수 리뷰를 병합 대기 단계로 운영하지 않습니다.

## 운영 시 주의할 범위

MR 번호 라벨 조회는 순차 재전송의 중복 생성을 막는 용도입니다. 동시에 실행된 두 규칙의
조회와 생성을 원자적으로 묶지는 못하며 Jira 검색 색인 반영도 지연될 수 있습니다.
정확히 한 번 생성이 필수라면 고유키 잠금과 영속 상태를 가진 중계 서비스가 필요합니다.
없는 이슈 키, 다중 키, 타 프로젝트, 동시 이벤트의 모든 조합을 실서비스 조건에서
시험한 것은 아닙니다. 일반적인 생성·재사용·본문 복구 경로와 순차 재전송을 실제 검증했습니다.

토큰 만료·회수 후에는 사용자 또는 관리자가 새 토큰을 발급해 GET·PUT 두 헤더를 교체합니다.
웹후크 암호를 다시 생성하면 GitLab의 X-Automation-Webhook-Token 값도 함께 교체합니다.
인증 URL이나 토큰을 문서·MR·채팅·일반 CI 변수에 기록하지 않습니다.

## 공식 문서

- [Jira Incoming webhook과 인증 헤더](https://support.atlassian.com/cloud-automation/docs/configure-the-incoming-webhook-trigger-in-atlassian-automation/)
- [Jira 조건 스마트 값](https://support.atlassian.com/cloud-automation/docs/jira-smart-values-conditional-logic/)
- [Jira 문자열 스마트 값과 JSON 인코딩](https://support.atlassian.com/cloud-automation/docs/jira-smart-values-text-fields/)
- [GitLab MR API](https://docs.gitlab.com/api/merge_requests/)
- [GitLab MR 웹후크 이벤트](https://docs.gitlab.com/user/project/integrations/webhook_events/)
- [Jira 자동화의 완료 전환·삭제 액션](https://support.atlassian.com/cloud-automation/docs/jira-automation-actions/)
- [Jira 삭제 이슈 복구의 제한](https://support.atlassian.com/jira/kb/restore-deleted-work-items-in-jira-cloud-using-local-backup-files/)
