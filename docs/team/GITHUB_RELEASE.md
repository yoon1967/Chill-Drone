# GitHub 공개·연동 준비

현재 GitLab이 코드 개발 원본이고 Jira가 내부 작업 관리 원본이다.
GitHub 저장소·조직·공개 시점·라이선스는 미확정이며 자동 push나 공개 권한 변경을 활성화하지 않았다.

## 권장 단계

1. 팀이 소유할 GitHub 조직/저장소와 관리자를 정한다. 먼저 비공개 빈 저장소로 연결을 검증한다.
2. 공개할 브랜치/태그와 **그 이력이 포함하는 모든 파일**을 점검한다.
   현재 원격의 휴대폰 프레임 추출 브랜치에는 영상·사진·생성물이 이미 추적돼 있다.
   `.gitignore`만으로 이 자료가 이력에서 제거되지 않는다. 전체 브랜치를 무조건 mirror하지 않는다.
3. 코드·데이터·모델·자산의 출처와 재배포 조건을 확인하고 팀이 LICENSE를 선택한다.
   아직 LICENSE를 임의 추가하지 않았다. 모델·데이터의 권리는 코드 라이선스와 별도로 기록한다.
4. 공개 README·CONTRIBUTING·SECURITY의 지원 범위·연락 방법·개인정보·내부 링크를 검토한다.
   커밋 작성자 이메일과 과거 문서/설정도 점검한다. Jira나 EC2 접근권한을 공개 기여자에게 요구하지 않는다.
5. 승인된 브랜치만 GitLab→GitHub 한 방향으로 전송한다. 두 곳에서 동시에 직접 개발·merge하지 않는다.
6. 비공개 대상에서 커밋 SHA·기본 브랜치·태그·설정·LFS 자료를 검증한 뒤 공개 시점을 결정한다.

## 연결 방식

GitLab의 Settings → Repository → Mirroring repositories에서 Push mirror를 사용할 수 있다.
다만 설치 버전/요금제의 브랜치 필터·태그 동작을 확인한 후 공개 대상에 맞을 때만 활성화한다.
`Only mirror protected branches`는 공개 승인 목록과 같지 않다. 내부 배포/작업 브랜치가 protected일 수도 있다.
태그가 포함하는 이력도 따로 확인한다. 공개할 ref만 엄격히 보낼 수 없다면 전체 자동 mirror 대신 아래 방식을 쓴다.

최초 검증은 인증이 설정된 별도 clone에서 승인한 source commit/ref와 목적 branch를 명시한다.
아래 `<...>`는 실제 확정값으로 바꾸며 토큰을 URL에 넣지 않는다.

```text
git remote add github https://github.com/<organization>/<repository>.git
git push --dry-run github <approved-source-ref>:refs/heads/<public-branch>
```

출력과 공개 대상이 맞는지 확인한 후 같은 명시적 ref로 전송한다. `--mirror`, `--all`, 강제 push는 기본 절차에 넣지 않는다.
전체 이력을 공개할 수 없다면 원본 이력을 임의로 다시 쓰지 말고 공개용 별도 이력/패키지 제작 방식을 팀과 정한다.
선택적 전송도 source ref의 **모든 조상 커밋**을 보낸다. 최신 파일만 안전하다고 판단하지 않는다.

자동 mirror를 사용할 때에는 전용 최소 권한 GitHub 토큰/승인된 인증을 GitLab의 비밀 설정에 보관한다.
인증값은 로컬 설정 파일·remote URL·README·MR·Jira·스크립트 인자로 넣지 않는다.
GitLab 공식 가이드에 따라 대상 저장소 Contents 쓰기 권한을 확인한다.
`.github/workflows/`를 보낼 때 필요한 추가 권한도 확인하며 현재는 GitHub Actions를 추가하지 않았다.
분기된 ref를 덮어쓰지 않도록 Keep divergent refs를 사용하고 동기화 오류는 소유자가 해결한다.

## Jira와 외부 기여

| 시기 | 코드 반영 위치 | 이슈·상태 관리 |
| --- | --- | --- |
| 현재 | GitLab develop MR | 기존 Jira Webhook이 시작·MR·완료 관리 |
| GitHub 공개 mirror 운영 | GitLab, 공개 mirror는 직접 merge하지 않음 | GitHub 제안은 Maintainer가 내부 작업과 출처를 연결 |
| GitHub를 개발 원본으로 바꿀 때 | 확정한 GitHub 기본 브랜치 PR | Jira GitHub 연결·상태 규칙을 먼저 검증하고 기존 완료 트리거 전환 |

Jira 개발 패널에 GitLab/GitHub 활동을 표시하는 앱 연결은 별도 관리자 작업이다.
앱 연결만으로 같은 기능의 GitLab MR·GitHub PR 상태를 양쪽에서 완료 처리하지 않는다.
GitLab MR/댓글/보호 브랜치/멤버 권한은 Git 객체 mirror로 이전되지 않으므로 GitHub 쪽에서 별도 구성한다.
GitHub CODEOWNERS에는 확인한 GitHub 계정/팀만 사용한다. 현재 루트 CODEOWNERS는 GitLab 전용 사용자명이다.

## 공개 전 남은 입력

- GitHub 조직/저장소와 공개 담당자·시점.
- 코드 LICENSE, 제3자 모델·데이터·촬영 자료의 공개 허용 범위.
- 공개 브랜치/태그 목록과 그 전체 이력 검토 결과.
- GitHub 계정 매핑, 비공개 보안 제보 창구와 지원 버전.
- mirror 또는 원본 전환 중 어느 방식을 사용할지, Jira 관리자 앱 연결 여부.

## 공식 문서

- [GitLab push mirror와 GitHub 인증](https://docs.gitlab.com/user/project/repository/mirror/push/)
- [GitHub 저장소 복제·LFS](https://docs.github.com/en/repositories/creating-and-managing-repositories/duplicating-a-repository)
- [Jira 개발 패널](https://docs.gitlab.com/integration/jira/development_panel/)
