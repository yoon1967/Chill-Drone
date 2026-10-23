# GitHub 공개·연동 준비

현재 GitLab이 코드 개발 원본이고 Jira가 내부 작업 관리 원본이다.
2026-10-23 사용자가 공개 대상 `yoon1967/Chill-Drone`과 동기화 범위 `develop`·`master`를 지정했다.
GitLab 기본 Push mirror는 서버 관리자 전용으로 제한돼 있어 아래 CI 방식을 준비했다.
라이선스 선택과 GitHub의 기존 기본 브랜치·권한 변경은 별도다.

## GitLab CI 자동 동기화

- 작업: [S15P31B205-262](https://ssafy.atlassian.net/browse/S15P31B205-262).
- 대상: [Chill-Drone](https://github.com/yoon1967/Chill-Drone).
- `.gitlab/ci/github-mirror.yml`의 `github-mirror` 작업이 `scripts/github_mirror.py`를 실행한다.
- GitLab의 `develop`·`master` 두 브랜치만 현재 SHA 그대로 전송한다. 작업 브랜치·태그는 제외하며
  GitHub의 기존 `main`과 GitHub에만 있는 다른 브랜치를 삭제하거나 변경하지 않는다.
- 브랜치의 모든 조상 커밋도 함께 전송하므로 최초 연결 전의 변경·작성자·커밋 SHA도 유지된다.
  GitLab에는 `main` 브랜치가 없으며 `master`를 같은 이름으로 전송한다. MR 본문·리뷰 댓글·권한은 복제하지 않는다.
- 원본의 두 브랜치를 매번 새로 가져오므로 대기 중이던 예전 파이프라인도 최신 원본을 확인한다.
  두 브랜치는 atomic fast-forward로 함께 전송하고 전송 뒤 GitHub의 SHA를 대조한다.
  GitHub 전용 변경으로 이력이 갈라지면 강제 덮어쓰기 없이 작업이 실패한다.
- `resource_group: github-mirror`로 동시 실행을 막는다. 보호된 `develop`·`master`의 push/수동 실행과
  `develop`의 예약 실행만 허용하고 MR·작업 브랜치에서는 인증 작업을 실행하지 않는다.
- 일반 파이프라인에서는 verify 이후 동기화하며 기존 빌드·배포 흐름을 이어간다.
  미러링 실패는 작업 실패/파이프라인 경고로 표시하고 GitHub 장애가 기존 빌드·배포를 막지 않도록 한다.

### 인증과 최초 적용

1. GitHub 소유자가 대상 저장소만 선택한 전용 토큰을 준비한다. Contents 읽기/쓰기가 필요하며,
   `.github/workflows/`를 전송하게 되면 Workflows 쓰기도 필요하다. 승인된 기존 인증 사용 시에는
   그 인증이 가진 실제 접근 범위를 먼저 확인한다. 토큰을 채팅·문서·원격 URL·로그에 적지 않는다.
2. GitLab CI/CD 변수 `GITHUB_MIRROR_TOKEN`을 **Protected, Masked and hidden, Expand variable reference 꺼짐,
   Environment scope `github-mirror`**로 등록한다. 다른 환경 작업에 전달하지 않는다.
   인증 헤더는 실행 중 환경에만 두고 파일·Git config·명령 인자에 쓰지 않는다. CI_DEBUG_TRACE는 사용하지 않는다.
3. 이 변경을 검증한 뒤 develop 대상 MR을 Squash로 병합한다. 보호 브랜치를 해제하거나 develop에 직접 push하지 않는다.
4. develop에서 실행한 `github-mirror` 작업의 성공과 GitHub 두 브랜치 SHA 일치를 확인한다.
   GitHub의 초기 기본 브랜치는 `main`이므로 저장소 소유자가 원하면 기본 브랜치를 `develop`으로 바꾼다.
5. develop 대상으로 8시간 간격 예약 파이프라인을 등록하고 변수 `GITHUB_MIRROR_ONLY=true`를 지정한다.
   이 모드는 미러링 작업만 실행하며 기존 verify·빌드·배포·rollback 작업은 실행하지 않는다.
   기존 CI 설정을 쓰는 master의 단독 변경과 실패 재시도도 다음 예약 실행에서 반영된다.
   실행 시각은 Asia/Seoul 기준 매일 00시·08시·16시이며 Runner 대기에 따라 늦어질 수 있다.
   develop의 push 뒤 자동 동기화는 별도로 유지한다.

인증 변수만 등록하거나 MR을 작성한 상태는 동기화 완료가 아니다. 실제 develop 병합과 파이프라인의
최초 성공을 확인해야 활성화된 것으로 보고한다. 인증이 없으면 작업은 분명한 오류로 종료한다.
중지할 때는 예약 실행을 비활성화하고 인증 변수를 제거한다. 이때 GitHub 이력은 보존한다.

### 검증과 장애 처리

`python -B -m unittest discover -s tests -p test_github_mirror.py -v`는 네트워크·토큰 없이
격리된 Git 저장소에서 선택 브랜치/기존 main 보존, 후속 변경·반복 실행, 분기 이력의 atomic 거부,
원본 브랜치 누락 시 전송 중단을 검증한다. 문서/CI 변경에는 workflow 정책 검사와 diff 검사도 실행한다.

인증·저장소 규칙·분기 이력 문제는 GitLab 작업 실패로 나타난다. 토큰이나 원본 URL이 로그에 섞이지 않게
Git의 원시 오류 출력은 표시하지 않는다. 소유자가 인증 유효기간/권한과 GitHub branch rules,
두 원본·대상 브랜치의 SHA/이력을 확인한 뒤 재실행한다. 강제 push로 자동 복구하지 않는다.

최초 준비 시 원격 develop/master에서 접근 가능한 전체 이력의 촬영 영상·사진·DB·ZIP·개인 키
파일명과 실제 토큰/개인 키 패턴을 점검한다. 이는 이후 팀원이 추가할 모든 자료의 공개 허용을 대신하지 않는다.
아래 공개 절차와 라이선스·출처 확인은 계속 적용한다.

### 최초 로컬 검증 기록 · 2026-10-23

- 원격 develop `49ef00e0af977ba6e38d2ae2f20e541afed50af6`, master
  `67e150b17d0e414766da4aa958e1e34bfa884923`에서 접근 가능한 이력을 확인했다.
- develop 이력 27개 커밋의 범위에서 영상·사진·DB·ZIP·실제 env·개인 키 파일명 검색 결과가 없었다.
  두 브랜치의 고유 blob 155개를 대상으로 한 제한된 GitHub/GitLab 토큰·AWS 키·개인 키 패턴 검색과
  100MiB 초과 blob 검사에서도 발견 사항이 없었다. 전체 자산의 재배포 권리를 판정하는 검사는 아니다.
- 미러링 통합 시험 4개, workflow 템플릿 시험 2개를 포함한 전체 unittest 19개와 diff 검사를 통과했다.
- 두 CI 파일을 결합한 GitLab CI Lint 결과 valid=true, 오류/경고 0개였다.
- 이 기록 시점에는 MR 병합·인증 등록·최초 원격 동기화·예약 활성화를 아직 수행하지 않았다.

### 인증 등록·주기 변경 확인 · 2026-10-23

- 사용자가 전용 인증 등록을 알린 후 API에서 GITHUB_MIRROR_TOKEN의 Protected=true, Masked=true,
  Hidden=true, Raw=true, Environment scope=github-mirror, Type=env_var를 확인했다.
  숨김 값의 유효성은 최초 GitLab CI 실행으로 확인한다.
- 사용자 요청에 따라 기존 예약 작업을 5분에서 8시간 간격으로 변경했다.
  인증 등록 시점까지 비활성 상태를 유지하고 최초 원격 동기화 성공 후 활성화한다.

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

- GitHub 공개 담당자와 GitHub 기본 브랜치 변경 여부. 대상 저장소·전송 브랜치는 위에서 확정했다.
- 코드 LICENSE, 제3자 모델·데이터·촬영 자료의 공개 허용 범위.
- 승인된 두 브랜치의 이력 검토 결과와 이후 자료 추가 시의 공개 조건. 태그는 자동 전송하지 않는다.
- GitHub 계정 매핑, 비공개 보안 제보 창구와 지원 버전.
- mirror 또는 원본 전환 중 어느 방식을 사용할지, Jira 관리자 앱 연결 여부.

## 공식 문서

- [GitLab push mirror와 GitHub 인증](https://docs.gitlab.com/user/project/repository/mirror/push/)
- [GitHub 저장소 복제·LFS](https://docs.github.com/en/repositories/creating-and-managing-repositories/duplicating-a-repository)
- [Jira 개발 패널](https://docs.gitlab.com/integration/jira/development_panel/)
