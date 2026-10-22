# 도메인·네트워크·접근 정책

작업: [S15P31B205-12](https://ssafy.atlassian.net/browse/S15P31B205-12).
2026-10-22 현행 설정과 제한된 외부 연결 점검에 근거한 정책이다. 새 포트·계정·공개 권한은 추가하지 않았다.

## 주소·포트·허용 주체

| 경로 | 현재 공개 범위 | 허용 주체·용도 |
| --- | --- | --- |
| https://k15b205.p.ssafy.io/ | 외부 TCP 443 | 누구나 읽을 수 있는 상태 화면. 앱 사용자 인증 화면이 아님 |
| /api/status | 위 HTTPS와 동일 | 선택된 이름·이미지·상태·집계·자원 비율·최근 이벤트. 비밀값/명령/로그/내부 주소 제외 |
| /healthz | 위 HTTPS와 동일 | 현재 웹 프로세스 생존·배포 revision. 호스트 전체 정상 판정과 구분 |
| SSH 22 | UFW에서 IPv4/IPv6 원본 주소 제한 없이 허용 | 승인된 SSH 키 소유자의 관리 접근. 인터넷 출발지 제한이 적용됐다고 주장하지 않음 |
| 호스트 127.0.0.1:18065 | 서버 내부 | Caddy가 웹으로 연결 |
| 컨테이너 18066 | 루프백 publish를 통해 연결 | 상태 웹 실행. 외부 직접 연결은 허용하지 않음 |
| Docker·DB·Jenkins 관리 포트 | 외부 비허용 | Docker API 2375/2376·DB 기본 포트·8080 등을 새로 열지 않음 |

향후 실제 서비스 API·운영 UI·DB가 들어오면 공개 경로·인증·권한·내부 포트를 별도로 확정한다.
프로젝트 역할이나 CODEOWNERS가 서버 관리자 접근권한을 자동 부여하지 않는다.

## 방화벽 경계

사용자 제공 클라우드 운영 안내에는 넓은 포트 허용 범위가 있으나 콘솔에서 현재 보안 그룹을 직접 조회하지 못했다.
호스트는 UFW active, 기본 incoming/routed deny를 유지한다. SSH와 HTTPS가 현재 허용되어 있다.
Docker bridge publish는 UFW INPUT보다 먼저 처리될 수 있으므로 DOCKER-USER의 별도 차단을 함께 쓴다.

Caddy만 host network를 사용하고 UFW를 따른다. 일반 앱·DB는 루프백 publish 또는 내부 Docker 네트워크를 사용한다.
Docker 서비스 재시작 후 차단 규칙을 복원하는 설정은 운영 문서와 스크립트에서 관리한다.
host network나 새 인터페이스/방화벽 backend를 추가할 때에는 기존 bridge 규칙만으로 보호된다고 가정하지 않는다.

## TLS

- Caddy의 ACME TLS-ALPN-01로 공개 인증서를 발급·관리한다. 외부 443으로 challenge를 수행한다.
- HTTP 80 리다이렉트·HTTP challenge와 HTTP/3·Caddy 관리 API는 현재 구성에서 비활성화했다.
- TLS 개인 키는 Caddy named volume에 보관하고 애플리케이션·Git·MR·Jira에 전달하지 않는다.
- 조사 시 호스트명 검증이 성공했고 TLS 1.3으로 연결됐다. 인증서 notAfter는 2027-01-20 00:35:51 UTC였다.
  이는 조사 시점 값이며 자동 갱신 성공을 미리 보장하지 않는다. 갱신 오류·만료 전 점검과 안전한 재발급 절차가 필요하다.

## 관리·인증정보

| 항목 | 정책·현재 상태 |
| --- | --- |
| SSH 개인 키 | 제한된 소유자 로컬 저장, 팀 공용 Git 인증/CI 변수로 재사용하지 않음 |
| 팀원 서버 접근 | 필요 시 관리자가 각자 공개 키와 권한·회수 범위를 기록. 이번에는 새 접근권한을 부여하지 않음 |
| Runner 인증 | root 전용 파일. 배포 Runner는 Docker를 제어하므로 관리자 수준 권한으로 취급 |
| CI 권한 | 검증과 보호된 배포 Runner 분리. 배포는 신뢰된 protected 브랜치만 사용 |
| 웹 컨테이너 | 비루트·읽기 전용·capability 제거·자원 제한·Docker 소켓 미공유 |
| 향후 비밀값 | 보호된 CI 변수/권한 제한 서버 파일에 공급, 저장소에는 값 없는 설정 이름만 기록 |
| 노출·분실 대응 | 소유자가 해당 인증을 회수/교체하고 영향·이력 범위를 확인. 파일 삭제·권한 설정만으로 기존 노출을 회수할 수 없음 |

Runner 토큰 교체·회수의 실제 검증은 이번 정책 문서 작업에서 수행하지 않았다.
이전에 등록 과정의 출력 노출 가능성과 토큰 교체 미실행을 별도로 안내한 이력이 있으므로
운영 소유자의 교체 확인이 남아 있다. 이 문서에 토큰 값은 기록하지 않는다.
관리자 출발지 IP 제한·개인별 SSH 권한 세분화도 실제 팀 접근 목록을 받은 뒤 적용할 개선 사항이다.

## 검증과 한계

2026-10-22 팀 PC에서 이 호스트의 22·443 연결 성공,
80·18065·2375·8080·9000 연결 실패와 TLS 호스트명 검증 성공을 확인했다.
UFW active·Docker/Runner/timer active 및 상시 컨테이너 2개도 확인했다.
이는 조사한 경로·시점의 결과이며 모든 IPv6·서브넷·외부 인터페이스에 대한 침투 시험은 아니다.
기존의 Docker 재시작/명시적 외부 publish 차단 시험은 운영 문서에 별도 기록되어 있다.

- [설정·복구·기존 실제 검증](../EC2_CICD.md)
- [Docker와 UFW의 경계](https://docs.docker.com/engine/network/packet-filtering-firewalls/)
- [Caddy HTTPS·challenge](https://caddyserver.com/docs/automatic-https)
- [Runner 권한과 보호](https://docs.gitlab.com/ci/runners/configure_runners/)
