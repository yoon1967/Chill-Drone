# EC2 배포·상태 기반

Docker·프로젝트 전용 GitLab Runner·HTTPS·실패 복구와 Docker 상태 수집을 관리한다.

- [운영·재구성·앱 연결·복구 가이드](../docs/EC2_CICD.md)
- [실제 서버·Docker 상태](https://k15b205.p.ssafy.io/)
- [배포 확인](https://k15b205.p.ssafy.io/healthz)

현재 `status/`는 실제 BE/FE 앱이 들어오기 전의 상태 서비스다.
Docker 소켓은 웹 컨테이너에 연결하지 않으며 선택한 상태 필드만 읽기 전용으로 전달한다.
최상위 README와 팀 협업·공개 전환 안내는 협업 구조 MR에서 관리한다.
