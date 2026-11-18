# 인프라 기술 비교·선정

작업: [S15P31B205-10](https://ssafy.atlassian.net/browse/S15P31B205-10).
판단 기준: 2026-10-22 제공 EC2 1대, 4 vCPU·약 16 GiB, 기존 GitLab/Jira, 단일 staging,
짧은 개발 기간, 공개 가능한 코드와 비공개 실험 데이터의 분리. 아래 선택 근거는 현재 제약에 대한 설계 판단이다.

## 선택표

| 영역 | 현재 채택 | 비교한 대안 | 현재 선택 근거·한계 |
| --- | --- | --- | --- |
| 실행 서버 | 제공 Ubuntu 24.04 EC2 | 추가 VM·관리형 플랫폼·GPU 서버 | 새 계정/비용을 늘리지 않고 이미 배포 검증한 환경 사용. 단일 장애점·CPU 크레딧·사용 종료일 제약 |
| 서비스 구성 | Docker Engine + Compose | 호스트 직접 실행·Kubernetes | 현재 두 서비스의 이미지/네트워크/볼륨을 파일로 재구성 가능. 다중 노드·자동 확장은 요구 미확정 |
| 파이프라인 | GitLab CI + 프로젝트 Runner | Jenkins·GitHub Actions | 현재 코드 원본/MR/실행 결과를 한곳에서 관리하고 이미 자동 배포 검증. Runner 호스트 관리는 필요 |
| HTTPS | Caddy, ACME TLS-ALPN-01 | Nginx + 인증서 갱신 구성 | 외부 443으로 인증서 운영·reverse proxy 구성. 단일 프록시 장애·갱신 실패 점검 필요 |
| 이미지 전달 | 같은 EC2의 커밋 SHA 로컬 이미지 | GitLab/GH Registry·별도 registry | 현재 1대에서 토큰/전송 단계를 줄임. 다중 서버·외부 복구 시 registry/이미지 보관 재설계 |
| 배포·실패 복구 | Compose 교체 + 이전 release 복구 | 수동 복사·blue/green | 코드화된 건강/HTTPS 검사와 실패 복구를 실제 확인. 짧은 중단 가능, DB rollback은 포함하지 않음 |
| 상태 확인 | 제한된 호스트 snapshot + 읽기 전용 웹 | Portainer·Prometheus/Grafana | 현재 확인할 필드에 맞추고 웹에 Docker 관리자 소켓을 주지 않음. 장기 시계열·알림·감사 로그는 별도 |
| 현재 실험 결과 | 로컬/비공개 회차별 파일 | EC2 파일·객체 저장소 | 휴대폰 PoC와 원본 촬영 분리 우선. 공유·보존·복원 요구가 나오면 비공개 저장 방식 확정 |

Docker Compose는 서비스·네트워크·볼륨을 선언하는 도구이고, Kubernetes는 컨테이너 워크로드를
클러스터에서 관리하는 플랫폼이다. 현재 요구가 작고 제공 서버가 한 대라는 이유로 Compose를 선택했다.
Kubernetes가 동작 불가능하다는 판단이나 성능 비교 실험을 했다는 뜻은 아니다.

Jenkins와 GitHub Actions도 파이프라인을 구성할 수 있다. 이번 선택은 기능 부재 때문이 아니라
이미 운영 중인 GitLab 경로와 계정·권한·실행기를 중복 관리하지 않기 위한 것이다.
교육/평가에서 특정 도구가 필수로 지정되거나 GitHub를 원본으로 옮길 때 다시 판단한다.

## 보류한 결정

| 미정 항목 | 보류 이유 | 결정에 필요한 입력 |
| --- | --- | --- |
| BE 언어·프레임워크·DB 제품/버전 | 현재 폴더만 존재하고 최신 확정 명세를 확보하지 못함 | 실제 API·ERD·조회/업로드 요구·팀 환경 |
| FE 스택·3D 뷰어 | UI 구현과 API/출력 계약이 미정 | 표시할 지도/사진·지원 기기·데이터 크기 |
| 모델·학습 자원·가중치 배포 | YOLO 작업 계획은 있으나 확정 버전·라이선스·성능 결과 미확보 | 모델/데이터 출처·지표·메모리·실행 시간 |
| S3/외부 DB/Registry/추가 VM | 계정·예산·보존·복원·다중 서버 요구 미확정 | 담당자 승인·전체 비용과 운영 기간 |
| 운영 HA·무중단·중앙 관측 | 단일 staging 검증 단계 | 서비스 수준 목표·트래픽·장애/알림 요구 |
| GitHub 공개·원본 전환 | 조직/저장소·라이선스·과거 데이터 이력 검토 미정 | 공개 승인 ref·인증 범위·외부 기여 경로 |

미확정 항목을 기본값으로 설치하거나 유료 서비스를 만들지 않았다.
구현된 도구 버전과 설치·복구 절차는 [운영 문서](../EC2_CICD.md)를 사용한다.
팀별 계약·현재 분담은 [작업 지도](../team/JIRA_WORK_MAP.md)를 근거로 삼았다.

## 검증과 재평가 조건

현재 GitLab 자동 검증·빌드·배포와 HTTPS·실패 복구의 실제 기록을 확인했다.
후보 전부를 같은 부하로 벤치마크하지 않았고 라이선스·요금의 법적/재무적 판단을 대신하지 않는다.
CPU/메모리 경합, 데이터 용량, 서버 종료, 다중 서버 요구가 확인되면 측정 결과로 선택표를 갱신한다.

- [Docker Compose 모델](https://docs.docker.com/compose/intro/compose-application-model/)
- [Kubernetes 개요](https://kubernetes.io/docs/concepts/overview/)
- [GitLab Runner](https://docs.gitlab.com/ci/runners/)
- [Jenkins Pipeline](https://www.jenkins.io/doc/book/pipeline/)
- [GitHub Actions](https://docs.github.com/en/actions/get-started/understand-github-actions)
- [Caddy 자동 HTTPS](https://caddyserver.com/docs/automatic-https)
