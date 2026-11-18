# 포팅 매뉴얼과 시연 시나리오

Chill-Drone을 새 PC에서 설치·실행하고 발표 화면을 재현하기 위한 제출 문서입니다.

| 제출 항목 | 문서 | 내용 |
| --- | --- | --- |
| 1. 빌드·배포 | [포팅 매뉴얼](BUILD_AND_DEPLOY.md) | 런타임, 환경 변수, 설치·실행, Windows·WSL·EC2 구성 |
| 2. 외부 서비스 | [외부 서비스와 모델 준비](EXTERNAL_SERVICES.md) | 서비스 접근, 비행·영상 복원 모델 준비 |
| 3. DB 덤프 | [데이터와 기록 복원](DATABASE.md) | 운영 DB 덤프 대상 확인, 회차 재생, 오프라인 COLMAP SQLite 인계 |
| 4. 시연 시나리오 | [발표용 시연 시나리오](PRESENTATION_DEMO.md) | 자율 탐색 → 요구조자 탐지 → 실시간 3D 지도 |

[루트 README](../README.md)에서 처음 시작하는 방법을, [실행 가이드](../docs/RUN_GUIDE.md)에서 모드별 준비와 실행을 확인할 수 있습니다. 실기체 운용에 필요한 리허설 동선과 역할은 [시연 운용 참고](DEMO_SCENARIO.md), 세부 포트와 문제 진단은 [Windows 통합 실행 참고](PORTING_MANUAL.md)에 기록돼 있습니다.
