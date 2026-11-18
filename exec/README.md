# 제출물 (exec)

실내 탐색 드론(자율 비행·사람 탐지·3D 지도·관제)을 다른 PC에서 켜고 시연하기 위한 문서다.
기준은 통합 브랜치 `feat/final-integration`(2026-11-17)이다.

| 문서 | 내용 |
| --- | --- |
| [포팅 매뉴얼](PORTING_MANUAL.md) | 구성, 필요한 것과 버전, 처음 한 번 준비(Windows·WSL·모델), 실행 명령, 포트, 방화벽, 끄기, 기록, 문제 해결, 라이선스 |
| [시연 시나리오](DEMO_SCENARIO.md) | 실기체 시연(교실 밖 출발 → 정찰 → 교실 → 출발점 착륙)의 무대·역할·순서·말할 거리·안 될 때, 실기체 없이 하는 시연 |
| [루트 README "통합 실행(최종)"](../README.md#통합-실행최종) | `scripts\start.cmd` 네 모드 한눈에 |

파트별 자세한 문서:

| 파트 | 문서 |
| --- | --- |
| 실행기 | [apps/launcher/README.md](../apps/launcher/README.md) |
| 관제 게이트웨이 | [modules/autonomy/tello_link/README.md](../modules/autonomy/tello_link/README.md) |
| 관제 화면 | [apps/controller/README.md](../apps/controller/README.md), [apps/controller/DESIGN.md](../apps/controller/DESIGN.md) |
| 자율 비행 | [modules/autonomy/tello_driver/README.md](../modules/autonomy/tello_driver/README.md), [simulation/tt-sim/README.md](../simulation/tt-sim/README.md) |
| 지도 콘솔 | [apps/console/README.md](../apps/console/README.md) |

아직 없는 것: 코드 정적 분석 보고서.
