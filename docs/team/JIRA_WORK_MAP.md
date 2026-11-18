# Jira 담당·작업 지도

2026-10-22 Jira의 실제 이슈와 GitLab 멤버 화면을 확인한 시작 안내다.
업무 배정이나 완료 상태를 이 문서로 변경하지 않는다. 이후 변경은 Jira를 우선한다.
메일·토큰·Jira accountId는 이 문서에 기록하지 않는다.

| 팀원 / 확인한 GitLab | 현재 확인한 작업 | 주 작업 폴더 |
| --- | --- | --- |
| 김수호 / `suho6347` | 휴대폰 3D 복원·좌표/스케일·위치 추정, EC2/협업 기반 | `modules/mapping/localization/`, `contracts/mapping/`, `infra/` |
| 박수경 / `wu2078` | 촬영/프레임·2D 특징점·매칭·실측 입력, 지도 저장/전달. API는 별도 백로그 | `modules/mapping/data/`, `contracts/mapping/`, `services/api/` |
| 허병우 / `byoungwooheo` | ROS 2·Gazebo·TT 시뮬레이션·경로 계획·비행 제어 | `simulation/`, `modules/autonomy/` |
| 송승우 / `dorem96` | 탐지 입력·실시간 추론·중복 억제·탐지 이벤트/위치·평가 | `modules/perception/inference/`, `contracts/perception/` |
| 이수진 / `tnwls134319` | 휴대폰 실시간 영상·재난 상황 사람 탐지·학습 데이터/모델 평가 | `modules/perception/inference/`, `modules/perception/training/` |
| 윤병현 / `ybh3044` | 기존 센서·지도/데이터 계획과 문서·발표 작업. 현재 지도 A/B 분담과 겹치는 범위는 Jira에서 확인 | `docs/architecture/`, `data/samples/` 및 해당 실제 작업 폴더 |

위 폴더는 코드 경계·문의 담당 안내이며 기존 Jira의 담당·스프린트·점수를 바꾸지 않는다.
화면 폴더 `apps/console/`의 담당은 아직 지정하지 않는다. 이수진의 과거 FE 분류만으로 현재 UI 업무를 배정하지 않는다.

## 지금 이어갈 실제 작업

- 김수호: [트랙 A](https://ssafy.atlassian.net/browse/S15P31B205-213),
  [입력·좌표·스케일 계약](https://ssafy.atlassian.net/browse/S15P31B205-215),
  [기준 환경](https://ssafy.atlassian.net/browse/S15P31B205-216),
  [실제 3D 복원](https://ssafy.atlassian.net/browse/S15P31B205-217),
  [품질·스케일 평가](https://ssafy.atlassian.net/browse/S15P31B205-235).
- 박수경: [트랙 B](https://ssafy.atlassian.net/browse/S15P31B205-214),
  [출력·미리보기](https://ssafy.atlassian.net/browse/S15P31B205-218),
  [촬영·프레임](https://ssafy.atlassian.net/browse/S15P31B205-219),
  [회차별 실행](https://ssafy.atlassian.net/browse/S15P31B205-220),
  [영상 품질](https://ssafy.atlassian.net/browse/S15P31B205-236),
  [2D 특징점·매칭](https://ssafy.atlassian.net/browse/S15P31B205-253),
  [실측 2D 관측·입력 전달 검증](https://ssafy.atlassian.net/browse/S15P31B205-254).
- 허병우: [시뮬레이션 기준 환경](https://ssafy.atlassian.net/browse/S15P31B205-201),
  [실기체 SDK 어댑터](https://ssafy.atlassian.net/browse/S15P31B205-207),
  [추정 위치 적용](https://ssafy.atlassian.net/browse/S15P31B205-208),
  [탐색·반복 평가](https://ssafy.atlassian.net/browse/S15P31B205-212).
- 송승우: [탐지 설정별 성능](https://ssafy.atlassian.net/browse/S15P31B205-169),
  [영상 입력 어댑터](https://ssafy.atlassian.net/browse/S15P31B205-170),
  [실시간 추론](https://ssafy.atlassian.net/browse/S15P31B205-171),
  [탐지 이벤트](https://ssafy.atlassian.net/browse/S15P31B205-172),
  [평가](https://ssafy.atlassian.net/browse/S15P31B205-176).
- 이수진: [영상 표시](https://ssafy.atlassian.net/browse/S15P31B205-249),
  [기준 탐지 성능](https://ssafy.atlassian.net/browse/S15P31B205-250),
  [데이터 수집·라벨링](https://ssafy.atlassian.net/browse/S15P31B205-251),
  [파인튜닝·평가](https://ssafy.atlassian.net/browse/S15P31B205-252).
- 윤병현: [기존 위치 추정·지도 에픽](https://ssafy.atlassian.net/browse/S15P31B205-135),
  [센서 관측 범위 조사](https://ssafy.atlassian.net/browse/S15P31B205-141),
  [데이터 원천·이용 조건](https://ssafy.atlassian.net/browse/S15P31B205-107).
  과거 계획과 현재 휴대폰 복원 분담이 공존하므로 구현 시작 전에 실제 우선 작업을 확인한다.

## 병렬 진행의 연결점

1. 지도: 박수경의 프레임·2D 관측 → 공용 계약 → 김수호의 3D 점군·카메라 pose·스케일 →
   박수경의 회차별 저장·미리보기. 양쪽은 작은 합성 입력/출력으로 먼저 개발한다.
2. 탐지: 영상/추론·이벤트 출력과 데이터/학습·평가는 하위 폴더와 실제 Jira 작업을 나눠 진행한다.
   송승우·이수진이 같은 파일·lockfile을 동시에 수정할 때는 변경 경계를 MR에 명시한다.
3. 자율비행: Gazebo 토픽·가상 입력과 실제 TT 어댑터 경계를 둔다. 정답 위치 기반 결과와 추정 위치 결과를 구분한다.
4. 서버/API·화면 연결은 계약과 샘플로 먼저 진행하며 이번 주 로컬 파일 PoC에 실제 DB·서버를 필수 의존으로 추가하지 않는다.

이 표의 다른 이슈 키를 MR 본문에 통째로 붙이지 않는다. MR에는 해결하는 단일 작업 키만 사용한다.
