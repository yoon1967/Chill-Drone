# Chill-Drone 실행기 (`scripts/start.cmd`)

프로젝트를 켜고 끄는 실행 파일을 하나로 모았다. `scripts\start.cmd`를 더블클릭하면 메뉴가 뜨고,
명령줄에서는 `scripts\start.cmd <명령>`으로 바로 실행한다. 내용은 이 폴더의 Python 모듈(`python -m apps.launcher`)이다.
매핑 계측 도구는 [modules/mapping/benchmarks](../../modules/mapping/benchmarks/)에 있다.

| 메뉴 | 명령 | 하는 일 |
| --- | --- | --- |
| 1 체험 | `start.cmd fake` | 가짜 드론 게이트웨이(8030) + 사람 탐지 + 가짜 지도(8795)·지도 연결기 → 관제 화면 `#/live`. 드론 없이 화면을 볼 때 |
| 2 실기체 관제 | `start.cmd real` | 실기체 게이트웨이(8010) + 고른 보조 프로그램(사람 탐지·3D 지도·MoNA 자율) → 관제 화면 |
| 3 3D 지도 촬영 | `start.cmd map` | 지도 파트 통합 실행: 이 창 안에서 게이트웨이를 지도 통합 모드로 켜고 지도 콘솔(`#/map`)을 연다. `--fake`, `--direct`, `--gateway URL` |
| 4 저장 지도·영상 열기 | `start.cmd map <파일·폴더>` | 지도 콘솔에서 연다 |
| 5 자율비행 웹(ROS 2) | `start.cmd team-autonomy docker` (또는 `wsl`) | 허병우 팀원 웹 서버([안내](../../docs/autonomy/WEB_LAUNCHERS.md)) |
| 6 준비 | `start.cmd prepare` | 인터넷이 될 때 지도 콘솔 웹 의존성(pnpm), 지도·사람 탐지·게이트웨이 패키지, 모델 파일을 준비한다(처음 한 번) |
| 0 모두 끄기 | `start.cmd stop` | 실행기가 켠 프로그램을 끈다. **실기체가 공중이면 아무것도 끄지 않는다** |

## 실기체 관제(`real`)

1. 드론을 켜고 PC를 드론 Wi-Fi(`TELLO-…`/`RMTT-…`)에 연결한다. 연결 전이어도 게이트웨이는 켜지고, 드론이 붙으면 알아서 연결된다.
2. 8010 포트나 드론 UDP 포트(8889·8890·11111)를 다른 프로그램(점검 화면 `check_server.py`, 다른 게이트웨이, 직접 연결 지도)이
   쓰고 있으면 켜지 않고 그 프로그램을 보여 준다. Tello는 처음 `command`를 보낸 쪽에만 답하기 때문이다.
3. 게이트웨이를 새 창으로 띄우고 드론이 붙으면 배터리·온도를 보여 준다. 기록은 `data/outputs/tello/`.
4. 같이 켤 것을 묻는다: 사람 탐지(기본 예), 3D 지도(지도 콘솔 + 지도 연결기, 기본 아니오), MoNA 자율(기본 안 켬, `h` 손에 든 시험, `f` 비행).
   명령줄에서는 `--detect/--no-detect`, `--map`, `--mona hand|flight`로 고르면 묻지 않는다.
5. 관제 화면을 연다. MoNA를 켰으면 "자율 탐색", 3D 지도를 켰으면 "지도 만들기" 프리셋으로 연다.

`--phone`을 붙이면 게이트웨이를 같은 망에 열고(`--host 0.0.0.0`) 폰 주소를 알려 준다. 인증이 없으니 같은 망 시험에서만 쓴다.
방화벽 규칙은 보안 설정이라 실행기가 만들지 않고, 필요한 명령만 보여 준다(관리자 PowerShell에서 직접 추가·삭제).

## 모두 끄기(`stop`)

- 켜져 있는 게이트웨이(실기체·체험·3D 지도 촬영의 내장 게이트웨이)의 상태를 먼저 읽는다. 실기체가 이륙·비행·착륙 중이면
  아무것도 끄지 않고 "먼저 착륙하세요"로 끝난다. 드론이 연결돼 있으면(땅) 한 번 묻는다(`--yes`로 생략).
- MoNA 컨테이너 → 사람 탐지 → 지도 연결기 → 가짜 지도 → 지도 콘솔 → 자율비행 웹 → 3D 지도 촬영 실행기 → 게이트웨이 순으로
  각 창에 Ctrl+C를 보내 정상 종료시킨다(게이트웨이가 회차 `metrics.json`과 영상 조각을 마무리한다). 안 끝나면 강제로 끈다.
  3D 지도 촬영·자율비행 웹처럼 `start.cmd` 창 안에서 돌던 것은, 안의 프로그램이 끝난 뒤 "일괄 작업을 끝내시겠습니까"로 남은 창도 닫는다.

## Python 환경

`scripts\start.cmd`는 이 checkout의 `.venv`, 없으면 main worktree의 `.venv`, 그것도 없으면 환경 변수 `CHILL_DRONE_PYTHON`의
Python을 쓴다. 처음에는 `.venv`를 만들고 인터넷이 될 때 준비를 한 번 실행한다.

```powershell
py -3.12 -m venv .venv
scripts\start.cmd prepare
```

지도 콘솔 웹 의존성(`apps/console/node_modules`)이 없으면 준비가 `pnpm --dir apps/console install --frozen-lockfile`로 설치한다(Node.js·pnpm 필요).
실기체 게이트웨이는 이 Python이 방화벽에서 UDP 수신(8890 상태, 11111 영상)을 허용받아야 한다. 규칙이 없으면 `real`이 추가 명령을 알려 준다.

## 이전 실행 파일과의 대응

| 이전 | 지금 |
| --- | --- |
| `scripts/start_3d_mapping.cmd [옵션]` | `scripts/start.cmd map [옵션]` |
| `scripts/start_3d_mapping.cmd --prepare-models` | `scripts/start.cmd prepare` |
| `scripts/start_autonomy.cmd`(샘플 화면만) | `scripts/start.cmd fake`(가짜 드론으로 실제 동작) |
| `scripts/start_team_autonomy_docker.cmd`, `_wsl.cmd` | `scripts/start.cmd team-autonomy docker`, `wsl` |
