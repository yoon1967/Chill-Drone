# 운영·결과 확인 화면

휴대폰 영상에서 계산한 실제 희소 점군과 카메라 경로를 확인하는 로컬 웹이다.
이번 복원 작업의 미리보기 구현이며 기존 EC2 상태 화면은 `infra/status/`에 유지한다.

## 설치와 실행

검증 환경: Python 3.12.14, Node.js 24.19.0, pnpm 11.25.0, Three.js 0.180.0, Chromium 기반 브라우저.
웹 서버는 Python 표준 라이브러리만 사용한다. Three.js는 설치 후 로컬에서 제공하므로
뷰어를 열 때 CDN·외부 API에 실제 자료를 보내지 않는다.

저장소 루트에서 [복원 실행](../../modules/mapping/localization/README.md)을 먼저 마친다.

```powershell
pnpm --dir apps/console install --frozen-lockfile
.\.venv\Scripts\python.exe apps/console/serve.py `
  --result output/phone_mapping/2f1_sfm_review --port 8790
```

브라우저에서 `http://127.0.0.1:8790/`을 연다. 서버를 종료하려면 실행 터미널에서 Ctrl+C를 누른다.
`--result`는 완료 상태의 결과 폴더, `--port` 기본값은 8790이다. 원본 프레임은 복원 당시 위치를 유지한다.
자료를 옮겼다면 새 위치에서 복원하거나 로컬 전용 `local_assets.json`의 `images_root`를 갱신한다.

## 화면 사용

- 드래그로 회전, 휠로 확대, 오른쪽 드래그로 이동한다. 입체·정면·위 시점을 선택할 수 있다.
- 최초 시점은 중심 구조에 맞춘다. 모든 점은 계속 렌더링되며 `전체 보기`는 외곽 점까지 포함해 맞춘다.
- 촬영 시점 슬라이더·재생은 원본 PTS와 카메라 위치를 연결한다. `이 위치에서 보기`로 해당 카메라 방향을 확인한다.
- 점 크기·색상·오차 필터, 촬영 경로·카메라 방향·격자를 조절한다. 필터는 표시만 바꾸고 원본 출력을 바꾸지 않는다.
- PLY, 카메라 자세 JSON, 경로 CSV를 내려받아 다른 도구에서 읽을 수 있다.

`reference_scaled`는 44cm 기준으로 맞춘 미터 단위, `arbitrary`는 미보정 임의 단위다.
미터 단위 격자의 한 칸은 0.5m다. 격자 평면은 표시용이며 실측 바닥·중력 방향이 아니다.
표면 메시·독립 치수 정확도·실시간 SLAM을 제공하는 화면으로 표시하지 않는다.

## 접근 범위와 검증

서버는 127.0.0.1에만 바인딩하고 정해진 정적 자원, 완료된 결과, 등록된 원본 프레임 이름만 제공한다.
저장소·실제 자료 디렉터리 열람, `.env`, 원본 DB, 로컬 절대 경로 JSON은 제공하지 않는다.
실제 촬영과 복원 결과는 Git·Sites·EC2에 업로드하지 않는다.

```powershell
python -B -m unittest discover -s apps/console/tests -v
node --check apps/console/app.js
```

입출력은 [복원 결과 계약](../../contracts/mapping/PHONE_SFM_RESULT_V1.md), 실제 수치는
[실행 기록](../../docs/phone_mapping/reconstruction_report_20261022.md)을 참고한다.
팀 작업 절차는 저장소 루트의 `docs/team/START_HERE.md`를 따른다.
