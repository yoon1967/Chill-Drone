# 실시간 체크보드 보정 튜토리얼

작업: https://ssafy.atlassian.net/browse/S15P31B205-350

이미 구한 JSON을 등록하는 보정 프로필과 별도로, 사용할 카메라에서 실제 코너를 수집해 보정값을 계산한다.
사진을 따로 촬영·업로드할 필요 없이 사이트의 원본 프리뷰를 보며 체크보드를 움직인다.

## 사용 순서

1. 현재 지도·탐지 영상 수신을 종료한다. **카메라 보정 프로필 → 처음 사용하는 카메라 보정하기**를 펼친다.
2. 종이의 **내부 교차점** 가로·세로 수와 실측 한 칸 크기를 입력한다. 기본 9×6 교차점은 10×7개 칸이다.
3. 필요하면 인쇄용 SVG를 내려받아 실제 크기 100%로 인쇄한다. 기본 25mm 패턴은 270×205mm로 A4 가로에 들어간다.
   인쇄 프로그램의 용지 맞춤/자동 축소를 끄고 자로 한 칸을 확인한다. 다른 설정은 더 큰 용지가 필요할 수 있다.
   흰 여백을 남겨 평평한 판에 붙인다. 화면에 띄운 패턴 촬영보다 평평한 인쇄물을 사용한다.
4. 연결된 카메라의 장치·요청 해상도 또는 Tello를 고른다. 카메라·인쇄물 확인란을 체크하고 연결한다.
   브라우저 카메라 권한은 이때 요청한다. 실제 장치가 반환한 해상도를 화면과 결과에 기록한다.
5. 전체 무늬와 여백을 보이게 정면 → 좌/우/상/하 → 가까이/멀리 → 좌우·상하 기울임 순서로 움직인다.
   카메라 자체를 움직여도 되지만 초점·줌·방향·크롭 설정을 고정해야 한다. 각 자세에서 잠시 멈춘다.
   Tello는 비행하지 않고 기체를 고정한 채 종이를 움직인다. Tello Wi-Fi에 연결하고 다른 SDK/UDP 영상 수신을 종료한다.
6. 원본 프리뷰와 마지막 분석 장면의 코너 표시를 확인한다. 흐림·무늬 미검출·잘림·너무 작은 무늬·중복 자세는 장수에 넣지 않는다.
7. 유효 장면 최소 18장(최대 40장), 화면 4개 영역과 좌우/상하 분포, 크기 변화, 기울임 3장 조건을 채운다.
   안내가 부족한 조건으로 바뀐다. 필요하면 마지막 장면을 제외하고 다시 모은다.
8. **모은 장면으로 보정값 계산**을 누른다. 계산과 별도 검증을 통과하면 수치와 JSON 다운로드가 표시된다.
9. **이 결과를 새 프로필로 등록 준비**를 누르면 기존 등록 폼에 값을 채운다. 이름·실제 기체·방향/크롭을 확인해 저장한다.
   기본 선택은 별도로 저장해야 한다. 기존 실측 기본 프로필·기존 세션·지도는 변경하지 않는다.

지도 탭을 바꾸거나 중단·페이지 종료·장치 연결 끊김이 발생하면 촬영을 종료한다. 재시작은 새 수집이다.
원본 수집 영상·사진은 디스크에 저장하지 않으며, 코너와 최근 분석 JPEG만 한 세션의 메모리에 보관한다.
결과도 서버 메모리이므로 서버 종료/다음 수집 전에 JSON을 내려받거나 프로필로 등록한다.
프로필로 등록한 파일은 기존 `output/camera_profiles/`에 영속 저장한다.

## 계산과 검증

OpenCV의 체크보드 탐지와 원본 픽셀 subpixel 코너 정밀화를 사용한다.
내부 모델은 기존 매핑과 동일한 `fx, fy, cx, cy, k1, k2, p1, p2`이며 k3를 0으로 고정한다.
수집 순서상 매 다섯 번째 장면은 내부 파라미터 계산에서 제외한다. 나머지 장면으로 계산한 뒤
제외한 장면의 자세만 solvePnP로 구해 재투영 RMS를 확인한다. 검증용 장면으로 내부 파라미터를 다시 맞추지 않는다.
계산 RMS 및 검증 RMS 1px 이하·검증 장면별 RMS 1.5px 이하와 유한한 수치·초점거리 범위를 확인한다.
통과하지 않으면 결과를 저장하지 않고 수집으로 돌아가 추가 촬영/장면 제외/재시작을 안내한다.

결과 JSON은 기존 카메라 형식을 유지하고 `calibration_summary`에 계산/검증 장수·오차·보드·분포·모델을 기록한다.
칸 크기는 보정 대상의 실제 크기를 정의하며 **단안 SLAM 지도를 자동으로 미터 단위로 바꾸지 않는다**.
분포/기울기 검사는 수집 안내를 위한 영상 기반 휴리스틱이다. 낮은 재투영 오차도 실물 치수 정확도를 보장하지 않는다.
어안 전용 모델·강한 고차 왜곡·서로 다른 초점/줌/크롭을 섞은 자료는 이 튜토리얼의 지원 범위가 아니다.

참고: [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

## 실행·자원 관리

기존 `scripts/start_3d_mapping.cmd`를 현재 작업 브랜치에서 다시 실행한다. 별도 설치/서비스는 추가하지 않는다.
브라우저는 한 번에 한 프레임만 전송하고 약 0.8초 간격으로 새 분석을 시작한다. 프레임은 2MB, 해상도는 최대 2560×1920이다.
Tello는 기존 DirectCamera/GatewayCamera를 재사용하고 비행·SLAM 명령을 추가하지 않는다. Gateway 사용 시 기존 실행기의 `--gateway` 설정을 따른다.
서버는 지도/탐지/보정 입력을 같은 잠금으로 검사하고 동시 시작을 거부한다. 보정은 45초 heartbeat 만료 또는 최대 15분에 종료한다.
중단 요청은 세션 ID로 제한한다. 늦게 도착한 권한/HTTP 응답은 취소한 실행에만 정리하며 다음 수집에 적용하지 않는다.

API는 loopback과 기존 Origin/Host/X-Mapping-Client 검사를 유지한다.
`/api/calibration`: 기능 조회, `/board.svg`: 인쇄물, `/start`, `/frame`, `/state`, `/undo`, `/solve`, `/stop`, `/preview`, `/camera.json`.

## 두 MR 검토·병합 순서

선행 보정 프로필 MR !64는 최종 커밋 `7bd11690078b7981e3cc3f8e984cd12837f01f24`로 보존한다.
이 튜토리얼은 그 커밋에서 분기했다. !64가 develop의 14211add로 병합된 뒤 156b3d7에서 실제 develop을 동기화했고, MR !65의 비교 대상도 develop으로 바꿨다. 동기화 커밋의 파일 내용은 기존 검증 커밋 c9a52fa와 같다.
!64는 병합 완료, !65는 Draft·미병합 상태다. !65는 사용자 검토 후 별도 병합 지시가 필요하다.

아침 검토 시 먼저 !64를 develop에 병합한다. 선행 MR이 Squash되면 Git의 조상 관계가 달라져 단순 merge 시
app/capture/index/profile-view/serve/style 파일의 중복 추가·수정 충돌이 날 수 있다. 선행 최종 커밋을 공통 **파일 기준**으로
명시한 3-way 비교는 현재 모의 Squash에서 충돌 없이 현재 튜토리얼과 같은 최종 트리를 만드는 것을 검증했다.
공유 브랜치를 강제 rebase/push하거나 충돌 파일을 무조건 ours/theirs로 고르지 않는다.

!64의 실제 merged 상태와 develop 반영을 확인한 후, 깨끗한 튜토리얼 브랜치에서 아래 순서로 동기화한다.
다른 develop 변경도 공통 파일 기준으로 3-way 병합하며, 새 충돌이 보고되면 중단하고 해당 변경을 검토한다.
아래는 !64 병합 후 이미 실행·검증한 동기화 절차의 기록이다. 현재는 develop이 튜토리얼 브랜치의 실제 조상이므로 같은 절차를 반복할 필요가 없고, 이후 develop 변경은 일반 merge로 동기화한다.

```powershell
git fetch origin develop
# !64가 실제 develop에 병합됐는지 먼저 확인한다.
# 현재 브랜치가 튜토리얼이고 git status --porcelain 출력이 없는지 확인한다.
$calibrationBase = '7bd11690078b7981e3cc3f8e984cd12837f01f24'
$calibrationHead = git rev-parse HEAD
$calibrationDevelop = git rev-parse origin/develop
$calibrationMergedTree = git merge-tree --write-tree "--merge-base=$calibrationBase" $calibrationHead $calibrationDevelop
if ($LASTEXITCODE -ne 0) { throw '추가 변경 충돌: 동기화를 중단하고 검토하세요.' }
git diff $calibrationDevelop $calibrationMergedTree
# diff에 튜토리얼 변경만 남는지 검토한 후, 양쪽 부모를 보존한 merge commit을 만든다.
$calibrationMergeCommit = git commit-tree $calibrationMergedTree -p $calibrationHead -p $calibrationDevelop -m 'Merge develop after profile dependency; preserve tutorial changes'
if ($LASTEXITCODE -ne 0) { throw 'merge commit 작성 실패' }
git merge --ff-only $calibrationMergeCommit
```

회귀 검사 후 일반 push하고 MR 대상을 develop으로 바꾼 뒤, 튜토리얼 변경만 남는지 다시 확인해 별도로 병합한다.
선행 브랜치는 의존 MR 확인 전 수동으로 삭제하지 않는다. 이 절차는 앞선 두 MR의 사전 병합 승인이 아니다.

## 검증 범위

실제 합성 JPEG 탐지→수집→최적화, 알려진 카메라 값 복원과 검증 장면 오류 거부,
HTTP 격리/장치 점유/중단, 브라우저 권한·늦은 응답·프로필 준비 경로를 검사한다.
합성 검사는 실제 인쇄물과 카메라로 촬영한 보정 정확도 시험을 대체하지 않는다.
