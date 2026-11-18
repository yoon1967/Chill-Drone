# 저장 SfM 결과의 PNG 미리보기 1.0

생산자: modules/mapping/data/render_preview.py. 결과 검사기: result_io.validate_saved_result().
입력은 [SfM 결과](PHONE_SFM_RESULT_V1.md)의 complete/partial 폴더다. 기존 결과와 분리된 새 출력 폴더만 생성한다.

| 파일 | 내용 |
| --- | --- |
| manifest.json | schema_version=phone-preview/1.0, input_run_id, scene_sha256, unit, result_status, 도구/코드 해시, 파일 해시/크기 |
| overview.png | 저장된 전체 점의 RGB와 원본 PTS 순서 카메라 경로의 입체 시점. 녹색 시작점/빨간 끝점 |
| projections.png | 전체 점/경로의 XY, XZ, YZ 투영. 바닥/실측 평면도를 의미하지 않음 |
| validation.json | 게시 상태·생산 해시·JSON/PLY/CSV 좌표/색상·개수·PTS·단위·raw/display 카메라 변환 대조 결과 |
| status.json | running / preview_ready / failed. preview_ready도 3D 전체 등록을 뜻하지 않음 |
| timing.json | 저장 결과 재로딩/검사부터 PNG/보고서 저장까지. 마지막 timing 쓰기는 제외 |
| .matplotlib/ | 회차별 로컬 폰트 캐시. 데이터 계약이나 공유 필수 자료가 아님 |

PNG는 matplotlib 3.10.8의 Agg로 생성한다. 원본 영상/사진·브라우저·GPU가 없어도 만들 수 있다.
모든 점을 그리며 삭제/보간/밀집화를 하지 않는다. 먼 외곽 점도 그림 범위에 포함한다.
축은 저장 좌표계이고 실제 바닥/중력 방향이 아니며, arbitrary는 m·cm로 해석할 수 없다.
그림 제목의 partial과 등록 수/전체 입력 수는 원래 결과 범위를 유지한다.

검사기는 생산자의 체크섬만 신뢰하지 않고 PLY XYZ/RGB와 scene, trajectory CSV의 카메라 위치/PTS/단위와
scene, poses/quality/scale JSON의 일치를 독립 대조한다. 카메라 raw 자세와 표시 자세/스케일 수식도 검사한다.
이 대조는 저장/포맷 일관성 검사이며 실제 공간 정확도나 잘못 대응된 점의 정답 검증은 아니다.
native raw 모델의 재로딩은 별도 종단 검증에서 수행한다.

manifest의 scene_sha256으로 결과와 묶는다. 결과 폴더를 옮겨도 좌표/PNG는 읽을 수 있다.
웹의 원본 프레임 사진은 reconstruction/local_assets.json에 연결된 로컬 사진이 있어야 표시된다.
