# 새 영상 하나로 희소 3D 복원 실행

새 로컬 영상 파일의 자동 처리 안내다. 영상 파일 하나를 입력하면 프레임 추출 → SIFT 특징점·매칭 →
김수호의 SfM 복원 → 저장 결과 재로딩/대조 → PNG 미리보기 → 결과 파일 검사를 실행한다. 실측이나 수동 좌표 지정은 필요하지 않다.
영상 업로드·실시간 드론 SLAM·밀집 표면 복원은 이 명령의 처리 범위에 포함하지 않는다.

## 설치와 실행

Python 3.12 환경에서 저장소 루트를 기준으로 실행한다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/data/requirements-pipeline.txt
.\.venv\Scripts\python.exe -B modules/mapping/data/run_video_pipeline.py --video "C:/videos/new_video.mp4"
```

기본값은 2fps 선택, 원본 표시 해상도 유지, JPEG 품질 95, CPU 4스레드, 다음 선택 프레임 4장까지 매칭,
SIFT max_features=4000이다. 특징점 한도의 실제 적용 방식은 PyCOLMAP을 따른다.
자동 생성한 UTC run_id와 결과 폴더를 콘솔에 출력한다. 출력 기본 위치는 저장소의 output/phone_mapping/<run_id>/다.
다른 디렉터리에서 실행할 때는 스크립트·Python 경로를 절대 경로로 지정한다. 입력/명시적 출력의 상대 경로는 호출 위치 기준이다.

선택 옵션 예시:

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/data/run_video_pipeline.py `
  --video "C:/videos/new_video.mp4" --run-id hallway_20261023_run01 `
  --output output/phone_mapping/hallway_20261023_run01 `
  --sample-fps 2 --max-width 960 --threads 4 --pair-window 4 --max-features 4000
$LASTEXITCODE
```

--max-width 0은 크기 유지, 960은 표시 방향의 폭이 960보다 클 때만 비율대로 축소한다.
축소는 좌표·특징점·복원 결과와 처리 시간에 영향을 준다. 같은 설정의 시간끼리 비교한다.
--output은 input/features/reconstruction/preview를 담는 **새 회차 루트**다. 이미 존재하면 변경 없이 거부한다.
실패한 회차도 유지하고 재실행에는 새 run_id와 새 출력 폴더를 사용한다. 자동 재시작/이어하기는 제공하지 않는다.
기존 snapshot/44cm 보정 명령은 [복원 모듈 안내](../../modules/mapping/localization/README.md)에 별도로 남아 있다.

## 결과를 확인하는 순서

```text
output/phone_mapping/<run_id>/
├─ status.json, manifest.json       # 전체 상태, 설정, 단계 결과, 요약과 산출물 참조/해시
├─ timing.json, execution.log       # 전체/단계 시간, 이벤트 기록
├─ local_source.json               # 원본의 로컬 절대 경로
├─ logs/
│  ├─ input.log                    # Python + FFmpeg 등 native 로그
│  ├─ features.log                 # SIFT/매칭·검사 로그
│  ├─ reconstruction.log           # SfM/내보내기 로그
│  ├─ preview.log                  # 저장 결과 재로딩·PNG 로그
│  └─ pipeline_error.log           # 실패/중단 시 traceback
├─ input/                          # manifest, frames/, 원본 PTS/변환·선택 결정 기록
├─ features/                       # database.db, 2D CSV, 설정·검증·매칭 미리보기
├─ reconstruction/                 # 선택 모델의 scene/PLY/poses/trajectory/quality/scale
│                                  # 및 raw/, 모델별 sparse/, 출처·시간·상태
└─ preview/                        # overview.png, projections.png, validation.json,
                                   # manifest/status/timing 및 로컬 폰트 캐시
```

1. 루트 status.json의 status를 확인한다. failed/interrupted이면 stage와 error, 같은 단계의 logs 파일을 연다.
2. manifest.json의 summary에서 입력/등록 프레임 수·3D 점 수·미등록 프레임·단위를 본다.
3. timing.json에서 전체 시간과 부모가 측정한 stages[*].elapsed_seconds를 읽는다.
4. reconstruction/quality.json과 scale.json으로 복원 범위와 단위를 확인한다.
5. points.ply는 3D 점군, poses.json/trajectory.csv는 같은 좌표계의 카메라 위치/원본 PTS다.
   [로컬 뷰어](../../apps/console/README.md)는 reconstruction/을 --result로 받는다.

새 영상은 scale.unit=arbitrary다. 점과 카메라는 같은 임의 좌표계에 있지만 m·cm 거리나 실제 높이로 읽을 수 없다.
SIFT/기하 검증 대응을 이용해 카메라와 점을 함께 추정한다. 별도의 YOLO 실행이나 이전 실험의 44cm/136cm 입력은 요구하지 않는다.
여러 native 모델이 생기면 가장 많은 카메라(동률이면 점)를 가진 모델을 선택하고 나머지는 따로 보존한다.
다른 모델의 좌표를 합산하지 않는다. 재투영 오차가 작아도 실제 공간 정확도가 검증된 것은 아니다.

## 상태와 종료 코드

| 루트 상태 | 종료 코드 | 뜻 |
| --- | --- | --- |
| complete | 0 | 네 작업 단계와 파일 검사가 끝나고 모든 제출 프레임이 선택 모델에 등록됨 |
| partial | 3 | 선택 모델은 생성됐으나 미등록 제출 프레임이 있음. 결과 확인 가능 |
| failed | 2 | 입력 열기/특징점/복원/파일 검사 등 실패. 다음 단계 실행 중지 |
| interrupted | 130 | Ctrl+C 또는 worker 중단. 진행하던 작업을 종료하고 기존 부분 파일 보존 |

부분 복원의 3은 전체 성공으로 처리하지 않도록 구분한 종료 코드다. 각 단계의 status=complete는 실행 완료이며,
루트 partial과 함께 존재할 수 있다. 원래 worker 상태는 manifest.stages[*].worker_status를 읽는다.
강제 종료된 worker의 상태 파일이 running으로 남았으면 루트 interrupted가 최종 실행 상태다.
OS 강제 종료/전원 차단은 보고서를 마무리할 수 없으므로 running 상태가 남을 수 있다.

run_id/옵션/위험한 출력 경로/기존 출력 폴더를 검사하다 거부된 경우에는 새 회차를 만들지 않는다.
그때는 CLI 오류와 종료 코드 2를 확인한다. 유효한 새 출력 경로에서 원본 파일이 없으면 preflight 실패 보고서를 남긴다.

## 시간 측정 범위

전체 시간은 run_pipeline() 진입의 입력/설정 검사부터 최종 산출물 검사와 manifest/status/이벤트 저장 완료까지다.
마지막 timing.json 자체 쓰기와 콘솔 출력은 제외한다. 부모 Python 시작/인자 파싱, 패키지 설치, 사용자의 뷰어 확인 시간도 제외한다. PNG 생성·저장 결과 대조는 포함한다.

- preflight_seconds: 설정·새 회차 준비·원본 파일 존재 검사·작업 명령 준비.
- stages[*].elapsed_seconds: 각 worker 시작/모듈 import·native 처리·로그 저장·종료 및 해당 상태/시간 보고서 읽기.
- finalization_seconds: 최종 scene의 회차/단위/등록 범위·필수 산출물 체크섬 대조와 요약 준비.
- total_seconds: 위 과정과 부모의 상태/이벤트 파일 저장 비용을 포함한 벽시계 시간.
- worker_timings: 각 worker가 자체 측정한 상세 시간. 특징점 추출, 매칭, CSV 내보내기/검사, SfM, PNG 등을 읽는다.
  worker 상세 시간은 부모의 단계 시간 안에 포함되어 있으므로 total에 다시 더하지 않는다.

features와 reconstruction은 각 입력 경계에서 DB/CSV를 검사하므로 두 검증 시간도 실제 전체 시간에 포함된다.
실패/중단 시 실행한 단계의 경과 시간과 total을 남기고 미실행 단계 시간은 null이다. 실행 중 긴 대기/시간 이상치가 관찰된 회차는 원인을 확인하기 전 정상 처리 속도 기준으로 쓰지 않는다.
처리 중 데이터나 설정을 바꾸지 않는다. 산출물은 Git에 올리지 않고 로컬 output/ 또는 회차별 Drive에 보관한다.

manifest의 산출물 경로는 회차 루트 기준 상대 경로다. 회차 폴더를 통째로 이동해도 참조 관계가 유지된다.
원본/뷰어 연결의 local_source.json과 reconstruction/local_assets.json은 로컬 절대 경로이므로 옮긴 후 현재 위치로 연결한다.

## 저장 결과 재사용과 검사

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s modules/mapping/data/tests -p test_video_pipeline.py -v
```

별도 fixture 프로세스로 상태·순서·시간·stdout/stderr·체크섬·중단 정리를 검사한다.
실제 디코더/특징 추출/복원 실패 검사도 포함한다. 실제 촬영 영상의 성공/부분 복원 확인은 로컬 실행 기록에 남긴다.
새 회차는 PNG까지 자동 생성하며 [미리보기 계약](../../contracts/mapping/PHONE_PREVIEW_V1.md)을 따른다.
이전 5단계 회차는 preview/가 없을 수 있으므로 아래의 단독 생성 명령을 사용한다.
브라우저 시각 검토는 자동 실행 단계가 아니므로 새 회차의 not_performed에 남는다. 제작자의 수동 검토는 별도 실행 기록으로 관리한다.
실제 실행 결과와 검사 범위는 [검증 요약](validation_summary.md)을 참고한다. 실제 공간 정확도·실측 보정·밀집 복원은 미검증이다.

### PNG를 다시 만들 때

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/data/render_preview.py `
  --result output/phone_mapping/hallway_20261023_run01/reconstruction `
  --output output/phone_mapping/hallway_20261023_run01_preview_again
.\.venv\Scripts\python.exe -B -m unittest discover -s modules/mapping/data/tests -p test_result_preview.py -v
```

새 출력 폴더만 지정한다. result_io.validate_saved_result(result_dir)는 (scene, report)를 반환한다.
원본 영상/사진 없이도 이 재로딩/PNG 검사는 가능하다. 원본 사진이 없으면 웹의 촬영 프레임 이미지는 표시할 수 없다.
검사에 실패하면 PNG를 성공으로 게시하지 않고 기존 reconstruction 파일은 유지한다.

- preview/overview.png: 전체 점과 원본 PTS 순서 카메라 경로의 입체 그림.
- preview/projections.png: 전체 점과 경로의 XY/XZ/YZ 투영. 실측 바닥/평면도를 뜻하지 않는다.
- preview/validation.json: JSON/PLY/CSV의 XYZ·RGB·개수·PTS·단위·raw/display 카메라 변환·해시 대조.

외곽의 불안정한 점도 삭제하지 않고 그린다. PNG의 축은 저장된 좌표계이며 실제 중력/바닥 방향이 아니다.
