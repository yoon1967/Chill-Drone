# 지도 입력·데이터·전달

## 인명 구조 의미 지도

`scripts/start_3d_mapping.cmd`로 실행한 화면의 **인명 구조 지도**에서 사람·공간 구분 복원을
기본 사용한다. 사람은 빨간 복셀과 3D 외곽선이며 색상·외곽선·구조물 구분 색을 변경할 수 있다.
일반 실행은 깊이·YOLO·SegFormer 가중치를 로컬 캐시의 실제 파일 경로로만 읽는다.
캐시가 없으면 다운로드 재시도를 기다리지 않고 준비 명령을 안내하며 종료한다.
최초 설치·파일 누락 때만 인터넷 연결 상태에서 `scripts/start_3d_mapping.cmd --prepare-models`를 실행한다.
패키지와 학습 가중치는 별개이며 가중치를 프로젝트나 워크트리에 복사하지 않는다.
기존 서버를 종료하고 실행기를 다시 열어 새 코드를 적용한다.

**지도에 사용할 초당 프레임**에서 0.1~15 사이 목표량을 설정한다(기본 5).
1은 약 1초마다, 2.5는 약 0.4초마다 한 장이다. 저장 영상의 **영상 1초당 복원할 프레임**과 같은 설정을 사용한다.
설정은 브라우저에 저장되고 다음 지도부터 적용된다. 카메라 촬영·녹화·화면 FPS와 별개이며 모델 처리 속도에 따라 실제 반영량은 낮을 수 있다.
위치 추적 입력은 별도 최대 15Hz를 유지한다. 기존 SfM 저장 영상 경로는 별도 최대 30프레임/초를 계속 지원한다.

의존성은 CUDA torch와 일치하는 torchvision 설치 후 `requirements-stream.txt`로 준비한다.
깊이·인식 모델은 GPU, 점군 통합은 NumPy CPU 연산으로 실행한다. 메시를 생성하지 않는다.
공간 구분을 꺼도 같은 복셀 통합을 사용한다. 기본 정적 점 32,000개를 넘으면 전체 지도 점 간격에
고정 증가 폭을 더해 합친다. 증가 폭은 지도 최초 깊이 배율로 `.005`를 좌표 변환한 값이며 촬영 중에는 고정된다.
배율 1의 예는 `.04 → .045 → .05`다. 현재 간격에 비례해서 증가하지 않는다.
오래된 구간을 지우지 않으며 지도 범위가 커지면 세부 묘사가 줄 수 있다.
화면에서 4,000 / 8,000 / 10,000 / 16,000 / 32,000점 예산을 선택한다. 최종 지도도 같은 예산으로 다시 통합한다.
점 예산의 측정 조건·근거·재현 명령은 [점 예산 조정 기록](../../../docs/phone_mapping/point-budget-tuning.md)에 있다.
새 실시간 UI는 표시 대기 없이 복원·기록하며, 저장 영상의 단계별 표시와 이전 API 동작은 유지한다.

원본 영상과 달리 지도 위치·깊이는 추정치다. 사람 표식은 현재 관측/마지막 발견 위치이며
확정 인원수·예측 위치를 뜻하지 않는다. 실제 비행·치수 정확도는 별도 검증해야 한다.
상용 배포 전 SegFormer·YOLO 코드와 가중치의 사용 조건을 확인한다.
필드·좌표·저장 호환성은 [의미 지도 계약](../../../contracts/mapping/RESCUE_SEMANTIC_MAP_V1.md)을 따른다.

촬영/프레임·2D 특징점/매칭·실측 입력, 점군 처리·회차별 저장·재생·미리보기·업로드 코드를 둔다. 문의: 박수경. 원본 영상·생성물은 data의 비공개 경로에 둔다.

## 휴대폰 영상의 2D 좌표 인계 시험

- 코드: `modules/mapping/data/phone_mapping/` (휴대폰 특징점·좌표 입력·전달 도구).
- 설치·실행·로컬 데이터 준비: [실행 안내](../../../docs/phone_mapping/README.md).
- 팀원이 Git과 실제 Drive 자료로 다음 작업을 시작하는 순서: [팀원 안내서](../../../docs/phone_mapping/team_guide.md).
- 공통 자료 분류와 등록 방법: [Drive 구조](../../../docs/team/DRIVE_STRUCTURE.md).
- 입출력·좌표 규칙·품질 한계: [좌표 인계 계획](../../../docs/phone_mapping/coordinate_handoff_plan.md).
- 생성 환경: Windows, Python 3.12.14, PyCOLMAP 4.2.1. 세부 패키지는 `modules/mapping/data/phone_mapping/requirements-features.txt`로 고정한다.
- 자동 SIFT 특징점·여러 프레임의 대응·수동 실측 기준점을 전달한다. 실제 3D 복원과 수신자 환경 확인은 후속 단계다.
- 확정 JSON/CSV·설정·검증 자료는 `data/mapping/phone_video/2f1_20261021/`에 공유한다. 원본 프레임·DB·ZIP과 중간 결과는 로컬/Drive에 보관한다.
- `python -B modules/mapping/data/phone_mapping/shared_inputs.py`로 Git 자료를 검사한다. `--package-root`로 어느 위치의 전달 패키지든 연결하고 `--workspace`로 새 작업 사본을 만들 수 있다.
- 파트 검사: `python -B -m unittest discover -s modules/mapping/data/tests -v`.

팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.

## 새 영상의 프레임 입력 준비 (파일 파이프라인 2단계)

Python 3.12 환경에서 저장소 루트 기준으로 설치/실행한다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/data/requirements-video.txt
.\.venv\Scripts\python.exe -B modules/mapping/data/video_input.py --video "C:/videos/new_video.mp4"
```

기본 출력은 `output/phone_mapping/<자동 run_id>/input/`이다. 영상 경로만 필수이며 별도의 실측이나
프레임 수 지정이 필요 없다. 원본은 복사/변경하지 않고 읽는다. 기본적으로 2fps 시간 격자로 선택하고
표시 회전을 적용하며 영상 해상도를 유지한다. 모든 디코딩 프레임의 선택/제외 이유를 기록한다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/data/video_input.py `
  --video "C:/videos/new_video.mp4" --run-id hallway_20261023_run01 `
  --sample-fps 2 --max-width 1280 `
  --output output/phone_mapping/hallway_20261023_run01/input
```

출력 폴더는 새 경로여야 한다. 다른 작업 디렉터리에서 실행할 때 상대 입력/명시적 출력 경로는 해당
작업 디렉터리 기준이며, 기본 출력만 저장소 루트를 기준으로 한다. 원본이 이동했으면 local_source.json의
경로와 해시를 확인하고 새 원본 위치를 사용한다.

`manifest.json`은 영상/프레임 체크섬·PTS·해상도·회전/축소 좌표 변환을 담고,
`frame_decisions.jsonl`은 모든 디코딩 프레임의 선택/제외 기록이다. `status.json`의 `frames_ready`를
확인한 뒤 다음 단계로 연결한다. 실패하면 status/manifest/execution.log에서 단계와 사유를 확인한다.
최소 3장이 안 나오면 더 긴 영상이나 높은 sample-fps를 사용한다.

이 명령은 입력 준비까지 수행한다. 아래 3단계 명령으로 특징점·매칭을 이어서 생성한다.
새 입력의 3D 복원은 아래 4단계로 연결한다. 한 명령 실행은 아래 5단계를 따른다.
[입력 명세](../../../contracts/mapping/PHONE_VIDEO_INPUT_V1.md)에 PTS 오류, 회전 지원 범위와 시간 범위를 설명한다.

실제 디코더·가변 프레임 시각·회전/축소·오류·출력 보호 검사:

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s modules/mapping/data/tests -p test_video_input.py -v
```

## 새 입력의 특징점·매칭 (파일 파이프라인 3단계)

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/data/phone_mapping/requirements-features.txt
.\.venv\Scripts\python.exe -B modules/mapping/data/phone_mapping/full_features.py `
  --input-dir output/phone_mapping/hallway_20261023_run01/input `
  --output-dir output/phone_mapping/hallway_20261023_run01/features
```

프레임 수·해상도·PTS는 manifest에서 읽는다. 기본 CPU 4스레드, 매칭 범위는 앞의 선택 프레임 4장이다.
`--threads`, `--pair-window`, `--max-features`로 바꿀 수 있다. 새 출력 폴더를 사용하며 기존 결과를 덮어쓰지 않는다.
추출·매칭·DB/CSV 내보내기와 전체 대조를 수행하고 status.json의 features_ready를 확인한다.
metadata/frames_manifest에 처리 이미지 좌표와 카메라 크기가 기록된다. 원본 1920×1080으로 고정하지 않는다.

기존 DB에서 CSV/보고서만 다시 만들 때는 같은 명령에 `--export-only`를 추가한다. 입력/DB가 달라진 경우
거부하며, 기존 DB·좌표는 유지한다. 검사만 따로 실행할 수도 있다.

```powershell
.\.venv\Scripts\python.exe -B modules/mapping/data/phone_mapping/validate_features.py `
  --input-dir output/phone_mapping/hallway_20261023_run01/input `
  --features-dir output/phone_mapping/hallway_20261023_run01/features
```

[결과 명세](../../../contracts/mapping/PHONE_FEATURE_RESULT_V1.md)에 DB/CSV와 좌표·시간·상태를 설명한다.
실측 자료 없이 아래 4단계 3D 복원까지 연결할 수 있다.
기존 첫 실험 형식도 지원하며, 고정 Git snapshot은 변경하지 않는다.

실제 디코딩·회전/축소된 5장 입력에서 native SIFT/매칭, DB/CSV 대조, 재내보내기·변조 거부 검사:

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s modules/mapping/data/tests -p test_feature_pipeline.py -v
```

## 새 입력의 실측 없는 3D 복원 (파일 파이프라인 4단계)

[복원 모듈 안내](../localization/README.md#새-영상-입력과-연결-파일-파이프라인-4단계)의 명령으로
2단계 input/과 3단계 features/를 연결한다. 2D 특징점의 대응과 카메라 모델을 이용해
김수호의 SfM 코드가 카메라 위치와 희소 3D 점을 함께 추정한다. 수동 기준점이나 YOLO는 필요하지 않다.
실측 없는 결과는 임의 단위다. 모든 입력 프레임이 한 모델에 등록된 complete와 일부만 등록된 partial을 구분한다.
partial도 선택 모델을 열어 확인할 수 있으며 실패(failed)와 다르다.

전체 실행 진입점·통합 시간/로그는 아래 5단계에서 제공한다. PNG·저장 결과 대조는 아래 6단계로 연결한다.

## 영상 하나로 실행 (파일 파이프라인 5단계)

```powershell
.\.venv\Scripts\python.exe -m pip install -r modules/mapping/data/requirements-pipeline.txt
.\.venv\Scripts\python.exe -B modules/mapping/data/run_video_pipeline.py --video "C:/videos/new_video.mp4"
```

기존 세 작업을 같은 Python 환경의 별도 프로세스로 순서대로 실행한다. 원본·native 로그를 파일로 남기고
전체/단계 벽시계 시간과 상세 내부 시간을 함께 기록한다. 실패하면 다음 단계는 실행하지 않는다.
새 회차 루트에 input/, features/, reconstruction/, logs/와 전체 manifest/status/timing을 저장한다.
종료 코드는 전체 등록 0, 부분 복원 3, 실패 2, 중단 130이다. partial은 결과가 있지만 전체 등록은 아닌 상태다.
명령/옵션·폴더·실패 확인 순서·시간 범위는 [실행 안내](../../../docs/phone_mapping/video_pipeline.md),
프로그램용 결과 형식은 [실행 결과 계약](../../../contracts/mapping/PHONE_VIDEO_PIPELINE_V1.md)을 따른다.
이 명령은 아래 6단계의 저장 결과 대조와 PNG 생성도 이어서 수행한다.

## 저장 결과 대조·PNG (파일 파이프라인 6단계)

result_io.py가 원본 사진 없이 저장 결과를 다시 읽고 JSON/PLY/CSV 좌표·카메라·단위·해시를 비교한다.
render_preview.py는 검사된 모든 점과 카메라 경로로 overview.png와 projections.png를 만든다.
전체 시간에는 preview worker의 시작/import·재로딩·PNG 저장도 포함한다. 실패는 전체 failed로 기록하며 3D 부분 파일은 보존한다.
PNG는 바닥 추정/실측 평면도가 아니며 실제 촬영 생성물과 함께 로컬 output/ 또는 Drive로 관리한다.
단독 재로딩·재생성 명령은 [실행 안내](../../../docs/phone_mapping/video_pipeline.md#저장-결과-재사용과-검사)를 따른다.
