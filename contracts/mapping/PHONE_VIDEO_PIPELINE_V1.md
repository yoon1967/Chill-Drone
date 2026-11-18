# 파일 영상 파이프라인 실행 결과 1.0

생산자: modules/mapping/data/run_video_pipeline.py. 소비자: CLI 호출자·회차 관리 도구.
루트 manifest/status의 schema_version은 phone-video-pipeline/1.0이다.
하위 입력·특징점·복원의 개별 계약과 필드/단위는 유지한다.

## 입력과 출력 위치

- 필수 --video 로컬 영상 경로. 선택 --output 새 회차 루트, --run-id 및 추출·매칭 설정.
- 기본 2fps, 크기 유지, JPEG 95, CPU 4스레드, pair-window 4, max-features 4000.
- --run-id 미지정 시 UTC 기반 고유 이름. 출력 기본값은 저장소 output/phone_mapping/<run_id>/.
- 같은 Python 실행 파일의 worker를 input → features → reconstruction → preview 순서로 시작한다. 명령 shell 확장을 사용하지 않는다.
- 기존 폴더/원본/실제 Git 입력을 덮어쓰지 않는다. 회차 루트는 새 폴더여야 한다.

## 파일

| 경로 | 계약 |
| --- | --- |
| manifest.json | run_id, status, stage, settings, source(추출 후 원본 해시 등), stages, summary, artifacts, 코드 해시·환경 |
| status.json | schema_version, run_id, status, stage, 실패/중단 시 error |
| timing.json | 전체·준비·단계·마무리 벽시계 시간과 worker의 내부 시간·측정 범위 |
| execution.log | UTF-8 JSONL. run/stage 시작·완료·오류 이벤트, UTC와 진입 후 경과 시간 |
| logs/<stage>.log | argv 기록 및 worker stdout+stderr 전체(native 출력 포함) |
| logs/pipeline_error.log | 실패/중단의 Python traceback |
| local_source.json | 로컬 원본 절대 경로. 공유 수치 계약 제외 |
| input/ | [영상 프레임 입력](PHONE_VIDEO_INPUT_V1.md) |
| features/ | [2D 특징점·매칭](PHONE_FEATURE_RESULT_V1.md) |
| reconstruction/ | [SfM 결과](PHONE_SFM_RESULT_V1.md) |
| preview/ | [저장 결과 대조·PNG](PHONE_PREVIEW_V1.md) |

artifacts의 path는 회차 루트 기준 상대 경로다. 선택 모델의 6개 필수 내보내기 파일과 입력 manifest,
feature metadata, reconstruction provenance와 PNG 2개/검증 JSON/preview manifest의 SHA-256/크기를 연결한다. 전체 이미지/DB 파일 목록을 복제하지 않는다.
생산자의 provenance와 최종 산출물 해시, 회차 ID/임의 단위/등록 수와 상태를 대조한 뒤 전체 결과를 게시한다.

## 상태와 오류

- running: preflight/input/features/reconstruction/preview/finalization 중 처리 중.
- complete, 종료 0: 결과/참조 검사를 통과하고 제출한 모든 프레임이 선택 모델에 등록됨.
- partial, 종료 3: 결과/참조 검사는 통과했지만 미등록 제출 프레임이 있음. 빈 모델은 허용하지 않음.
- failed, 종료 2: 작업/검사 실패. error.stage/type/reason과 가능한 worker_status를 보존. 이후 작업은 not_started.
- interrupted, 종료 130: Ctrl+C/worker 중단. 자식 프로세스를 종료하고 부분 파일/경과 시간을 보존.
- 단계 status는 not_started/running/complete/failed/interrupted다. complete는 그 작업의 종료를 뜻하며
  복원의 범위는 worker_status와 루트 complete/partial을 읽는다. 실패한 finalization 이전 단계들이 complete일 수 있다.
- worker returncode=0만으로 성공 처리하지 않는다. 기대하는 상태 파일과 timing이 있어야 다음 단계로 간다.
- 설정/출력 경로가 부적절하거나 출력이 이미 있으면 생성 없이 거부한다. CLI 오류/종료 2로 알린다.
  유효한 새 회차의 영상 누락/열기 실패는 보고서를 저장한다. OS 강제 종료/디스크 쓰기 불가는 최종 보고서를 보장하지 못한다.

현재 네 작업 단계는 PNG까지다. preview는 preview_ready 상태와 timing을 게시해야 한다.
not_performed에는 browser_visual_review/metric_scaling/dense_reconstruction을 기록한다.
complete도 이 작업들의 완료나 실제 공간 정확도를 뜻하지 않는다. 이전 5단계의 3-worker 결과에는 preview가 없을 수 있다.
새 입력의 스케일은 arbitrary다. 원본 PTS·픽셀 좌표 변환은 하위 계약으로 추적한다.

## 시간

clock=perf_counter. total_seconds는 run_pipeline 진입부터 최종 manifest/status/event 저장까지다.
마지막 timing 자체 쓰기·콘솔 출력·부모 Python 시작/CLI 인자 파싱·설치·뷰어/사용자 대기는 제외한다. PNG 렌더링/import와 저장 결과 대조는 포함한다.
preflight_seconds, stages[*].elapsed_seconds(자식 시작/import·로그·상태 읽기 포함), finalization_seconds를 제공한다.
부모 상태/이벤트 저장 비용은 total에 포함하므로 하위 시간의 합과 정확히 같을 필요는 없다.
worker_timings의 추출/매칭/SfM 상세 시간은 부모 단계 시간에 포함된다. 합산하여 중복 계산하지 않는다.
실패/중단한 단계에는 경과 시간을 남기고 미실행 시간은 null이다. 모든 결과·로그는 실제 촬영 자료와 함께 로컬/Drive로 관리한다.

preview manifest의 회차·scene 해시·단위·등록 범위도 복원 결과와 대조한다.
