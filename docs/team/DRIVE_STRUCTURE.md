# 팀 공통 Drive 자료 구조와 등록 규칙

갱신: 2026-10-22. 촬영 원본과 기능별 처리 결과를 함께 찾고, 여러 팀원이 같은 이름의 자료를 덮어쓰지 않도록 관리한다.
저장소 코드 구조는 [REPOSITORY_STRUCTURE.md](REPOSITORY_STRUCTURE.md), 휴대폰 입력 실행은 [팀원 안내서](../phone_mapping/team_guide.md)를 따른다.

## 1. 실제 공유 위치와 현재 범위

- [팀 Drive 상위 폴더](https://drive.google.com/drive/folders/1r8iNz97yyZ7lFQWRyoheZbO969RAhkw1)
- [프로젝트_공유자료](https://drive.google.com/drive/folders/1jEIgRg0_VBrY0HGN1zNfgYDebDsSPvCx)
- [이번 촬영 원본·측정 참고](https://drive.google.com/drive/folders/1Qbt3hr7y0XrFo3st_ZGYVqgBzCZ_o5t6)
- [이번 처리 결과](https://drive.google.com/drive/folders/1AiaXk37HcsF27v9ZXuak8WIDfSJ_FEIz)
- [선택 프레임 31장](https://drive.google.com/drive/folders/1WqPJynCYONbAghIn8ghUXrlD1E30C5tk)

현재 생성한 최상위 분류는 `01_captures`와 `02_runs`다. 아래 실제 구조에 **81개 파일, 59,031,760 bytes**를 게시했다.
영상 1개·측정 사진 1개·선택 프레임 31장·DB 1개·확인 그림 42개·로그 3개·메타데이터 JSON 2개다.

```text
프로젝트_공유자료/
├─ 01_captures/real/2f1_20261021/
│  ├─ capture.json
│  ├─ raw/phone_01/videos/
│  │  └─ source_video.mp4
│  └─ references/measurements_v2_floor/
│     └─ measurement_reference.png
└─ 02_runs/mapping/2f1_20261021/
   └─ 2f1_20261021_features_v1_full/
      ├─ artifact_manifest.json
      ├─ frames/                  # 원본 선택 31장
      ├─ database.db
      ├─ previews/                # 매칭 30장 + 측정 검토 12장
      └─ logs/
         ├─ execution.log
         ├─ export.log
         └─ raw_assets_check.json
```

확인 범위는 업로드 전 로컬 SHA-256/크기 검사, Drive 전체 업로드 완료 표시, 주요 경로와 프레임/그림/로그 목록 확인이다.
원격 파일 전체를 다시 다운로드해 바이트를 비교하지는 않았다. 팀원은 받은 프레임과 DB를 [안내서의 검사 예제](../phone_mapping/team_guide.md)로 검증한다.
로그의 DB 읽기 성공은 제작자 환경의 검사이며 실제 3D 복원·수신자 환경·실측 정확도 검증과 구분한다.

다른 촬영 원본 5개, 전체 구간 55장, 개인 재검증 사본은 이번 업로드에 포함하지 않았다.
아래 미래 분류 폴더도 아직 생성하지 않았다.

## 2. 전체 팀이 사용할 분류

상위는 자료의 역할, 원본은 촬영 회차, 결과는 기능과 처리 회차로 구분한다.
담당자 이름은 메타데이터에 기록하고 개인 임시 실험에서 이름별 폴더를 사용한다.

| 분류 | 용도 | 현재 상태 |
| --- | --- | --- |
| `00_catalog/` | 촬영·처리 목록, 위치·담당자·접근 방법 | 향후 필요 시 생성 |
| `01_captures/real/<capture_id>/` | 실제 휴대폰·드론·센서 촬영 원본과 현장 기록 | 이번 휴대폰 회차 생성됨 |
| `01_captures/simulation/<capture_id>/` | 시뮬레이션 생성 원본 | 향후 필요 시 생성 |
| `02_runs/<기능>/<capture_id>/<run_id>/` | 원본에서 파생한 프레임·탐지·지도·로그 | mapping의 이번 처리 회차 생성됨 |
| `03_datasets/<dataset_id>/<version>/` | 여러 촬영을 조합한 학습·평가 자료 | 향후 필요 시 생성 |
| `04_models/<task>/<model_id>/` | 모델 가중치와 학습 산출물 | 향후 필요 시 생성 |
| `05_demo_media/<시연 회차>/` | 발표·시연용 편집 영상과 이미지 | 향후 필요 시 생성 |
| `90_scratch/<팀원 이름>/` | 개인 임시 실험과 작업 자료 | 향후 필요 시 생성 |

촬영 장비와 기능을 섞지 않는다. 예를 들어 드론 영상도 `01_captures/real/`에 원본을 한 번 두고,
매핑과 객체 인식은 각각 `02_runs/mapping/`, `02_runs/perception/`에서 같은 `capture_id`를 참조한다.
비행 계획/제어 실행 기록은 해당 `02_runs/autonomy/` 회차에 둔다. 실제 촬영과 시뮬레이션의 근거를 구분한다.

## 3. 촬영 회차와 처리 회차

| 식별자 | 의미 | 이번 자료 |
| --- | --- | --- |
| `capture_id` | 어떤 촬영 원본인가 | `2f1_20261021` |
| 처리 `run_id` | 그 원본을 어떤 실행에서 처리했는가 | `2f1_20261021_features_v1_full` |
| 주석 버전 | 어떤 실측 기준점 정의/관측인가 | `2f1_20261021_measurement_anchors_v2_floor` |

새 촬영/처리 ID는 UUID를 사용하고 사람이 읽을 이름·날짜·장소는 메타데이터와 목록에 적는다.
이번 기존 ID는 변경하지 않는다. Drive의 회차 식별자는 입력 참조용이며 기존 phone 자료를 공통 PoC 형식으로 자동 변환하지 않는다.
공통 교환 결과를 만들 때 ID·프레임 이름·시각 단위 변환은 [PoC 계약](../../contracts/POC_V0_1.md)에 맞춰 명시한다.

- 여러 팀원이 같은 원본으로 실험하면 서로 다른 처리 `run_id`를 사용한다.
- 확정된 처리 폴더와 입력 ID를 덮어쓰지 않는다. 재처리는 새 회차/버전으로 추가한다.
- `최종`, `최종2` 대신 고유 ID와 메타데이터의 처리 상태로 구분한다.
- 원본은 한 위치에 유지하고 다른 기능에서는 원본 ID/위치를 참조한다.
- 프레임 파일명은 Git의 `frame_id`와 같게 유지한다. 다른 촬영의 같은 파일명은 상위 회차 폴더로 구분된다.
- 같은 촬영 세션의 연속 파일은 같은 촬영 회차에 둘 수 있다. 여러 장비를 썼다면 `raw/<device_id>/`로 나눈다.
- 촬영 시각 미상은 null/미상으로 기록한다. 이번 ID의 날짜를 실제 촬영 시각으로 추정하지 않는다.

## 4. 촬영 폴더의 내용

새 촬영의 기본 형태는 다음과 같다. 해당 장비에 실제 자료가 있는 종류부터 생성한다.

```text
01_captures/real/<capture_id>/
  capture.json
  raw/<device_id>/
    videos/
    photos/
    telemetry/
  references/<참고 자료 버전>/
```

- `videos/`, `photos/`: 장비가 직접 만든 원본. 영상에서 추출한 프레임은 처리 결과에 둔다.
- `telemetry/`: 비행·센서 기록이 있을 때 사용한다. RGB만 있는 입력에 없는 센서값을 채우지 않는다.
- `references/`: 실측 표시 사진, 현장 참고 자료. 복원용 원본 이미지와 구분한다.

`capture.json`에는 촬영 ID·설명·담당자·장비 ID/설정·촬영 시각·파일 목록/크기/SHA-256을 기록한다.
현재 게시한 `drive-capture/1.0`에는 `capture_id`, `owner`, `captured_at_utc`, `camera`, `video_information`, `git`, `files` 등이 있다.
이는 Drive 자료 관리용 메타데이터이며 공통 복원 결과 JSON을 대신하지 않는다.

## 5. 처리 폴더와 메타데이터

처리 폴더에는 해당 실행이 실제로 생성한 결과를 넣는다. 기능에 따라 frames, DB, 점군, 카메라 경로,
탐지 결과, 확인 그림과 로그를 보관한다. 입력/결과 담당 경계를 유지하고 다른 실행의 파일을 섞지 않는다.

`artifact_manifest.json`에 다음 정보를 기록한다.

- 처리 ID, 입력 `capture_id`, 입력으로 참조한 다른 처리 회차와 주석 버전.
- 담당자, 도구/버전과 설정 출처, 연결되는 Git 커밋·데이터 경로.
- 파일별 처리 폴더 기준 상대 경로, 크기와 SHA-256.
- 프레임 이름·PTS·해상도·DB image_id 등 입력 연결 정보.
- 처리 단계와 성공/부분 성공/실패, 실제로 검증하지 않은 항목.

현재 게시한 형식은 `drive-artifacts/1.0`이며 처리 ID 필드는 `processing_run_id`, 입력 회차 필드는 `input_capture_id`다.
현재 `reconstruction_executed=false`, `height_correspondence_ready=false` 상태를 보존했다.
가로 44cm 스케일 시험용 자료이며, 세로 136cm의 바닥 C 동일점 대응은 미확인이다.

## 6. Git·Drive·로컬의 기준

| 내용 | 기준 위치 |
| --- | --- |
| 처리 코드, 의존성, 실행 설명, 합의된 형식 | Git의 module/contracts/docs |
| 팀원이 소비하는 확정 좌표·매칭·실측·설정·검증 기록 | Git의 `data/mapping/` 회차/버전 |
| 원본 영상·사진·DB·큰 점군·전체 로그·확인 그림 | Drive의 촬영/처리 회차 |
| 생성 중인 결과·재실행 사본 | 로컬 `output/` 등 ignore 경로 |

Git 수치 입력을 Drive에서도 보관할 때는 대응 커밋/해시를 고정한 사본으로 취급하고 독립적으로 수정하지 않는다.
Drive manifest는 Git의 좌표 파일 위치와 버전을 연결한다. 데이터 구조/단위 계약을 Drive에서 별도로 바꾸지 않는다.
현재 확정 snapshot의 `external_artifacts.json`에는 예전 배포 기록과 `folder_id=null`이 남아 있다.
활성 Drive 위치는 이 문서의 링크를 사용한다. 확정 파일의 해시를 깨뜨리는 임의 수정은 하지 않는다.

팀원이 자료를 다운로드할 때 로컬 폴더 위치는 자유롭게 선택한다.
현재 휴대폰 회차는 [팀원 안내서](../phone_mapping/team_guide.md)의 원본 프레임/DB 직접 검사로 연결한다.
모든 Drive 회차를 자동 다운로드·읽기하는 일반 CLI가 구현됐다는 뜻은 아니다.

## 7. 새 자료를 올리는 순서

1. 촬영/처리 ID를 정하고 기존 폴더·목록에 같은 ID가 있는지 확인한다.
2. 자신의 작업 영역에서 결과를 정리하고 입력과 산출물의 버전·이름을 확정한다.
3. 메타데이터에 입력 ID·담당자·설정·Git 커밋과 파일 해시를 기록한다.
4. 같은 ID의 확정 결과가 있으면 덮어쓰지 않고 새 처리 회차를 만든다.
5. 해당 Drive 경로에 업로드하고 완료 상태·파일 개수·원본 보존을 확인한다.
6. 실제 폴더 링크, 자료 버전, 필요한 접근 방법과 후속 담당자가 할 검사를 문서/작업에 남긴다.
7. 수신자는 다운로드한 자료를 해시와 연결 정보로 검사하고 본인 환경의 결과를 기록한다.

Drive가 같은 이름의 폴더를 허용하더라도 같은 ID의 확정 폴더를 중복 생성하지 않는다.
큰 자료의 원본 위치를 바꿀 때는 참조와 목록도 함께 갱신한다. 담당자가 바뀌어도 ID/자료 위치는 유지한다.
공용 목록은 필요 시 `00_catalog/`에서 관리하며 다른 파트의 기존 항목을 덮어쓰지 않는다.

## 8. 문서 관리

- 전 파트 공통 Drive 구조/등록 규칙: 이 문서.
- 휴대폰 실험 실행·검사·다음 복원 단계: [team_guide.md](../phone_mapping/team_guide.md).
- 개별 모듈 실행 방법과 구현 상태: 해당 모듈 README.
- 입력/출력 필드·단위의 합의: `contracts/`.

구조·실행 방법을 설명하는 문서는 Git에 관리하고 실제 대용량 자료는 Drive에 연결한다.
공통 규칙 변경은 영향받는 파트와 형식/경로의 전환 방법을 MR에 적는다.
