# 휴대폰 2D 입력·3D 복원 팀원 안내

갱신: 2026-10-22. Git의 확정 입력을 읽고 Drive의 프레임·DB를 연결해 다음 복원 작업을 진행하는 안내다.
확정 수치 입력이 develop에 반영된 기준 커밋은 `07fe7c64281fa0b4457cfdee115c172766d61f5f`다.
이는 문서 전체의 최신 커밋을 고정한다는 뜻은 아니다. 이후 구현 상태는 실제 코드와 담당 작업에서 확인한다.

- [문서 목록과 빠른 시작](README.md)
- [팀 공통 Drive 구조와 등록 규칙](../team/DRIVE_STRUCTURE.md)
- [현재 phone 입력 형식](../../contracts/mapping/PHONE_INPUT_SNAPSHOT_V1.md)
- [확정 수치 입력](../../data/mapping/phone_video/2f1_20261021/README.md)

## 1. 현재 무엇이 준비됐는가 (2026-10-22 확인)

- Git: 확정 2D 좌표·프레임 간 매칭·실측 기준점·프레임 목록·처리 설정·검증 기록과 코드.
- 로컬: 원본 영상·이미지·DB·확인 그림·중간 결과와 재실행 사본.
- Drive: 이번 촬영/처리 자료 81개(약 59MB)를 업로드했다. 원격 화면에서 전체 완료와 선택 프레임 31장·확인 그림 42개·로그 3개를 확인했다. 다른 촬영 원본 5개 등은 이번 업로드 범위가 아니다.
- 실제 3D 카메라/점 복원과 김수호님 환경 검증은 아직 완료되지 않았다.
- 지금부터 팀원은 develop을 받아 작업 브랜치에서 개발한다. 예전 작업 브랜치는 더 이상 선행 조건이 아니다.

Git만으로 바로 할 수 있는 작업은 좌표 파일 읽기, 데이터 구조 연결, 형식/참조 검사다.
현재 COLMAP 기반 복원 경로에는 원본 프레임 31장과 동일한 특징점 DB가 추가로 필요하다.
프레임이나 DB를 이미 갖고 있다면 아래 해시 검사를 거쳐 사용할 수 있다.

## 2. Git 폴더 구조 (경로는 저장소 루트 기준)

```text
data/mapping/
  README.md
  phone_video/2f1_20261021/
    README.md
    run_manifest.json
    external_artifacts.json
    features_v1/
      keypoints.csv
      matches.csv
      frames_manifest.json
      feature_metadata.json
      quality_report.json
      validation_report.json
    measurements_v2_floor/
      manual_picks.json
      measurement_anchors.json
      measurement_observations.csv
      annotation_quality_report.json

scripts/phone_mapping/
  shared_inputs.py          # Git 수치 입력 검사와 현재 경로 조회
  full_features.py          # 특징점·매칭 추출/내보내기
  validate_features.py      # DB·프레임·CSV 대조
  annotate_measurements.py  # 수동 기준점 처리
  path_options.py           # 재생성 출력 위치 보호
  requirements-features.txt

contracts/mapping/
  PHONE_INPUT_SNAPSHOT_V1.md # 현재 휴대폰 입력 형식
  README.md
contracts/POC_V0_1.md       # 이후 파트 간 공통 교환 형식

modules/mapping/
  data/                    # 수경: 입력·좌표·저장·전달 구현
    tests/test_shared_inputs.py
  localization/            # 김수호: 카메라·3D·스케일 복원 구현
```

이 트리는 이번 협업에 필요한 파일을 추린 것이다. 저장소의 모든 파일을 나열한 목록은 아니다.
기존 처리 코드는 경로 호환을 위해 scripts/phone_mapping/에 남겨 두었다.
확인 시점 localization/에는 담당 안내가 있으며, 이번 실험을 실제 복원하는 실행 CLI는 아직 구현되어 있지 않다. 후속 구현이 들어오면 해당 모듈 README를 우선 확인한다.

## 3. 팀원이 Git을 받은 직후 할 일

저장소가 없다면 복제하고, 이미 있다면 자신의 미커밋 변경을 먼저 확인한다.

```powershell
git clone https://lab.ssafy.com/s15-final/S15P31B205.git
cd S15P31B205
git switch develop
git pull --ff-only origin develop
python -B scripts/phone_mapping/shared_inputs.py
```

기존 저장소 사용자는 clone/cd를 생략한다. develop에서 직접 코드를 수정하지 않고 작업할 실제 Jira 키에 맞는 브랜치를 만든다.
Git 전용 검사는 Python 표준 라이브러리만 사용하며 사진/DB나 별도 pip 설치가 필요 없다.

기대 결과:

| 항목 | 기대값 |
| --- | --- |
| status | passed_with_declared_height_limitation |
| frames | 31 |
| keypoints | 178584 |
| matches | 59175 |
| geometric_inliers | 52159 |
| manual_observations | 15 |
| external_package | not_requested |

마지막 값은 기본 검사가 외부 자료까지 검사하지 않았다는 뜻이다. 입력 오류나 3D 복원 성공을 뜻하지 않는다.

파일은 다음 순서로 읽는다.

1. 회차 README와 run_manifest.json: 활성 파일·회차·버전·해시.
2. frames_manifest.json: 프레임 이름·DB image_id·원본 PTS·해상도·해시.
3. keypoints.csv: 프레임별 2D 점.
4. matches.csv: 서로 다른 프레임의 어떤 점이 연결되는지.
5. measurement_anchors.json: A/B/C 정의, 길이, 관측 좌표와 사용 제한.
6. feature_metadata.json 및 보고서: 도구/카메라 초기 설정과 당시 검사 범위.

## 4. Git 입력을 코드에서 읽는 방법

```python
from pathlib import Path
import sys

repo = Path('/실제/저장소/경로')
sys.path.insert(0, str(repo / 'scripts/phone_mapping'))
from shared_inputs import load_run, validate, csv_rows

run = load_run(repo / 'data/mapping/phone_video/2f1_20261021')
validate(run)
for point in csv_rows(run['paths']['keypoints']):
    frame_id = point['frame_id']
    point_id = int(point['keypoint_id'])
    x, y = float(point['x_px']), float(point['y_px'])
    # (frame_id, point_id)를 프레임 내 점 식별자로 사용한다.
```

활성 경로는 load_run() 결과와 run_manifest.json을 따른다.
복사한 생산자 JSON에 남은 ../../... 경로는 과거 실행 당시 출처라서 새 Git 폴더에 직접 붙이지 않는다.
특징점 CSV에는 SIFT 기술자가 없고 매칭 CSV에는 DB의 모든 기하 정보가 없으므로 기존 DB의 완전한 대체물은 아니다.

## 5. 실제 Drive 위치와 업로드된 구조

- [프로젝트 공유자료](https://drive.google.com/drive/folders/1jEIgRg0_VBrY0HGN1zNfgYDebDsSPvCx)
- [촬영 원본·측정 참고](https://drive.google.com/drive/folders/1Qbt3hr7y0XrFo3st_ZGYVqgBzCZ_o5t6)
- [프레임·DB·확인 그림·로그](https://drive.google.com/drive/folders/1AiaXk37HcsF27v9ZXuak8WIDfSJ_FEIz)
- [선택 프레임 31장](https://drive.google.com/drive/folders/1WqPJynCYONbAghIn8ghUXrlD1E30C5tk)

```text
프로젝트_공유자료/
  01_captures/real/2f1_20261021/
    capture.json
    raw/phone_01/videos/source_video.mp4
    references/measurements_v2_floor/measurement_reference.png
  02_runs/mapping/2f1_20261021/2f1_20261021_features_v1_full/
    artifact_manifest.json
    frames/                 # 원본 선택 31장
    database.db
    previews/               # 매칭 30장 + 측정 검토 12장
    logs/                   # execution.log, export.log, raw_assets_check.json
```

두 메타데이터 JSON에 회차/처리 버전·Git 커밋·파일별 크기/SHA-256·미검증 항목을 기록했다.
원격 파일을 다시 내려받아 모든 바이트를 대조한 것은 아니며, 업로드 전 로컬 해시 대조와 Drive 전체 완료/파일 목록을 확인했다.
팀원은 frames/ 폴더와 database.db를 다운로드한 후 아래 예제로 본인 환경에서 검사한다.
확정 좌표/실측/설정은 Git의 data/mapping/phone_video/2f1_20261021/를 기준으로 읽는다.

이번에는 source_video.mp4 한 개만 업로드했다. 다른 촬영 원본 5개와 전체 구간 55장 등은 별도 보관 대상이다.
현재 Drive에는 위 회차 자료만 게시되어 있다. 개인 재검증 사본은 로컬 자료다.
확정 snapshot의 external_artifacts.json에 남은 folder_id=null과 예전 경로는 당시 출처 기록이다. 실제 Drive 위치는 이 문서와 공통 Drive 구조 문서를 따른다.
위 실제 폴더 링크와 아래 원본 프레임/DB 직접 검사를 사용하면 된다.

## 6. Git 좌표와 Drive의 프레임·DB가 같은 자료인지 확인

아래 예제는 Git의 확정 해시와 **프레임 폴더/DB 파일을 직접 비교**한다. 일반 폴더 형태의 자료만 있으면 된다.
이 문서의 확인 시점에는 저장소 CLI에 아래 예제와 같은 raw 폴더용 인자가 별도로 구현된 것은 아니다. 아래 Python 예제를 직접 실행하는 방식이다.

먼저 저장소 루트에서 환경을 구성하고 받은 자료의 실제 로컬 경로를 입력한다.
네이티브 검사는 제작 환경과 같은 Python 3.12 사용을 권장한다. 생성 당시 Python은 3.12.14였고,
의존성은 requirements-features.txt의 PyCOLMAP 4.2.1·NumPy 2.3.5·Pillow 12.3.0으로 고정한다.

```powershell
python --version
python -m venv .venv
$mappingPython = (Resolve-Path '.venv/Scripts/python.exe').Path
& $mappingPython -m pip install -r scripts/phone_mapping/requirements-features.txt
$env:MAPPING_REPO_ROOT = (Get-Location).Path
$env:MAPPING_FRAMES_DIR = Read-Host '원본 31장이 있는 로컬 폴더 경로'
$env:MAPPING_DB_FILE = Read-Host '원본 database.db의 로컬 파일 경로'
$env:MAPPING_CHECK_REPORT = Join-Path (Get-Location).Path 'output/phone_mapping/local_receipts/raw_assets_check.json'
```

이어서 같은 PowerShell에서 다음 블록을 실행한다.

```powershell
@'
"""Check Git numeric inputs against the original frame folder and DB."""
import json
import os
from pathlib import Path
import subprocess
import sys

repo = Path(os.environ["MAPPING_REPO_ROOT"]).resolve()
sys.path.insert(0, str(repo / "scripts/phone_mapping"))
from shared_inputs import check_file, load_run, read_json, require, sha256, validate
from check_handoff import native_read

run = load_run(repo / "data/mapping/phone_video/2f1_20261021")
git_result = validate(run)
images = Path(os.environ["MAPPING_FRAMES_DIR"]).resolve()
database = Path(os.environ["MAPPING_DB_FILE"]).resolve()
frames = read_json(run["paths"]["frames"])["frames"]
names = {frame["frame_id"] for frame in frames}
require({path.name for path in images.glob("*.jpg")} == names,
        "Select the exact 31-frame folder, not all extracted frames")
for frame in frames:
    check_file(images / frame["frame_id"], {
        "sha256": frame["sha256"],
        "bytes": run["external"]["frame_bytes"][frame["frame_id"]],
    })
check_file(database, run["external"]["artifacts"]["database"])
before = sha256(database)
native_result = native_read(database, names)
require(sha256(database) == before, "Original DB changed")
result = {
    "input_run_id": run["manifest"]["input_run_id"],
    "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
    "git_inputs": git_result,
    "raw_assets": "passed",
    "frames_checked": len(frames),
    "database_sha256": before,
    "native_sfm_input_read": native_result,
    "reconstruction_executed": False,
    "floor_C_identity_confirmed": False,
    "scope": "Only the environment actually running this check; no 3D reconstruction or metric validation",
}
report_name = os.environ.get("MAPPING_CHECK_REPORT")
if report_name:
    report = Path(report_name).resolve()
    require(not report.is_relative_to(repo / "data") and report != database
            and not report.is_relative_to(images), "Keep the report separate from inputs")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False, indent=2))
'@ | & $mappingPython -B -
```

검사 내용은 Git 입력 일관성, 선택 프레임 31장의 정확한 파일명/해시/크기, DB 해시/크기,
PyCOLMAP 4.2.1 DB 사본 읽기와 SfM 입력 캐시의 31장 연결이다. 원본 DB를 보존한다.
`raw_assets=passed`, `native_sfm_input_read.status=passed`를 확인하고 결과 JSON을 본인 실행 환경의 기록으로 회신한다.
다른 도구로 DB를 수정한 사본은 원래 Git 입력과 일치하지 않을 수 있으므로 원본 DB를 따로 보존한다.
위 예제는 제작자 PC에서 직접 실행 검증했다. 이것이 김수호님의 환경까지 검증했다는 뜻은 아니다.

## 7. 김수호님이 실제 3D 복원으로 이어갈 순서

1. develop의 Git 검사 및 위 원본 프레임/DB 검사를 통과한다.
2. DB를 새로운 로컬 작업 디렉터리에 복사한다. 프레임은 원본 경로를 읽거나 원본 그대로 복사한다.
3. localization 영역에서 해당 DB와 image_path를 받는 복원 실행 코드를 구현한다. 이미 특징점/매칭이 준비되어 있으므로 이번 입력을 사용할 때 다시 추출할 필요는 없다.
4. 카메라 자세와 희소 3D 점을 복원하고 등록/실패 프레임, 카메라 모델·추정 파라미터, 점 수, 재투영 오차를 기록한다.
5. 등록된 카메라에서 수동 A/B 관측을 별도로 읽고 왜곡을 일관되게 처리해 삼각측량한다.
6. A–B 복원 길이가 d_raw이면 0.44/d_raw를 점군과 카메라 중심/이동에 일관되게 적용한다. 회전은 바꾸지 않는다.
7. C의 실제 동일점 대응을 확인하기 전까지 136cm 높이 검증은 수행하지 않는다.
8. 결과/실패 로그·형식·사용 설정을 남기고 결과의 위치와 확인 방법을 수경에게 전달한다.

현재 카메라는 SIMPLE_RADIAL의 미보정 초기값이다. iPhone의 실측 보정값이 아니므로 복원에서 추정/보정이 필요하다.
유리 반사·반복 사물함·평면 중심 장면에 따른 오매칭/퇴화가 있어 점 개수가 많아도 성공이 보장되지 않는다.
실패할 경우 로그와 실패 프레임을 근거로 원본 구간 재선정/재촬영 여부를 함께 판단한다.

## 8. 다음 결과를 어디에 올리는가

| 내용 | 저장 위치 |
| --- | --- |
| 복원/변환/검사 코드와 의존성·실행 방법 | modules/mapping/localization/ 중심으로 Git |
| 입력/출력 형식의 합의 | contracts/mapping/ |
| 확정 복원 메타데이터·카메라 경로·작은 품질 요약 | 해당 Git 회차의 reconstruction/<새 버전>/ |
| 큰 점군·DB·원본 영상·사진·상세 로그·확인 그림 | Drive의 해당 회차/처리 버전 |
| 자동 생성된 중간 결과·재실행 사본 | 로컬 output/ |

reconstruction/은 앞으로 추가할 영역이며 아직 실제 결과는 없다.
공통 PoC 결과 형식을 쓸 때는 현재 phone 입력의 기존 run/frame ID와 초 단위 PTS를 명시적으로 연결한다.
공통 PoC는 UUID 회차·다른 frame ID·마이크로초 PTS를 사용하므로 이름만 바꿔서 같은 형식으로 취급하지 않는다.
실제 결과 파일은 계약에 맞는 단위·축척 근거·카메라 변환 방향과 외부 큰 파일의 식별자/해시를 함께 기록한다.
Git에 결과 메타데이터만 있어도 필요한 큰 파일을 Drive에서 연결해야 실제 결과를 열 수 있다.

김수호님 회신의 최소 항목:

- 사용한 Git 커밋·input/feature/annotation 버전과 실행 도구/버전.
- 원본 읽기 검사 JSON 및 성공/실패 여부.
- 기대 프레임 수와 등록 프레임 수, 3D 점 수, 재투영 오차 또는 미측정 사유.
- 보정 전 임의 단위인지, 44cm 스케일 적용 후 m 단위인지.
- 결과 메타데이터의 Git 경로와 큰 자료의 Drive 위치/해시.
- C 동일점 확인 여부와 136cm 검증 실행 여부. 미실행을 성공으로 표시하지 않는다.

## 9. 회차/버전과 팀 작업 경계

- 수경 입력 버전과 김수호 복원 버전을 나누고 확정 좌표/ID를 덮어쓰지 않는다.
- 새 실험은 새 회차, 같은 입력 재처리는 새 처리 버전으로 관리한다.
- 향후 드론 입력은 data/mapping/drone_video/<회차>/처럼 분리한다.
- 다른 팀의 객체 인식·자율비행 코드는 각자 module 영역에 두고 공용 계약으로 연결한다.
- 공용 계약 수정은 생산자·소비자 영향과 변환 방법을 별도 작은 MR로 남긴다.
- 현재 추출 도구는 첫 실험 31장에 맞춰져 있다. 새 드론 입력을 받으려면 별도 입력/설정 확장이 필요하다.

## 10. 문서 갱신

이 안내서는 `docs/phone_mapping/team_guide.md`를 원본으로 관리한다. 개인 전달 사본보다 저장소의 최신 문서를 우선 확인한다.
Drive 공통 구조/등록 규칙은 [DRIVE_STRUCTURE.md](../team/DRIVE_STRUCTURE.md), 기존 처리 이력은 [좌표 인계 계획](coordinate_handoff_plan.md)에 남긴다.
새 구현의 실행 방법과 실제 검사 결과가 바뀌면 해당 모듈 README와 이 문서의 관련 절을 같은 작업에서 갱신한다.
대용량 원본이나 개인 로컬 파일 목록은 문서와 함께 커밋하지 않는다.
