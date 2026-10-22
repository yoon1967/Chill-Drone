# 휴대폰 영상의 2D 좌표 추출·인계 도구

## 먼저 읽을 문서

| 문서 | 용도 |
| --- | --- |
| [팀원 실행·다음 작업 안내](team_guide.md) | Git 입력 확인 → 실제 Drive 자료 받기 → 프레임/DB 검사 → 3D 복원 담당의 다음 단계 |
| [팀 공통 Drive 구조](../team/DRIVE_STRUCTURE.md) | 실제 공유 위치, 촬영/처리 회차, 이름 충돌 방지, Git·Drive 역할 |
| [좌표 인계 계획과 실행 이력](coordinate_handoff_plan.md) | 기존 단계별 처리 결과와 미확인 사항 |
| [확정 수치 입력](../../data/mapping/phone_video/2f1_20261021/README.md) | 회차·버전·원본 좌표 파일 |
| [현재 입력 형식](../../contracts/mapping/PHONE_INPUT_SNAPSHOT_V1.md) | 좌표·점 ID·PTS·경로 해석 규칙 |

팀원이 새로 시작할 때는 `team_guide.md`를 따른다. 아래는 기존 도구와 재처리 경로의 참고다.
현재 특징 추출 도구는 첫 촬영 run에 맞춘 시험 도구이며 다른 촬영에 그대로 적용하는 범용 CLI는 아니다.

## 데이터 보관

확정 좌표 CSV·실측 JSON·설정·검증 기록은 `data/mapping/phone_video/2f1_20261021/`에서 Git으로 공유한다.
실제 영상·프레임·특징점 DB·주석 그림·ZIP·전체 로그는 기존 로컬/Drive에 유지한다.
clone만으로 수치 입력 검사와 소비 코드 개발은 가능하다. 실제 영상 기반 재처리/복원에는 외부 프레임과 DB가 필요하다.
Git 수치 입력 위치는 `shared_inputs.py --run-dir`로 지정한다. Drive의 원본 프레임/DB 폴더는 [팀원 안내서](team_guide.md)의 직접 검사 예제로 연결한다.
기존 로컬 경로는 아래와 같지만 새 실행 위치를 강제하지 않는다.

| 로컬 경로 (`output/phone_mapping/` 아래) | 용도 |
| --- | --- |
| `2f1_20261021/` | 원본 영상, 선택 프레임, manifest, 실측 표시 사진 |
| `2f1_20261021_features_v1/full/` | 자동 특징점·매칭 DB, CSV, 품질/검증 보고서 |
| `2f1_20261021_features_v1/measurements/` | 수동 입력 `manual_picks.json`, 기준점 JSON/CSV, 확인 그림 |
| `2f1_20261021_handoff_v1/` | 통합 전달 ZIP, 체크섬, 로컬 수신 검사 결과 |

공유한 Git 이력은 재작성하지 않았다. 이전 커밋의 자료는 이력에 남으므로 이번 추적 제외가 이력의 용량 축소나 자료 삭제를 의미하지 않는다.

## 환경

Windows의 Python 3.12.14 환경에서 PyCOLMAP 4.2.1로 생성했다. 저장소 루트에서 실행한다.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r scripts/phone_mapping/requirements-features.txt
```

## 기존 로컬 입력 폴더에서 재처리

Git 전용 입력 검사는 `python -B scripts/phone_mapping/shared_inputs.py`이며 추가 패키지가 필요 없다.
기존 전달 폴더를 보관한 경우에는 `--package-root <ZIP 해제 루트> --workspace <새 작업 폴더>`를 사용할 수 있다.
이 옵션은 예전 manifest와 폴더 구성을 요구한다. Drive에서 받은 프레임과 DB만 사용할 때는 `team_guide.md`의 직접 검사 예제를 따른다.
자세한 형식과 경로 해석은 `contracts/mapping/PHONE_INPUT_SNAPSHOT_V1.md`를 따른다.

아래는 기존 기본 경로를 사용하는 재처리 순서다. 임의 위치에서는 `full_features.py`에 `--input-dir/--output-dir`,
`validate_features.py`에 `--input-dir/--features-dir/--report`, `annotate_measurements.py`에
`--input-dir/--features-dir/--output-dir/--picks-file`을 지정한다. Git 입력 폴더를 출력 위치로 쓰지 않는다.
재처리와 기존 검증기는 작업 사본에서 실행한다. 기본 전체 추출은 첫 회차 31장에 맞춰져 있다.

1. 입력 폴더의 `manifest.json`, `frames_first_test/`, `measurement_reference.png`와 수동 `manual_picks.json`을 준비한다. manifest의 프레임 목록과 SHA-256이 실제 파일과 일치해야 한다.
2. 자동 특징점·매칭을 생성한다. 기존 DB가 있으면 덮어쓰지 않고 중단한다. 기존 DB에서 CSV만 다시 만들 때는 `--export-only`를 사용한다.

   ```powershell
   .\.venv\Scripts\python.exe -B scripts/phone_mapping/full_features.py
   # 기존 DB에서 CSV/보고서 재생성:
   # .\.venv\Scripts\python.exe -B scripts/phone_mapping/full_features.py --export-only
   ```

3. DB와 CSV의 좌표/인덱스/체크섬을 검증하고 수동 기준점 결과를 만든다. 수동 입력은 자동으로 정답을 찾는 과정이 아니며 별도 확인이 필요하다.

   ```powershell
   .\.venv\Scripts\python.exe -B scripts/phone_mapping/validate_features.py
   .\.venv\Scripts\python.exe -B scripts/phone_mapping/annotate_measurements.py
   ```

4. 필요할 때 전달 ZIP을 새 출력 폴더에 재생성한다. 기존 ZIP이 있으면 덮어쓰지 않고 중단한다.
   `--input-dir`, `--features-dir`(full/과 measurements/를 포함하는 상위 폴더), `--output-dir`를 지정할 수 있다.
   ZIP 해시·압축 해제·네이티브 DB 읽기와 미확인 높이 사용 방지 검사를 수행한다.

   ```powershell
   .\.venv\Scripts\python.exe -B scripts/phone_mapping/package_handoff.py --output-dir output/phone_mapping/handoff_rebuild
   ```

5. 수신자는 ZIP을 풀고 ZIP 안의 README대로 `scripts/check_handoff.py --native`를 실행한다. 수신자 검사기는 표준 라이브러리 검사만 할 수도 있지만 네이티브 검사는 PyCOLMAP 4.2.1 환경이 필요하다. 실제 수신 결과는 별도 회신으로 확인한다.

## 결과 해석

좌표는 원본 이미지의 2D 픽셀 좌표이며 첫 픽셀 중심은 `(0.5, 0.5)`다.
여러 프레임의 대응과 카메라 설정을 이용해 후속 SfM에서 카메라 자세와 3D 점을 추정한다.
가로 0.44m는 복원 결과의 크기를 맞추는 기준이다.
세로 1.36m의 바닥 C는 여러 시점에서 같은 실제 점인지 미확인이라 치수 검증에 사용할 수 없다.
카메라의 실측 보정값과 실제 3D 복원 결과도 아직 없다.

로컬 전달물의 체크섬/읽기 검사는 통과했지만 수신자 환경의 복원 성공을 뜻하지 않는다.
