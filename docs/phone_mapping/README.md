# 휴대폰 영상의 2D 좌표 추출·인계 도구

자세한 처리 순서와 결과의 한계는 [좌표 인계 계획](coordinate_handoff_plan.md)을 참고한다.
현재 스크립트는 첫 촬영 run의 경로와 설정을 고정한 시험 도구다. 다른 촬영에 그대로 적용하는 범용 CLI는 아니다.

## 데이터 보관

실제 영상·프레임·특징점 DB·좌표 CSV·주석·생성 ZIP은 Git에 포함하지 않는다.
기존 로컬 자료는 `output/phone_mapping/`에 유지하고 비공개 전달 경로로 공유한다.
저장소를 새로 clone한 것만으로 이 촬영의 결과를 재생성할 수는 없다.
원본 자료와 수동 입력을 먼저 같은 상대 경로에 준비해야 한다.

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

## 실행 순서

다음 명령은 해당 로컬 입력이 준비되어 있을 때 사용한다. 기존 결과를 보존한 뒤 재생성한다.

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

4. 전달 ZIP을 재생성한다. ZIP 해시·압축 해제·네이티브 DB 읽기와 미확인 높이 사용 방지 검사를 수행한다.

   ```powershell
   .\.venv\Scripts\python.exe -B scripts/phone_mapping/package_handoff.py
   ```

5. 수신자는 ZIP을 풀고 ZIP 안의 README대로 `scripts/check_handoff.py --native`를 실행한다. 수신자 검사기는 표준 라이브러리 검사만 할 수도 있지만 네이티브 검사는 PyCOLMAP 4.2.1 환경이 필요하다. 실제 수신 결과는 별도 회신으로 확인한다.

## 결과 해석

좌표는 원본 이미지의 2D 픽셀 좌표이며 첫 픽셀 중심은 `(0.5, 0.5)`다.
여러 프레임의 대응과 카메라 설정을 이용해 후속 SfM에서 카메라 자세와 3D 점을 추정한다.
가로 0.44m는 복원 결과의 크기를 맞추는 기준이다.
세로 1.36m의 바닥 C는 여러 시점에서 같은 실제 점인지 미확인이라 치수 검증에 사용할 수 없다.
카메라의 실측 보정값과 실제 3D 복원 결과도 아직 없다.

로컬 전달물의 체크섬/읽기 검사는 통과했지만 수신자 환경의 복원 성공을 뜻하지 않는다.
