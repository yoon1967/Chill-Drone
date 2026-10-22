"""Build the stage-5 ZIP and test the extracted package outside the repository."""

from __future__ import annotations

import hashlib
import argparse
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tempfile
from zipfile import ZipFile, ZIP_DEFLATED
from path_options import writable_output


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "output/phone_mapping"
INPUT = BASE / "2f1_20261021"
FEATURES = BASE / "2f1_20261021_features_v1"
PACKAGE_ID = "2f1_20261021_handoff_v1"
OUTPUT = BASE / PACKAGE_ID

README = """# 김수호 전달용: 2층 1 영상의 3D 복원 입력

이 ZIP을 새 폴더에 풀고 이 README가 있는 폴더를 기준으로 사용한다. 원본 31장과 **이미 추출한 2D 특징점·매칭 DB**가 함께 있다. 생성 버전은 PyCOLMAP 4.2.1이다. 좌표는 원본 1920×1080 픽셀이며 첫 픽셀 중심은 (0.5,0.5)다.

## 첫 시험의 범위

- 자동 특징점 178,584개, 114쌍의 매칭 후보 59,175개와 기하 검증 대응 52,159개. 이 수는 고유 3D 점 수가 아니다.
- 가로 A–B=0.44m를 스케일 기준으로 사용한다. 수동 A/B 관측은 원본 5장에 있고 자동 SIFT 점 번호와 구분한다.
- **세로 1.36m는 위쪽 A에서 받침대 하단의 바닥 접점 C까지다. C의 다른 프레임 대응은 미확인이다.** 기준 사진 1개와 추정 4개를 구분했고 모든 C의 metric_validation_eligible=false, 높이 선분 correspondence_ready=false다. 추정 C를 확인된 제어점으로 사용하거나 136cm 정확도 검증을 완료했다고 기록하지 않는다.
- 카메라는 실측 보정값 없이 SIMPLE_RADIAL, f=2304px 초깃값을 사용했다. 복원 시 추정/보정이 필요하다. 영상은 iPhone 13 일반 동영상·1× 고정·HD/30fps 설정이라는 사용자 정보이며 실제 프레임은 1920×1080이다.
- 이 전달물은 카메라 자세·3D 점·m 좌표·스케일 보정 결과를 포함하지 않는다. 김수호가 이 입력으로 실제 복원과 스케일 검증을 진행한다.

## 폴더와 입력 경로

| 경로 | 용도 |
| --- | --- |
| output/phone_mapping/2f1_20261021/frames_first_test/ | 복원용 원본 31장. 주석 그림을 섞어 넣지 않는다. |
| output/phone_mapping/2f1_20261021/source_video.mp4 | 원본 영상 보관용 |
| output/phone_mapping/2f1_20261021/manifest.json | 원본 PTS·이미지 체크섬·카메라 정보·수정 측정 참조 |
| output/phone_mapping/2f1_20261021_features_v1/full/database.db | 특징점·SIFT 기술자·매칭·기하 검증·카메라/이미지 정보 |
| 같은 full/ 폴더의 keypoints.csv, matches.csv, candidate_tracks.csv | 좌표·프레임 쌍의 대응과 여러 프레임의 2D 연결 후보 |
| output/phone_mapping/2f1_20261021_features_v1/measurements/measurement_anchors.json | 수동 A/B/C 관측, 44cm/136cm 길이와 사용 제한 |
| full/와 measurements/의 README·보고서·확인 그림 | 품질·미확인 사항·파일별 설명 |
| handoff_manifest.json | 활성 경로와 모든 전달 파일의 SHA-256/크기 |
| scripts/check_handoff.py | 이 패키지를 다른 위치에서 읽는 수신 검사기 |
| provenance/ | 실제 추출·주석 코드와 설정 버전의 보관 사본. 수신 검사 명령과 구분한다. |

기존 절대 촬영 경로와 일부 보고서의 입력 ZIP 이름/manifest 해시는 실행 당시의 출처다. 현재 사용하는 경로는 handoff_manifest.json의 paths를 따른다. source manifest의 측정 사진은 리비전 2이며, 특징점 추출 당시 manifest 해시와 달라도 원본 31장과 특징점 DB는 그대로다. 이전 3프레임 OpenCV pilot 결과는 이 패키지에 넣지 않았다. 같은 프레임 이름이라도 pilot 점 번호와 섞지 않는다.

## 수신 검사 (PowerShell)

이 README가 있는 폴더에서 실행한다. Python 3.12 환경을 권장하며 별도 가상 환경을 만들 수 있다.

```powershell
python -m venv ../mapping-env
& ../mapping-env/Scripts/python.exe -m pip install -r requirements.txt
& ../mapping-env/Scripts/python.exe scripts/check_handoff.py --native --receiver-name "김수호" --report ../recipient_receipt.json
```

Linux/macOS에서는 가상 환경의 Python 경로를 ../mapping-env/bin/python으로 바꾼다. 기본 파일/SQLite/좌표 검사는 NumPy와 Pillow로 실행되고, --native는 PyCOLMAP 4.2.1로 **DB 사본**을 읽어 SfM 입력 캐시에 원본 31장이 로드되는지 확인한다. --native가 없으면 네이티브 읽기 검사는 미실행으로 기록된다.

검사기는 모든 파일 해시, 원본 이미지/영상, CSV↔DB 점/매칭, 후보 트랙, 프레임/시각/경로, 수정된 측정 참조와 C 사용 제한을 검사한다. C의 검증 제한이 임의로 해제된 패키지는 거부한다. 원본 DB를 변경하지 않으며 보고서는 패키지 폴더 바깥에 저장한다.

기대 출력은 status=passed_with_declared_height_limitation, native_sfm_input_read.status=passed, original_frames_checked=31이다. 이는 파일과 SfM 입력 읽기 확인이며 실제 3D 복원 성공을 의미하지 않는다. 다른 PyCOLMAP 버전에서는 이 버전의 네이티브 호환성을 자동으로 성공 처리하지 않는다.

제작자는 ZIP을 저장소 밖 임시 폴더에 풀어 동일한 검사를 수행하고 별도의 local_acceptance_report.json에 기록한다. **제작자 로컬 검사는 김수호 환경의 확인을 대신하지 않는다.** 실제 수신 결과는 위 명령으로 만든 recipient_receipt.json과 함께 회신한다.

## 실제 복원으로 연결하기

1. 검사 통과 후 database.db를 별도 작업 폴더에 복사하고, 위 원본 31장 경로를 image_path로 지정해 기존 특징점/매칭에서 SfM 복원 단계로 연결한다. 원본 전달 DB는 보관한다.
2. 등록 프레임 수, 카메라 모델/내부 파라미터, 3D 점 수, 재투영 오차·실패 로그를 기록한다. 반복 사물함·유리 반사·평면 중심 장면의 오매칭과 퇴화를 살핀다.
3. 카메라 복원 후 수동 A/B 관측을 별도로 읽고 왜곡을 일관되게 처리해 삼각측량한다. 안정적으로 복원된 A–B 길이가 d_raw이면 0.44/d_raw를 점군과 카메라 위치에 적용한다.
4. C는 같은 실제 바닥점의 여러 시점 대응을 확인할 때까지 세로 길이 검증에서 제외한다. 카메라/3D 끝점이 복원됐다는 사실만으로 추정 C가 맞다고 판단하지 않는다.

김수호가 회신할 항목은 수신 환경/도구 버전, 검사 JSON, 31장 경로/DB 읽기 여부, 복원 착수/결과 또는 실패 로그다. 기본 양식은 recipient_receipt_template.json, 전달 문구는 recipient_message.md에 있다.

Jira: S15P31B205-253은 실제 수신 읽기 확인이 남아 진행 중, -254는 C 동일점 확인과 수신 확인·리뷰/머지가 남아 진행 중이다. 실제 3D 복원은 김수호의 -217 범위다.
"""

MESSAGE = """수호님, 2층 1 영상의 3D 복원 시험 입력을 묶었습니다.

ZIP을 풀면 원본 31장, PyCOLMAP 4.2.1 특징점/매칭 database.db, 2D 좌표·매칭 CSV, 수동 기준점과 설정/품질 보고서가 있습니다. 루트 README의 수신 검사 명령을 실행하고 recipient_receipt.json을 회신해 주세요.

가로 A–B 44cm로 스케일 시험을 진행하면 됩니다. 세로 136cm는 위쪽 A에서 받침대 하단의 바닥 C까지로 정정했습니다. C는 기준 사진 1장 외의 대응이 추정이라 현재 세로 길이 검증 사용을 막았습니다. 이 제한은 그대로 두고 첫 복원을 진행해 주세요.

카메라 보정값은 실측값이 없고 SIMPLE_RADIAL 초깃값에서 추정해야 합니다. 자동 SIFT 점 번호와 수동 A/B/C는 구분됩니다. 복원 후 수동 관측을 따로 삼각측량해야 합니다.

원본 31장/DB 읽기 여부, 사용 도구·버전, 등록 프레임/점 수 또는 실패 로그를 알려 주세요. 제작자 로컬 압축 해제 검사는 별도 수행했지만 실제 수신 환경 확인은 아직입니다.
"""


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write_json(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    global INPUT, FEATURES, OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=INPUT)
    parser.add_argument("--features-dir", type=Path, default=FEATURES, help="Directory containing full/ and measurements/")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT)
    args = parser.parse_args()
    INPUT, FEATURES, OUTPUT = args.input_dir.resolve(), args.features_dir.resolve(), writable_output(args.output_dir)
    if (OUTPUT / (PACKAGE_ID + ".zip")).exists():
        raise FileExistsError("Preserve the delivered ZIP. Choose a new --output-dir")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Path | bytes] = {}
    source = json.loads((INPUT / "manifest.json").read_text(encoding="utf-8"))
    meta = json.loads((FEATURES / "full/feature_metadata.json").read_text(encoding="utf-8"))
    anchors = json.loads((FEATURES / "measurements/measurement_anchors.json").read_text(encoding="utf-8"))
    input_names = ["source_video.mp4", "measurement_reference.png", "manifest.json", "frames.csv", "frame_quality.csv", "README.md", "contact_sheet.jpg"]
    input_names += ["frames_first_test/" + f["frameId"] for f in source["frames"] if f["selectedForFirstTest"]]
    for name in input_names:
        payload["output/phone_mapping/2f1_20261021/" + name] = INPUT / name
    for section in ("full", "measurements"):
        for path in sorted((FEATURES / section).rglob("*")):
            if not path.is_file():
                continue
            if path.name.endswith(("-wal", "-shm")) or "__pycache__" in path.parts:
                raise RuntimeError(f"Unexpected temporary file in source: {path}")
            payload["output/phone_mapping/2f1_20261021_features_v1/" + path.relative_to(FEATURES).as_posix()] = path
    for name in ("full_features.py", "annotate_measurements.py", "validate_features.py", "requirements-features.txt", "package_handoff.py", "path_options.py"):
        payload["provenance/" + name] = ROOT / "scripts/phone_mapping" / name
    payload["scripts/check_handoff.py"] = ROOT / "scripts/phone_mapping/check_handoff.py"
    payload["requirements.txt"] = ROOT / "scripts/phone_mapping/requirements-features.txt"
    payload["README.md"] = README.encode("utf-8")
    payload["recipient_message.md"] = MESSAGE.encode("utf-8")
    template = {"status": "pending_actual_recipient_run", "receiver_name": "", "tool_and_version": "",
                "receiver_os": "", "receipt_json": "", "image_and_database_paths_resolved": None,
                "sfm_input_read_result": None, "reconstruction_started": None,
                "registered_frames": None, "points3D": None, "failure_log": "",
                "floor_C_identity_confirmed": False, "height_validation_performed": False}
    payload["recipient_receipt_template.json"] = (json.dumps(template, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    active = {"images": "output/phone_mapping/2f1_20261021/frames_first_test", "source_manifest": "output/phone_mapping/2f1_20261021/manifest.json",
              "source_video": "output/phone_mapping/2f1_20261021/source_video.mp4", "measurement_reference": "output/phone_mapping/2f1_20261021/measurement_reference.png"}
    for key, filename in (("database", "database.db"), ("keypoints_csv", "keypoints.csv"), ("matches_csv", "matches.csv"),
                          ("candidate_tracks_csv", "candidate_tracks.csv"), ("frames_manifest", "frames_manifest.json"),
                          ("feature_metadata", "feature_metadata.json"), ("quality_report", "quality_report.json")):
        active[key] = "output/phone_mapping/2f1_20261021_features_v1/full/" + filename
    active["measurement_anchors"] = "output/phone_mapping/2f1_20261021_features_v1/measurements/measurement_anchors.json"
    active["measurement_observations_csv"] = "output/phone_mapping/2f1_20261021_features_v1/measurements/measurement_observations.csv"
    inventory = {}
    for name, item in payload.items():
        data = item.read_bytes() if isinstance(item, Path) else item
        inventory[name] = {"bytes": len(data), "sha256": sha256(data)}
    manifest = {"schema_version": "1.0", "package_id": PACKAGE_ID, "input_run_id": source["runId"],
                "feature_run_id": meta["feature_run_id"], "annotation_run_id": anchors["annotation_run_id"],
                "producer_pycolmap_version": "4.2.1", "data_baseline_commit": "4b0bbf6",
                "status": "first_3d_trial_ready_floor_height_identity_pending", "paths": active,
                "width_scale_reference_m": 0.44, "height_target_m": 1.36, "height_correspondence_ready": False,
                "actual_recipient_environment_test": "pending", "reconstruction": "not_executed",
                "files": inventory}
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    archive = OUTPUT / (PACKAGE_ID + ".zip")
    with ZipFile(archive, "w", compression=ZIP_DEFLATED, compresslevel=6) as z:
        for name, item in payload.items():
            if isinstance(item, Path):
                z.write(item, arcname=PACKAGE_ID + "/" + name)
            else:
                z.writestr(PACKAGE_ID + "/" + name, item)
        z.writestr(PACKAGE_ID + "/handoff_manifest.json", manifest_bytes)
    (OUTPUT / "README.md").write_text(README, encoding="utf-8")
    (OUTPUT / "recipient_message.md").write_text(MESSAGE, encoding="utf-8")
    (OUTPUT / "handoff_manifest.json").write_bytes(manifest_bytes)
    write_json(OUTPUT / "recipient_receipt_template.json", template)
    archive_sha = sha256(archive.read_bytes())
    (OUTPUT / (archive.name + ".sha256")).write_text(archive_sha + "  " + archive.name + "\n", encoding="ascii")

    temp_base = Path(tempfile.gettempdir()).resolve()
    with tempfile.TemporaryDirectory(prefix="phone-handoff-roundtrip-", dir=temp_base) as temporary:
        temp_root = Path(temporary)
        if not temp_root.resolve().is_relative_to(temp_base):
            raise RuntimeError("Unexpected package-test cleanup path")
        with ZipFile(archive) as z:
            if z.testzip() is not None:
                raise RuntimeError("ZIP CRC failure")
            for name in z.namelist():
                parts = PurePosixPath(name).parts
                if not parts or parts[0] != PACKAGE_ID or ".." in parts or PurePosixPath(name).is_absolute():
                    raise RuntimeError("Unsafe ZIP path")
            z.extractall(temp_root)
        extracted = temp_root / PACKAGE_ID
        receipt = temp_root / "local_receipt.json"
        command = [sys.executable, str(extracted / "scripts/check_handoff.py"), "--native", "--receiver-name", "local-producer-simulation", "--report", str(receipt)]
        result = subprocess.run(command, cwd=extracted, capture_output=True, text=True, encoding="utf-8", errors="replace")
        (OUTPUT / "local_receiver_check.log").write_text(result.stdout + "\n--- native reader stderr ---\n" + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            raise RuntimeError("Extracted receiver check failed; see local_receiver_check.log")
        accepted = json.loads(receipt.read_text(encoding="utf-8"))
        if accepted["status"] != "passed_with_declared_height_limitation" or accepted["native_sfm_input_read"]["status"] != "passed":
            raise RuntimeError("Native receiver read was not verified")
        # Exercise the real risk: even a checksum-consistent package must reject
        # an accidental promotion of unconfirmed floor C to metric validation.
        anchor_path = extracted / active["measurement_anchors"]
        old_anchor, old_manifest = anchor_path.read_bytes(), (extracted / "handoff_manifest.json").read_bytes()
        tampered = json.loads(old_anchor)
        next(s for s in tampered["segments"] if s["segment_id"] == "height_136cm")["correspondence_ready"] = True
        new_anchor = (json.dumps(tampered, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        anchor_path.write_bytes(new_anchor)
        new_manifest = json.loads(old_manifest)
        new_manifest["files"][active["measurement_anchors"]] = {"bytes": len(new_anchor), "sha256": sha256(new_anchor)}
        write_json(extracted / "handoff_manifest.json", new_manifest)
        rejected = temp_root / "negative_receipt.json"
        bad = subprocess.run([sys.executable, str(extracted / "scripts/check_handoff.py"), "--report", str(rejected)], cwd=extracted,
                             capture_output=True, text=True, encoding="utf-8", errors="replace")
        failure = json.loads(rejected.read_text(encoding="utf-8")) if rejected.exists() else {}
        if bad.returncode == 0 or "Unconfirmed floor height must stay disabled" not in failure.get("error", ""):
            raise RuntimeError("Unconfirmed C gating negative check failed")
        anchor_path.write_bytes(old_anchor)
        (extracted / "handoff_manifest.json").write_bytes(old_manifest)
        if anchor_path.read_bytes() != old_anchor or (extracted / "handoff_manifest.json").read_bytes() != old_manifest:
            raise RuntimeError("Temporary test restoration failed")
    report = {"scope": "local_producer_zip_roundtrip_not_Kim_Suho_environment", "status": "passed_with_declared_height_limitation",
              "package_id": PACKAGE_ID, "zip_name": archive.name, "zip_bytes": archive.stat().st_size, "zip_sha256": archive_sha,
              "zip_crc_check": "passed", "extraction_location": "temporary directory outside repository",
              "receiver_check": accepted,
              "negative_check": {"case": "height correspondence_ready promoted to true, with inventory hashes updated", "result": "correctly_rejected", "error": failure["error"]},
              "actual_recipient_environment_test": "pending", "actual_transfer": "not_sent_by_assistant", "reconstruction_executed": False}
    write_json(OUTPUT / "local_acceptance_report.json", report)
    print(json.dumps({"status": report["status"], "archive": str(archive.resolve()), "zip_bytes": report["zip_bytes"],
                      "files": len(inventory), "native": accepted["native_sfm_input_read"], "negative_check": report["negative_check"]["result"],
                      "actual_recipient_environment_test": "pending"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
