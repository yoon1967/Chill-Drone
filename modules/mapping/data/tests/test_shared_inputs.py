"""Exercise relocation and rejection of corrupt shared inputs without images/COLMAP."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts/phone_mapping"))
from shared_inputs import DEFAULT_RUN, inside, load_run, prepare_workspace, sha256, validate


class SharedInputTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="mapping-snapshot-test-")
        self.addCleanup(self.temporary.cleanup)
        self.parent = Path(self.temporary.name)
        self.run_dir = self.parent / "relocated run"
        shutil.copytree(DEFAULT_RUN, self.run_dir)

    def update_inventory(self, key):
        path = self.run_dir / "run_manifest.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        record = manifest["files"][key]
        item = self.run_dir / record["path"]
        record.update(bytes=item.stat().st_size, sha256=sha256(item))
        path.write_text(json.dumps(manifest), encoding="utf-8")

    def test_relocated_snapshot_from_unrelated_cwd_without_images(self):
        result = subprocess.run([sys.executable, "-B", str(ROOT / "scripts/phone_mapping/shared_inputs.py"),
                                 "--run-dir", str(self.run_dir)], cwd=self.parent,
                                capture_output=True, text=True, check=True)
        report = json.loads(result.stdout)
        self.assertEqual(report["keypoints"], 178584)
        self.assertEqual(report["matches"], 59175)
        self.assertEqual(report["external_package"], "not_requested")
        self.assertFalse(any(self.run_dir.rglob("*.jpg")))

    def test_checksum_corruption_rejected(self):
        with (self.run_dir / "features_v1/matches.csv").open("ab") as file:
            file.write(b"corruption")
        with self.assertRaisesRegex(ValueError, "checksum"):
            load_run(self.run_dir)

    def test_dangling_point_rejected_even_with_updated_checksum(self):
        path = self.run_dir / "features_v1/matches.csv"
        lines = path.read_text(encoding="utf-8").splitlines()
        fields = lines[1].split(",")
        fields[1] = "999999999"
        lines[1] = ",".join(fields)
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.update_inventory("matches")
        with self.assertRaisesRegex(ValueError, "unknown point"):
            validate(load_run(self.run_dir))

    def test_unconfirmed_height_promotion_rejected_with_updated_checksum(self):
        path = self.run_dir / "measurements_v2_floor/measurement_anchors.json"
        anchors = json.loads(path.read_text(encoding="utf-8"))
        next(s for s in anchors["segments"] if s["segment_id"] == "height_136cm")["correspondence_ready"] = True
        path.write_text(json.dumps(anchors), encoding="utf-8")
        self.update_inventory("measurement_anchors")
        with self.assertRaisesRegex(ValueError, "height must stay disabled"):
            validate(load_run(self.run_dir))

    def test_path_escape_and_windows_absolute_paths_rejected(self):
        for name in ("../outside.csv", "C:/outside.csv", "/outside.csv", "..\\outside.csv"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                inside(self.run_dir, name)

    def test_existing_workspace_cannot_be_overwritten(self):
        target = self.parent / "existing"
        target.mkdir()
        marker = target / "keep.txt"
        marker.write_text("original", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "already exists"):
            prepare_workspace({"package_root": self.parent / "package"}, target)
        self.assertEqual(marker.read_text(encoding="utf-8"), "original")


if __name__ == "__main__":
    unittest.main()
