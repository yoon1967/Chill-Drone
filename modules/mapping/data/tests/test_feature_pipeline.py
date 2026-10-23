"""Run native SIFT/matching on stage-2 inputs and reject corrupt handoffs."""
import contextlib
import csv
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import av
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts/phone_mapping"))
sys.path.insert(0, str(ROOT / "modules/mapping/data"))
from feature_input import digest, load_feature_input
from full_features import run_features
from validate_features import validate_features
from video_input import extract_video


def make_textured_video(path):
    random = np.random.default_rng(42)
    texture = Image.fromarray(random.integers(0, 256, (440, 720, 3), dtype=np.uint8))
    draw = ImageDraw.Draw(texture)
    for _ in range(300):
        x, y = map(int, random.integers([0, 0], [690, 410]))
        draw.rectangle((x, y, x + 12, y + 12), fill=tuple(map(int, random.integers(0, 256, 3))))
    with av.open(str(path), "w") as container:
        stream = container.add_stream("mpeg4", rate=4)
        stream.width, stream.height, stream.pix_fmt = 512, 320, "yuv420p"
        stream.set_display_rotation(90)
        for index in range(20):
            image = texture.crop((index * 4, index * 2, index * 4 + 512, index * 2 + 320))
            for packet in stream.encode(av.VideoFrame.from_image(image)):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)


class FeaturePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="feature-pipeline-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.parent = Path(cls.temporary.name)
        cls.video = cls.parent / "new.mp4"
        make_textured_video(cls.video)
        cls.input_dir = cls.parent / "input"
        extract_video(cls.video, cls.input_dir, run_id="portrait_run", sample_fps=1, max_width=240)
        cls.features = cls.parent / "features"
        with contextlib.redirect_stdout(io.StringIO()):
            cls.report = run_features(cls.input_dir, cls.features, pair_window=2, max_features=500, threads=1)

    def test_stage2_portrait_input_native_db_and_csv(self):
        source = load_feature_input(self.input_dir)
        self.assertEqual(len(source["selected"]), 5)
        self.assertEqual((source["width"], source["height"]), (240, 384))
        self.assertEqual(self.report["summary"]["attempted_pairs"], 7)
        self.assertGreater(self.report["summary"]["keypoints"], 100)
        self.assertGreater(self.report["summary"]["geometric_inlier_correspondences"], 0)
        metadata = json.loads((self.features / "feature_metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["input_run_id"], "portrait_run")
        self.assertEqual(metadata["coordinate_space"], "processed_image")
        self.assertEqual(len(metadata["input_transforms"]), 5)
        self.assertTrue(all((c["width"], c["height"]) == (240, 384) for c in metadata["camera_records"]))
        self.assertEqual(metadata["database_sha256"], digest(self.features / "database.db"))
        with (self.features / "keypoints.csv").open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                self.assertEqual(row["run_id"], "portrait_run")
                self.assertTrue(0 <= float(row["x_px"]) < 240)
                self.assertTrue(0 <= float(row["y_px"]) < 384)

    def test_export_only_retains_native_database_and_coordinates(self):
        target = self.parent / "reexport"
        shutil.copytree(self.features, target)
        before = {name: digest(target / name) for name in ("database.db", "keypoints.csv", "matches.csv", "candidate_tracks.csv")}
        with contextlib.redirect_stdout(io.StringIO()):
            run_features(self.input_dir, target, export_only=True)
        self.assertEqual(before, {name: digest(target / name) for name in before})
        timing = json.loads((target / "timing.json").read_text())
        self.assertIsNone(timing["feature_extraction_seconds"])
        self.assertIsNone(timing["matching_and_geometry_seconds"])

    def test_validator_rejects_changed_coordinate_and_preserves_db(self):
        target = self.parent / "corrupt_export"
        shutil.copytree(self.features, target)
        path = target / "keypoints.csv"
        lines = path.read_text().splitlines()
        fields = lines[1].split(",")
        fields[-2] = str(float(fields[-2]) + 5)
        lines[1] = ",".join(fields)
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        before = digest(target / "database.db")
        with self.assertRaisesRegex(ValueError, "coordinates differ"):
            validate_features(self.input_dir, target)
        self.assertEqual(digest(target / "database.db"), before)

    def test_input_hash_count_path_size_pts_and_status_rejection(self):
        mutations = [
            lambda m: m.update(status="failed"),
            lambda m: m.update(selected_frames=31),
            lambda m: m["frames"][0].update(path="../escape.jpg"),
            lambda m: m["frames"][0].update(width=1920),
            lambda m: m["frames"][1].update(source_pts_s=m["frames"][0]["source_pts_s"]),
            lambda m: m["frames"][1].update(source_pts_s=m["frames"][1]["source_pts_s"] + .1),
            lambda m: m["frames"][0].update(sha256="0" * 64),
        ]
        for index, mutation in enumerate(mutations):
            with self.subTest(index=index):
                target = self.parent / f"bad_input_{index}"
                shutil.copytree(self.input_dir, target)
                path = target / "manifest.json"
                manifest = json.loads(path.read_text(encoding="utf-8"))
                mutation(manifest)
                path.write_text(json.dumps(manifest), encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_feature_input(target)

    def test_legacy_input_does_not_require_31_frames_or_hd(self):
        target = self.parent / "legacy"
        target.mkdir()
        shutil.copytree(self.input_dir / "frames", target / "frames_first_test")
        source = load_feature_input(self.input_dir)
        manifest = {"runId": "legacy", "video": {"width": 240, "height": 384},
                    "camera": {}, "frames": [{"frameId": f["frameId"], "sourcePtsSeconds": f["sourcePtsSeconds"],
                    "sha256": f["sha256"], "selectedForFirstTest": True} for f in source["selected"]]}
        (target / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        actual = load_feature_input(target)
        self.assertEqual(actual["schema"], "legacy-phone-first-test")
        self.assertEqual(len(actual["selected"]), 5)
        self.assertEqual((actual["width"], actual["height"]), (240, 384))

    def test_existing_output_cli_rejected_without_changes(self):
        before = digest(self.features / "database.db")
        command = [sys.executable, "-B", str(ROOT / "scripts/phone_mapping/full_features.py"),
                   "--input-dir", str(self.input_dir), "--output-dir", str(self.features)]
        result = subprocess.run(command, cwd=self.parent, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(digest(self.features / "database.db"), before)

    def test_untextured_frames_fail_without_features_ready(self):
        target = self.parent / "blank_input"
        shutil.copytree(self.input_dir, target)
        path = target / "manifest.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        for row in manifest["frames"]:
            image_path = target / row["path"]
            Image.new("RGB", (240, 384), "white").save(image_path)
            row.update(sha256=digest(image_path), bytes=image_path.stat().st_size)
        path.write_text(json.dumps(manifest), encoding="utf-8")
        output = self.parent / "blank_features"
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
            run_features(target, output, threads=1, max_features=500)
        self.assertEqual(json.loads((output / "status.json").read_text())["status"], "failed")


if __name__ == "__main__":
    unittest.main()
