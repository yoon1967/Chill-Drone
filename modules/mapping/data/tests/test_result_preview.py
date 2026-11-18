"""Compare independent saved formats, then render the verified data without source media."""
import contextlib
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import pycolmap
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "modules/mapping/data"))
sys.path.insert(0, str(ROOT / "modules/mapping/localization"))
from reconstruct import export_result, write_json
from result_io import digest, validate_saved_result, EXPORTS
from render_preview import render_preview


class ResultPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="saved-result-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.result = self.root / "result"
        self.result.mkdir()
        options = pycolmap.SyntheticDatasetOptions()
        options.num_rigs, options.num_frames_per_rig, options.num_points3D = 1, 4, 40
        model = pycolmap.synthesize_dataset(options)
        frames = [{"frame_id": im.name, "source_pts_s": i * 0.5}
                  for i, im in enumerate(sorted(model.images.values(), key=lambda im: im.name))]
        run = {"input_mode": "video_features", "manifest": {"input_run_id": "synthetic_run"},
               "external": {"artifacts": {"database": {"sha256": "synthetic"}}}, "provenance": {"input_mode": "synthetic"}}
        with contextlib.redirect_stdout(io.StringIO()):
            export_result(model, run, self.root / "no_source_images", frames, self.result, 0.1,
                          [{"selected": True}], {"status": "synthetic"}, [])
        write_json(self.result / "status.json", {"status": "complete"})

    def rebind(self):
        p = self.result / "provenance.json"
        provenance = json.loads(p.read_text(encoding="utf-8"))
        provenance["outputs"] = {name: digest(self.result / name) for name in EXPORTS}
        write_json(p, provenance)

    def test_reload_without_source_images_and_after_directory_move(self):
        scene, report = validate_saved_result(self.result)
        self.assertEqual(report["points_checked"], 40)
        self.assertEqual(report["cameras_checked"], 4)
        moved = self.root / "relocated" / "result"
        moved.parent.mkdir()
        shutil.copytree(self.result, moved)
        scene2, report2 = validate_saved_result(moved)
        self.assertEqual(scene, scene2)
        self.assertEqual(report, report2)

    def test_raw_tamper_is_rejected_by_checksum(self):
        with (self.result / "points.ply").open("a") as stream:
            stream.write("0 0 0 0 0 0\n")
        with self.assertRaisesRegex(ValueError, "checksum"):
            validate_saved_result(self.result)

    def test_rebound_csv_tamper_is_rejected_by_coordinate_comparison(self):
        p = self.result / "trajectory.csv"
        lines = p.read_text(encoding="utf-8").splitlines()
        fields = lines[1].split(",")
        fields[2] = str(float(fields[2]) + 1)
        lines[1] = ",".join(fields)
        p.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.rebind()
        with self.assertRaisesRegex(ValueError, "Trajectory coordinates"):
            validate_saved_result(self.result)

    def test_rebound_nan_point_is_rejected(self):
        p = self.result / "scene.json"
        scene = json.loads(p.read_text(encoding="utf-8"))
        scene["points"][0][1] = float("nan")
        p.write_text(json.dumps(scene), encoding="utf-8")
        self.rebind()
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            validate_saved_result(self.result)

    def test_pngs_readable_bound_to_scene_and_no_overwrite(self):
        before = {name: digest(self.result / name) for name in EXPORTS}
        output = self.root / "preview"
        render_preview(self.result, output)
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["scene_sha256"], digest(self.result / "scene.json"))
        self.assertEqual(manifest["unit"], "arbitrary")
        for name in ("overview.png", "projections.png"):
            with Image.open(output / name) as image:
                self.assertEqual(image.format, "PNG")
                self.assertGreater(image.width, 1000)
                image.verify()
            self.assertEqual(manifest["files"][name]["sha256"], digest(output / name))
        self.assertEqual(before, {name: digest(self.result / name) for name in EXPORTS})
        with self.assertRaises(FileExistsError):
            render_preview(self.result, output)

    def test_failed_result_cannot_generate_preview(self):
        write_json(self.result / "status.json", {"status": "failed"})
        output = self.root / "failed_preview"
        with self.assertRaisesRegex(ValueError, "not complete or partial"):
            render_preview(self.result, output)
        self.assertFalse((output / "overview.png").exists())
        self.assertEqual(json.loads((output / "status.json").read_text())["status"], "failed")


if __name__ == "__main__":
    unittest.main()
