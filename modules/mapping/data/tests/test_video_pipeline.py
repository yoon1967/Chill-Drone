"""Exercise subprocess lifecycle/report contracts; native video smoke runs are separate."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "modules/mapping/data"))
import run_video_pipeline as pipeline
REAL_SCRIPTS = pipeline.SCRIPTS.copy()

WORKER = '''
import argparse, hashlib, json
from pathlib import Path
import sys
p=argparse.ArgumentParser()
for flag in ('video','output','output-dir','input-dir','features-dir','run-id','result'):
    p.add_argument('--'+flag)
a,extra=p.parse_known_args()
name=Path(__file__).stem
out=Path(a.output or a.output_dir); out.mkdir(parents=True)
def write(name,data): (out/name).write_text(json.dumps(data),encoding='utf-8')
print('native stdout marker',flush=True)
print('native stderr marker',file=sys.stderr,flush=True)
if name=='input':
    run_id=a.run_id
    write('manifest.json',{'run_id':run_id,'source':{'filename':Path(a.video).name,'sha256':'fixture'}})
    status='frames_ready'
elif name=='features':
    source=json.loads((Path(a.input_dir)/'manifest.json').read_text())
    run_id=source['run_id']
    if run_id=='crash': sys.exit(7)
    write('feature_metadata.json',{'input_run_id':run_id})
    status='processing' if 'false_success' in run_id else 'features_ready'
elif name=='reconstruction':
    source=json.loads((Path(a.input_dir)/'manifest.json').read_text())
    run_id=source['run_id']
    partial='partial' in run_id
    status='partial' if partial else 'complete'
    quality={'input_frames':3,'submitted_frames':3,'registered_frames':2 if partial else 3,
             'registration_ratio':2/3 if partial else 1,'points3D':1,
             'unregistered_submitted_frames':['c.jpg'] if partial else [],
             'mean_reprojection_error_px':0.2,'components':[{'selected':True}], 'result_status':status}
    scene={'input_run_id':'other' if 'wrongrun' in run_id else run_id,
           'scale':{'unit':'arbitrary'},'quality':quality,
           'points':[[1,0,0,1]],'cameras':[{'frame_id':str(i)} for i in range(quality['registered_frames'])]}
    for name,value in [('scene.json',scene),('poses.json',{'cameras':scene['cameras']}),
                       ('quality.json',quality),('scale.json',scene['scale'])]: write(name,value)
    (out/'points.ply').write_text('ply fixture')
    (out/'trajectory.csv').write_text('frame_id')
    exports={name:hashlib.sha256((out/name).read_bytes()).hexdigest() for name in
             ('scene.json','poses.json','quality.json','scale.json','points.ply','trajectory.csv')}
    if 'bad_hash' in run_id: exports['points.ply']='wrong'
    write('provenance.json',{'outputs':exports})
else:
    result=Path(a.result)
    scene=json.loads((result/'scene.json').read_text())
    run_id=scene['input_run_id']
    if 'preview_crash' in run_id: sys.exit(9)
    for name in ('overview.png','projections.png'): (out/name).write_bytes(b'fixture image')
    write('validation.json',{'status':'passed'})
    write('manifest.json',{'input_run_id':run_id,'scene_sha256':hashlib.sha256((result/'scene.json').read_bytes()).hexdigest(),
        'unit':'arbitrary','result_status':scene['quality']['result_status'],
        'files':{name:{'sha256':hashlib.sha256((out/name).read_bytes()).hexdigest()} for name in
                 ('overview.png','projections.png','validation.json')}})
    status='preview_ready'
write('status.json',{'status':status})
write('timing.json',{'elapsed_seconds':0.01,'scope':'fixture worker'})
'''


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


class VideoPipelineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="pipeline-한글-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.video = self.root / "한글 영상.mp4"
        self.video.write_bytes(b"fixture input; not a real video")
        self.scripts = {}
        for name in pipeline.SCRIPTS:
            path = self.root / f"{name}.py"
            path.write_text(WORKER, encoding="utf-8")
            self.scripts[name] = path
        self.patcher = patch.object(pipeline, "SCRIPTS", self.scripts)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    def run_case(self, run_id, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return pipeline.run_pipeline(self.video, self.root / run_id, run_id=run_id, **kwargs)

    def test_complete_preserves_order_logs_bindings_and_timer_scope(self):
        result = self.run_case("normal")
        out = self.root / "normal"
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["exit_code"], 0)
        self.assertEqual([s["name"] for s in result["stages"]], ["input", "features", "reconstruction", "preview"])
        for stage in result["stages"]:
            log = (out / stage["log"]).read_text(encoding="utf-8")
            self.assertIn("native stdout marker", log)
            self.assertIn("native stderr marker", log)
            self.assertEqual(stage["status"], "complete")
            command = json.loads(log.splitlines()[0])["command"]
            self.assertEqual(command[:2], [sys.executable, "-B"])
        for artifact in result["artifacts"].values():
            self.assertFalse(Path(artifact["path"]).is_absolute())
            self.assertEqual(pipeline.sha256(out / artifact["path"]), artifact["sha256"])
        timing = read(out / "timing.json")
        self.assertGreater(timing["total_seconds"], sum(s["elapsed_seconds"] for s in timing["stages"]))
        self.assertEqual(set(timing["worker_timings"]), set(self.scripts))
        self.assertEqual(read(out / "status.json")["status"], "complete")
        events = [json.loads(line)["event"] for line in (out / "execution.log").read_text().splitlines()]
        self.assertEqual(events[0], "run_started")
        self.assertEqual(events[-1], "run_finished")
        self.assertEqual(self.video.read_bytes(), b"fixture input; not a real video")

    def test_partial_is_not_complete_and_cli_returns_three(self):
        args = ["run_video_pipeline.py", "--video", str(self.video), "--output", str(self.root / "partial"),
                "--run-id", "partial"]
        with patch.object(sys, "argv", args), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(pipeline.main(), 3)
        result = read(self.root / "partial/manifest.json")
        self.assertEqual(result["status"], "partial")
        self.assertEqual(result["summary"]["unregistered_submitted_frames"], ["c.jpg"])

    def test_worker_crash_stops_following_stage_preserves_prior_output(self):
        result = self.run_case("crash")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "features")
        self.assertEqual(result["stages"][1]["returncode"], 7)
        self.assertEqual(result["stages"][2]["status"], "not_started")
        self.assertTrue((self.root / "crash/input/manifest.json").is_file())
        self.assertFalse((self.root / "crash/reconstruction").exists())
        self.assertTrue((self.root / "crash/logs/pipeline_error.log").is_file())
        self.assertGreater(read(self.root / "crash/timing.json")["total_seconds"], 0)

    def test_preview_failure_preserves_successful_3d_files(self):
        result = self.run_case("preview_crash")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "preview")
        self.assertEqual(result["stages"][2]["status"], "complete")
        self.assertTrue((self.root / "preview_crash/reconstruction/scene.json").is_file())

    def test_exit_zero_with_wrong_status_is_rejected(self):
        result = self.run_case("false_success")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "features")
        self.assertIn("accepted status", result["error"]["reason"])

    def test_wrong_run_and_bad_final_hash_fail_finalization(self):
        for run_id in ("wrongrun", "bad_hash"):
            with self.subTest(run_id=run_id):
                result = self.run_case(run_id)
                self.assertEqual(result["status"], "failed")
                self.assertEqual(result["error"]["stage"], "finalization")
                self.assertTrue(all(s["status"] == "complete" for s in result["stages"]))

    def test_missing_video_has_preflight_report(self):
        result = pipeline.run_pipeline(self.root / "missing.mp4", self.root / "missing", run_id="missing")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "preflight")
        self.assertTrue(all(s["status"] == "not_started" for s in result["stages"]))
        self.assertTrue((self.root / "missing/timing.json").is_file())

    def test_unsafe_existing_output_and_invalid_settings_do_not_overwrite(self):
        self.run_case("saved")
        before = pipeline.sha256(self.root / "saved/manifest.json")
        with self.assertRaises(FileExistsError):
            self.run_case("saved")
        self.assertEqual(before, pipeline.sha256(self.root / "saved/manifest.json"))
        for kwargs in ({"sample_fps": float("nan")}, {"threads": 0}, {"max_width": -1}, {"jpeg_quality": 101}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.run_case("bad_settings", **kwargs)
        self.assertFalse((self.root / "bad_settings").exists())
        with self.assertRaises(ValueError):
            pipeline.run_pipeline(self.root / "future/video.mp4", self.root / "future")
        with self.assertRaises(ValueError):
            pipeline.run_pipeline(self.video, ROOT / "data/unsafe_pipeline_result")

    def test_default_output_auto_id_and_option_forwarding(self):
        with patch.object(pipeline, "ROOT", self.root), contextlib.redirect_stdout(io.StringIO()):
            result = pipeline.run_pipeline(self.video, sample_fps=3, max_width=800, threads=2, pair_window=7)
        out = self.root / "output/phone_mapping" / result["run_id"]
        self.assertEqual(result["status"], "complete")
        self.assertRegex(result["run_id"], r"^video_\d{8}T")
        command = json.loads((out / "logs/features.log").read_text(encoding="utf-8").splitlines()[0])["command"]
        self.assertEqual(command[command.index("--pair-window") + 1], "7")
        self.assertEqual(command[command.index("--threads") + 1], "2")

    def test_interruption_records_status_and_stops_next_stages(self):
        for index, side_effect in enumerate((KeyboardInterrupt(), 130)):
            with patch.object(pipeline, "run_command", side_effect=[side_effect]), contextlib.redirect_stdout(io.StringIO()):
                result = self.run_case(f"interrupted_{index}")
            self.assertEqual(result["status"], "interrupted")
            self.assertEqual(result["exit_code"], 130)
            self.assertEqual(result["stages"][1]["status"], "not_started")

    def test_cancelled_wait_terminates_worker(self):
        worker = Mock()
        worker.wait.side_effect = [KeyboardInterrupt(), 0]
        worker.poll.return_value = None
        with patch.object(pipeline.subprocess, "Popen", return_value=worker):
            with self.assertRaises(KeyboardInterrupt):
                pipeline.run_command(["fixture"], self.root / "cancel.log")
        worker.terminate.assert_called_once()
        worker.wait.assert_called_with(timeout=5)

    def test_interrupt_between_stages_keeps_finished_stage_complete(self):
        original_write = pipeline.write_json
        interrupted = False

        def interrupt_once(path, value):
            nonlocal interrupted
            if (not interrupted and path.name == "manifest.json" and value.get("status") == "running"
                    and value["stages"][0]["status"] == "complete"):
                interrupted = True
                raise KeyboardInterrupt()
            original_write(path, value)

        with patch.object(pipeline, "write_json", side_effect=interrupt_once):
            result = self.run_case("between_stages")
        self.assertEqual(result["status"], "interrupted")
        self.assertEqual(result["stages"][0]["status"], "complete")
        self.assertEqual(result["stages"][1]["status"], "not_started")

    def test_real_corrupt_video_stops_at_input_with_decoder_error(self):
        with patch.object(pipeline, "SCRIPTS", REAL_SCRIPTS):
            result = self.run_case("real_corrupt")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "input")
        self.assertEqual(result["stages"][1]["status"], "not_started")
        self.assertEqual(result["error"]["worker_status"]["status"], "failed")
        self.assertTrue((self.root / "real_corrupt/input/manifest.json").is_file())

    def test_real_textureless_video_stops_at_features(self):
        import av
        from PIL import Image
        with av.open(str(self.video), "w") as container:
            stream = container.add_stream("mpeg4", rate=2)
            stream.width, stream.height, stream.pix_fmt = 160, 120, "yuv420p"
            for _ in range(8):
                for packet in stream.encode(av.VideoFrame.from_image(Image.new("RGB", (160, 120)))):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
        with patch.object(pipeline, "SCRIPTS", REAL_SCRIPTS):
            result = self.run_case("real_no_features", threads=1)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "features")
        self.assertEqual(result["stages"][0]["worker_status"]["status"], "frames_ready")
        self.assertEqual(result["stages"][2]["status"], "not_started")

    def test_real_unrelated_frames_fail_reconstruction_and_keep_features(self):
        import av
        import numpy as np
        random = np.random.default_rng(16)
        with av.open(str(self.video), "w") as container:
            stream = container.add_stream("mpeg4", rate=2)
            stream.width, stream.height, stream.pix_fmt = 320, 240, "yuv420p"
            for _ in range(6):
                pixels = random.integers(0, 256, (240, 320, 3), dtype=np.uint8)
                for packet in stream.encode(av.VideoFrame.from_ndarray(pixels, format="rgb24")):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
        with patch.object(pipeline, "SCRIPTS", REAL_SCRIPTS):
            result = self.run_case("real_no_model", threads=1)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["stage"], "reconstruction")
        self.assertEqual(result["stages"][1]["worker_status"]["status"], "features_ready")
        self.assertEqual(result["stages"][2]["worker_status"]["status"], "failed")
        self.assertTrue((self.root / "real_no_model/features/database.db").is_file())
        self.assertFalse((self.root / "real_no_model/reconstruction/scene.json").exists())


if __name__ == "__main__":
    unittest.main()
