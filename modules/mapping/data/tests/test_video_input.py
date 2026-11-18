"""Exercise real decoding, PTS sampling, orientation and protected run outputs."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import wave

import av
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "modules/mapping/data"))
from video_input import VideoInputError, extract_video, geometry, presentation_time


def make_video(path, timestamps=None, rotation=0, mirror=False, codec="mpeg4"):
    timestamps = timestamps or list(range(0, 2000, 100))
    with av.open(str(path), "w", format="mp4") as container:
        stream = container.add_stream(codec, rate=10)
        stream.width, stream.height, stream.pix_fmt = 96, 64, "yuv420p"
        stream.time_base = stream.codec_context.time_base = Fraction(1, 1000)
        stream.set_display_rotation(rotation, hflip=mirror)
        for stamp in timestamps:
            image = Image.new("RGB", (96, 64), (20, 200, 20))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 47, 31), fill=(230, 10, 10))
            draw.rectangle((48, 32, 95, 63), fill=(10, 10, 230))
            frame = av.VideoFrame.from_image(image)
            frame.pts, frame.time_base = stamp, Fraction(1, 1000)
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)


class VideoInputTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="video-input-")
        self.addCleanup(self.temporary.cleanup)
        self.parent = Path(self.temporary.name)
        self.video = self.parent / "새 공간 영상.mp4"
        self.output = self.parent / "new run/input"

    def test_vfr_pts_sampling_and_full_decode_audit(self):
        make_video(self.video, [500, 600, 900, 1000, 1400, 1500, 1900, 2000, 2400])
        before = hashlib.sha256(self.video.read_bytes()).hexdigest()
        with av.open(str(self.video)) as container:
            pts = [float(f.pts * f.time_base) for f in container.decode(video=0)]
        result = extract_video(self.video, self.output, run_id="vfr")
        manifest = json.loads((self.output / "manifest.json").read_text(encoding="utf-8"))
        decisions = [json.loads(line) for line in (self.output / "frame_decisions.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertEqual(result["decoded_frames"], 9)
        self.assertEqual(result["selected_frames"], 4)
        self.assertEqual([row["source_pts_s"] for row in decisions], pts)
        self.assertEqual([r["decoded_frame_index"] for r in manifest["frames"]], [0, 3, 5, 7])
        self.assertEqual([r["timestamp_s"] for r in manifest["frames"]], [0, .5, 1, 1.5])
        self.assertEqual(sum(r["reason"] == "sampling_interval" for r in decisions), 5)
        self.assertEqual(manifest["source"]["sha256"], before)
        self.assertTrue(manifest["source"]["unchanged_after_decode"])
        self.assertEqual(hashlib.sha256(self.video.read_bytes()).hexdigest(), before)
        self.assertIsNone(manifest["metric_reference"])
        for row in manifest["frames"]:
            path = self.output / row["path"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), row["sha256"])
            with Image.open(path) as image:
                self.assertEqual(image.size, (96, 64))

    def test_display_rotation_resize_and_coordinate_transform(self):
        make_video(self.video, rotation=90)
        extract_video(self.video, self.output, run_id="portrait", max_width=32)
        manifest = json.loads((self.output / "manifest.json").read_text(encoding="utf-8"))
        row = manifest["frames"][0]
        self.assertEqual(row["rotation_applied_ccw_degrees"], 90)
        self.assertEqual((row["width"], row["height"]), (32, 48))
        self.assertEqual(row["coded_to_processed_pixel_centers"], [[0, .5, 0], [-.5, 0, 48], [0, 0, 1]])
        with Image.open(self.output / row["path"]) as image:
            red = image.getpixel((8, 36))
            self.assertGreater(red[0], 200)
            self.assertLess(red[1], 40)
        self.assertEqual(geometry(96, 64, 270, 0)[2], [[0, -1, 64], [1, 0, 0], [0, 0, 1]])

    def test_unspecified_h264_pixel_aspect_is_reported_as_unknown(self):
        # iPhone input uses FFmpeg's 0/1 sentinel, rather than None, for unknown SAR.
        make_video(self.video, codec="libx264")
        extract_video(self.video, self.output, run_id="unknown_sar")
        source = json.loads((self.output / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(source["source"]["raw_sample_aspect_ratio"], "0/1")
        self.assertIsNone(source["source"]["sample_aspect_ratio"])
        self.assertIn("sample_aspect_ratio_unknown", source["warnings"][0])

    def test_cli_from_another_cwd_and_existing_output_rejection(self):
        make_video(self.video)
        command = [sys.executable, "-B", str(ROOT / "modules/mapping/data/video_input.py"),
                   "--video", str(self.video), "--output", str(self.output), "--run-id", "cli"]
        first = subprocess.run(command, cwd=self.parent, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(json.loads(first.stdout)["selected_frames"], 4)
        manifest = (self.output / "manifest.json").read_bytes()
        second = subprocess.run(command, cwd=self.parent, capture_output=True, text=True)
        self.assertEqual(second.returncode, 2)
        self.assertEqual((self.output / "manifest.json").read_bytes(), manifest)

    def test_invalid_and_truncated_video_leave_failed_reports(self):
        valid = self.parent / "valid.mp4"
        make_video(valid)
        cases = [b"this is not a video", valid.read_bytes()[:len(valid.read_bytes()) // 2]]
        for index, data in enumerate(cases):
            with self.subTest(index=index):
                self.video.write_bytes(data)
                target = self.parent / f"broken{index}"
                with self.assertRaises(Exception):
                    extract_video(self.video, target, run_id=f"broken{index}")
                status = json.loads((target / "status.json").read_text(encoding="utf-8"))
                self.assertEqual(status["status"], "failed")
                self.assertIsNotNone(status["error"])
                self.assertTrue((target / "manifest.json").exists())
                self.assertTrue((target / "execution.log").exists())

    def test_audio_only_and_mirrored_input_are_explicit_failures(self):
        audio = self.parent / "audio.wav"
        with wave.open(str(audio), "wb") as stream:
            stream.setnchannels(1)
            stream.setsampwidth(2)
            stream.setframerate(8000)
            stream.writeframes(b"\x00\x00" * 8000)
        with self.assertRaisesRegex(VideoInputError, "no video stream"):
            extract_video(audio, self.output, run_id="audio")
        make_video(self.video, mirror=True)
        target = self.parent / "mirrored"
        with self.assertRaises(VideoInputError) as error:
            extract_video(self.video, target, run_id="mirror")
        self.assertEqual(error.exception.code, "unsupported_display_matrix")

    def test_short_input_preserves_partial_frames_without_ready_status(self):
        make_video(self.video)
        with self.assertRaises(VideoInputError) as error:
            extract_video(self.video, self.output, run_id="short", sample_fps=.1)
        self.assertEqual(error.exception.code, "insufficient_frames")
        manifest = json.loads((self.output / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(len(manifest["frames"]), 1)

    def test_unknown_or_reversed_pts_are_not_fabricated(self):
        for pts, base, previous in [(None, Fraction(1, 10), None), (1, None, None),
                                    (1, Fraction(1, 10), Fraction(2, 10)),
                                    (1, Fraction(1, 10), Fraction(1, 10))]:
            with self.subTest(pts=pts, base=base, previous=previous), self.assertRaises(VideoInputError):
                presentation_time(pts, base, previous)

    def test_invalid_settings_and_tracked_output_are_rejected(self):
        make_video(self.video)
        for options in [{"sample_fps": float("nan")}, {"sample_fps": 0},
                        {"run_id": "../escape"}, {"max_width": -1}, {"jpeg_quality": 101}]:
            with self.subTest(options=options), self.assertRaises(VideoInputError):
                extract_video(self.video, self.output, **options)
            self.assertFalse(self.output.exists())
        with self.assertRaises(VideoInputError):
            extract_video(self.video, ROOT / "data/mapping/never_write_here", run_id="protected")


if __name__ == "__main__":
    unittest.main()
