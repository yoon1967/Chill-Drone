import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from onnx_depth import MODELS, DepthModel, prepare_bgr

DA = "da_v2_metric_indoor_small"  # fp16 inside
DA32 = "da_v2_metric_indoor_small_fp32"


def reference(kind):
    return Path(MODELS[kind].path.replace(".onnx", ".reference.npz"))


@unittest.skipUnless(Path(MODELS[DA].path).is_file(), "DA-V2 ONNX not exported (export_da2_onnx.py)")
class DepthAnythingTests(unittest.TestCase):
    def test_preprocessing_uses_imagenet_normalization_and_518x686(self):
        image = np.zeros((720, 960, 3), dtype=np.uint8)
        image[:, :, 2] = 255  # BGR red
        tensor = prepare_bgr(image, MODELS[DA])
        self.assertEqual(tensor.shape, (1, 3, 518, 686))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertAlmostEqual(float(tensor[0, 0, 5, 5]), (1 - .485) / .229, places=4)
        self.assertAlmostEqual(float(tensor[0, 1, 5, 5]), -.456 / .224, places=4)

    def test_onnx_matches_pytorch_reference_from_export(self):
        # fp32 on CPU must match PyTorch fp32; fp16 on CUDA must match PyTorch fp16 (half rounding).
        cases = [(DA32, "cpu", 1e-3), (DA, "cuda", 1e-2)]
        for kind, device, tolerance in cases:
            with self.subTest(kind=kind):
                if not reference(kind).is_file() or not Path(MODELS[kind].path).is_file():
                    self.skipTest(f"{kind} export missing")
                try:
                    model = DepthModel(kind=kind, device=device)
                except RuntimeError as error:  # e.g. no --gpus
                    self.skipTest(str(error))
                expected = np.load(reference(kind))
                actual = model.session.run(None, {model.input.name: expected["input"]})[0]
                relative = np.abs(actual - expected["depth"]) / expected["depth"]
                self.assertLess(float(np.percentile(relative, 99)), tolerance)

    def test_fp16_matches_fp32_on_cuda(self):
        try:
            half, full = DepthModel(kind=DA, device="cuda"), DepthModel(kind=DA32, device="cuda")
        except (RuntimeError, FileNotFoundError) as error:
            self.skipTest(str(error))
        image = np.random.default_rng(9).integers(0, 256, (720, 960, 3), dtype=np.uint8)
        a, _ = full.infer(image)
        b, _ = half.infer(image)
        self.assertLess(float(np.percentile(np.abs(b - a) / a, 99)), 1e-2)

    def test_metric_output_is_resized_to_camera_resolution(self):
        rng = np.random.default_rng(5)
        depth, _ = DepthModel(kind=DA32).infer(rng.integers(0, 256, (720, 960, 3), dtype=np.uint8))
        self.assertEqual(depth.shape, (480, 640))
        self.assertTrue(np.all(np.isfinite(depth) & (depth > 0) & (depth <= 20)))

    def test_hash_is_checked_per_model(self):
        with self.assertRaisesRegex(ValueError, DA):
            DepthModel(MODELS["sc_depth_v2_nyu"].path, kind=DA)

    def test_unknown_model_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "model must be one of"):
            DepthModel(kind="midas")


class OnnxTests(unittest.TestCase):
    def test_preprocessing_uses_rgb_and_training_normalization(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        image[:, :, 0] = 255  # BGR blue
        tensor = prepare_bgr(image)
        self.assertEqual(tensor.shape, (1, 3, 480, 640))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertAlmostEqual(float(tensor[0, 0, 0, 0]), -2, places=5)
        self.assertAlmostEqual(float(tensor[0, 2, 0, 0]), (1 - .45) / .225, places=5)

    def test_invalid_image_is_rejected(self):
        with self.assertRaises(ValueError):
            prepare_bgr(np.zeros((480, 640), dtype=np.uint8))

    def test_wrong_model_is_rejected_before_loading(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "wrong.onnx"
            path.write_bytes(b"not the verified model")
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                DepthModel(path)

    def test_real_model_produces_valid_relative_depth(self):
        depth, elapsed = DepthModel().infer(np.full((480, 640, 3), 127, dtype=np.uint8))
        self.assertEqual(depth.shape, (480, 640))
        self.assertTrue(np.all(np.isfinite(depth) & (depth > 0)))
        self.assertGreater(elapsed, 0)

    def test_unknown_device_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "cpu or cuda"):
            DepthModel(device="tensorrt")

    def test_cuda_matches_cpu_and_reports_its_provider(self):
        import onnxruntime
        if "CUDAExecutionProvider" not in onnxruntime.get_available_providers():
            self.skipTest("onnxruntime build without CUDA provider")
        try:
            gpu = DepthModel(device="cuda")
        except Exception as error:  # e.g. container started without --gpus
            self.skipTest(f"CUDA session unavailable: {error}")
        cpu = DepthModel(device="cpu")
        self.assertEqual(gpu.provider, "CUDAExecutionProvider")
        self.assertEqual(cpu.provider, "CPUExecutionProvider")
        rng = np.random.default_rng(7)
        image = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
        expected, _ = cpu.infer(image)
        actual, _ = gpu.infer(image)
        relative = np.abs(actual - expected) / expected
        self.assertLess(float(np.percentile(relative, 99)), 1e-2)

    def test_ros_node_preserves_stamp_and_uses_expected_topics(self):
        processes = []
        try:
            processes.append(subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            for _ in range(40):
                if subprocess.run(["rosparam", "list"], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL).returncode == 0:
                    break
                time.sleep(.25)
            else:
                self.fail("ROS master not ready")
            processes.append(subprocess.Popen([sys.executable, "/opt/mona_runtime/onnx_depth.py"],
                                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            rospy.init_node("test_onnx_depth", disable_signals=True)
            depths, colors = [], []
            rospy.Subscriber("/sc_depth_pl/depth/image_raw", Image, depths.append)
            rospy.Subscriber("/sc_depth_pl/color/image_raw", Image, colors.append)
            publisher = rospy.Publisher("/tello/image_raw", Image, queue_size=1)
            for _ in range(100):
                if publisher.get_num_connections():
                    break
                time.sleep(.1)
            self.assertGreater(publisher.get_num_connections(), 0)
            time.sleep(.3)
            image = CvBridge().cv2_to_imgmsg(np.full((480, 640, 3), 127, dtype=np.uint8), "rgb8")
            image.header.stamp = rospy.Time(123, 456)
            image.header.frame_id = "camera_link"
            publisher.publish(image)
            for _ in range(100):
                if depths and colors:
                    break
                time.sleep(.1)
            self.assertTrue(depths and colors, "node did not produce synchronized RGB/depth")
            self.assertEqual(depths[0].encoding, "32FC1")
            self.assertEqual(depths[0].header.stamp, image.header.stamp)
            self.assertEqual(colors[0].header.stamp, image.header.stamp)
            self.assertEqual(colors[0].encoding, "bgr8")
            self.assertEqual(depths[0].header.frame_id, "camera_link")
        finally:
            for process in reversed(processes):
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()


if __name__ == "__main__":
    unittest.main()
