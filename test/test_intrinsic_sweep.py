#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from unittest.mock import Mock, patch
from pathlib import Path

from sensor_msgs.msg import Image


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/drive_intrinsic_calibration.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("drive_intrinsic_calibration", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class IntrinsicSweepTest(unittest.TestCase):
    def test_started_sweep_aborts_without_replay_after_world_loss(self):
        args = MODULE.parser().parse_args(["--simulation-endpoint", "/run/" + "a" * 32 + ".sock",
                                          "--target-id", "fixture"])
        controller = Mock()
        controller.set_view.side_effect = ConnectionError("world stopped during sweep")
        controller.set_pose.side_effect = ConnectionError("world still unavailable during restore")
        with patch.object(MODULE, "GazeboModelController", return_value=controller), \
                patch.object(MODULE.rospy, "is_shutdown", return_value=False), \
                patch.object(MODULE.rospy, "loginfo"), patch.object(MODULE.rospy, "logerr"):
            self.assertEqual(MODULE.run_calibration(args, Mock()), 2)
        self.assertEqual(controller.set_view.call_count, 1)
        self.assertEqual(controller.set_pose.call_count, 1)

    def test_progress_matches_ros_camera_calibration_ranges(self):
        samples = [(0.1, 0.1, 0.1, 0.0), (0.8, 0.8, 0.4, 0.5)]
        self.assertEqual(MODULE.calibration_progress(samples), (1.0, 1.0, 1.0, 1.0))

    def test_sample_distance_matches_ros_camera_calibration(self):
        samples = [(0.5, 0.5, 0.2, 0.0)]
        self.assertFalse(MODULE.is_new_sample((0.55, 0.55, 0.25, 0.0), samples))
        self.assertTrue(MODULE.is_new_sample((0.8, 0.5, 0.2, 0.0), samples))

    def test_bgr_image_with_row_padding_converts_without_cv_bridge(self):
        message = Image()
        message.width = 2
        message.height = 1
        message.encoding = "bgr8"
        message.step = 8
        message.data = bytes([0, 0, 255, 0, 255, 0, 99, 99])
        gray = MODULE.image_to_gray(message)
        self.assertEqual(gray.shape, (1, 2))
        self.assertGreater(int(gray[0, 0]), 70)
        self.assertGreater(int(gray[0, 1]), 140)

    def test_board_size_validation(self):
        self.assertEqual(MODULE.parse_board_size("7x5"), (7, 5))
        with self.assertRaises(Exception):
            MODULE.parse_board_size("7")


if __name__ == "__main__":
    unittest.main()
