#!/usr/bin/env python3
"""Exercise the deployed world-camera intrinsic identity contract without ROS."""

import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _load_publisher_module():
    stubs = {
        "rospy": types.ModuleType("rospy"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "tf": types.ModuleType("tf"),
        "tf.transformations": types.ModuleType("tf.transformations"),
        "tf2_msgs": types.ModuleType("tf2_msgs"),
        "tf2_msgs.msg": types.ModuleType("tf2_msgs.msg"),
    }
    stubs["geometry_msgs.msg"].TransformStamped = object
    stubs["sensor_msgs.msg"].CameraInfo = object
    stubs["sensor_msgs.msg"].CompressedImage = object
    stubs["tf.transformations"].quaternion_from_euler = lambda *args: args
    stubs["tf2_msgs.msg"].TFMessage = object
    previous = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    try:
        spec = importlib.util.spec_from_file_location(
            "camera_contract_publisher_under_test",
            PACKAGE / "scripts/camera_contract_publisher.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


MODULE = _load_publisher_module()


def _document(camera_name="usb_cam", complete=True):
    value = {
        "schema": "xgc2.camera.intrinsic.v1",
        "camera_name": camera_name,
        "image_width": 3840,
        "image_height": 2160,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1332.0, 0.0, 1920.0, 0.0, 1332.0, 1080.0, 0.0, 0.0, 1.0],
        },
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0] * 5},
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
    }
    if complete:
        value["projection_matrix"] = {
            "rows": 3,
            "cols": 4,
            "data": [1332.0, 0.0, 1920.0, 0.0, 0.0, 1332.0, 1080.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        }
    return value


class CameraIntrinsicSelectionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.directory = self.root / "sim" / "usb_cam"
        self.directory.mkdir(parents=True)
        self.file = self.directory / "intrinsics-20260830T010203.000000Z.yaml"

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, document):
        self.file.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    def test_accepts_complete_matching_timestamped_document(self):
        self.write(_document())
        camera_matrix, distortion = MODULE._selected_intrinsics(
            str(self.file), (3840, 2160), "usb_cam"
        )
        self.assertEqual(len(camera_matrix), 9)
        self.assertEqual(len(distortion), 5)

    def test_rejects_invalid_camera_name_before_using_the_path(self):
        self.write(_document())
        for camera_name in ("../outside", "usb/cam", "usb cam", ".usb_cam"):
            with self.subTest(camera_name=camera_name), self.assertRaisesRegex(
                ValueError, "camera_name"
            ):
                MODULE._selected_intrinsics(str(self.file), (3840, 2160), camera_name)

    def test_rejects_incomplete_or_mismatched_documents(self):
        self.write(_document(complete=False))
        with self.assertRaises(ValueError):
            MODULE._selected_intrinsics(str(self.file), (3840, 2160), "usb_cam")
        self.write(_document(camera_name="other"))
        with self.assertRaisesRegex(ValueError, "camera_name"):
            MODULE._selected_intrinsics(str(self.file), (3840, 2160), "usb_cam")

    def test_rejects_the_physical_partition(self):
        physical = self.root / "phy" / "usb_cam" / self.file.name
        physical.parent.mkdir(parents=True)
        physical.write_text(
            yaml.safe_dump(_document(), sort_keys=False), encoding="utf-8"
        )
        with self.assertRaisesRegex(ValueError, "sim"):
            MODULE._selected_intrinsics(str(physical), (3840, 2160), "usb_cam")

    def test_rejects_a_symlink_that_resolves_outside_the_camera_partition(self):
        outside = self.root / "outside" / self.file.name
        outside.parent.mkdir()
        outside.write_text(
            yaml.safe_dump(_document(), sort_keys=False), encoding="utf-8"
        )
        selected = self.directory / "intrinsics-20260830T020304.000000Z.yaml"
        selected.symlink_to(outside)
        with self.assertRaises(ValueError):
            MODULE._selected_intrinsics(str(selected), (3840, 2160), "usb_cam")


if __name__ == "__main__":
    unittest.main()
