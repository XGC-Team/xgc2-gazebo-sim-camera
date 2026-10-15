#!/usr/bin/env python3
"""Exercise the deployed world-camera intrinsic identity contract without ROS."""

import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))


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
        "camera_peer": types.ModuleType("camera_peer"),
    }
    stubs["geometry_msgs.msg"].TransformStamped = object
    stubs["sensor_msgs.msg"].CameraInfo = object
    stubs["sensor_msgs.msg"].CompressedImage = object
    stubs["tf.transformations"].quaternion_from_euler = lambda *args: args
    stubs["tf2_msgs.msg"].TFMessage = object
    stubs["camera_peer"].CameraPeer = object
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
        "distortion_model": "plumb_bob",
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
    def configured(self, document, camera_name="usb_cam"):
        params = {"~camera_profiles_file":str(PACKAGE / "config/world_camera_profiles.yaml"),
                  "~camera_profile":"world_wide_4k30_110", "~camera_name":camera_name,
                  "~intrinsic_json":json.dumps(document) if document else ""}
        MODULE.rospy.get_param = lambda name, default=None: params.get(name, default)
        return MODULE._configured_intrinsics()

    def test_accepts_matching_pinned_document_without_a_calibration_file(self):
        width, height, matrix, distortion, model = self.configured(_document())
        self.assertEqual((width,height),(3840,2160))
        self.assertEqual(matrix,_document()["camera_matrix"]["data"])
        self.assertEqual(distortion,[0.]*5)
        self.assertEqual(model,"plumb_bob")

    def test_rejects_invalid_camera_identity(self):
        for name in ("../outside","usb/cam","usb cam",".usb_cam"):
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,"camera_name"):
                self.configured(_document(),name)
        with self.assertRaisesRegex(ValueError,"identity"):
            self.configured(_document("other"))

    def test_rejects_dimension_or_matrix_drift(self):
        document=_document();document["image_width"]=640
        with self.assertRaisesRegex(ValueError,"dimensions"):
            self.configured(document)
        document=_document();document["camera_matrix"]["data"]=[]
        with self.assertRaisesRegex(ValueError,"data"):
            self.configured(document)

    def test_rejects_invalid_focal_length_and_distortion_model(self):
        document=_document();document["camera_matrix"]["data"][0]=0
        with self.assertRaises(ValueError):self.configured(document)
        document=_document();document["distortion_model"]="unknown"
        with self.assertRaises(ValueError):self.configured(document)

    def test_uncalibrated_camera_keeps_product_profile_intrinsics(self):
        width,height,matrix,distortion,model=self.configured(None)
        self.assertEqual((width,height),(3840,2160))
        self.assertGreater(matrix[0],0)
        self.assertEqual(distortion,[0.]*5)
        self.assertEqual(model,"plumb_bob")


if __name__ == "__main__":
    unittest.main()
