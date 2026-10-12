#!/usr/bin/env python3
"""Placement contract and models captured from the base launch, without Gazebo."""
import contextlib
import importlib.machinery
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "scripts/place_world_camera"
loader = importlib.machinery.SourceFileLoader("place_world_camera", str(CLI))
spec = importlib.util.spec_from_loader(loader.name, loader)
placement_cli = importlib.util.module_from_spec(spec)
loader.exec_module(placement_cli)
REQUIRED = ["--world-endpoint", "/granted/world.sock", "--target-id", "fixture",
            "--media-control-endpoint", "/granted/camera.sock",
            "--media-control-target-id", "fixture", "--media-rtp-port", "26417"]


def normalized_xml(document):
    root = ET.fromstring(document)
    for element in root.iter():
        element.text = (element.text or "").strip() or None
        element.tail = None
        attributes = sorted(element.attrib.items())
        element.attrib.clear()
        element.attrib.update(attributes)
    return ET.tostring(root, encoding="unicode")


class PlacementTest(unittest.TestCase):
    def invoke(self, arguments, failure=None, acknowledged=None):
        calls = []
        package = types.ModuleType("xgc2_gazebo_scene")
        package.__path__ = []
        module = types.ModuleType("xgc2_gazebo_scene.placement")
        result = {"entities": [{"ref": {"id": "returned-camera", "generation": 7}}]}

        def place_entity(endpoint, target_id, entity, timeout):
            calls.append((endpoint, target_id, entity, timeout))
            if failure:
                raise RuntimeError(failure)
            if acknowledged is not None:
                result.update(acknowledged(entity))
            return result, "1" * 32

        module.place_entity = place_entity
        stdout, stderr = io.StringIO(), io.StringIO()
        injected = {package.__name__: package, module.__name__: module}
        previous = {name: sys.modules.get(name) for name in injected}
        sys.modules.update(injected)
        try:
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = placement_cli.main(arguments)
        finally:
            for name, value in previous.items():
                if value is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = value
        return code, stdout.getvalue(), stderr.getvalue(), calls, result

    def test_six_models_equal_the_base_launch_description_parameter(self):
        reference = json.loads((ROOT / "test/placement_reference/base_launch.json").read_text())
        self.assertTrue(reference["base_commit"].startswith("14bfb28"))
        self.assertGreaterEqual(len(reference["cases"]), 6)
        for row in reference["cases"]:
            with self.subTest(case=row["name"]):
                args = REQUIRED.copy()
                for name, value in row["arguments"].items():
                    if value == "@profiles@":
                        value = str(ROOT / "test/placement_reference/profiles.yaml")
                    args.extend(["--" + name.replace("_", "-"), value])
                code, stdout, stderr, calls, result = self.invoke(args)
                self.assertEqual(code, 0, stderr)
                self.assertEqual(stderr, "")
                self.assertEqual(len(calls), 1)
                entity = calls[0][2]
                self.assertEqual(entity["role"], "sensor")
                self.assertEqual(entity["asset"]["id"], entity["id"] + "-definition")
                self.assertEqual(entity["asset"]["realization"]["media_type"], "application/urdf+xml")
                self.assertEqual(normalized_xml(entity["asset"]["realization"]["content"]), row["normalized_urdf"])
                self.assertEqual(json.loads(stdout), {"schema": "xgc2.placement.v1", "kind": "camera",
                    "id": entity["id"], "entity": result, "instance": "1" * 32})
                self.assertEqual(len(stdout.splitlines()), 1)

    def test_pose_and_remaining_whole_run_budget_reach_the_shared_helper(self):
        code, _, stderr, calls, _ = self.invoke(REQUIRED + ["--model-name", "yard_camera",
            "--x", "1", "--y", "-2", "--z", "3", "--roll", "0.2", "--pitch", "-0.4", "--yaw", "0.6", "--timeout", "3"])
        self.assertEqual(code, 0, stderr)
        endpoint, target, entity, remaining = calls[0]
        self.assertEqual((endpoint, target), ("/granted/world.sock", "fixture"))
        self.assertEqual(entity["pose"]["position"], [1, -2, 3])
        sys.path.append(placement_cli.ROS_PYTHON)
        from tf.transformations import quaternion_from_euler
        for actual, expected in zip(entity["pose"]["orientation"], quaternion_from_euler(0.2, -0.4, 0.6)):
            self.assertAlmostEqual(actual, expected)
        self.assertGreater(remaining, 0)
        self.assertLess(remaining, 3)

    def frozen_arguments(self, root, choice):
        from xgc_camera_calibration.extrinsic_resolver import encode_frozen, resolve_selection
        roles = {"parentFrame": "world", "opticalFrames": {
            "sim": "xgc_world_camera_optical_frame", "phy": "usb_cam_optical_frame"}}
        frozen = resolve_selection(root, "world", choice,
            {"frame": "world", "worldOffset": [10, -3, 1]}, roles, "placement-test")
        payload = encode_frozen(frozen)
        return REQUIRED + ["--camera-name", "world", "--media-source-id", "world",
            "--optical-frame", roles["opticalFrames"]["sim"],
            "--resolved-extrinsic-json", payload, "--frame-roles-json", json.dumps(roles)], frozen

    @staticmethod
    def ready_entity(entity):
        return {"entities": [{"ref": {"id": entity["id"], "generation": 7},
                             "lifecycle": "ready", "state": {"pose": entity["pose"]}}]}

    def test_uncalibrated_context_uses_acknowledged_pose_without_saving_calibration(self):
        with tempfile.TemporaryDirectory() as root:
            arguments, frozen = self.frozen_arguments(root, {"mode": "auto"})
            def acknowledged(entity):
                result = self.ready_entity(entity)
                result["entities"][0]["state"]["pose"] = {
                    "position": [7, 8, 9], "orientation": [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]}
                return result
            code, stdout, stderr, calls, _ = self.invoke(arguments, acknowledged=acknowledged)
            self.assertEqual(code, 0, stderr)
            context = json.loads(stdout)["cameraContext"]
            self.assertEqual(json.loads(context["resolvedExtrinsicJson"]), frozen)
            self.assertEqual(frozen["status"], "uncalibrated")
            self.assertNotIn("resolvedOpticalPose", frozen)
            self.assertEqual(context["appliedLinkPose"]["x"], 7)
            self.assertAlmostEqual(context["appliedOpticalPose"]["translation"][1], 8.067)
            self.assertEqual(context["cameraInfoTopic"], "/xgc/camera/world/camera_info")
            self.assertNotIn("imageTopic", context)
            self.assertNotIn("applicationStateParam", context)
            self.assertEqual(len(calls), 1)
            self.assertFalse((Path(root) / "selections").exists())

    def test_selected_optical_pose_rebases_once_and_places_matching_camera_link(self):
        from xgc_camera_calibration.transforms import quaternion_to_rotation_matrix, link_to_optical_rotation
        with tempfile.TemporaryDirectory() as root:
            arguments, frozen = self.frozen_arguments(root, {"mode": "pose", "pose": {
                "convention": "world_T_camera_optical", "translation": [2, -1, 3],
                "quaternionXyzw": [0, 0, 0, 1], "coordinates": {"schemaVersion": 1,
                "kind": "experiment-world", "frame": "world", "savedWorldOffset": [13, -5, 2]}}})
            code, stdout, stderr, calls, _ = self.invoke(arguments, acknowledged=self.ready_entity)
            self.assertEqual(code, 0, stderr)
            context = json.loads(stdout)["cameraContext"]
            self.assertEqual(json.loads(context["resolvedExtrinsicJson"]), frozen)
            self.assertIn("appliedOpticalPose", context)
            pose = calls[0][2]["pose"]
            rotation = quaternion_to_rotation_matrix(pose["orientation"])
            optical = rotation.dot((0.067, 0, 0)) + pose["position"]
            for actual, expected in zip(optical, frozen["resolvedOpticalPose"]["translation"]):
                self.assertAlmostEqual(actual, expected)
            self.assertTrue((abs(rotation.dot(link_to_optical_rotation()) - quaternion_to_rotation_matrix(
                frozen["resolvedOpticalPose"]["quaternionXyzw"])) < 1e-10).all())
            self.assertFalse((Path(root) / "selections").exists())

    def test_frozen_identity_drift_sends_nothing_and_invalid_acknowledgment_is_not_replayed(self):
        with tempfile.TemporaryDirectory() as root:
            arguments, _ = self.frozen_arguments(root, {"mode": "auto"})
            code, stdout, _, calls, _ = self.invoke(arguments + ["--camera-name", "other"])
            self.assertEqual((code, stdout, calls), (2, "", []))
            code, stdout, stderr, calls, _ = self.invoke(arguments)
            self.assertEqual((code, stdout, len(calls)), (1, "", 1))
            self.assertIn("acknowledge", stderr)

    def test_validation_errors_send_nothing_and_have_one_diagnostic(self):
        cases = [REQUIRED[:-2]]
        for name, value in (("world-endpoint", "relative.sock"), ("media-control-endpoint", "relative.sock"),
                            ("world-endpoint", "/tmp/../world.sock"), ("world-endpoint", "//world.sock"),
                            ("world-endpoint", "/" + "a" * 108), ("media-rtp-port", "0"),
                            ("media-rtp-port", "65536"), ("timeout", "0"), ("timeout", "55.1"),
                            ("timeout", "nan"), ("x", "inf"), ("static", "yes"),
                            ("model-name", "bad name"), ("media-rtp-host", "192.0.2.1"),
                            ("width", "15"), ("height", "2161"), ("fps", "0"), ("hfov-degrees", "180"),
                            ("far-clip", "0.01"), ("noise-stddev", "-1"), ("media-bitrate", "100"),
                            ("media-max-bitrate", "128000"), ("media-pacing-bitrate", "128000"),
                            ("media-vbv-buffer-milliseconds", "49"), ("snapshot-jpeg-quality", "49"),
                            ("encoded-publisher-queue-capacity", "129"), ("camera-profile", "absent"),
                            ("width", "invalid"), ("snapshot-jpeg-backend", "invalid"), ("unknown", "value"),
                            ("media-rtp-p", "20001")):
            cases.append(REQUIRED + ["--" + name, value])
        for args in cases:
            with self.subTest(arguments=args):
                code, stdout, stderr, calls, _ = self.invoke(args)
                self.assertEqual(code, 2)
                self.assertEqual(stdout, "")
                self.assertEqual(calls, [])
                self.assertEqual(len(stderr.splitlines()), 1)
                self.assertTrue(stderr.startswith("place_world_camera: "))
                self.assertNotIn("Traceback", stderr)

    def test_world_failure_is_not_replayed_and_names_the_entity(self):
        for failure in ("conflict: already exists", "creation refused", "world timeout"):
            with self.subTest(failure=failure):
                code, stdout, stderr, calls, _ = self.invoke(REQUIRED, failure)
                self.assertEqual(code, 1)
                self.assertEqual(stdout, "")
                self.assertEqual(len(calls), 1)
                self.assertEqual(stderr, "place_world_camera: gazebo_static_camera: " + failure + "\n")

    def test_rendering_time_is_charged_before_any_placement_call(self):
        with patch.object(placement_cli.time, "monotonic", side_effect=[100, 151]):
            code, stdout, stderr, calls, _ = self.invoke(REQUIRED)
        self.assertEqual(code, 2)
        self.assertEqual(stdout, "")
        self.assertEqual(calls, [])
        self.assertIn("deadline", stderr)

    def test_help_runs_directly_with_minimal_environment_without_placement(self):
        result = subprocess.run([str(CLI), "--help"], input="unused stdin\n", text=True,
                                env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"},
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--media-rtp-port", result.stdout)
        self.assertEqual(result.stderr, "")
        self.assertTrue(os.access(CLI, os.X_OK))
        # Even an explicitly unavailable helper must not be imported for help.
        with patch.dict(sys.modules, {"xgc2_gazebo_scene.placement": None}), \
                contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as exit:
            placement_cli.main(["--help"])
        self.assertEqual(exit.exception.code, 0)

    def test_placement_renders_with_minimal_environment_and_ignores_stdin(self):
        bootstrap = """import runpy, sys, types
package = types.ModuleType('xgc2_gazebo_scene')
package.__path__ = []
module = types.ModuleType('xgc2_gazebo_scene.placement')
def place_entity(endpoint, target, entity, timeout):
    assert endpoint == '/granted/world.sock' and target == 'fixture'
    assert 0 < timeout < 50
    return entity, '11111111111111111111111111111111'
module.place_entity = place_entity
sys.modules[package.__name__] = package
sys.modules[module.__name__] = module
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
"""
        result = subprocess.run(["/usr/bin/python3", "-c", bootstrap, str(CLI), *REQUIRED],
                                input="unused stdin\n", text=True,
                                env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"},
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        self.assertEqual(len(result.stdout.splitlines()), 1)
        self.assertEqual(json.loads(result.stdout)["entity"]["role"], "sensor")


if __name__ == "__main__":
    unittest.main()
