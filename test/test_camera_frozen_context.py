"""World metadata publishes one frozen TF chain before its Record fact."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_camera_intrinsic_selection import _load_publisher_module
from xgc_camera_calibration.extrinsic_resolver import encode_frozen, resolve_selection
from xgc_camera_calibration.transforms import _rpy_matrix, quaternion_to_rotation_matrix
from xgc_camera_calibration.solver import rotation_matrix_to_quaternion

ROLES = {"parentFrame": "world", "opticalFrames": {
    "sim": "xgc_world_camera_optical_frame", "phy": "usb_cam_optical_frame"}}


class Transform:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.transform = SimpleNamespace(translation=SimpleNamespace(x=0, y=0, z=0),
            rotation=SimpleNamespace(x=0, y=0, z=0, w=1))


class NoThread:
    def __init__(self, **_): pass
    def start(self): pass
    def join(self): pass


class NoPeer:
    def __init__(self, *_): pass
    def close(self): pass


class FrozenWorldCameraTest(unittest.TestCase):
    def make_publisher(self, calibrated, placement=None):
        module = _load_publisher_module()
        with tempfile.TemporaryDirectory() as root:
            choice = {"mode": "auto"}
            if calibrated:
                choice = {"mode": "pose", "pose": {"convention": "world_T_camera_optical",
                    "translation": [1, 2, 3], "quaternionXyzw": [0, 0, 0, 1],
                    "coordinates": {"schemaVersion": 1, "kind": "experiment-world",
                    "frame": "world", "savedWorldOffset": [4, 5, 6]}}}
            frozen = resolve_selection(root, "world", choice,
                {"frame": "world", "worldOffset": [1, 2, 3]}, ROLES, "world-camera-test")
        params = {"~optical_frame": ROLES["opticalFrames"]["sim"], "~parent_frame": "world",
            "~camera_link_frame": "xgc_world_camera_link", "~x": 7., "~y": 8., "~z": 9.,
            "~roll": 0., "~pitch": 0., "~yaw": 0., "~output_transform_topic": "/xgc/camera/world/tf",
            "~resolved_extrinsic_json": encode_frozen(frozen), "~frame_roles_json": json.dumps(ROLES),
            "~camera_name": "world", "~media_control_endpoint": "/private/camera.sock",
            "~media_control_target_id": "fixture", "~placement_receipt": json.dumps(placement or {})}
        events = []
        def publisher(topic, *_args, **_kwargs):
            return SimpleNamespace(publish=lambda message: events.append((topic, message)))
        module.rospy.get_param = lambda name, default=None: params.get(name, default)
        module.rospy.Publisher = publisher
        module.rospy.Time = SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 1234))
        module.rospy.on_shutdown = lambda _: None
        module.rospy.logwarn_throttle = lambda *_: None
        module.TransformStamped = Transform
        module.TFMessage = lambda **kwargs: SimpleNamespace(**kwargs)
        module.quaternion_from_euler = lambda *values: rotation_matrix_to_quaternion(_rpy_matrix(*values))
        string_module = ModuleType("std_msgs.msg")
        string_module.String = lambda **kwargs: SimpleNamespace(**kwargs)
        with patch.object(module, "_configured_intrinsics", return_value=(640, 480, [1]*9, [0]*5, "plumb_bob")), \
                patch.object(module, "CameraPeer", NoPeer), patch.object(module.threading, "Thread", NoThread), \
                patch.dict(sys.modules, {"std_msgs.msg": string_module}), \
                patch("xgc_camera_calibration.extrinsic_resolver.SelectionStore", side_effect=AssertionError("runtime read global selection")), \
                patch("xgc_camera_calibration.extrinsic_resolver.read_version", side_effect=AssertionError("runtime read history")):
            runtime = module.CameraContractPublisher()
        self.addCleanup(runtime.close)
        return runtime, frozen, events

    def test_selected_pose_and_record_fact_use_same_complete_chain_and_frozen_resolution(self):
        runtime, frozen, events = self.make_publisher(True)
        self.assertEqual([topic for topic, _ in events], ["/xgc/camera/world/tf", "/xgc/record_facts"])
        transforms = events[0][1].transforms
        fact = json.loads(events[1][1].data)
        self.assertEqual(fact["values"]["resolvedExtrinsic"], frozen)
        self.assertEqual(fact["effectiveAt"]["rosTimeNs"], "1234")
        self.assertTrue(fact["values"]["calibrated"])
        chain = fact["values"]["transforms"]
        self.assertEqual(len(chain), 2)
        self.assertEqual(chain[0]["parentFrame"], "world")
        self.assertEqual(chain[0]["childFrame"], chain[1]["parentFrame"])
        self.assertEqual(chain[1]["childFrame"], ROLES["opticalFrames"]["sim"])
        rotation = quaternion_to_rotation_matrix(chain[0]["quaternionXyzw"])
        optical = rotation.dot(chain[1]["translation"]) + chain[0]["translation"]
        for actual, expected in zip(optical, frozen["resolvedOpticalPose"]["translation"]):
            self.assertAlmostEqual(actual, expected)
        self.assertIs(transforms[0].header.stamp, transforms[1].header.stamp)
        self.assertEqual(fact["provenance"]["selectionConfirmation"], "owned-by-SelectionStore-not-this-fact")
        runtime._publish_transforms()
        self.assertEqual(len(events), 3)  # Same frozen values publish TF, without a second applied fact.

    def test_uncalibrated_native_pose_does_not_claim_saved_calibration(self):
        _, frozen, events = self.make_publisher(False)
        fact = json.loads(events[1][1].data)
        self.assertEqual(frozen["status"], "uncalibrated")
        self.assertNotIn("resolvedOpticalPose", frozen)
        self.assertFalse(fact["values"]["calibrated"])
        self.assertEqual(fact["values"]["transforms"][0]["translation"], [7, 8, 9])

    def test_close_removes_only_the_owned_world_instance_and_entity_generation_once(self):
        receipt = {"world": {"service": "xgc2.simulation", "api_version": "v1", "profile": "http.v1",
            "target_id": "fixture", "instance_id": "1" * 32,
            "endpoint": {"kind": "unix", "address": "/granted/world.sock"}},
            "entity": {"entities": [{"ref": {"id": "gazebo_static_camera", "generation": 7}}]}}
        sdk = ModuleType("xgc2_xrpc")
        sdk.Runtime, sdk.ServiceRef = Mock(), Mock()
        reference = SimpleNamespace(target_id="fixture")
        sdk.ServiceRef.from_dict.return_value.validate.return_value = reference
        simulation = ModuleType("xgc2_gazebo_scene.simulation_client")
        simulation.SimulationClient = Mock()
        with patch.dict(sys.modules, {sdk.__name__: sdk, simulation.__name__: simulation}):
            runtime, _, _ = self.make_publisher(False, receipt)
            runtime.close()
            runtime.close()
            sdk.ServiceRef.from_dict.assert_called_once_with(receipt["world"])
            sdk.ServiceRef.from_dict.return_value.validate.assert_called_once_with()
            simulation.SimulationClient.assert_called_once_with(reference,
                runtime=sdk.Runtime.return_value, local_target="fixture", timeout=3)
            simulation.SimulationClient.return_value.operation.assert_called_once_with("/v1/entities/gazebo_static_camera",
                {"generation": 7, "operation_timeout_ms": 3000}, method="DELETE")
            simulation.SimulationClient.return_value.close.assert_called_once_with()
            sdk.Runtime.return_value.close.assert_called_once_with()

    def test_tf_send_failure_never_emits_record_success(self):
        runtime, _, events = self.make_publisher(False)
        before = len(events)
        def fail(_): raise RuntimeError("TF publication failed")
        runtime._transform_publisher.publish = fail
        with self.assertRaisesRegex(RuntimeError, "TF publication failed"):
            runtime._publish_transforms()
        self.assertEqual(len(events), before)


if __name__ == "__main__":
    unittest.main()
