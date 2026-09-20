"""Saved estimates only update XGC TF; they never move Gazebo's truth camera."""
import sys
import threading
import types
import unittest
from unittest.mock import patch, Mock

from test_camera_intrinsic_selection import _load_publisher_module


class CameraExtrinsicUpdateTest(unittest.TestCase):
    def setUp(self):
        transforms = types.ModuleType('xgc_camera_calibration.transforms')
        transforms.split_parent_to_optical_pose = Mock(return_value={
            'parent_t_link': (2, 3, 3.933), 'parent_q_link_xyzw': (0, 0, 0, 1)})
        modules = patch.dict(sys.modules, {
            'xgc_camera_calibration': types.ModuleType('xgc_camera_calibration'),
            transforms.__name__: transforms})
        modules.start()
        self.addCleanup(modules.stop)
        self.transforms = transforms
        self.module = _load_publisher_module()
        self.publisher = self.module.CameraContractPublisher.__new__(self.module.CameraContractPublisher)
        self.publisher._pose_lock = threading.Lock()
        self.publisher._translation = (0, 0, 0)
        self.publisher._pose_rotation = (0, 0, 0, 1)
        self.params = {}
        self.module.rospy.set_param = self.params.__setitem__
        self.module.rospy.logwarn_throttle = lambda *args: None
        self.sent = []
        self.publisher._publish_transforms = lambda: self.sent.append(tuple(self.publisher._translation))
        self.frozen = {'resolvedOpticalPose': {'translation': [2, 3, 4], 'quaternionXyzw': [0, 0, 0, 1]},
                       'targetCoordinates': {'worldOffset': [10, 20, 30]}}

    def test_complete_tf_publish_precedes_application_confirmation(self):
        def tick(apply):
            self.assertEqual(self.sent, [])
            apply(self.frozen)
            self.assertEqual(self.sent, [(2, 3, 3.933)])
        self.publisher._application = types.SimpleNamespace(tick=tick)
        self.publisher._refresh_estimate()
        self.transforms.split_parent_to_optical_pose.assert_called_once_with([2, 3, 4], [0, 0, 0, 1], (.067, 0., 0.))
        self.assertEqual(self.params['~extrinsic_update_error'], '')
        # No ROS service/model movement path is involved in the actual callback.
        self.assertFalse(hasattr(self.publisher, '_model_state_client'))

    def test_failed_publish_restores_estimate_and_cannot_confirm(self):
        def fail(): raise OSError('TF transport failed')
        self.publisher._publish_transforms = fail
        def tick(apply):
            apply(self.frozen)
            self.fail('publish failure cannot reach confirmation')
        self.publisher._application = types.SimpleNamespace(tick=tick)
        self.publisher._refresh_estimate()
        self.assertEqual(self.publisher._translation, (0, 0, 0))
        self.assertEqual(self.params['~extrinsic_update_error'], 'TF transport failed')

    def test_standalone_intrinsic_camera_has_no_global_selection_consumer(self):
        self.publisher._application = None
        self.publisher._refresh_estimate()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.params, {})


if __name__ == '__main__': unittest.main()
