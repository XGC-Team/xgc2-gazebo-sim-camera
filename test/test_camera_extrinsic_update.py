"""Frozen estimates split optical pose once; publication owns no model movement."""
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

    def test_frozen_estimate_splits_the_model_owned_optical_offset_once(self):
        self.publisher._set_frozen_estimate(self.frozen)
        self.transforms.split_parent_to_optical_pose.assert_called_once_with([2, 3, 4], [0, 0, 0, 1], (.067, 0., 0.))
        self.assertEqual(tuple(self.publisher._translation), (2, 3, 3.933))
        self.assertEqual(self.sent, [])
        self.assertEqual(self.params, {})
        self.assertFalse(hasattr(self.publisher, '_model_state_client'))

    def test_invalid_optical_pose_does_not_replace_the_existing_estimate(self):
        self.transforms.split_parent_to_optical_pose.side_effect = ValueError('invalid optical pose')
        with self.assertRaisesRegex(ValueError, 'invalid optical pose'):
            self.publisher._set_frozen_estimate(self.frozen)
        self.assertEqual(self.publisher._translation, (0, 0, 0))
        self.assertEqual(self.params, {})

    def test_publisher_has_no_retired_hot_selection_or_confirmation_callbacks(self):
        for name in ('_refresh_estimate', '_apply_estimate', '_close_application'):
            self.assertFalse(hasattr(self.publisher, name))


if __name__ == '__main__': unittest.main()
