"""Saved estimate updates must leave the simulation truth camera independent."""
import sys
import threading
import types
import unittest
from unittest.mock import patch, Mock
from pathlib import Path

from test_camera_intrinsic_selection import _load_publisher_module



class CameraExtrinsicUpdateTest(unittest.TestCase):
    def setUp(self):
        # The calibration owner separately tests SE(3) arithmetic. Here exercise
        # the publisher's real admission/error/update path at that package boundary.
        transforms = types.ModuleType('xgc_camera_calibration.transforms')
        transforms.split_parent_to_optical_pose = Mock(return_value={
            'parent_t_link':(2, 3, 3.933), 'parent_q_link_xyzw':(0, 0, 0, 1)})
        coordinates = types.ModuleType('xgc_camera_calibration.extrinsic_coordinates')
        coordinates.optical_translation_in_world = Mock(return_value=(2, 3, 4))
        modules = patch.dict(sys.modules, {
            'xgc_camera_calibration':types.ModuleType('xgc_camera_calibration'),
            transforms.__name__:transforms, coordinates.__name__:coordinates})
        modules.start()
        self.addCleanup(modules.stop)
        self.coordinates = coordinates
        self.transforms = transforms
        self.module = _load_publisher_module()
        self.publisher = self.module.CameraContractPublisher.__new__(self.module.CameraContractPublisher)
        self.publisher._pose_lock = threading.Lock()
        self.publisher._parent_frame = 'world'
        self.publisher._optical_frame = 'camera_optical'
        self.publisher._translation = (0, 0, 0)
        self.publisher._pose_rotation = (0, 0, 0, 1)
        self.params = {}
        self.module.rospy.set_param = self.params.__setitem__
        self.module.rospy.logwarn_throttle = lambda *args: None
        self.sent = []
        self.publisher._publish_transforms = lambda: self.sent.append(tuple(self.publisher._translation))

    def revision(self, frame='world'):
        return types.SimpleNamespace(path=Path('/evidence/extrinsics-example.yaml'), document={
            'parent_frame':frame, 'child_frame':'camera_optical',
            'translation_array':[2, 3, 4], 'quaternion_xyzw_array':[0, 0, 0, 1],
            'metadata': {'pose_coordinates': {'schema_version':1, 'kind':'experiment-world',
                'frame':'world', 'world_offset':[-8, 0, 0]}},
        })

    def test_save_applies_stored_world_pose_and_retains_last_good_on_error(self):
        revision = self.revision()
        self.publisher._selection_watcher = types.SimpleNamespace(next_revision=lambda: revision)
        self.publisher._refresh_estimate()
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.params['~extrinsic_update_error'], '')
        self.coordinates.optical_translation_in_world.assert_called_once_with(revision.document, None)
        self.transforms.split_parent_to_optical_pose.assert_called_once_with((2, 3, 4), [0, 0, 0, 1], (.067, 0.0, 0.0))
        good = self.publisher._translation
        self.assertAlmostEqual(good[2], 4 - .067)
        self.assertLess(abs(good[0] - 2), .068)  # optical-link offset only, not world offset
        revision = self.revision('wrong-frame')
        self.publisher._refresh_estimate()
        self.assertEqual(tuple(self.publisher._translation), tuple(good))
        self.assertIn('frames', self.params['~extrinsic_update_error'])
        revision = self.revision()
        self.publisher._refresh_estimate()
        self.assertEqual(self.params['~extrinsic_update_error'], '')

    def test_corrupt_pointer_does_not_publish_or_replace_estimate(self):
        def fail(): raise ValueError('broken pointer')
        self.publisher._selection_watcher = types.SimpleNamespace(next_revision=fail)
        self.publisher._refresh_estimate()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.publisher._translation, (0, 0, 0))
        self.assertEqual(self.params['~extrinsic_update_error'], 'broken pointer')


if __name__ == '__main__': unittest.main()
