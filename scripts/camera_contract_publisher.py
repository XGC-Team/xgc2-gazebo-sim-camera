#!/usr/bin/env python3
"""Publish a world camera through the stable XGC camera contract.

The world-camera product owns this contract. It publishes the explicitly
selected intrinsic calibration and the current estimated simulation extrinsics.
Live and recorded video uses the source plugin's encoded H264 topics. Explicit
still-image capture uses the plugin's source-control snapshot transaction.
"""

import math
import re
import threading
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage
import yaml


_CAMERA_NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9._-]{0,63}$")
_INTRINSIC_NAME_PATTERN = re.compile(
    r"intrinsics-\d{8}T\d{6}\.\d{6}Z\.yaml"
)


def _profile_value(argument, fallback, convert):
    return convert(fallback if str(argument) == "profile" else argument)


def _stable_camera_name(value):
    camera_name = str(value).strip()
    if not _CAMERA_NAME_PATTERN.fullmatch(camera_name):
        raise ValueError(
            "camera_name must match ^[A-Za-z][A-Za-z0-9._-]{0,63}$"
        )
    return camera_name


def _matrix_data(document, field, expected_rows, expected_cols=None, minimum_cols=None):
    matrix = document.get(field)
    if not isinstance(matrix, dict):
        raise ValueError("intrinsic_file {} must be a matrix object".format(field))
    rows = matrix.get("rows")
    cols = matrix.get("cols")
    data = matrix.get("data")
    if rows != expected_rows:
        raise ValueError("intrinsic_file {} has invalid rows".format(field))
    if expected_cols is not None and cols != expected_cols:
        raise ValueError("intrinsic_file {} has invalid cols".format(field))
    if minimum_cols is not None and (not isinstance(cols, int) or cols < minimum_cols):
        raise ValueError("intrinsic_file {} has too few columns".format(field))
    if not isinstance(data, list) or len(data) != rows * cols:
        raise ValueError("intrinsic_file {} data length does not match rows/cols".format(field))
    values = [float(value) for value in data]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("intrinsic_file {} contains non-finite values".format(field))
    return values


def _selected_intrinsics(path_value, configured_size, camera_name):
    camera_name = _stable_camera_name(camera_name)
    path = Path(str(path_value).strip()).expanduser()
    if not path.is_absolute():
        raise ValueError("intrinsic_file must be an absolute YAML file path")
    try:
        path = path.resolve(strict=True)
    except OSError as error:
        raise ValueError("intrinsic_file must resolve to an existing YAML file") from error
    if (
        path.parent.parent.name != "sim"
        or path.parent.name != camera_name
        or not _INTRINSIC_NAME_PATTERN.fullmatch(path.name)
    ):
        raise ValueError(
            "intrinsic_file must be a concrete timestamped file under <root>/sim/<camera_name>/"
        )
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict) or document.get("schema") != "xgc2.camera.intrinsic.v1":
        raise ValueError("intrinsic_file must contain an xgc2.camera.intrinsic.v1 document")
    if str(document.get("camera_name", "")).strip() != camera_name:
        raise ValueError("intrinsic_file camera_name must match the configured camera_name")
    width = int(document.get("image_width", 0))
    height = int(document.get("image_height", 0))
    if (width, height) != configured_size:
        raise ValueError(
            "intrinsic_file is {}x{}, but the configured camera is {}x{}".format(
                width,
                height,
                configured_size[0],
                configured_size[1],
            )
        )
    camera_matrix = _matrix_data(document, "camera_matrix", 3, expected_cols=3)
    distortion = _matrix_data(
        document, "distortion_coefficients", 1, minimum_cols=4
    )
    _matrix_data(document, "rectification_matrix", 3, expected_cols=3)
    _matrix_data(document, "projection_matrix", 3, expected_cols=4)
    return camera_matrix, distortion


def _configured_intrinsics():
    profile_path = rospy.get_param("~camera_profiles_file")
    profile_name = rospy.get_param("~camera_profile")
    with open(profile_path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        profile = document["profiles"][profile_name]
        profile_image = profile["image"]
        profile_lens = profile["lens"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "unknown or malformed world-camera profile {!r}".format(profile_name)
        ) from error

    width = _profile_value(
        rospy.get_param("~width", "profile"),
        profile_image["width_px"],
        int,
    )
    height = _profile_value(
        rospy.get_param("~height", "profile"),
        profile_image["height_px"],
        int,
    )
    hfov_degrees = rospy.get_param("~hfov_degrees", "profile")
    if str(hfov_degrees) != "profile":
        horizontal_fov = math.radians(float(hfov_degrees))
    else:
        horizontal_fov = math.radians(
            float(profile_lens["horizontal_fov_degrees"])
        )
    if width <= 0 or height <= 0:
        raise ValueError("world-camera image dimensions must be positive")
    if not math.isfinite(horizontal_fov) or not 0.0 < horizontal_fov < math.pi:
        raise ValueError("world-camera horizontal FOV must be between 0 and pi")
    focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))
    camera_matrix = [
        focal_length,
        0.0,
        (width - 1.0) / 2.0,
        0.0,
        focal_length,
        (height - 1.0) / 2.0,
        0.0,
        0.0,
        1.0,
    ]
    intrinsic_file = str(rospy.get_param("~intrinsic_file", "")).strip()
    camera_name = _stable_camera_name(rospy.get_param("~camera_name", "usb_cam"))
    if intrinsic_file:
        camera_matrix, distortion = _selected_intrinsics(
            intrinsic_file,
            (width, height),
            camera_name,
        )
    else:
        distortion = [0.0] * 5
    return width, height, camera_matrix, distortion


def _transform(parent, child, translation, rotation, stamp):
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = translation[0]
    message.transform.translation.y = translation[1]
    message.transform.translation.z = translation[2]
    message.transform.rotation.x = rotation[0]
    message.transform.rotation.y = rotation[1]
    message.transform.rotation.z = rotation[2]
    message.transform.rotation.w = rotation[3]
    return message


class CameraContractPublisher:
    def __init__(self):
        self._optical_frame = rospy.get_param("~optical_frame")
        (
            self._width,
            self._height,
            self._camera_matrix,
            self._distortion,
        ) = _configured_intrinsics()
        self._camera_info_publisher = rospy.Publisher(
            rospy.get_param("~output_camera_info_topic"),
            CameraInfo,
            queue_size=1,
            latch=True,
        )
        self._transform_publisher = rospy.Publisher(
            rospy.get_param("~output_transform_topic"),
            TFMessage,
            queue_size=1,
            latch=True,
        )

        self._parent_frame = rospy.get_param("~parent_frame")
        self._camera_link_frame = rospy.get_param("~camera_link_frame")
        self._translation = (
            float(rospy.get_param("~x")),
            float(rospy.get_param("~y")),
            float(rospy.get_param("~z")),
        )
        self._pose_rotation = quaternion_from_euler(
            float(rospy.get_param("~roll")),
            float(rospy.get_param("~pitch")),
            float(rospy.get_param("~yaw")),
        )
        self._optical_rotation = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)

        self._pose_lock = threading.Lock()
        self._selection_watcher = None
        calibration_root = str(rospy.get_param("~calibration_root", "")).strip()
        if calibration_root:
            from xgc_camera_calibration.extrinsic_file_watcher import ExtrinsicSelectionWatcher
            # Startup pose remains the workflow's explicit choice. Only subsequent
            # saves replace the estimate; never move the Gazebo truth camera.
            self._selection_watcher = ExtrinsicSelectionWatcher(
                calibration_root, "sim", rospy.get_param("~camera_name"),
            )
            try:
                self._selection_watcher.next_revision()
            except Exception as error:
                rospy.set_param("~extrinsic_update_error", str(error))

        transform_rate = float(rospy.get_param("~transform_publish_rate", 10.0))
        if transform_rate <= 0.0:
            raise ValueError("transform_publish_rate must be positive")
        if not math.isfinite(transform_rate):
            raise ValueError("transform_publish_rate must be finite")
        self._stop_event = threading.Event()
        rospy.on_shutdown(self._stop_event.set)
        self._transform_period = 1.0 / transform_rate
        # CameraInfo comes from the selected calibration file when supplied;
        # it is never inferred from image transport metadata.
        self._publish_camera_info(rospy.Time.now())
        self._publish_transforms()
        self._transform_thread = threading.Thread(target=self._transform_loop, daemon=True)
        self._transform_thread.start()

    def _refresh_estimate(self):
        if self._selection_watcher is None:
            return
        try:
            revision = self._selection_watcher.next_revision()
            if revision is None:
                return
            from xgc_camera_calibration.transforms import split_parent_to_optical_pose
            from xgc_camera_calibration.extrinsic_coordinates import optical_translation_in_world
            document = revision.document
            if (document["parent_frame"] != self._parent_frame
                    or document["child_frame"] != self._optical_frame):
                raise ValueError("saved extrinsic frames do not match the active camera")
            chain = split_parent_to_optical_pose(
                optical_translation_in_world(document, None),
                document["quaternion_xyzw_array"], (0.067, 0.0, 0.0),
            )
            with self._pose_lock:
                self._translation = chain["parent_t_link"]
                self._pose_rotation = chain["parent_q_link_xyzw"]
            self._publish_transforms()
            rospy.set_param("~active_extrinsic_file", str(revision.path))
            rospy.set_param("~extrinsic_update_error", "")
        except Exception as error:
            rospy.set_param("~extrinsic_update_error", str(error))
            rospy.logwarn_throttle(5.0, "Retaining camera estimate: %s", error)

    def _transform_loop(self):
        # Wall time keeps save consumption alive when Gazebo /clock is paused.
        next_refresh = 0.0
        while not self._stop_event.wait(self._transform_period):
            if rospy.is_shutdown():
                return
            if time.monotonic() >= next_refresh:
                self._refresh_estimate()
                next_refresh = time.monotonic() + 1.0
            self._publish_transforms()

    def _publish_camera_info(self, stamp):
        output = CameraInfo()
        output.header.stamp = stamp
        output.header.frame_id = self._optical_frame
        output.height = self._height
        output.width = self._width
        output.distortion_model = "plumb_bob"
        output.D = list(self._distortion)
        output.K = list(self._camera_matrix)
        output.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        output.P = [
            self._camera_matrix[0],
            0.0,
            self._camera_matrix[2],
            0.0,
            0.0,
            self._camera_matrix[4],
            self._camera_matrix[5],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self._camera_info_publisher.publish(output)

    def _publish_transforms(self, _event=None):
        stamp = rospy.Time.now()
        with self._pose_lock:
            translation, rotation = self._translation, self._pose_rotation
        self._transform_publisher.publish(TFMessage(transforms=[
            _transform(
                self._parent_frame,
                self._camera_link_frame,
                translation,
                rotation,
                stamp,
            ),
            _transform(
                self._camera_link_frame,
                self._optical_frame,
                (0.067, 0.0, 0.0),
                self._optical_rotation,
                stamp,
            ),
        ]))


def main():
    rospy.init_node("xgc_camera_contract_publisher")
    CameraContractPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
