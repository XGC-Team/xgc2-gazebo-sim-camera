#!/usr/bin/env python3
"""Publish a world camera through the stable XGC camera contract.

The world-camera product owns this contract. It publishes the explicitly
selected intrinsic calibration and the current estimated simulation extrinsics.
Live and recorded video uses the source plugin's encoded H264 topics. Explicit
still-image capture uses the plugin's source-control snapshot transaction.
"""

import json
import math
import sys
if sys.version_info < (3, 8):
    raise RuntimeError("the selected camera interpreter must provide Python >= 3.8 and the formal XRPC wheel")
import re
import threading
import uuid
from pathlib import Path
from urllib.parse import quote

import rospy
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage
import yaml
from camera_peer import CameraPeer


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
    model = document.get("distortion_model", "plumb_bob")
    if model not in {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4} or len(distortion) != {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}[model]:
        raise ValueError("intrinsic distortion model and coefficient count differ")
    return camera_matrix, distortion, model


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
    intrinsic_json = str(rospy.get_param("~intrinsic_json", "")).strip()
    if intrinsic_json:
        document = json.loads(intrinsic_json)
        if document.get("camera_name") != camera_name:
            raise ValueError("frozen intrinsic camera identity differs")
        if (document.get("image_width"), document.get("image_height")) != (width, height):
            raise ValueError("frozen intrinsic dimensions differ from the camera source")
        camera_matrix = _matrix_data(document, "camera_matrix", 3, expected_cols=3)
        distortion = _matrix_data(document, "distortion_coefficients", 1, minimum_cols=4)
        model = document["distortion_model"]
        counts = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}
        if model not in counts or len(distortion) != counts[model] or camera_matrix[0] <= 0 or camera_matrix[4] <= 0:
            raise ValueError("frozen intrinsic model or focal length is invalid")
    elif intrinsic_file:
        camera_matrix, distortion, model = _selected_intrinsics(
            intrinsic_file,
            (width, height),
            camera_name,
        )
    else:
        distortion = [0.0] * 5
        model = "plumb_bob"
    return width, height, camera_matrix, distortion, model


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
        self._closed = False
        self._placement = json.loads(rospy.get_param("~placement_receipt", "{}"))
        self._optical_frame = rospy.get_param("~optical_frame")
        (
            self._width,
            self._height,
            self._camera_matrix,
            self._distortion,
            self._distortion_model,
        ) = _configured_intrinsics()
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
        self._frozen = None
        self._facts = None
        frozen_json = str(rospy.get_param("~resolved_extrinsic_json", ""))
        if frozen_json:
            from xgc_camera_calibration.extrinsic_application import parse_frame_roles
            from xgc_camera_calibration.extrinsic_resolver import decode_frozen
            from xgc_camera_calibration.record_facts import AppliedTransformFacts
            from std_msgs.msg import String
            roles = parse_frame_roles(rospy.get_param("~frame_roles_json"))
            if self._parent_frame != roles["parentFrame"] or self._optical_frame != roles["opticalFrames"]["sim"]:
                raise ValueError("Gazebo output frames do not match the controlled camera roles")
            self._frozen = decode_frozen(frozen_json, rospy.get_param("~camera_name"), roles)
            if self._frozen["status"] != "uncalibrated":
                self._set_frozen_estimate(self._frozen)
            self._fact_publisher = rospy.Publisher(AppliedTransformFacts.TOPIC, String, queue_size=64, latch=True)
            self._facts = AppliedTransformFacts({"id": rospy.get_param("~camera_name") + ":world:" + self._optical_frame,
                "instanceId": uuid.uuid4().hex, "kind": "camera-extrinsic", "resolutionId": self._frozen["resolutionId"]},
                lambda text: self._fact_publisher.publish(String(data=text)),
                lambda message: rospy.logwarn_throttle(5.0, message))
        # Non-calibrating standalone/intrinsic camera launches retain their
        # explicit spawn pose and have no selection consumer at all.

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
        self._publish_transforms()
        self._media_peer = CameraPeer(rospy.get_param("~media_control_endpoint"),
            rospy.get_param("~media_control_target_id"), "camera-source", self._apply_camera_info_metadata)
        self._transform_thread = threading.Thread(target=self._transform_loop, daemon=True)
        self._transform_thread.start()
        rospy.on_shutdown(self.close)

    def _set_frozen_estimate(self, frozen):
        from xgc_camera_calibration.transforms import split_parent_to_optical_pose
        pose = frozen["resolvedOpticalPose"]
        chain = split_parent_to_optical_pose(pose["translation"], pose["quaternionXyzw"], (0.067, 0., 0.))
        with self._pose_lock:
            self._translation = chain["parent_t_link"]
            self._pose_rotation = chain["parent_q_link_xyzw"]

    def _transform_loop(self):
        # Keep the frozen TF publication cadence independent of paused /clock.
        while not self._stop_event.wait(self._transform_period):
            if rospy.is_shutdown():
                return
            self._publish_transforms()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        try:
            self._media_peer.close()
            self._transform_thread.join()
            if self._facts is not None:
                self._facts.stop(rospy.Time.now().to_nsec())
        finally:
            if self._placement:
                from xgc2_xrpc import Runtime, ServiceRef
                from xgc2_gazebo_scene.simulation_client import SimulationClient
                reference = ServiceRef.from_dict(self._placement["world"]).validate()
                entity = self._placement["entity"]["entities"][0]["ref"]
                runtime = Runtime(blocking_workers=1, max_calls=2, max_connections=2,
                                  max_sessions=2, shutdown_timeout=1, log_level="error")
                try:
                    client = SimulationClient(reference, runtime=runtime, local_target=reference.target_id,
                                              timeout=3)
                    try:
                        client.operation("/v1/entities/" + quote(entity["id"], safe=""),
                                         {"generation": entity["generation"], "operation_timeout_ms": 3000},
                                         method="DELETE")
                    finally:
                        client.close()
                finally:
                    runtime.close()

    def _apply_camera_info_metadata(self, client, description, stop):
        source_id = _stable_camera_name(rospy.get_param("~media_source_id"))
        if source_id not in description["facts"]["sources"]:
            raise RuntimeError("camera source has not registered")
        route = "/v1/media/sources/" + source_id
        descriptor = client.json(route + "/describe", method="GET")
        if "calibration-metadata" not in descriptor.get("capabilities", []):
            raise RuntimeError("native camera source cannot publish calibration metadata")
        calibration = {"scope": "calibration-metadata", "model": self._distortion_model,
                       "width": self._width, "height": self._height,
                       "camera_matrix": list(self._camera_matrix), "distortion": list(self._distortion)}
        status = client.json(route + "/config", method="GET")
        if status.get("pending"):
            raise RuntimeError("camera metadata application is still pending")
        # Reconcile a lost reply before any write. The provider's revision and
        # publication acknowledgment fence a single replay of this owned state.
        if status.get("applied", {}).get("calibration") != calibration:
            status = client.json(route + "/config", {
                "expected_revision": status["desired_revision"], "persist": False,
                "config": {"calibration": calibration}}, method="PATCH", timeout=8)
        if status.get("pending") or status["published_calibration_revision"] != status["applied_revision"]:
            raise RuntimeError("native CameraInfo publication is not yet acknowledged")
        return route + "/observe/", status["event_revision"]

    def _publish_transforms(self, _event=None):
        stamp = rospy.Time.now()
        with self._pose_lock:
            translation, rotation = self._translation, self._pose_rotation
        transforms = [
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
        ]
        self._transform_publisher.publish(TFMessage(transforms=transforms))
        if self._facts is not None:
            from xgc_camera_calibration.record_facts import transform_values
            self._facts.applied(transform_values(transforms), self._frozen, stamp.to_nsec())


def main():
    rospy.init_node("xgc_camera_contract_publisher")
    publisher = CameraContractPublisher()
    try:
        rospy.spin()
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
