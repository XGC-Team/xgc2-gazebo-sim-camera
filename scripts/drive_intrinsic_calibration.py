#!/usr/bin/env python3
"""Move the Gazebo camera through views that fill ROS calibration progress."""

import argparse
import copy
import math
import sys
if sys.version_info < (3, 10):
    raise RuntimeError("the selected camera interpreter must provide Python >= 3.10 and the formal XRPC wheel")
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from xgc2_xrpc import Client, Fault, Runtime
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler
from native_camera_entity import bind_simulation_service


PARAMETER_RANGES = (0.7, 0.7, 0.4, 0.5)
SAMPLE_DISTANCE = 0.2


def parse_board_size(value):
    try:
        columns, rows = (int(part) for part in value.lower().split("x", 1))
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("board size must look like 7x5")
    if columns < 2 or rows < 2:
        raise argparse.ArgumentTypeError("board dimensions must both be at least 2")
    return columns, rows


def image_to_gray(message):
    """Convert common 8-bit sensor_msgs/Image encodings without cv_bridge."""
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
    }
    if encoding not in channels_by_encoding:
        raise ValueError("unsupported image encoding: {}".format(message.encoding))
    channels = channels_by_encoding[encoding]
    packed_width = message.width * channels
    if message.step < packed_width:
        raise ValueError("image step is smaller than its packed row width")
    expected = message.step * message.height
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size < expected:
        raise ValueError("image data is shorter than height * step")
    rows = data[:expected].reshape(message.height, message.step)
    pixels = rows[:, :packed_width]
    if channels == 1:
        return pixels.reshape(message.height, message.width).copy()
    image = pixels.reshape(message.height, message.width, channels)
    conversions = {
        "bgr8": cv2.COLOR_BGR2GRAY,
        "rgb8": cv2.COLOR_RGB2GRAY,
        "bgra8": cv2.COLOR_BGRA2GRAY,
        "rgba8": cv2.COLOR_RGBA2GRAY,
    }
    return cv2.cvtColor(image, conversions[encoding])


def checkerboard_parameters(gray, board_size, maximum_width=960):
    """Return ROS camera_calibration's normalized X/Y/Size/Skew metrics."""
    if gray.shape[1] > maximum_width:
        scale = float(maximum_width) / float(gray.shape[1])
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    columns, _ = board_size
    upper_left = corners[0, 0]
    upper_right = corners[columns - 1, 0]
    lower_right = corners[-1, 0]
    lower_left = corners[-columns, 0]
    edge_a = upper_right - upper_left
    edge_b = lower_right - upper_right
    edge_c = lower_left - lower_right
    diagonal_p = edge_b + edge_c
    diagonal_q = edge_a + edge_b
    area = abs(
        diagonal_p[0] * diagonal_q[1] - diagonal_p[1] * diagonal_q[0]
    ) / 2.0
    border = math.sqrt(area)
    width = float(gray.shape[1])
    height = float(gray.shape[0])
    mean_x = float(np.mean(corners[:, :, 0]))
    mean_y = float(np.mean(corners[:, :, 1]))
    p_x = min(1.0, max(0.0, (mean_x - border / 2.0) / (width - border)))
    p_y = min(1.0, max(0.0, (mean_y - border / 2.0) / (height - border)))
    p_size = math.sqrt(area / (width * height))
    vector_a = upper_left - upper_right
    vector_b = lower_right - upper_right
    cosine = float(np.dot(vector_a, vector_b)) / (
        float(np.linalg.norm(vector_a)) * float(np.linalg.norm(vector_b))
    )
    angle = math.acos(min(1.0, max(-1.0, cosine)))
    p_skew = min(1.0, 2.0 * abs(math.pi / 2.0 - angle))
    return p_x, p_y, p_size, p_skew


def is_new_sample(parameters, samples):
    if not samples:
        return True
    distance = min(
        sum(abs(left - right) for left, right in zip(parameters, sample))
        for sample in samples
    )
    return distance > SAMPLE_DISTANCE


def calibration_progress(samples):
    if not samples:
        return 0.0, 0.0, 0.0, 0.0
    minimum = [min(sample[index] for sample in samples) for index in range(4)]
    maximum = [max(sample[index] for sample in samples) for index in range(4)]
    minimum[2] = 0.0
    minimum[3] = 0.0
    return tuple(
        min(1.0, (high - low) / required)
        for low, high, required in zip(minimum, maximum, PARAMETER_RANGES)
    )


def look_at_orientation(position, target, yaw_offset=0.0, pitch_offset=0.0, roll=0.0):
    delta_x = target[0] - position[0]
    delta_y = target[1] - position[1]
    delta_z = target[2] - position[2]
    horizontal = math.hypot(delta_x, delta_y)
    yaw = math.atan2(delta_y, delta_x) + yaw_offset
    pitch = -math.atan2(delta_z, horizontal) + pitch_offset
    return quaternion_from_euler(roll, pitch, yaw)


class GazeboModelController:
    """Camera pose control through the world-owned simulation-v1 provider.

    An EntityRef generation fences removal/replacement. Completion comes from
    the provider's native operation wait, never from a ROS state sample.
    User images and calibration observations continue using ROS data topics.
    """
    def __init__(self, model_name, reference_frame="world", connection_timeout=10.0,
                 service_ref_json=None, target_id=None, runtime=None):
        if reference_frame != "world":
            raise ValueError("simulation-v1 camera poses use the declared world frame")
        self.model_name = model_name
        self._owns_runtime = runtime is None
        self._runtime = runtime or Runtime(blocking_workers=1, max_calls=8, max_connections=8)
        self._client = None
        try:
            self._client, _ = bind_simulation_service(service_ref_json, target_id, self._runtime)
            self._reference = self._entity()["ref"]
        except BaseException:
            if self._client is not None:
                self._client.close()
            if self._owns_runtime:
                self._runtime.close()
            raise

    def close(self):
        self._client.close()
        if self._owns_runtime:
            self._runtime.close()

    def _entity(self):
        result = self._client.json("/v1/entities/" + self.model_name, method="GET", timeout=5)
        entities = result.get("entities", [])
        if len(entities) != 1 or entities[0]["ref"]["id"] != self.model_name:
            raise RuntimeError("simulation provider did not return the camera EntityRef")
        if hasattr(self, "_reference") and entities[0]["ref"] != self._reference:
            raise RuntimeError("camera entity was replaced; generation no longer matches")
        return entities[0]

    def current_pose(self):
        state = self._entity()["state"]["pose"]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = state["position"]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = state["orientation"]
        return pose

    def set_pose(self, pose):
        state = {"position": [pose.position.x, pose.position.y, pose.position.z],
                 "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}
        operation = self._client.json("/v1/entities/" + self.model_name + "/state",
                                      {"generation": self._reference["generation"],
                                       "operation_timeout_ms": 5000, "state": {"pose": state}}, timeout=3)
        completed = self._client.json("/v1/operations/" + operation["id"] + "/wait", {}, timeout=6)
        if completed.get("state") != "succeeded":
            raise RuntimeError("native camera pose application failed: " + str(completed.get("error", completed)))
        return completed

    def set_view(self, position, target, yaw_offset=0.0, pitch_offset=0.0, roll=0.0):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = look_at_orientation(
            position, target, yaw_offset=yaw_offset, pitch_offset=pitch_offset, roll=roll)
        return self.set_pose(pose)


def calibration_views(target=(2.0, 0.0, 2.2)):
    """Measured views that fill coverage without taking the camera near ground."""
    tx, ty, tz = target
    specs = [
        ("far_lower_left", (tx - 6.0, ty + 0.3, tz + 0.1), -0.76, -0.24),
        ("far_lower_right", (tx - 6.0, ty - 0.3, tz + 0.1), 0.76, -0.24),
        ("far_bottom", (tx - 6.0, ty, tz - 0.1), 0.0, -0.48),
        ("far_lower_center", (tx - 6.0, ty, tz), 0.0, -0.24),
        ("upper_perspective", (tx - 2.5, ty - 1.2, tz - 0.8), 0.0, 0.44),
        ("upper_oblique", (tx - 3.5, ty + 2.0, tz + 1.0), 0.0, 0.44),
        ("center_oblique_left", (tx - 2.5, ty + 1.2, tz - 0.8), 0.0, 0.0),
        ("center_oblique_right", (tx - 2.5, ty - 1.2, tz + 0.8), 0.0, 0.0),
        ("medium_center", (tx - 4.0, ty, tz), 0.0, -0.08),
        ("near_center", (tx - 2.0, ty, tz), 0.0, -0.08),
        ("near_large", (tx - 1.4, ty, tz), 0.0, -0.08),
        ("near_maximum", (tx - 1.2, ty, tz), 0.0, -0.08),
        ("diagonal_high", (tx - 2.5, ty + 2.6, tz + 1.7), 0.0, 0.0),
        ("lower_right_perspective", (tx - 4.0, ty - 1.0, tz - 0.8), 0.0, 0.20),
        ("upper_left_perspective", (tx - 2.5, ty + 1.2, tz + 0.8), 0.0, 0.28),
    ]
    return [(name, position, yaw, pitch, 0.0) for name, position, yaw, pitch in specs]


def wait_for_detection(image_topic, board_size, timeout, maximum_width):
    deadline = time.monotonic() + timeout
    last_error = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        remaining = max(0.1, deadline - time.monotonic())
        try:
            message = rospy.wait_for_message(image_topic, Image, timeout=min(1.0, remaining))
            parameters = checkerboard_parameters(
                image_to_gray(message), board_size, maximum_width=maximum_width
            )
            if parameters is not None:
                return parameters
        except (rospy.ROSException, ValueError) as error:
            last_error = error
    if last_error:
        rospy.logwarn("No checkerboard detection: %s", last_error)
    return None


def parser():
    result = argparse.ArgumentParser(
        description="Move a Gazebo camera until ROS camera_calibration progress is covered."
    )
    result.add_argument("--model-name", default="gazebo_static_camera")
    result.add_argument("--simulation-service-ref-json", required=True)
    result.add_argument("--target-id", required=True)
    result.add_argument("--image-topic", default="/usb_cam/image_raw")
    result.add_argument("--board-size", type=parse_board_size, default=parse_board_size("7x5"))
    result.add_argument("--board-x", type=float, default=2.0)
    result.add_argument("--board-y", type=float, default=0.0)
    result.add_argument("--board-z", type=float, default=2.2)
    result.add_argument("--settle-seconds", type=float, default=1.0)
    result.add_argument("--hold-seconds", type=float, default=0.8)
    result.add_argument("--detection-timeout", type=float, default=4.0)
    result.add_argument("--maximum-detection-width", type=int, default=960)
    result.add_argument("--keep-final-pose", action="store_true")
    return result


def main():
    args = parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("gazebo_camera_intrinsic_calibration_driver")
    controller = GazeboModelController(args.model_name, service_ref_json=args.simulation_service_ref_json, target_id=args.target_id)
    original_pose = controller.current_pose()
    target = (args.board_x, args.board_y, args.board_z)
    samples = []
    complete = False
    try:
        for name, position, yaw_offset, pitch_offset, roll in calibration_views(target):
            if rospy.is_shutdown():
                break
            rospy.loginfo("Calibration view %s at %s", name, position)
            controller.set_view(
                position,
                target,
                yaw_offset=yaw_offset,
                pitch_offset=pitch_offset,
                roll=roll,
            )
            rospy.sleep(args.settle_seconds)
            parameters = wait_for_detection(
                args.image_topic,
                args.board_size,
                args.detection_timeout,
                args.maximum_detection_width,
            )
            if parameters is None:
                rospy.logwarn("Skipping %s: checkerboard was not detected", name)
                continue
            if is_new_sample(parameters, samples):
                samples.append(parameters)
            progress = calibration_progress(samples)
            rospy.loginfo(
                "%s X=%.3f Y=%.3f Size=%.3f Skew=%.3f | progress %.0f%% %.0f%% %.0f%% %.0f%%",
                name,
                *parameters,
                *(100.0 * value for value in progress)
            )
            rospy.sleep(args.hold_seconds)
            if all(value >= 1.0 for value in progress):
                complete = True
                rospy.loginfo(
                    "Intrinsic coverage complete; camera_calibration should now enable CALIBRATE."
                )
                break
    finally:
        if not args.keep_final_pose:
            try:
                controller.set_pose(original_pose)
                rospy.loginfo("Restored %s to its original pose", args.model_name)
            except (rospy.ROSException, RuntimeError, Fault) as error:
                rospy.logerr("Could not restore original camera pose: %s", error)
        controller.close()
    if not complete:
        rospy.logerr("Pose sweep ended before all four progress ranges were covered")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
