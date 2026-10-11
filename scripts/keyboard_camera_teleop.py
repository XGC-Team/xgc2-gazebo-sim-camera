#!/usr/bin/env python3
"""Fly the simulated camera from keypresses captured by Gazebo Classic GUI."""

import argparse
import copy
import math
import re
import subprocess
import sys
import json
import random
import threading
import time
import xml.etree.ElementTree as ET
if sys.version_info < (3, 8):
    raise RuntimeError("the selected camera interpreter must provide Python >= 3.8 and the formal XRPC wheel")

import numpy as np
import rospy
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply

from geometry_msgs.msg import Pose
from drive_intrinsic_calibration import GazeboModelController
from camera_peer import CameraPeer


KEY_LEFT = 0x01000012
KEY_UP = 0x01000013
KEY_RIGHT = 0x01000014
KEY_DOWN = 0x01000015
KEY_ESCAPE = 0x01000000


def letter(key, value):
    return key in (ord(value.lower()), ord(value.upper()))


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    return [value / norm for value in quaternion]


def translate_local(pose, forward=0.0, left=0.0, up=0.0):
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    rotation = quaternion_matrix(quaternion)[:3, :3]
    delta = rotation.dot(np.array([forward, left, up], dtype=float))
    pose.position.x += float(delta[0])
    pose.position.y += float(delta[1])
    pose.position.z += float(delta[2])


def rotate_local(pose, roll=0.0, pitch=0.0, yaw=0.0):
    current = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    delta = quaternion_from_euler(roll, pitch, yaw)
    result = normalize_quaternion(quaternion_multiply(current, delta))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = result


def pose_summary(pose):
    return "position=(%.2f, %.2f, %.2f)" % (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )


def print_controls(linear_step, angular_step):
    rospy.loginfo(
        "Drone-style camera keys: W/S throttle up/down, A/D yaw, "
        "arrows forward-back / strafe, Q/E pitch, Z/C roll, +/- speed, "
        "Space reset, H help, Esc quit (linear %.3f m, angular %.3f rad)",
        linear_step,
        angular_step,
    )


def resolve_keyboard_topic(requested, world):
    if not requested.startswith("~"):
        return requested
    # The prepared world is an owned launch input; resolving its name needs no
    # process, network probe or topic-list polling.
    name = ET.parse(world).getroot().find("world").attrib["name"]
    return "/gazebo/" + name + requested[1:]


class CameraPosePublisher:
    def __init__(self, model_name, service_ref_json, target_id):
        self.model_name, self.target_id = model_name, target_id
        self.reference = json.loads(service_ref_json)
        self._lock = threading.Lock()
        self._pose, self._revision, self._applied = None, 0, 0
        self._peer = None

    def set_pose(self, pose):
        with self._lock:
            self._pose = copy.deepcopy(pose)
            self._revision += 1
        # GUI idle starts no control connection. One bounded slot holds the
        # latest desired pose while the world or camera entity is unavailable.
        if self._peer is None:
            self._peer = CameraPeer(self.reference["endpoint"]["address"], self.target_id,
                                    "xgc2.simulation", self._apply)
        else:
            self._peer.wake()

    def _apply(self, client, description, stop):
        reference = description.get("service_ref", description.get("facts", {}).get("service_ref"))
        controller = GazeboModelController(self.model_name, service_ref_json=json.dumps(reference),
                                          target_id=self.target_id, runtime=client.runtime)
        try:
            while not stop.is_set():
                with self._lock:
                    pose, revision = copy.deepcopy(self._pose), self._revision
                # Reading the actual pose reconciles a lost write reply first.
                current = controller.current_pose()
                if current != pose:
                    controller.set_pose(pose)
                with self._lock:
                    self._applied = revision
                    if revision == self._revision:
                        return
        finally:
            controller.close()

    def close(self):
        if self._peer is not None:
            self._peer.close()


def keyboard_stream_command(topic):
    """Command that streams Gazebo keypress messages one line at a time.

    `gz topic -e` block-buffers its stdout when it writes to a pipe, so single
    keypress messages (~tens of bytes) sit in the buffer until kilobytes pile
    up.  The teleop then receives nothing for a long time and finally a whole
    batch at once -- the camera looks frozen, then lurches erratically.  Wrap
    the command in `stdbuf -oL` so gz flushes every line immediately and each
    keypress is acted on as it happens.
    """
    return ["stdbuf", "-oL", "gz", "topic", "-e", topic]


def apply_key(key, pose, initial_pose, linear_step, angular_step):
    # Drone RC "Mode 2" layout.  Left stick = W/S throttle (altitude) + A/D yaw;
    # right stick = arrows forward-back / strafe left-right.  Q/E pitch and Z/C
    # roll remain for aiming the camera up/down and levelling.
    if letter(key, "w"):
        pose.position.z += linear_step        # throttle up
    elif letter(key, "s"):
        pose.position.z -= linear_step        # throttle down
    elif letter(key, "a"):
        rotate_local(pose, yaw=angular_step)   # yaw left
    elif letter(key, "d"):
        rotate_local(pose, yaw=-angular_step)  # yaw right
    elif key == KEY_UP:
        translate_local(pose, forward=linear_step)
    elif key == KEY_DOWN:
        translate_local(pose, forward=-linear_step)
    elif key == KEY_LEFT:
        translate_local(pose, left=linear_step)
    elif key == KEY_RIGHT:
        translate_local(pose, left=-linear_step)
    elif letter(key, "q"):
        rotate_local(pose, pitch=-angular_step)  # aim up
    elif letter(key, "e"):
        rotate_local(pose, pitch=angular_step)   # aim down
    elif letter(key, "z"):
        rotate_local(pose, roll=-angular_step)
    elif letter(key, "c"):
        rotate_local(pose, roll=angular_step)
    elif key == ord(" "):
        pose = copy.deepcopy(initial_pose)
    else:
        return pose, False
    return pose, True


def parser():
    result = argparse.ArgumentParser(description="Control a Gazebo camera from GUI keypresses.")
    result.add_argument("--model-name", default="gazebo_static_camera")
    result.add_argument("--simulation-service-ref-json", required=True)
    result.add_argument("--target-id", required=True)
    result.add_argument("--linear-step", type=float, default=0.15)
    result.add_argument("--angular-step", type=float, default=0.05)
    result.add_argument("--minimum-step-scale", type=float, default=0.125)
    result.add_argument("--maximum-step-scale", type=float, default=8.0)
    result.add_argument("--topic", default="~/keyboard/keypress")
    result.add_argument("--world", required=True)
    return result


def main():
    args = parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("gazebo_camera_keyboard_teleop")
    controller = CameraPosePublisher(args.model_name, args.simulation_service_ref_json, args.target_id)
    initial = rospy.get_param("~initial_pose")
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = initial["position"]
    rotation = quaternion_from_euler(*initial["rotation"])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
    initial_pose = copy.deepcopy(pose)
    linear_step = args.linear_step
    angular_step = args.angular_step
    print_controls(linear_step, angular_step)
    key_pattern = re.compile(r"(?:int_value|data):\s*(-?\d+)")
    process = None
    quit_requested = False
    stopped = threading.Event()
    def stop():
        stopped.set()
        if process is not None and process.poll() is None:
            process.terminate()
    rospy.on_shutdown(stop)
    resolved_topic = resolve_keyboard_topic(args.topic, args.world)
    ceiling, last_log, suppressed = 0.25, None, 0
    try:
        while not rospy.is_shutdown() and not quit_requested:
            process = subprocess.Popen(
                keyboard_stream_command(resolved_topic),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
            )
            for line in process.stdout:
                if rospy.is_shutdown():
                    break
                match = key_pattern.search(line)
                if not match:
                    continue
                ceiling, last_log, suppressed = 0.25, None, 0
                key = int(match.group(1))
                if key == KEY_ESCAPE:
                    quit_requested = True
                    break
                if letter(key, "h"):
                    print_controls(linear_step, angular_step)
                    continue
                if key in (ord("+"), ord("=")):
                    maximum = args.linear_step * args.maximum_step_scale
                    linear_step = min(maximum, linear_step * 2.0)
                    angular_step = min(
                        args.angular_step * args.maximum_step_scale, angular_step * 2.0
                    )
                    print_controls(linear_step, angular_step)
                    continue
                if key in (ord("-"), ord("_")):
                    minimum = args.linear_step * args.minimum_step_scale
                    linear_step = max(minimum, linear_step / 2.0)
                    angular_step = max(
                        args.angular_step * args.minimum_step_scale, angular_step / 2.0
                    )
                    print_controls(linear_step, angular_step)
                    continue
                pose, changed = apply_key(
                    key,
                    pose,
                    initial_pose,
                    linear_step=linear_step,
                    angular_step=angular_step,
                )
                if changed:
                    controller.set_pose(pose)
                    rospy.loginfo("Requested camera %s", pose_summary(pose))
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            process.stdout.close()
            if not rospy.is_shutdown() and not quit_requested:
                suppressed += 1
                now = time.monotonic()
                if last_log is None or now - last_log >= 10:
                    rospy.logwarn("Gazebo keyboard stream closed (%d failures since last summary)", suppressed)
                    last_log, suppressed = now, 0
                stopped.wait(random.uniform(0, ceiling))
                ceiling = min(30, ceiling * 2)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
            process.wait()
        if process is not None:
            process.stdout.close()
        controller.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
