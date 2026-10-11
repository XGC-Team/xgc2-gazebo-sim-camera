#!/usr/bin/env python3

import copy
import importlib.util
import math
import json
import os
import random
import select
import signal
import socket
import subprocess
import tempfile
import sys
import time
import unittest
from unittest import mock
from pathlib import Path

from geometry_msgs.msg import Pose


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location(
    "keyboard_camera_teleop", str(SCRIPT_DIR / "keyboard_camera_teleop.py")
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class KeyboardCameraTeleopTest(unittest.TestCase):
    def pose(self):
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    def test_w_is_throttle_up(self):
        pose = self.pose()
        result, changed = MODULE.apply_key(ord("W"), pose, copy.deepcopy(pose), 0.2, 0.1)
        self.assertTrue(changed)
        self.assertAlmostEqual(result.position.z, 0.2)  # drone throttle up

    def test_arrow_up_moves_forward(self):
        pose = self.pose()
        result, changed = MODULE.apply_key(MODULE.KEY_UP, pose, copy.deepcopy(pose), 0.2, 0.1)
        self.assertTrue(changed)
        self.assertAlmostEqual(result.position.x, 0.2)  # right stick forward

    def test_space_restores_initial_pose(self):
        pose = self.pose()
        pose.position.x = 4.0
        initial = self.pose()
        initial.position.x = -4.0
        result, changed = MODULE.apply_key(ord(" "), pose, initial, 0.2, 0.1)
        self.assertTrue(changed)
        self.assertAlmostEqual(result.position.x, -4.0)
        self.assertIsNot(result, initial)

    def test_yaw_key_keeps_quaternion_normalized(self):
        pose = self.pose()
        result, changed = MODULE.apply_key(ord("A"), pose, copy.deepcopy(pose), 0.2, 0.1)
        self.assertTrue(changed)
        norm = math.sqrt(
            result.orientation.x ** 2
            + result.orientation.y ** 2
            + result.orientation.z ** 2
            + result.orientation.w ** 2
        )
        self.assertAlmostEqual(norm, 1.0)

    def test_keyboard_stream_is_line_buffered(self):
        # gz block-buffers its pipe stdout, so keypresses must be un-batched with
        # stdbuf -oL or the camera freezes and then lurches.
        command = MODULE.keyboard_stream_command("/gazebo/calibration/keyboard/keypress")
        self.assertEqual(command[:3], ["stdbuf", "-oL", "gz"])
        self.assertEqual(command[-4:], ["gz", "topic", "-e", "/gazebo/calibration/keyboard/keypress"])

    def test_private_keyboard_topic_is_resolved_from_owned_world(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "calibration.world"
            world.write_text('<sdf version="1.6"><world name="calibration"/></sdf>')
            self.assertEqual(MODULE.resolve_keyboard_topic("~/keyboard/keypress", world),
                             "/gazebo/calibration/keyboard/keypress")


def stream_worker(directory):
    root = Path(directory)
    random.seed(23)
    callbacks, stopped = [], [False]
    MODULE.rospy.init_node = lambda *args: None
    MODULE.rospy.is_shutdown = lambda: stopped[0]
    MODULE.rospy.on_shutdown = callbacks.append
    MODULE.rospy.get_param = lambda name: {"position": [0, 0, 0], "rotation": [0, 0, 0]}
    MODULE.rospy.myargv = lambda **kwargs: ["keyboard", "--world", str(root / "world.sdf"),
        "--simulation-endpoint", str(root / "world.sock"), "--target-id", "fixture"]
    MODULE.rospy.logwarn = lambda fmt, *args: print(fmt % args, file=sys.stderr, flush=True)
    MODULE.rospy.loginfo = lambda *args: None
    def stop(*args):
        stopped[0] = True
        for callback in callbacks:
            callback()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    class Poses:
        def __init__(self, *args):
            pass
        def set_pose(self, pose):
            print(json.dumps({"z": pose.position.z}), flush=True)
        def close(self):
            pass
    MODULE.CameraPosePublisher = Poses
    # This executable is the disposable native input stream, not a probe. Its
    # socket closes reproduce the EOF events consumed by the actual main loop.
    MODULE.keyboard_stream_command = lambda topic: [sys.executable, str(root / "stream.py"), str(root / "peer.sock")]
    launch = MODULE.subprocess.Popen
    def counted(*args, **kwargs):
        child = launch(*args, **kwargs)
        with (root / "attempts").open("a") as output:
            output.write(str(child.pid) + "\n")
        return child
    MODULE.subprocess.Popen = counted
    return MODULE.main()


class KeyboardStreamStartOrderTest(unittest.TestCase):
    def exercise(self, duration):
        def resources(pid):
            fields = Path("/proc/%d/stat" % pid).read_text().rsplit(")", 1)[1].split()
            return ((int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK"),
                    len(list(Path("/proc/%d/task" % pid).iterdir())), len(list(Path("/proc/%d/fd" % pid).iterdir())))
        with tempfile.TemporaryDirectory(prefix="keyboard-stream-test-") as directory:
            root = Path(directory)
            endpoint = root / "peer.sock"
            (root / "world.sdf").write_text('<sdf version="1.6"><world name="fixture"/></sdf>')
            (root / "stream.py").write_text('''import socket, sys
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
try:
    sock.connect(sys.argv[1])
except OSError:
    sys.exit(1)
try:
    while True:
        data = sock.recv(128)
        if not data:
            break
        sys.stdout.write(data.decode())
        sys.stdout.flush()
finally:
    sock.close()
''')
            with (root / "worker.log").open("w+") as log:
                worker = subprocess.Popen([sys.executable, __file__, "--stream-worker", directory],
                    stdout=subprocess.PIPE, stderr=log, universal_newlines=True)
                listener = connection = None
                try:
                    deadline = time.monotonic() + 5
                    while not (root / "attempts").exists() and time.monotonic() < deadline:
                        time.sleep(.05)
                    self.assertTrue((root / "attempts").exists())
                    time.sleep(.5)
                    before = resources(worker.pid)
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        time.sleep(min(.25, max(0, deadline - time.monotonic())))
                    after = resources(worker.pid)
                    attempts = len((root / "attempts").read_text().splitlines())
                    log.seek(0)
                    lines = log.read().splitlines()
                    self.assertLessEqual(attempts, 18 if duration == 60 else 8)
                    self.assertLessEqual(len(lines), 7 if duration == 60 else 2)
                    self.assertEqual(after[1:], before[1:])
                    if duration == 60:
                        self.assertLess(after[0] - before[0], 1.2)
                    for instance in range(2):
                        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        listener.bind(str(endpoint))
                        listener.listen(1)
                        listener.settimeout(35)
                        connection, _ = listener.accept()
                        connection.sendall(b"int_value: 87\n")
                        self.assertTrue(select.select([worker.stdout], [], [], 5)[0])
                        self.assertAlmostEqual(json.loads(worker.stdout.readline())["z"], .15 * (instance + 1))
                        if instance == 0:
                            connection.close()
                            connection = None
                            listener.close()
                            listener = None
                            endpoint.unlink()
                    connection.sendall(("int_value: %d\n" % MODULE.KEY_ESCAPE).encode())
                    self.assertEqual(worker.wait(timeout=5), 0)
                    for pid in (root / "attempts").read_text().splitlines():
                        self.assertFalse(Path("/proc/" + pid).exists(), "input stream child was not reaped")
                    print(json.dumps({"peer": "native keyboard stream", "outage_seconds": duration,
                        "cpu_seconds": after[0] - before[0], "attempts": attempts, "log_lines": len(lines),
                        "threads": [before[1], after[1]], "fds": [before[2], after[2]]}))
                finally:
                    if worker.poll() is None:
                        worker.terminate()
                    try:
                        worker.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        worker.kill()
                        worker.wait()
                    worker.stdout.close()
                    if connection is not None:
                        connection.close()
                    if listener is not None:
                        listener.close()

    def test_stream_started_late_and_restarted(self):
        self.exercise(1)

    @unittest.skipUnless(os.environ.get("XGC_LONG_TESTS") == "1", "60-second outage cell")
    def test_stream_down_for_sixty_seconds(self):
        self.exercise(60)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--stream-worker":
        sys.exit(stream_worker(sys.argv[2]))
    unittest.main()
