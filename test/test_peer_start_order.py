#!/usr/bin/env python3
"""Real XRPC transport, late/restarted fake world and source, no Gazebo.

Set XGC_LONG_TESTS=1 to include both 60-second outage measurements.
"""
import asyncio
import json
import os
from pathlib import Path
import random
import select
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from xgc2_xrpc import Fault, Host, Runtime
from test_camera_intrinsic_selection import _load_publisher_module


def worker(kind, endpoint):
    random.seed(23)
    module = _load_publisher_module()
    module.rospy.logwarn = lambda fmt, *args: print(fmt % args, file=sys.stderr, flush=True)
    import camera_peer
    camera_peer.rospy = module.rospy
    params = {"~media_source_id": "usb_cam", "~media_control_endpoint": endpoint,
              "~media_control_target_id": "fixture", "~optical_frame": "optical",
              "~parent_frame": "world", "~camera_link_frame": "link",
              "~x": 0, "~y": 0, "~z": 0, "~roll": 0, "~pitch": 0, "~yaw": 0}
    module.rospy.get_param = lambda name, default=None: params.get(name, default)
    module.rospy.on_shutdown = lambda callback: None
    module.rospy.is_shutdown = lambda: False
    module.rospy.Publisher = lambda *args, **kwargs: None
    module._configured_intrinsics = lambda: (32, 18, [100., 0., 16., 0., 100., 9., 0., 0., 1.], [0.] * 5, "plumb_bob")
    transforms = []
    module.CameraContractPublisher._publish_transforms = lambda self: transforms.append(1)
    if kind == "source":
        owner = module.CameraContractPublisher()
        peer = owner._media_peer
    elif kind == "pose":
        from geometry_msgs.msg import Pose
        from keyboard_camera_teleop import CameraPosePublisher
        reference = {"target_id": "fixture", "service": "xgc2.simulation", "api_version": "v1",
                     "instance_id": "0" * 32, "profile": "http.v1",
                     "endpoint": {"kind": "unix", "address": endpoint}}
        owner = CameraPosePublisher("camera", json.dumps(reference), "fixture")
        assert owner._peer is None
        # Saturate wake notifications during an outage; only the latest pose
        # survives, and the caller continues serving without pipe contention.
        for x in [1, 2] * 10000 + [3]:
            pose = Pose()
            pose.position.x, pose.orientation.w = x, 1
            owner.set_pose(pose)
        peer = owner._peer
    else:
        # Only the ROS imports are substituted; the actual entity owner and its
        # XRPC client run unchanged in this independent process.
        import types
        sys.modules["rospy"] = module.rospy
        tf = types.ModuleType("tf.transformations")
        tf.quaternion_from_euler = lambda *args: [0, 0, 0, 1]
        sys.modules["tf.transformations"] = tf
        from native_camera_entity import NativeCameraEntity
        reference = {"target_id": "fixture", "service": "xgc2.simulation", "api_version": "v1",
                     "instance_id": "0" * 32, "profile": "http.v1",
                     "endpoint": {"kind": "unix", "address": endpoint}}
        owner = NativeCameraEntity(json.dumps(reference), "fixture", "camera", "<robot name='camera'/>",
                                   [0, 0, 0], [0, 0, 0, 1])
        peer = owner._peer
    try:
        for line in sys.stdin:
            if line.strip() == "stop":
                break
            print(json.dumps(dict(peer.status, transforms=len(transforms))), flush=True)
    finally:
        owner.close()


class FakePeer:
    def __init__(self, kind, endpoint, instance, lose_reply=False):
        self.kind, self.endpoint = kind, endpoint
        self.lose_reply = lose_reply
        self.writes = 0
        self.config = {"desired_revision": 1, "applied_revision": 1,
                       "published_calibration_revision": 1, "event_revision": 1, "pending": False,
                       "applied": {"calibration": {}}}
        service = "camera-source" if kind == "source" else "xgc2.simulation"
        self.reference = {"target_id": "fixture", "service": service, "api_version": "v1",
                          "instance_id": instance, "profile": "http.v1",
                          "endpoint": {"kind": "unix", "address": endpoint}}
        self.entity = None
        self.runtime = Runtime(blocking_workers=1, max_calls=8, max_connections=8)
        routes = {("GET", "/v1/describe"): self.describe}
        if kind == "source":
            route = "/v1/media/sources/usb_cam"
            routes.update({("GET", route + "/describe"): lambda ctx, value: {"capabilities": ["calibration-metadata"]},
                           ("GET", route + "/config"): lambda ctx, value: self.config,
                           ("PATCH", route + "/config"): self.patch,
                           ("GET", route + "/observe/2"): self.observe})
        else:
            if kind == "pose":
                self.entity = {"ref": {"id": "camera", "generation": 1},
                               "state": {"pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}}}
            routes.update({("POST", "/v1/entities"): self.create,
                           ("GET", "/v1/entities/camera"): lambda ctx, value: {"entities": [self.entity]},
                           ("DELETE", "/v1/entities/camera"): lambda ctx, value: {"id": "create"},
                           ("POST", "/v1/operations/create/wait"): lambda ctx, value: {
                               "state": "succeeded", "result": {"entities": [self.entity]}},
                           ("POST", "/v1/entities/camera/state"): self.pose})
        self.host = Host(endpoint, routes, runtime=self.runtime, instance_id=instance,
                         discovery_routes={"/v1/describe"})
        self.host.start()

    def describe(self, context, value):
        assert context.query == "wait_ready_ms=30000"
        return {"service": self.reference["service"], "api_version": "v1",
                "instance_id": self.reference["instance_id"], "ready": True,
                "facts": {"service_ref": self.reference, "sources": ["usb_cam"],
                          "artifact_types": ["application/urdf+xml"]}}

    def patch(self, context, value):
        assert value["expected_revision"] == self.config["desired_revision"]
        self.writes += 1
        self.config = dict(self.config, desired_revision=2, applied_revision=2,
                           published_calibration_revision=2, event_revision=2,
                           applied=value["config"], receipt={"stage": "completed"})
        if self.lose_reply:
            self.lose_reply = False
            raise Fault("unavailable", "reply lost after applying metadata")
        return self.config

    async def observe(self, context, value):
        # An observation is held instead of immediately returning ready again.
        await asyncio.sleep(1)
        return self.config

    def create(self, context, value):
        self.writes += 1
        self.entity = {"ref": {"id": "camera", "generation": 1}, "specification": value["entity"], "lifecycle": "ready"}
        if self.lose_reply:
            self.lose_reply = False
            raise Fault("unavailable", "reply lost after creating entity")
        return {"id": "create"}

    def pose(self, context, value):
        self.writes += 1
        self.entity["state"] = value["state"]
        return {"id": "create"}

    def close(self):
        self.host.close()
        self.runtime.close()


def resources(pid):
    fields = Path("/proc/%d/stat" % pid).read_text().rsplit(")", 1)[1].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    return cpu, len(list(Path("/proc/%d/task" % pid).iterdir())), len(list(Path("/proc/%d/fd" % pid).iterdir()))


class PeerStartOrderTest(unittest.TestCase):
    def exercise(self, kind, duration, lose_reply=False):
        with tempfile.TemporaryDirectory(prefix="camera-peer-test-") as directory:
            endpoint = str(Path(directory) / "peer.sock")
            # A leftover socket path exercises actual transport failure/backoff,
            # rather than only the zero-connect socket-appearance path.
            Path(endpoint).touch()
            with (Path(directory) / "worker.log").open("w+") as log:
                process = subprocess.Popen([sys.executable, __file__, "--worker", kind, endpoint],
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                                           universal_newlines=True)
                peer = None
                def status():
                    process.stdin.write("status\n")
                    process.stdin.flush()
                    self.assertTrue(select.select([process.stdout], [], [], 5)[0], "worker stopped serving")
                    return json.loads(process.stdout.readline())
                def wait_connected():
                    deadline = time.monotonic() + 35
                    while time.monotonic() < deadline:
                        state = status()
                        if state["state"] == "connected":
                            return state
                        time.sleep(.05)
                    self.fail("peer did not reconnect: " + repr(state))
                try:
                    state = status()
                    self.assertNotEqual(state["state"], "connected")
                    # Let the runtime allocate its single shared transport pool.
                    time.sleep(.5)
                    before = resources(process.pid)
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        time.sleep(min(.25, deadline - time.monotonic()))
                    state = status()
                    after = resources(process.pid)
                    log.flush()
                    log.seek(0)
                    lines = log.read().splitlines()
                    self.assertLessEqual(state["attempts"], 18 if duration == 60 else 8)
                    self.assertLessEqual(len(lines), 7 if duration == 60 else 2, "\n".join(lines))
                    self.assertLessEqual(after[1], before[1])
                    self.assertLessEqual(after[2], before[2])
                    if duration == 60:
                        self.assertLess(after[0] - before[0], 1.2)
                    Path(endpoint).unlink()
                    peer = FakePeer(kind, endpoint, "1" * 32, lose_reply=lose_reply)
                    wait_connected()
                    self.assertEqual(peer.writes, 1)
                    if kind == "pose":
                        self.assertEqual(peer.entity["state"]["pose"]["position"], [3, 0, 0])
                    if kind == "source":
                        self.assertGreater(status()["transforms"], 5)
                    peer.close()
                    peer = None
                    peer = FakePeer(kind, endpoint, "2" * 32)
                    connected = wait_connected()
                    # The old connected status can briefly precede the socket event.
                    deadline = time.monotonic() + 35
                    while connected.get("instance_id") != "2" * 32 and time.monotonic() < deadline:
                        time.sleep(.05)
                        connected = status()
                    self.assertEqual(connected["instance_id"], "2" * 32)
                    self.assertEqual(peer.writes, 1)
                    time.sleep(.2)
                    self.assertEqual(peer.writes, 1)
                    print(json.dumps({"kind": kind, "outage_seconds": duration,
                        "cpu_seconds": after[0] - before[0], "attempts": state["attempts"],
                        "log_lines": len(lines), "threads": [before[1], after[1]],
                        "fds": [before[2], after[2]], "replays_per_instance": 1}))
                finally:
                    if process.poll() is None:
                        process.stdin.write("stop\n")
                        process.stdin.flush()
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    if peer is not None:
                        peer.close()
                    process.stdin.close()
                    process.stdout.close()
                self.assertEqual(process.returncode, 0, (Path(directory) / "worker.log").read_text())

    def test_source_late_and_restarted(self):
        self.exercise("source", 1)

    def test_world_late_and_restarted(self):
        self.exercise("world", 1)

    def test_pose_is_lazy_latest_wins_and_replayed_after_restart(self):
        self.exercise("pose", 1)

    def test_applied_metadata_is_reconciled_after_a_lost_reply(self):
        self.exercise("source", 1, lose_reply=True)

    def test_created_entity_is_reconciled_after_a_lost_reply(self):
        self.exercise("world", 1, lose_reply=True)

    @unittest.skipUnless(os.environ.get("XGC_LONG_TESTS") == "1", "60-second outage cell")
    def test_source_down_for_sixty_seconds(self):
        self.exercise("source", 60)

    @unittest.skipUnless(os.environ.get("XGC_LONG_TESTS") == "1", "60-second outage cell")
    def test_world_down_for_sixty_seconds(self):
        self.exercise("world", 60)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--worker":
        worker(*sys.argv[2:])
    else:
        unittest.main()
