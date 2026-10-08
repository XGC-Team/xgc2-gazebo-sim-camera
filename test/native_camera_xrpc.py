#!/usr/bin/env python3
"""Isolated native Gazebo camera + official Python XRPC client acceptance.

No active station/master is contacted. Every child process, endpoint and log is
owned by this fixture and lives in a temporary directory.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from email.parser import BytesParser
from email.policy import default
import io
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import tempfile
import time
import threading
import xmlrpc.client

from PIL import Image
from xgc2_xrpc import Client, Fault, Runtime, ServiceRef, TransportError


def expect_fault(status, client, route, value=None, **options):
    try:
        response = client.call(route, value, **options)
    except Fault as error:
        assert error.status == status, (error.status, str(error))
    else:
        assert response.status == status, (response.status, response.body)


def run(args):
    with tempfile.TemporaryDirectory(prefix="sol6-native-camera-") as directory:
        root = Path(directory)
        isolated_environment = dict(os.environ)
        isolated_environment.pop("DISPLAY", None)
        isolated_environment.pop("WAYLAND_DISPLAY", None)
        endpoint = root / "camera.sock"
        log = root / "gazebo.log"
        with socket.socket() as free_port:
            free_port.bind(("127.0.0.1", 0))
            master_port = free_port.getsockname()[1]
        read_fd, write_fd = os.pipe()
        xvfb = subprocess.Popen([args.xvfb, "-displayfd", str(write_fd), "-screen", "0", "640x480x24", "-nolisten", "tcp"],
                                env=isolated_environment, pass_fds=(write_fd,), stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, start_new_session=True)
        os.close(write_fd)
        gazebo = roscore = None
        camera_info_subscription = None
        camera_info_event = threading.Event()
        received_info = []
        old_ros_master = os.environ.get("ROS_MASTER_URI")
        runtime = Runtime(blocking_workers=1, max_calls=16, max_connections=16)
        client = discovery = None
        try:
            if not select.select([read_fd], [], [], 8)[0]:
                raise RuntimeError("isolated Xvfb did not select a display")
            display = os.read(read_fd, 128).decode().strip()
            os.close(read_fd)
            if args.ros_calibration:
                with socket.socket() as port:
                    port.bind(("127.0.0.1", 0))
                    ros_port = port.getsockname()[1]
                os.environ["ROS_MASTER_URI"] = f"http://127.0.0.1:{ros_port}"
                with (root / "roscore.log").open("wb") as output:
                    ros_environment = dict(isolated_environment, ROS_MASTER_URI=os.environ["ROS_MASTER_URI"], ROS_HOME=str(root / "ros"), ROS_LOG_DIR=str(root / "ros-logs"),
                        ROS_PACKAGE_PATH="/opt/ros/noetic/share", ROS_ROOT="/opt/ros/noetic/share/ros",
                        PATH="/opt/ros/noetic/bin:" + os.environ.get("PATH", ""))
                    roscore = subprocess.Popen([args.rosmaster, "--core", "-p", str(ros_port)], env=ros_environment, stdout=output,
                        stderr=subprocess.STDOUT, start_new_session=True)
                ros_deadline = time.monotonic() + 12
                while True:
                    try:
                        if xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getPid("native_fixture")[0] == 1:
                            break
                    except OSError:
                        pass
                    if time.monotonic() >= ros_deadline:
                        raise RuntimeError("isolated ROS master startup failed: " + (root / "roscore.log").read_text()[-2000:])
                    time.sleep(0.05)
                import rospy
                from sensor_msgs.msg import CameraInfo
                rospy.init_node("native_camera_metadata_fixture", anonymous=True, disable_signals=True, disable_rosout=True)
                def record_info(message):
                    received_info.append(message)
                    if abs(message.K[0] - 123.0) < 1e-10:
                        camera_info_event.set()
                camera_info_subscription = rospy.Subscriber("/xgc/native_fixture/camera_info", CameraInfo, record_info, queue_size=2)
            models = []
            for index, source in enumerate(("front", "world")):
                models.append(f'''<model name="camera_{source}"><static>true</static><pose>0 {index} 2 0 0 0</pose>
<link name="link"><sensor name="sensor" type="camera"><always_on>false</always_on><update_rate>10</update_rate>
<camera><horizontal_fov>1.1</horizontal_fov><image><width>320</width><height>180</height><format>R8G8B8</format></image><clip><near>0.1</near><far>100</far></clip></camera>
<plugin name="xgc_media_camera" filename="{Path(args.plugin).resolve()}"><sourceId>{source}</sourceId><controlEndpoint>{endpoint}</controlEndpoint><controlInstanceId>native-camera-fixture</controlInstanceId><controlTargetId>native-camera-test</controlTargetId><rosPublishEnabled>{str(args.ros_calibration and source == "world").lower()}</rosPublishEnabled><rosCameraInfoTopic>/xgc/native_fixture/camera_info</rosCameraInfoTopic><rtpPort>{18004+index*2}</rtpPort><snapshotJpegBackend>cpu</snapshotJpegBackend></plugin>
</sensor></link></model>''')
            world = root / "camera.world"
            world.write_text('<sdf version="1.6"><world name="camera_test"><physics type="ode"><max_step_size>0.01</max_step_size><real_time_update_rate>100</real_time_update_rate></physics><scene><ambient>0.8 0.8 0.8 1</ambient><background>0.3 0.1 0.6 1</background></scene>' + ''.join(models) + '</world></sdf>')
            environment = dict(isolated_environment, DISPLAY=":" + display, LIBGL_ALWAYS_SOFTWARE="1",
                               GAZEBO_MASTER_URI=f"http://127.0.0.1:{master_port}", GAZEBO_LOG_PATH=str(root / "logs"),
                               XGC2_XRPC_HOST_MAX_CONNECTIONS="6")
            if args.ros_calibration:
                environment["ROS_MASTER_URI"] = os.environ["ROS_MASTER_URI"]
            with log.open("wb") as output:
                gazebo = subprocess.Popen([args.gzserver, "--verbose", str(world)], env=environment,
                                          stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            discovery = Client(str(endpoint), runtime=runtime)
            deadline = time.monotonic() + 25
            while True:
                if gazebo.poll() is not None:
                    raise RuntimeError("isolated Gazebo exited: " + log.read_text()[-4000:])
                try:
                    discovered = discovery.json("/v1/describe", method="GET", timeout=1)
                    if len(discovered.get("sources", [])) == 2:
                        break
                except (TransportError, OSError):
                    pass
                if time.monotonic() > deadline:
                    raise RuntimeError("camera source discovery timed out: " + log.read_text()[-5000:])
                time.sleep(0.1)
            assert set(discovered["sources"]) == {"front", "world"}, discovered
            client = Client.from_service(ServiceRef.from_dict(discovered["service_ref"]), runtime=runtime, local_target="native-camera-test")
            policy = client.json("/v1/runtime-policy", method="GET")
            fields = {field["name"]: field for field in policy["fields"]}
            assert fields["HOST_MAX_CONNECTIONS"]["value"] == discovered["limits"]["connections"] == 6
            assert fields["HOST_MAX_CONNECTIONS"]["source"] == "environment", fields["HOST_MAX_CONNECTIONS"]
            assert fields["HOST_MAX_CONNECTIONS"]["ceiling"] == 8
            route = "/v1/media/sources/world"
            status = client.json(route + "/status", method="GET")
            descriptor = client.json(route + "/describe", method="GET")
            assert descriptor["sourceId"] == "world" and "fresh-snapshot" in descriptor["capabilities"]
            expect_fault(404, client, route, method="GET")
            assert status["desired_revision"] == status["applied_revision"] == 1
            assert status["persisted_revision"] is None and not status["rtp_requested_active"]
            expect_fault(400, client, route + "/config", {"expected_revision": 1, "persist": True, "config": {"jpeg_quality": 70}}, method="PATCH")
            expect_fault(400, client, route + "/config", {"expected_revision": 1, "config": {"misspelled": 70}}, method="PATCH")
            expect_fault(409, client, route + "/config", {"expected_revision": 0, "config": {"jpeg_quality": 70}}, method="PATCH")
            expect_fault(404, client, "/v1/set-active", {"active": True})
            observation_revision = status["event_revision"]
            with ThreadPoolExecutor(max_workers=1) as worker:
                observation = worker.submit(client.json, route + "/observe/" + str(observation_revision), method="GET", timeout=5)
                applied = client.json(route + "/config", {"expected_revision": 1, "persist": False, "config": {"jpeg_quality": 70}}, method="PATCH", request_id="native-config")
                observed = observation.result(6)
            assert applied["receipt"]["stage"] == "completed", applied
            assert applied["applied_revision"] == 2 and applied["applied"]["jpeg_quality"] == 70
            assert observed["event_revision"] > observation_revision
            assert client.json(route + "/receipts/native-config", method="GET")["stage"] == "completed"
            expect_fault(409, client, route + "/config", {"expected_revision": 2, "config": {"jpeg_quality": 70}}, method="PATCH", request_id="native-config")
            if args.hardware_lifecycle:
                started = client.json(route + "/start", {}, request_id="native-start", timeout=8)
                assert started["active"] and started["completion"] == "applied" and started["state"] == "active"
                keyframe = client.json(route + "/request-keyframe", {}, timeout=8)
                assert keyframe["completion"] == "applied"
                expected_revision = keyframe["applied_revision"]
            else:
                expected_revision = applied["applied_revision"]
            stopped = client.json(route + "/stop", {}, request_id="native-stop", timeout=8)
            assert not stopped["active"] and stopped["state"] == "idle" and stopped["completion"] == "applied"
            expected_revision = stopped["applied_revision"]
            frames = []
            for index in range(2):
                response = client.call(route + "/capture", {"snapshotId": f"frame-{index}", "includeRgb": True, "requireFresh": True, "requestKeyframe": False}, timeout=6)
                assert response.status == 200, response.body[:1000]
                message = BytesParser(policy=default).parsebytes(("Content-Type: " + response.content_type + "\r\nMIME-Version: 1.0\r\n\r\n").encode() + response.body)
                parts = list(message.iter_parts())
                assert len(parts) == 3
                metadata = json.loads(parts[0].get_payload(decode=True))
                jpeg, rgb = (part.get_payload(decode=True) for part in parts[1:])
                assert len(jpeg) == metadata["jpegBytes"] and len(rgb) == metadata["rgbBytes"] == 320 * 180 * 3
                assert Image.open(io.BytesIO(jpeg)).size == (320, 180)
                assert metadata["frameSequence"] > 0
                assert metadata["snapshotId"] == f"frame-{index}" and metadata["sourceId"] == "world"
                assert metadata["timestampClockDomain"] == "simulation" and metadata["receipt"]["stage"] == "completed"
                assert len(metadata["cameraMatrix"]) == 9 and "renderPose" in metadata
                frames.append(metadata["timestampNanoseconds"])
            assert frames[1] > frames[0], frames
            result = {"engine": "Gazebo Classic", "sources": discovered["sources"], "applied_revision": stopped["applied_revision"],
                      "effective_policy": policy, "process": {"pid": gazebo.pid, "executable": str(Path(args.gzserver).resolve()),
                          "command": [args.gzserver, "--verbose", str(world)], "rootfs": "native host", "display": ":" + display,
                          "xvfb_pid": xvfb.pid, "wayland_display": None, "gzclient": False},
                      "capture_source_times_ns": frames, "gates": ["effective-runtime-policy", "CAS", "unknown-field", "persist-rejection", "native-render-ack", "receipt", "event-observe", "old-route-removed", "same-frame-multipart", "fresh-capture"]}
            if args.ros_calibration:
                calibration = {"scope": "calibration-metadata", "model": "plumb_bob", "width": 320,
                    "height": 180, "camera_matrix": [123.,0.,159.5,0.,124.,89.5,0.,0.,1.], "distortion": [.01,0.,0.,0.,0.]}
                malformed = dict(calibration, width=321)
                expect_fault(400, client, route + "/config", {"expected_revision": expected_revision, "persist": False,
                    "config": {"calibration": malformed}}, method="PATCH")
                applied_calibration = client.json(route + "/config", {"expected_revision": expected_revision, "persist": False,
                    "config": {"calibration": calibration}}, method="PATCH", timeout=8, request_id="native-calibration")
                assert applied_calibration["receipt"]["stage"] == "completed"
                assert applied_calibration["applied_revision"] == applied_calibration["published_calibration_revision"] == expected_revision + 1
                assert camera_info_event.wait(4), "actual ROS CameraInfo subscriber did not receive native calibration"
                actual = next(info for info in reversed(received_info) if info.K[0] == 123.)
                assert actual.width == 320 and actual.height == 180 and list(actual.K) == calibration["camera_matrix"]
                assert list(actual.D) == calibration["distortion"] and actual.distortion_model == "plumb_bob"
                assert metadata["cameraMatrix"][0] != 123., "estimated calibration leaked into rendered optical truth"
                result["gates"] += ["native-calibration-metadata", "actual-ROS-CameraInfo", "render-optics-preserved"]
                result["camera_info"] = {"K": list(actual.K), "D": list(actual.D), "applied_revision": expected_revision + 1}
            if args.evidence:
                Path(args.evidence).write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
        finally:
            if client: client.close()
            if discovery: discovery.close()
            runtime.close()
            if camera_info_subscription is not None:
                camera_info_subscription.unregister()
                import rospy
                rospy.signal_shutdown("isolated camera fixture completed")
            for process in (gazebo, xvfb, roscore):
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(3)
                        raise RuntimeError("owned native process failed to drain after SIGINT")
            if old_ros_master is None: os.environ.pop("ROS_MASTER_URI", None)
            else: os.environ["ROS_MASTER_URI"] = old_ros_master


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugin", required=True)
    parser.add_argument("--gzserver", default="gzserver")
    parser.add_argument("--xvfb", default="Xvfb")
    parser.add_argument("--evidence")
    parser.add_argument("--ros-calibration", action="store_true")
    parser.add_argument("--hardware-lifecycle", action="store_true", help="requires actual NVENC device; verifies real native start and force-IDR completion")
    parser.add_argument("--rosmaster", default="/opt/ros/noetic/bin/rosmaster")
    run(parser.parse_args())
