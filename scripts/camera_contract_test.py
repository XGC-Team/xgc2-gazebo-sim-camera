#!/usr/bin/env python3
import errno
import json
import math
import socket
import time
import unittest

import rospy
import tf
from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import CameraInfo
from xgc_camera_msgs.msg import FrameTiming, StreamInfo


def receive_line(connection, maximum_bytes=65536):
    buffer = bytearray()
    while b"\n" not in buffer:
        chunk = connection.recv(65536)
        if not chunk:
            raise RuntimeError("camera control socket closed before its response header")
        buffer.extend(chunk)
        if len(buffer) > maximum_bytes:
            raise RuntimeError("camera control response header is too large")
    line, remainder = bytes(buffer).split(b"\n", 1)
    return line, remainder


def receive_exact(connection, size, initial=b""):
    buffer = bytearray(initial)
    while len(buffer) < size:
        chunk = connection.recv(min(65536, size - len(buffer)))
        if not chunk:
            raise RuntimeError(
                "camera control socket closed with {} of {} payload bytes".format(
                    len(buffer), size
                )
            )
        buffer.extend(chunk)
    if len(buffer) != size:
        raise RuntimeError("camera control response contains unexpected trailing bytes")
    return bytes(buffer)


class CameraContractTest(unittest.TestCase):
    def connect_to_camera(self, path, timeout):
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                connection.connect(path)
                connection.settimeout(15.0)
                return connection
            except OSError as error:
                connection.close()
                last_error = error
                if error.errno not in (errno.ENOENT, errno.ECONNREFUSED):
                    raise
                time.sleep(0.1)
        self.fail(
            "camera control socket {} did not become ready: {}".format(
                path, last_error
            )
        )

    def request_description(self, path):
        connection = self.connect_to_camera(path, timeout=90.0)
        try:
            connection.sendall(b'{"operation":"describe"}\n')
            encoded_header, remainder = receive_line(connection)
            self.assertEqual(remainder, b"")
            description = json.loads(encoded_header.decode("utf-8"))
            if not description.get("ok"):
                self.fail(
                    "camera description failed: {}".format(
                        description.get("error")
                    )
                )
            return description
        finally:
            connection.close()

    def request_snapshot(self, path):
        connection = self.connect_to_camera(path, timeout=90.0)
        try:
            request = {
                "operation": "snapshot",
                "snapshotId": "static-camera-contract",
            }
            connection.sendall(json.dumps(request).encode("utf-8") + b"\n")
            encoded_header, payload_prefix = receive_line(connection)
            header = json.loads(encoded_header.decode("utf-8"))
            if not header.get("ok"):
                self.fail("camera snapshot failed: {}".format(header.get("error")))
            jpeg_size = int(header["jpegBytes"])
            rgb_size = int(header["rgbBytes"])
            payload = receive_exact(
                connection,
                jpeg_size + rgb_size,
                initial=payload_prefix,
            )
            return header, payload[:jpeg_size], payload[jpeg_size:]
        finally:
            connection.close()

    def test_camera_contract(self):
        control_socket = rospy.get_param(
            "~control_socket", "/tmp/xgc2/media/contract_camera.sock"
        )
        source_id = rospy.get_param("~source_id", "contract_camera")
        rtp_host = rospy.get_param("~rtp_host", "127.0.0.1")
        rtp_port = int(rospy.get_param("~rtp_port", 15004))
        frame_id = rospy.get_param("~frame_id", "contract_camera_optical_frame")
        parent_frame = rospy.get_param("~parent_frame", "map")
        width = int(rospy.get_param("~width", 1280))
        height = int(rospy.get_param("~height", 720))
        fps = float(rospy.get_param("~fps", 20.0))
        hfov = float(rospy.get_param("~hfov", 1.3962634015954636))
        stream_info_topic = rospy.get_param(
            "~stream_info_topic", "/xgc/test/camera/stream_info"
        )
        video_topic = rospy.get_param(
            "~video_topic", "/xgc/test/camera/video_h264"
        )
        frame_timing_topic = rospy.get_param(
            "~frame_timing_topic", "/xgc/test/camera/frame_timing"
        )
        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/xgc/camera/world/camera_info"
        )

        stream_info = rospy.wait_for_message(
            stream_info_topic, StreamInfo, timeout=30.0
        )
        self.assertEqual(stream_info.contract_version, 1)
        self.assertEqual(stream_info.stream_id, source_id)
        self.assertEqual(stream_info.frame_id, frame_id)
        self.assertNotEqual(stream_info.epoch, 0)
        self.assertEqual(stream_info.codec, StreamInfo.CODEC_H264)
        self.assertEqual(
            stream_info.bitstream_format,
            StreamInfo.BITSTREAM_FORMAT_ANNEX_B,
        )
        self.assertEqual(
            stream_info.clock_domain,
            StreamInfo.CLOCK_DOMAIN_SIMULATION,
        )
        self.assertEqual(
            stream_info.timestamp_reference,
            StreamInfo.TIMESTAMP_REFERENCE_RENDER_COMPLETE,
        )
        self.assertEqual((stream_info.width, stream_info.height), (width, height))
        self.assertAlmostEqual(stream_info.nominal_frame_rate, fps)
        self.assertEqual(stream_info.rtp_clock_rate, 90000)
        self.assertEqual(stream_info.rtp_payload_type, 96)

        # Subscribing to the encoded topic activates the demand-driven encoder.
        # Verify the three per-frame/per-stream ROS messages and the separately
        # published CameraInfo all identify the same optical TF child.
        camera_info = rospy.wait_for_message(
            camera_info_topic, CameraInfo, timeout=30.0
        )
        received = {}

        def receive_video(message):
            received.setdefault("video", message)

        def receive_timing(message):
            received.setdefault("timing", message)

        # Keep both subscriptions alive together. The video connection is what
        # activates rendering, while timing is emitted beside each encoded AU.
        video_subscription = rospy.Subscriber(
            video_topic, CompressedVideo, receive_video, queue_size=1
        )
        timing_subscription = rospy.Subscriber(
            frame_timing_topic, FrameTiming, receive_timing, queue_size=1
        )
        deadline = time.monotonic() + 30.0
        try:
            while (
                ("video" not in received or "timing" not in received)
                and time.monotonic() < deadline
                and not rospy.is_shutdown()
            ):
                time.sleep(0.01)
        finally:
            video_subscription.unregister()
            timing_subscription.unregister()
        self.assertIn("video", received)
        self.assertIn("timing", received)
        video = received["video"]
        timing = received["timing"]
        self.assertEqual(camera_info.header.frame_id, frame_id)
        self.assertEqual(video.frame_id, frame_id)
        self.assertEqual(timing.frame_id, frame_id)

        # The plugin starts inactive. Describe must report the resolved Gazebo
        # sensor contract without activating rendering or allocating NVENC.
        description = self.request_description(control_socket)
        self.assertEqual(
            description,
            {
                "ok": True,
                "protocolVersion": 1,
                "sourceId": source_id,
                "codec": "H264",
                "rtpPayloadType": 96,
                "rtpClockRate": 90000,
                "rtpHost": rtp_host,
                "rtpPort": rtp_port,
                "width": width,
                "height": height,
                "fps": fps,
                "frameId": frame_id,
                "timestampClockDomain": "simulation",
                "capabilities": [
                    "set-active",
                    "request-keyframe",
                    "snapshot",
                ],
            },
        )

        header, jpeg, rgb = self.request_snapshot(control_socket)
        self.assertEqual(header["snapshotId"], "static-camera-contract")
        self.assertEqual(header["frameId"], frame_id)
        self.assertEqual((header["width"], header["height"]), (width, height))
        self.assertEqual(header["pixelFormat"], "rgb8")
        self.assertGreaterEqual(header["timestampNanoseconds"], 0)
        self.assertEqual(header["timestampClockDomain"], "simulation")
        self.assertEqual(len(rgb), width * height * 3)
        self.assertGreater(len(jpeg), 4)
        self.assertEqual(jpeg[:2], b"\xff\xd8")
        self.assertEqual(jpeg[-2:], b"\xff\xd9")

        camera_matrix = header["cameraMatrix"]
        self.assertEqual(len(camera_matrix), 9)
        expected_fx = width / (2.0 * math.tan(hfov / 2.0))
        self.assertAlmostEqual(
            camera_matrix[0],
            expected_fx,
            delta=max(2.0, expected_fx * 0.03),
        )
        self.assertAlmostEqual(camera_matrix[4], expected_fx, delta=max(2.0, expected_fx * 0.03))
        self.assertAlmostEqual(camera_matrix[8], 1.0, places=6)
        self.assertEqual(header["distortion"], [0.0] * 5)

        listener = tf.TransformListener()
        listener.waitForTransform(
            parent_frame, frame_id, rospy.Time(0), rospy.Duration(20.0)
        )
        translation, rotation = listener.lookupTransform(
            parent_frame, frame_id, rospy.Time(0)
        )
        self.assertEqual(len(translation), 3)
        self.assertAlmostEqual(
            sum(value * value for value in rotation), 1.0, delta=1.0e-4
        )


if __name__ == "__main__":
    import rostest

    rospy.init_node("gazebo_sim_camera_contract_test")
    rostest.rosrun(
        "gazebo_sim_camera", "camera_contract", CameraContractTest
    )
