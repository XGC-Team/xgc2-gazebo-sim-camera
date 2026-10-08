#!/usr/bin/env python3
import json
import math
import time
import unittest
from email.parser import BytesParser
from email.policy import default

from xgc2_xrpc import Client, Runtime, TransportError

import rospy
import tf
from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import CameraInfo
from xgc_camera_msgs.msg import FrameTiming, StreamInfo


class CameraContractTest(unittest.TestCase):
    def setUp(self):
        self._runtime = Runtime(blocking_workers=1, max_calls=8, max_connections=8)
        self._clients = {}

    def tearDown(self):
        for client in self._clients.values():
            client.close()
        self._runtime.close()

    def connect_to_camera(self, path, timeout):
        if path in self._clients:
            return self._clients[path]
        deadline = time.monotonic() + timeout
        discovery = Client(path, runtime=self._runtime)
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                try:
                    description = discovery.json("/v1/describe", method="GET", timeout=2)
                    client = Client(path, runtime=self._runtime,
                                    instance_id=description["service_ref"]["instance_id"])
                    self._clients[path] = client
                    return client
                except (OSError, TransportError):
                    time.sleep(0.1)
            self.fail("camera XRPC endpoint did not become ready: " + path)
        finally:
            discovery.close()

    def request_description(self, path):
        return self.connect_to_camera(path, 90).json(
            "/v1/media/sources/" + self.source_id + "/describe", method="GET", timeout=3)

    def request_snapshot(self, path, snapshot_id="static-camera-contract", **options):
        request = {"snapshotId": snapshot_id}
        request.update(options)
        response = self.connect_to_camera(path, 90).call(
            "/v1/media/sources/" + self.source_id + "/capture", request, timeout=6)
        self.assertEqual(response.status, 200, response.body[:1024])
        message = BytesParser(policy=default).parsebytes(
            ("Content-Type: " + response.content_type + "\r\nMIME-Version: 1.0\r\n\r\n").encode() + response.body)
        self.assertEqual(message.get_content_type(), "multipart/mixed")
        parts = list(message.iter_parts())
        self.assertEqual(parts[0].get_param("name", header="content-disposition"), "metadata")
        header = json.loads(parts[0].get_payload(decode=True))
        self.assertEqual(parts[1].get_content_type(), "image/jpeg")
        jpeg = parts[1].get_payload(decode=True)
        rgb = parts[2].get_payload(decode=True) if len(parts) == 3 else b""
        self.assertEqual(len(jpeg), header["jpegBytes"])
        self.assertEqual(len(rgb), header["rgbBytes"])
        self.assertEqual(header["receipt"]["stage"], "completed")
        return header, jpeg, rgb

    def test_camera_contract(self):
        control_socket = rospy.get_param(
            "~control_socket", "/tmp/xgc2/media/contract_camera.sock"
        )
        source_id = rospy.get_param("~source_id", "contract_camera")
        self.source_id = source_id
        rtp_host = rospy.get_param("~rtp_host", "127.0.0.1")
        rtp_port = int(rospy.get_param("~rtp_port", 15004))
        frame_id = rospy.get_param("~frame_id", "contract_camera_optical_frame")
        parent_frame = rospy.get_param("~parent_frame", "map")
        width = int(rospy.get_param("~width", 1280))
        height = int(rospy.get_param("~height", 720))
        fps = float(rospy.get_param("~fps", 20.0))
        hfov = float(rospy.get_param("~hfov", 1.9198621771937625))
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
        expect_encoded_frames = bool(
            rospy.get_param("~expect_encoded_frames", False)
        )
        expected_jpeg_policy = rospy.get_param("~expected_jpeg_policy", "auto")
        expected_jpeg_backend = rospy.get_param(
            "~expected_jpeg_backend", "libjpeg-turbo"
        )
        expected_jpeg_hardware_state = rospy.get_param(
            "~expected_jpeg_hardware_state", "unavailable"
        )
        expect_jpeg_fallback = bool(
            rospy.get_param("~expect_jpeg_fallback", True)
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
        self.assertAlmostEqual(stream_info.nominal_frame_rate, fps, delta=1e-4)
        self.assertEqual(stream_info.rtp_clock_rate, 90000)
        self.assertEqual(stream_info.rtp_payload_type, 96)

        # Package builders do not expose an NVIDIA encode device. Verify the
        # advertised ROS contract there without adding a forbidden CPU encoder
        # fallback; GPU-backed tests opt in to live access-unit validation.
        published_topics = dict(rospy.get_published_topics())
        self.assertEqual(
            published_topics.get(video_topic),
            "foxglove_msgs/CompressedVideo",
        )
        self.assertEqual(
            published_topics.get(frame_timing_topic),
            "xgc_camera_msgs/FrameTiming",
        )

        camera_info = rospy.wait_for_message(
            camera_info_topic, CameraInfo, timeout=30.0
        )
        self.assertEqual(camera_info.header.frame_id, frame_id)

        if expect_encoded_frames:
            received = {}

            def receive_video(message):
                received.setdefault("video", message)

            def receive_timing(message):
                received.setdefault("timing", message)

            # The video connection activates rendering; timing is emitted
            # beside each encoded access unit from that same GPU encode.
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
            self.assertEqual(received["video"].frame_id, frame_id)
            self.assertEqual(received["timing"].frame_id, frame_id)

        # The plugin starts inactive. Describe must report the resolved Gazebo
        # sensor contract without activating rendering or allocating NVENC.
        description = self.request_description(control_socket)
        self.assertIn("fps", description)
        self.assertAlmostEqual(description.pop("fps"), fps, delta=1e-4)
        expected = {
            "protocolVersion": 1, "sourceId": source_id, "codec": "H264",
            "rtpPayloadType": 96, "rtpClockRate": 90000, "rtpHost": rtp_host,
            "rtpPort": rtp_port, "width": width, "height": height, "frameId": frame_id,
            "snapshotJpegPolicy": expected_jpeg_policy,
            "snapshotJpegBackend": expected_jpeg_backend,
            "snapshotJpegHardwareState": expected_jpeg_hardware_state,
        }
        self.assertEqual({key: description[key] for key in expected}, expected)
        self.assertIn("capture", description["capabilities"])
        self.assertIn("fresh-snapshot", description["capabilities"])

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
        self.assertEqual(header["jpegBackend"], expected_jpeg_backend)
        # Mesa/Xvfb is the minimum portable accelerated-readback gate. The
        # synchronous path remains a runtime fallback, not the accepted CI
        # default for a build that advertises OpenGL PBO support.
        self.assertEqual(header["jpegReadback"], "opengl-pbo")
        self.assertGreaterEqual(header["jpegReadbackMilliseconds"], 0.0)
        self.assertGreaterEqual(header["jpegEncodeMilliseconds"], 0.0)
        if expect_jpeg_fallback:
            self.assertIn("jpegFallbackReason", header)
        else:
            self.assertNotIn("jpegFallbackReason", header)

        jpeg_header, jpeg_only, jpeg_rgb = self.request_snapshot(
            control_socket,
            snapshot_id="static-camera-contract-jpeg-only",
            includeRgb=False,
            requestKeyframe=False,
            requireFresh=True,
        )
        self.assertEqual(jpeg_header["snapshotId"], "static-camera-contract-jpeg-only")
        self.assertGreater(
            jpeg_header["timestampNanoseconds"], header["timestampNanoseconds"]
        )
        self.assertEqual(jpeg_header["rgbBytes"], 0)
        self.assertEqual(jpeg_rgb, b"")
        self.assertEqual(jpeg_only[:2], b"\xff\xd8")
        self.assertEqual(jpeg_only[-2:], b"\xff\xd9")

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
