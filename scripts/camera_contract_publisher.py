#!/usr/bin/env python3
"""Publish a world camera through the stable XGC camera contract.

The world-camera product owns this contract. It publishes simulation-truth
intrinsics and extrinsics immediately from its frozen launch configuration.
Periodic JPEG snapshot polling is a disabled-by-default compatibility mode;
live and recorded video uses the source plugin's encoded H264 topics.
"""

import json
import math
import socket
import threading

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage
import yaml


_MAX_CONTROL_HEADER_BYTES = 64 * 1024


def _receive_line(connection):
    data = bytearray()
    while True:
        chunk = connection.recv(4096)
        if not chunk:
            raise RuntimeError("media source closed before its response header")
        newline = chunk.find(b"\n")
        if newline >= 0:
            data.extend(chunk[:newline])
            return bytes(data), chunk[newline + 1 :]
        data.extend(chunk)
        if len(data) > _MAX_CONTROL_HEADER_BYTES:
            raise RuntimeError("media source response header is too large")


def _receive_exact(connection, size, initial=b""):
    data = bytearray(initial)
    if len(data) > size:
        del data[size:]
    while len(data) < size:
        chunk = connection.recv(min(65536, size - len(data)))
        if not chunk:
            raise RuntimeError("media source closed before its JPEG payload")
        data.extend(chunk)
    return bytes(data)


def _profile_value(argument, fallback, convert):
    return convert(fallback if str(argument) == "profile" else argument)


def _boolean(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError("expected a boolean value")


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
    hfov = rospy.get_param("~hfov", "profile")
    if str(hfov_degrees) != "profile":
        horizontal_fov = math.radians(float(hfov_degrees))
    elif str(hfov) != "profile":
        horizontal_fov = float(hfov)
    else:
        horizontal_fov = math.radians(
            float(profile_lens["horizontal_fov_degrees"])
        )
    if width <= 0 or height <= 0:
        raise ValueError("world-camera image dimensions must be positive")
    if not math.isfinite(horizontal_fov) or not 0.0 < horizontal_fov < math.pi:
        raise ValueError("world-camera horizontal FOV must be between 0 and pi")
    focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))
    return width, height, [
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
        self._media_control_socket = rospy.get_param("~media_control_socket")
        self._snapshot_timeout = float(rospy.get_param("~snapshot_timeout", 5.0))
        if self._snapshot_timeout <= 0.0:
            raise ValueError("snapshot_timeout must be positive")
        self._width, self._height, self._camera_matrix = _configured_intrinsics()
        self._snapshot_sequence = 0
        self._snapshot_lock = threading.Lock()
        self._camera_info_publisher = rospy.Publisher(
            rospy.get_param("~output_camera_info_topic"),
            CameraInfo,
            queue_size=1,
            latch=True,
        )
        self._image_publisher = rospy.Publisher(
            rospy.get_param("~output_compressed_image_topic"),
            CompressedImage,
            queue_size=1,
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

        transform_rate = float(rospy.get_param("~transform_publish_rate", 10.0))
        if transform_rate <= 0.0:
            raise ValueError("transform_publish_rate must be positive")
        self._transform_timer = rospy.Timer(
            rospy.Duration(1.0 / transform_rate),
            self._publish_transforms,
        )
        self._image_timer = None
        self._continuous_jpeg_preview = _boolean(
            rospy.get_param("~enable_continuous_jpeg_preview", False)
        )
        if self._continuous_jpeg_preview:
            image_rate = float(rospy.get_param("~image_publish_rate", 10.0))
            if image_rate <= 0.0:
                raise ValueError("image_publish_rate must be positive")
            self._image_timer = rospy.Timer(
                rospy.Duration(1.0 / image_rate),
                self._publish_media_snapshot,
            )
        # Intrinsics and extrinsics are truth owned by this product, not data
        # inferred from the simulator image transport. Publish both at startup.
        self._publish_camera_info(rospy.Time.now())
        self._publish_transforms()

    def _publish_camera_info(self, stamp):
        output = CameraInfo()
        output.header.stamp = stamp
        output.header.frame_id = self._optical_frame
        output.height = self._height
        output.width = self._width
        output.distortion_model = "plumb_bob"
        output.D = [0.0] * 5
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

    def _publish_media_snapshot(self, _event=None):
        # Foxglove subscribes only when an Image/AR view needs this topic.
        if self._image_publisher.get_num_connections() == 0:
            return
        if not self._snapshot_lock.acquire(False):
            return
        try:
            self._snapshot_sequence += 1
            request = {
                "operation": "snapshot",
                "snapshotId": "xgc-contract-{}".format(self._snapshot_sequence),
                "includeRgb": False,
                "requestKeyframe": False,
            }
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.settimeout(self._snapshot_timeout)
            try:
                connection.connect(self._media_control_socket)
                connection.sendall(
                    json.dumps(request, separators=(",", ":")).encode("utf-8")
                    + b"\n"
                )
                encoded_header, payload_prefix = _receive_line(connection)
                header = json.loads(encoded_header.decode("utf-8"))
                if not header.get("ok"):
                    raise RuntimeError(
                        header.get("error", "media snapshot request failed")
                    )
                jpeg_size = int(header["jpegBytes"])
                rgb_size = int(header["rgbBytes"])
                if jpeg_size <= 0 or rgb_size < 0:
                    raise RuntimeError("media source returned an invalid JPEG contract")
                # A gzserver that was already running when this product was
                # upgraded may still have the previous plugin mapped. It
                # ignores includeRgb=false and appends RGB. Consume that
                # backward-compatible payload but publish only its JPEG.
                payload = _receive_exact(
                    connection,
                    jpeg_size + rgb_size,
                    payload_prefix,
                )
                jpeg = payload[:jpeg_size]
            finally:
                connection.close()

            width = int(header["width"])
            height = int(header["height"])
            if (width, height) != (self._width, self._height):
                raise RuntimeError(
                    "media dimensions {}x{} do not match configured intrinsics {}x{}".format(
                        width,
                        height,
                        self._width,
                        self._height,
                    )
                )
            timestamp_nanoseconds = int(header["timestampNanoseconds"])
            stamp = rospy.Time(
                timestamp_nanoseconds // 1000000000,
                timestamp_nanoseconds % 1000000000,
            )
            output = CompressedImage()
            output.header.stamp = stamp
            output.header.frame_id = self._optical_frame
            output.format = "jpeg"
            output.data = jpeg
            self._image_publisher.publish(output)
            # Keep the latched startup truth, and also stamp calibration in the
            # source clock domain for clients that synchronize image/info.
            self._publish_camera_info(stamp)
        except (KeyError, TypeError, ValueError, OSError, RuntimeError) as error:
            rospy.logwarn_throttle(
                2.0,
                "XGC world-camera image contract is waiting for its media source: %s",
                error,
            )
        finally:
            self._snapshot_lock.release()

    def _publish_transforms(self, _event=None):
        stamp = rospy.Time.now()
        self._transform_publisher.publish(TFMessage(transforms=[
            _transform(
                self._parent_frame,
                self._camera_link_frame,
                self._translation,
                self._pose_rotation,
                stamp,
            ),
            _transform(
                self._camera_link_frame,
                self._optical_frame,
                (0.0, 0.0, 0.0),
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
