#!/usr/bin/env python3
import math
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "config/world_camera_profiles.yaml"
XACRO_PATH = ROOT / "urdf/fixed_rgb_camera.urdf.xacro"
EXPECTED_PROFILE_KEYS = {"image", "lens", "encoder", "snapshot"}


def load_profiles():
    return yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))


def expand_profile(profile_name, *mappings):
    command = [
        "/opt/ros/noetic/bin/xacro",
        str(XACRO_PATH),
        "camera_profile:={}".format(profile_name),
        *mappings,
    ]
    completed = subprocess.run(
        command,
        cwd=str(ROOT),
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return ET.fromstring(completed.stdout)


def sensor_from(robot):
    sensor = robot.find(".//sensor[@name='rgb_camera']")
    if sensor is None:
        raise AssertionError("expanded camera URDF has no rgb_camera sensor")
    return sensor


class WorldCameraProfilesTest(unittest.TestCase):
    def test_checked_in_profiles_have_a_strict_supported_shape(self):
        document = load_profiles()
        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(document["default_profile"], "world_wide_4k30_110")
        self.assertEqual(
            list(document["profiles"]),
            [
                "world_wide_4k30_110",
                "world_wide_1080p30_110",
                "calibration_wide_720p20_110",
            ],
        )
        world = document["profiles"]["world_wide_4k30_110"]
        self.assertEqual(
            (world["image"]["width_px"], world["image"]["height_px"]),
            (3840, 2160),
        )
        self.assertEqual(world["image"]["frame_rate_hz"], 30.0)
        self.assertEqual(world["lens"]["horizontal_fov_degrees"], 110.0)
        preview = document["profiles"]["world_wide_1080p30_110"]
        self.assertEqual(
            (preview["image"]["width_px"], preview["image"]["height_px"]),
            (1920, 1080),
        )
        self.assertEqual(preview["image"]["frame_rate_hz"], 30.0)
        self.assertEqual(preview["lens"]["horizontal_fov_degrees"], 110.0)

        for name, profile in document["profiles"].items():
            with self.subTest(profile=name):
                self.assertEqual(set(profile), EXPECTED_PROFILE_KEYS)
                self.assertEqual(
                    set(profile["image"]),
                    {"width_px", "height_px", "frame_rate_hz"},
                )
                self.assertEqual(
                    set(profile["lens"]),
                    {
                        "horizontal_fov_degrees",
                        "near_clip_m",
                        "far_clip_m",
                        "gaussian_noise_stddev",
                    },
                )
                self.assertEqual(
                    set(profile["encoder"]),
                    {
                        "average_bitrate_bps",
                        "maximum_bitrate_bps",
                        "pacing_bitrate_bps",
                        "vbv_buffer_ms",
                    },
                )
                self.assertEqual(set(profile["snapshot"]), {"jpeg_quality"})

                image = profile["image"]
                lens = profile["lens"]
                encoder = profile["encoder"]
                snapshot = profile["snapshot"]
                self.assertGreaterEqual(image["width_px"], 16)
                self.assertLessEqual(image["width_px"], 8192)
                self.assertEqual(image["width_px"] % 2, 0)
                self.assertGreaterEqual(image["height_px"], 16)
                self.assertLessEqual(image["height_px"], 8192)
                self.assertEqual(image["height_px"] % 2, 0)
                self.assertGreaterEqual(image["frame_rate_hz"], 0.1)
                self.assertLessEqual(image["frame_rate_hz"], 240.0)
                self.assertGreater(lens["horizontal_fov_degrees"], 0.0)
                self.assertLess(lens["horizontal_fov_degrees"], math.degrees(3.0))
                self.assertGreater(lens["near_clip_m"], 0.0)
                self.assertGreater(lens["far_clip_m"], lens["near_clip_m"])
                self.assertGreaterEqual(lens["gaussian_noise_stddev"], 0.0)
                self.assertGreaterEqual(encoder["average_bitrate_bps"], 128000)
                self.assertLessEqual(
                    encoder["average_bitrate_bps"],
                    encoder["maximum_bitrate_bps"],
                )
                self.assertLessEqual(
                    encoder["maximum_bitrate_bps"],
                    encoder["pacing_bitrate_bps"],
                )
                self.assertGreaterEqual(encoder["vbv_buffer_ms"], 50)
                self.assertLessEqual(encoder["vbv_buffer_ms"], 2000)
                self.assertGreaterEqual(snapshot["jpeg_quality"], 50)
                self.assertLessEqual(snapshot["jpeg_quality"], 100)

    def test_each_profile_expands_to_the_declared_sensor_and_encoder_values(self):
        for name, profile in load_profiles()["profiles"].items():
            with self.subTest(profile=name):
                sensor = sensor_from(expand_profile(name))
                image = profile["image"]
                lens = profile["lens"]
                encoder = profile["encoder"]
                snapshot = profile["snapshot"]
                plugin = sensor.find("plugin[@name='xgc_media_camera']")
                self.assertIsNotNone(plugin)
                self.assertEqual(int(sensor.findtext("camera/image/width")), image["width_px"])
                self.assertEqual(int(sensor.findtext("camera/image/height")), image["height_px"])
                self.assertAlmostEqual(
                    float(sensor.findtext("update_rate")),
                    image["frame_rate_hz"],
                )
                self.assertAlmostEqual(
                    float(sensor.findtext("camera/horizontal_fov")),
                    math.radians(lens["horizontal_fov_degrees"]),
                )
                self.assertAlmostEqual(
                    float(sensor.findtext("camera/clip/near")),
                    lens["near_clip_m"],
                )
                self.assertAlmostEqual(
                    float(sensor.findtext("camera/clip/far")),
                    lens["far_clip_m"],
                )
                self.assertEqual(
                    int(plugin.findtext("bitrate")),
                    encoder["average_bitrate_bps"],
                )
                self.assertEqual(
                    int(plugin.findtext("maxBitrate")),
                    encoder["maximum_bitrate_bps"],
                )
                self.assertEqual(
                    int(plugin.findtext("pacingBitrate")),
                    encoder["pacing_bitrate_bps"],
                )
                self.assertEqual(
                    int(plugin.findtext("vbvBufferMilliseconds")),
                    encoder["vbv_buffer_ms"],
                )
                self.assertEqual(
                    int(plugin.findtext("jpegQuality")),
                    snapshot["jpeg_quality"],
                )

    def test_explicit_launch_overrides_win_without_changing_the_selected_profile(self):
        sensor = sensor_from(
            expand_profile(
                "calibration_wide_720p20_110",
                "width:=640",
                "height:=480",
                "fps:=10",
                "hfov_degrees:=57.29577951308232",
                "media_bitrate:=2000000",
            )
        )
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "480")
        self.assertEqual(sensor.findtext("update_rate"), "10.0")
        self.assertEqual(sensor.findtext("camera/horizontal_fov"), "1.0")
        self.assertEqual(
            sensor.findtext("plugin[@name='xgc_media_camera']/bitrate"),
            "2000000",
        )
        self.assertEqual(
            sensor.findtext("plugin[@name='xgc_media_camera']/maxBitrate"),
            "6000000",
        )

    def test_managed_horizontal_fov_degrees_override_is_converted_to_radians(self):
        sensor = sensor_from(
            expand_profile(
                "world_wide_4k30_110",
                "hfov_degrees:=110",
            )
        )
        self.assertAlmostEqual(
            float(sensor.findtext("camera/horizontal_fov")),
            math.radians(110.0),
        )


if __name__ == "__main__":
    unittest.main()
