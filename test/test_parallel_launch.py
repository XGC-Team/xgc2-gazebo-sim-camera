#!/usr/bin/env python3
"""Check local launch composition and process readiness without any peer."""
import json
import os
import random
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ParallelLaunchTest(unittest.TestCase):
    def test_runtime_composes_without_a_world_or_master(self):
        with tempfile.TemporaryDirectory(prefix="camera-launch-") as ros_home:
            environment = dict(os.environ, ROS_HOME=ros_home,
                               ROS_MASTER_URI="http://127.0.0.1:%d" % random.randint(20000, 40000),
                               ROS_PACKAGE_PATH=str(ROOT) + ":" + os.environ.get("ROS_PACKAGE_PATH", "/opt/ros/noetic/share"))
            for mode in ("truth", "calibration", "validation"):
                command = ["/opt/ros/noetic/bin/roslaunch", "--nodes", str(ROOT / "launch/static_camera.launch"),
                           "media_control_endpoint:=/granted/camera.sock", "media_control_target_id:=fixture",
                           "python_executable:=/usr/bin/python3", "mode:=" + mode]
                result = subprocess.run(command, env=environment, check=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, timeout=10)
                nodes = result.stdout.splitlines()
                self.assertIn("/gazebo_static_camera_xgc_camera_contract", nodes)
                self.assertEqual(len(nodes), 1 if mode == "calibration" else 3)
                self.assertFalse(any("native_entity" in node or "gazebo_gui" in node for node in nodes))

    def test_launches_have_no_peer_start_barriers(self):
        paths = list(ROOT.rglob("*.launch"))
        self.assertTrue(paths)
        for path in paths:
            for element in ET.parse(path).getroot().iter():
                for value in element.attrib.values():
                    self.assertFalse(re.search(r"\bsleep\b|wait_for_|waitForService|await-service", value), str(path))
                if element.tag == "node":
                    self.assertNotEqual(element.get("required"), "true", str(path))
                    self.assertNotEqual(element.get("respawn"), "true", str(path))
                    if element.get("type") == "spawn_robot_model":
                        self.assertNotRegex(element.get("args", ""), "controller|scene_service|scene_adapter")

    def test_readiness_uses_only_owned_resources(self):
        for path in (ROOT / "process-definitions").glob("*.json"):
            for definition in json.loads(path.read_text())["definitions"]:
                self.assertEqual(definition["readiness"], {"kind": "none"}, str(path))
                args = definition["command"].get("args", [])
                for index, arg in enumerate(args):
                    if arg.endswith("/roslaunch"):
                        self.assertEqual(args[index + 1], "--wait")


if __name__ == "__main__":
    unittest.main()
