#!/usr/bin/env python3
"""Check local launch composition and process readiness without any peer."""
import json
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ParallelLaunchTest(unittest.TestCase):
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
