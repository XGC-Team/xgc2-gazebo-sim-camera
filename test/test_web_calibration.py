#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "web_calibration", str(SCRIPT_DIR / "web_calibration.py")
    )
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as error:
        # camera_calibration / ROS deps may be absent in a minimal CI image;
        # the pure helpers can only be exercised where they are installed.
        raise unittest.SkipTest("web_calibration deps unavailable: %s" % error)
    return module


MODULE = _load_module()


class KeyMappingTest(unittest.TestCase):
    def test_letters_map_to_ascii(self):
        self.assertEqual(MODULE.key_to_code("w"), ord("w"))
        self.assertEqual(MODULE.key_to_code("e"), ord("e"))

    def test_named_keys_map_to_teleop_constants(self):
        self.assertEqual(MODULE.key_to_code("ArrowLeft"), MODULE.KEY_LEFT)
        self.assertEqual(MODULE.key_to_code("ArrowDown"), MODULE.KEY_DOWN)
        self.assertEqual(MODULE.key_to_code("Space"), ord(" "))
        self.assertEqual(MODULE.key_to_code(" "), ord(" "))

    def test_unknown_key_is_none(self):
        self.assertIsNone(MODULE.key_to_code("Shift"))
        self.assertIsNone(MODULE.key_to_code(""))


class ProgressSerializationTest(unittest.TestCase):
    def test_params_become_label_progress_pairs(self):
        params = [("X", 0.1, 0.8, 0.5), ("Y", 0.0, 1.0, 1.0), ("Size", 0.2, 0.6, 0.4), ("Skew", 0.0, 0.5, 0.25)]
        bars = MODULE.serialize_progress(params)
        self.assertEqual([b["label"] for b in bars], ["X", "Y", "Size", "Skew"])
        self.assertAlmostEqual(bars[0]["progress"], 0.5)
        self.assertAlmostEqual(bars[1]["progress"], 1.0)

    def test_none_params_are_empty(self):
        self.assertEqual(MODULE.serialize_progress(None), [])


class PoseSerializationTest(unittest.TestCase):
    def test_pose_to_dict_roundtrips_fields(self):
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = 1.0, 2.0, 3.0
        pose.orientation.w = 1.0
        d = MODULE.pose_to_dict(pose)
        self.assertEqual((d["x"], d["y"], d["z"]), (1.0, 2.0, 3.0))
        self.assertEqual(d["qw"], 1.0)

    def test_none_pose_is_none(self):
        self.assertIsNone(MODULE.pose_to_dict(None))


class RecommendedViewsTest(unittest.TestCase):
    def test_views_are_distinct_and_cover_x_and_y(self):
        views = MODULE.recommended_views((2.0, 0.0, 1.5))
        self.assertEqual(len(views), 10)
        positions = {tuple(v["position"]) for v in views}
        self.assertEqual(len(positions), len(views))  # every pose is its own marker
        # X/Y only fill when the board is pushed off-centre via an aim offset.
        self.assertTrue(any(v["yaw_offset"] != 0 for v in views))
        self.assertTrue(any(v["pitch_offset"] != 0 for v in views))


class WebDirTest(unittest.TestCase):
    def test_resolve_web_dir_finds_source_assets(self):
        webdir = Path(MODULE.resolve_web_dir())
        index = (webdir / "index.html").read_text(encoding="utf-8")
        app = (webdir / "app.js").read_text(encoding="utf-8")
        styles = (webdir / "style.css").read_text(encoding="utf-8")
        self.assertIn('type="module"', index)
        self.assertIn('src="/app.js"', index)
        self.assertIn("xgc-app-shell", app)
        self.assertIn("/state", app)
        self.assertIn(".xgc-topbar", styles)


if __name__ == "__main__":
    unittest.main()
