#!/usr/bin/env python3
"""Serve the ROS camera_calibration workflow and camera teleop as a web UI.

The calibration CV engine is reused unchanged: this node subclasses
``camera_calibration.camera_calibrator.CalibrationNode`` (which already owns the
image subscription, the sample/goodenough/calibrate logic and the
``set_camera_info`` service) and only replaces its ``redraw_monocular`` GUI hook
so each annotated frame and the X/Y/Size/Skew progress can be pushed to a
browser instead of an OpenCV window.  Camera motion reuses the topic-based
``GazeboModelController`` and the exact key mapping of the keyboard teleop, so
the whole move -> sample -> calibrate -> commit flow lives in one web page.

The frontend has no dependencies: an MJPEG ``<img>`` for the annotated video and
``fetch`` polling of ``/state`` JSON.  The backend is Python stdlib only
(``http.server``); no Flask/WebSocket libraries are pulled in.
"""

import argparse
import copy
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rospy
from camera_calibration.calibrator import ChessboardInfo, Patterns
from camera_calibration.camera_calibrator import CalibrationNode

# Reuse the project's own movement math and pose controller.
from drive_intrinsic_calibration import (
    GazeboModelController,
    look_at_orientation,
    parse_board_size,
)
from keyboard_camera_teleop import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP, apply_key


# Browser KeyboardEvent.key -> the integer keycode apply_key() expects.
NAMED_KEYS = {
    "ArrowLeft": KEY_LEFT,
    "ArrowRight": KEY_RIGHT,
    "ArrowUp": KEY_UP,
    "ArrowDown": KEY_DOWN,
    "Space": ord(" "),
    " ": ord(" "),
}


def key_to_code(name):
    """Map a browser key name to the keycode apply_key()/speed control expects."""
    if name in NAMED_KEYS:
        return NAMED_KEYS[name]
    if isinstance(name, str) and len(name) == 1:
        return ord(name)
    return None


def serialize_progress(params):
    """MonoDrawable.params (label, lo, hi, progress) -> JSON-friendly bars."""
    bars = []
    if params:
        for label, _lo, _hi, progress in params:
            bars.append({"label": label, "progress": float(progress)})
    return bars


def recommended_views(target):
    """Spatially-distinct sample poses that together fill X / Y / Size / Skew.

    Filling X and Y needs the board *off-centre* in the image, so those poses
    carry a yaw / pitch aim offset (goto applies it through look_at_orientation)
    -- a camera that simply aims at the board keeps it centred and never moves
    the X/Y bars.  Size needs a near and a far view; Skew needs the oblique
    corners.  Every pose sits at its own point so each is a distinct, clickable
    marker in the 3D guide.
    """
    tx, ty, tz = target
    specs = [
        ("far lower left", (tx - 6.0, ty + 0.3, tz + 0.1), -0.76, -0.24),
        ("far lower right", (tx - 6.0, ty - 0.3, tz + 0.1), 0.76, -0.24),
        ("far bottom", (tx - 6.0, ty, tz - 0.1), 0.0, -0.48),
        ("far lower center", (tx - 6.0, ty, tz), 0.0, -0.24),
        ("upper perspective", (tx - 2.5, ty - 1.2, tz - 0.8), 0.0, 0.44),
        ("upper oblique", (tx - 3.5, ty + 2.0, tz + 1.0), 0.0, 0.44),
        ("center oblique left", (tx - 2.5, ty + 1.2, tz - 0.8), 0.0, 0.0),
        ("center oblique right", (tx - 2.5, ty - 1.2, tz + 0.8), 0.0, 0.0),
        ("medium center", (tx - 4.0, ty, tz), 0.0, -0.08),
        ("near center", (tx - 2.0, ty, tz), 0.0, -0.08),
        ("near large", (tx - 1.4, ty, tz), 0.0, -0.08),
        ("near maximum", (tx - 1.2, ty, tz), 0.0, -0.08),
        ("diagonal high", (tx - 2.5, ty + 2.6, tz + 1.7), 0.0, 0.0),
        ("lower right perspective", (tx - 4.0, ty - 1.0, tz - 0.8), 0.0, 0.20),
        ("upper left perspective", (tx - 2.5, ty + 1.2, tz + 0.8), 0.0, 0.28),
    ]
    return [{
        "name": name,
        "position": [round(v, 2) for v in position],
        "yaw_offset": yaw_offset,
        "pitch_offset": pitch_offset,
        "roll": 0.0,
    } for (name, position, yaw_offset, pitch_offset) in specs]


def pose_to_dict(pose):
    if pose is None:
        return None
    return {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "qx": pose.orientation.x,
        "qy": pose.orientation.y,
        "qz": pose.orientation.z,
        "qw": pose.orientation.w,
    }


class CameraController(object):
    """Move the Gazebo camera over the /gazebo/set_model_state topic.

    Reuses keyboard_camera_teleop.apply_key so browser keys behave identically
    to the native Gazebo keyboard teleop, plus the teleop's +/- speed scaling.
    """

    def __init__(self, model_name, linear_step, angular_step,
                 target=(2.0, 0.0, 2.2), min_scale=0.125, max_scale=8.0):
        self._controller = GazeboModelController(model_name)
        self.initial_pose = self._controller.current_pose()
        self.pose = copy.deepcopy(self.initial_pose)
        self.target = tuple(float(v) for v in target)
        self._base_linear = linear_step
        self._base_angular = angular_step
        self.linear_step = linear_step
        self.angular_step = angular_step
        self._min_scale = min_scale
        self._max_scale = max_scale
        self._lock = threading.Lock()

    def handle_key(self, code):
        """Apply one keystroke; return True if the camera pose changed."""
        with self._lock:
            if code in (ord("+"), ord("=")):
                self.linear_step = min(self._base_linear * self._max_scale, self.linear_step * 2.0)
                self.angular_step = min(self._base_angular * self._max_scale, self.angular_step * 2.0)
                return False
            if code in (ord("-"), ord("_")):
                self.linear_step = max(self._base_linear * self._min_scale, self.linear_step / 2.0)
                self.angular_step = max(self._base_angular * self._min_scale, self.angular_step / 2.0)
                return False
            self.pose, changed = apply_key(
                code, self.pose, self.initial_pose, self.linear_step, self.angular_step
            )
            if changed:
                self._controller.set_pose(self.pose)
            return changed

    def reset(self):
        with self._lock:
            self.pose = copy.deepcopy(self.initial_pose)
            self._controller.set_pose(self.pose)

    def goto(self, position, yaw_offset=0.0, pitch_offset=0.0, roll=0.0):
        """Fly to a position and auto-aim at the board (reuses look_at_orientation)."""
        with self._lock:
            quaternion = look_at_orientation(
                position, self.target, yaw_offset=yaw_offset, pitch_offset=pitch_offset, roll=roll
            )
            self.pose.position.x, self.pose.position.y, self.pose.position.z = position
            (
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w,
            ) = quaternion
            self._controller.set_pose(self.pose)

    def current(self):
        with self._lock:
            return copy.deepcopy(self.pose)


class WebCalibrationNode(CalibrationNode):
    """CalibrationNode whose GUI hook feeds a web page instead of an OpenCV window."""

    def __init__(self, *args, **kwargs):
        super(WebCalibrationNode, self).__init__(*args, **kwargs)
        self._engine_lock = threading.Lock()
        self._frame_cond = threading.Condition()
        self._jpeg = None
        self._frame_seq = 0
        self._params = None
        self._linear_error = -1.0
        self._last_scrib = None
        self._client_lock = threading.Lock()
        self._clients = 0
        # target-guide state (set by bind())
        self._camera = None
        self._views = []
        self._target_done = []
        self._refs = {}            # index -> reference JPEG bytes
        self._refs_dir = None
        self._prev_db_len = 0
        self._recording = False
        self._align_threshold = 1.8   # metres from a target to count as "aligned"

    def bind(self, camera, views, refs_dir):
        """Attach the camera + the target catalogue, and load any saved refs."""
        self._camera = camera
        self._views = views
        self._target_done = [False] * len(views)
        self._refs_dir = refs_dir
        self._load_refs()

    def _load_refs(self):
        if not self._refs_dir:
            return
        for i in range(len(self._views)):
            path = os.path.join(self._refs_dir, "%d.jpg" % i)
            if os.path.isfile(path):
                try:
                    with open(path, "rb") as handle:
                        self._refs[i] = handle.read()
                except OSError:
                    pass

    def _save_ref(self, index, jpeg):
        self._refs[index] = jpeg
        if not self._refs_dir:
            return
        try:
            os.makedirs(self._refs_dir, exist_ok=True)
            with open(os.path.join(self._refs_dir, "%d.jpg" % index), "wb") as handle:
                handle.write(jpeg)
        except OSError:
            pass

    def ref(self, index):
        return self._refs.get(index)

    def _nearest_target(self, pos):
        best, best_d = None, float("inf")
        for i, view in enumerate(self._views):
            t = view["position"]
            d = ((pos[0]-t[0])**2 + (pos[1]-t[1])**2 + (pos[2]-t[2])**2) ** 0.5
            if d < best_d:
                best_d, best = d, i
        return best, best_d

    def _mark_aligned_target(self):
        """Green the nearest target once the camera is aligned to it and the
        board is visible this frame -- independent of whether the calibrator
        kept the frame as a *new* sample.  is_good_sample() de-duplicates
        similar views, so a redundant-but-valid pose would otherwise stay grey
        even though its coverage is already filled by an equivalent sample."""
        if self._recording or self._camera is None or not self._views or self.c is None:
            return
        if getattr(self.c, "last_frame_corners", None) is None:
            return  # board not detected in the latest frame
        pose = self._camera.current()
        index, dist = self._nearest_target((pose.position.x, pose.position.y, pose.position.z))
        if index is None or dist > self._align_threshold or self._target_done[index]:
            return
        self._target_done[index] = True
        if self._last_scrib is not None:
            ok, buf = cv2.imencode(".jpg", self._last_scrib, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                self._save_ref(index, buf.tobytes())

    def _reset_calibration(self):
        with self._engine_lock:
            self.c = None
            self._prev_db_len = 0
            self._params = None
            self._target_done = [False] * len(self._views)

    def record_references(self, settle=1.3):
        """One-off: drive to every target, snapshot the view, then start fresh.

        Fills the reference-image library so the guide can show 'aim to see this'
        from the first alignment.  Pre-recorded samples are discarded afterwards
        so the user still calibrates manually with all spheres grey.
        """
        if self._camera is None or not self._views:
            return 0
        self._recording = True
        saved = 0
        try:
            for index, view in enumerate(self._views):
                self._camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
                time.sleep(settle)
                scrib = self._last_scrib
                if scrib is not None:
                    ok, buf = cv2.imencode(".jpg", scrib, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if ok:
                        self._save_ref(index, buf.tobytes())
                        saved += 1
            self._camera.reset()
        finally:
            self._recording = False
            self._reset_calibration()
        return saved

    def auto_run(self, settle=1.3):
        """Validate the algorithm: reset, then teleport through every target so
        the calibrator collects a full sample set hands-free (spheres green as
        it goes).  The user can then press Calibrate / Commit."""
        if self._camera is None or not self._views:
            return 0
        self._reset_calibration()
        for view in self._views:
            self._camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
            time.sleep(settle)
        with self._engine_lock:
            return len(self.c.db) if self.c is not None else 0

    # --- frame production ------------------------------------------------
    def add_client(self):
        with self._client_lock:
            self._clients += 1

    def remove_client(self):
        with self._client_lock:
            self._clients = max(0, self._clients - 1)

    def _has_client(self):
        with self._client_lock:
            return self._clients > 0

    def handle_monocular(self, msg):
        # Serialize all engine access (consumer thread vs HTTP action threads).
        with self._engine_lock:
            super(WebCalibrationNode, self).handle_monocular(msg)

    def redraw_monocular(self, drawable):
        # Called from handle_monocular, already under _engine_lock.
        self._params = drawable.params
        self._linear_error = drawable.linear_error
        self._last_scrib = drawable.scrib
        self._mark_aligned_target()
        if not self._has_client():
            return  # skip JPEG encoding when nobody is watching the stream
        ok, buf = cv2.imencode(".jpg", drawable.scrib, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        with self._frame_cond:
            self._jpeg = buf.tobytes()
            self._frame_seq += 1
            self._frame_cond.notify_all()

    def wait_for_frame(self, last_seq, timeout=1.0):
        """Block until a frame newer than last_seq; return (seq, jpeg_bytes|None)."""
        with self._frame_cond:
            if self._frame_seq == last_seq:
                self._frame_cond.wait(timeout)
            return self._frame_seq, self._jpeg

    # --- actions ---------------------------------------------------------
    def calibrate(self):
        with self._engine_lock:
            if self.c is not None and self.c.goodenough and not self.c.calibrated:
                self.c.do_calibration()
                return True
        return False

    def save(self):
        with self._engine_lock:
            if self.c is not None and self.c.calibrated:
                self.c.do_save()
                return "/tmp/calibrationdata.tar.gz"
        return None

    def commit(self):
        with self._engine_lock:
            if self.c is not None and self.c.calibrated:
                return bool(self.do_upload())
        return False

    def state(self):
        with self._engine_lock:
            c = self.c
            samples = len(c.db) if c is not None else 0
            goodenough = bool(c.goodenough) if c is not None else False
            calibrated = bool(c.calibrated) if c is not None else False
            params = self._params
            linear_error = self._linear_error
            result = None
            if calibrated:
                info = c.as_message()
                result = {
                    "width": info.width,
                    "height": info.height,
                    "K": list(info.K),
                    "D": list(info.D),
                    "P": list(info.P),
                    "yaml": c.yaml(),
                }
            done = list(self._target_done)
            targets = [{
                "name": view["name"],
                "position": view["position"],
                "done": done[i] if i < len(done) else False,
                "has_ref": i in self._refs,
            } for i, view in enumerate(self._views)]
            next_index = next((i for i, d in enumerate(done) if not d), None)
        return {
            "samples": samples,
            "goodenough": goodenough,
            "calibrated": calibrated,
            "progress": serialize_progress(params),
            "linear_error": None if linear_error is None or linear_error < 0 else float(linear_error),
            "result": result,
            "targets": targets,
            "next": next_index,
        }


class _Handler(BaseHTTPRequestHandler):
    server_version = "WebCalibration/1.0"

    def log_message(self, fmt, *args):  # quiet: route through rospy at debug level
        rospy.logdebug("web_calibration http: " + fmt, *args)

    # --- helpers ---------------------------------------------------------
    def _send_bytes(self, data, content_type, code=200):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj, code=200):
        self._send_bytes(json.dumps(obj).encode("utf-8"), "application/json", code)

    def _send_file(self, filename, content_type):
        try:
            with open(filename, "rb") as handle:
                data = handle.read()
        except OSError:
            self.send_error(404, "not found")
            return
        self._send_bytes(data, content_type)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    # --- routes ----------------------------------------------------------
    def do_GET(self):
        node = self.server.node
        webdir = self.server.webdir
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send_file(os.path.join(webdir, "index.html"), "text/html; charset=utf-8")
        elif path == "/app.js":
            self._send_file(os.path.join(webdir, "app.js"), "application/javascript")
        elif path == "/style.css":
            self._send_file(os.path.join(webdir, "style.css"), "text/css")
        elif path == "/state":
            state = node.state()
            state["camera_control"] = self.server.camera is not None
            state["pose"] = pose_to_dict(self.server.camera.current()) if self.server.camera else None
            self._send_json(state)
        elif path == "/stream.mjpg":
            self._stream_mjpeg(node)
        elif path == "/targets":
            camera = self.server.camera
            target = list(camera.target) if camera else [2.0, 0.0, 1.5]
            self._send_json({
                # Board is a vertical plane facing the cameras (8x6 squares * 0.2 m).
                "board": {"center": target, "width": 1.6, "height": 1.2, "plane": "yz"},
                "views": self.server.views,
                "camera_control": camera is not None,
            })
        elif path.startswith("/ref/"):
            try:
                index = int(path[len("/ref/"):].split(".", 1)[0])
            except ValueError:
                self.send_error(404, "not found")
                return
            jpeg = self.server.node.ref(index)
            if jpeg is None:
                self.send_error(404, "no reference")
                return
            self._send_bytes(jpeg, "image/jpeg")
        else:
            self.send_error(404, "not found")

    def do_POST(self):
        node = self.server.node
        camera = self.server.camera
        path = self.path.split("?", 1)[0]
        if path == "/calibrate":
            self._send_json({"ok": node.calibrate()})
        elif path == "/save":
            saved = node.save()
            self._send_json({"ok": saved is not None, "path": saved})
        elif path == "/commit":
            self._send_json({"ok": node.commit()})
        elif path == "/move":
            if camera is None:
                self._send_json({"ok": False, "error": "camera control disabled"}, 409)
                return
            code = key_to_code(self._read_json().get("key", ""))
            if code is None:
                self._send_json({"ok": False, "error": "unknown key"}, 400)
                return
            self._send_json({"ok": True, "changed": bool(camera.handle_key(code))})
        elif path == "/reset_pose":
            if camera is None:
                self._send_json({"ok": False, "error": "camera control disabled"}, 409)
                return
            camera.reset()
            self._send_json({"ok": True})
        elif path == "/goto":
            if camera is None:
                self._send_json({"ok": False, "error": "camera control disabled"}, 409)
                return
            views = self.server.views
            try:
                index = int(self._read_json().get("index"))
                view = views[index]
            except (TypeError, ValueError, IndexError):
                self._send_json({"ok": False, "error": "bad view index"}, 400)
                return
            camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
            self._send_json({"ok": True, "name": view["name"]})
        elif path == "/record_refs":
            if camera is None:
                self._send_json({"ok": False, "error": "camera control disabled"}, 409)
                return
            saved = self.server.node.record_references()
            self._send_json({"ok": True, "saved": saved})
        elif path == "/auto_run":
            if camera is None:
                self._send_json({"ok": False, "error": "camera control disabled"}, 409)
                return
            samples = self.server.node.auto_run()
            self._send_json({"ok": True, "samples": samples})
        else:
            self.send_error(404, "not found")

    def _stream_mjpeg(self, node):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        node.add_client()
        last_seq = -1
        try:
            while not rospy.is_shutdown():
                last_seq, frame = node.wait_for_frame(last_seq, timeout=1.0)
                if frame is None:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, ValueError):
            pass
        finally:
            node.remove_client()


def resolve_web_dir():
    """Locate the web/ asset dir from source layout first, then the install share."""
    here = os.path.dirname(os.path.abspath(__file__))
    source_candidate = os.path.normpath(os.path.join(here, "..", "web"))
    if os.path.isdir(source_candidate):
        return source_candidate
    try:
        import rospkg
        share = rospkg.RosPack().get_path("gazebo_sim_camera")
        return os.path.join(share, "web")
    except Exception:  # rospkg missing or package not found
        return source_candidate


def parser():
    result = argparse.ArgumentParser(description="Web UI for camera_calibration + camera teleop.")
    result.add_argument("--size", type=parse_board_size, default=parse_board_size("7x5"),
                        help="interior corners, e.g. 7x5")
    result.add_argument("--square", type=float, default=0.20, help="square size in meters")
    result.add_argument("--camera", default="usb_cam", help="calibrator camera name")
    result.add_argument("--port", type=int, default=8080)
    result.add_argument("--model-name", default="gazebo_static_camera")
    result.add_argument("--linear-step", type=float, default=0.15)
    result.add_argument("--angular-step", type=float, default=0.05)
    result.add_argument("--board-x", type=float, default=2.0)
    result.add_argument("--board-y", type=float, default=0.0)
    result.add_argument("--board-z", type=float, default=2.2)
    result.add_argument("--refs-dir", default=os.path.expanduser("~/.ros/web_calibration_refs"),
                        help="where per-target reference snapshots are stored")
    result.add_argument("--no-service-check", action="store_true",
                        help="do not wait for the set_camera_info service")
    result.add_argument("--no-camera-control", action="store_true",
                        help="serve calibration only, without moving the camera")
    return result


def main():
    args = parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("web_calibration")

    n_cols, n_rows = args.size
    boards = [ChessboardInfo("chessboard", n_cols, n_rows, float(args.square))]
    node = WebCalibrationNode(
        boards,
        service_check=not args.no_service_check,
        pattern=Patterns.Chessboard,
        camera_name=args.camera,
        checkerboard_flags=cv2.CALIB_CB_FAST_CHECK,
    )

    board_target = (args.board_x, args.board_y, args.board_z)
    camera = None
    if not args.no_camera_control:
        try:
            camera = CameraController(
                args.model_name, args.linear_step, args.angular_step, target=board_target
            )
        except (rospy.ROSException, RuntimeError) as error:
            rospy.logwarn("Camera control disabled (%s); serving calibration only.", error)

    webdir = resolve_web_dir()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), _Handler)
    server.node = node
    server.camera = camera
    server.views = recommended_views(board_target)
    server.webdir = webdir
    server.daemon_threads = True
    node.bind(camera, server.views, args.refs_dir)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo("Web calibration UI on http://localhost:%d (assets: %s)", args.port, webdir)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
