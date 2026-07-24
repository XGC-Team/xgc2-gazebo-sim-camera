# XGC2 Gazebo Sim Camera

Independent world-camera product for Gazebo Classic 11 and ROS Noetic. It owns
the simulated camera model and GPU H264 source, but contains no vehicle model
and does not modify FS150, Scout, UAV, or other onboard camera definitions.

## Media contract

The managed camera path is:

```text
Gazebo render texture -> OpenGL texture -> NVENC H264 -> loopback RTP
  -> XGC media edge -> WebRTC -> WebUI
```

The live WebUI path does not publish raw or JPEG video through ROS. The camera
plugin exposes a private Unix control socket under `/tmp/xgc2/media/`; the media
edge activates the sensor only while a consumer needs live video. An explicit
snapshot request renders one frame and returns its JPEG, RGB pixels, source
timestamp, pinhole camera matrix, and zero-distortion vector for calibration.
ROS image delivery is an optional downstream branch and is not owned by this
package.

The default instance uses:

- media source ID `usb_cam`;
- RTP destination `127.0.0.1:5004`;
- control socket `/tmp/xgc2/media/usb_cam.sock`;
- frames `usb_cam_link -> usb_cam_optical_frame`.

The optical-frame joint uses the REP-103 rotation
`rpy=(-pi/2, 0, -pi/2)`. A second instance must use a distinct model name,
source ID, RTP port, control socket, camera-link frame, and optical frame.

## World camera profiles

`config/world_camera_profiles.yaml` is the canonical source for simulated
optics, render cadence, encoder budgets, and snapshot quality. Endpoint
identity, network ports, Gazebo pose, and TF ownership deliberately remain
per-instance launch parameters.

| Profile | Image | Horizontal FOV | H264 average / max / pacing |
| --- | --- | --- | --- |
| `world_ultrawide_4k30_130` (default) | 3840×2160 at 30 fps | 130° | 24 / 36 / 72 Mbit/s |
| `calibration_standard_720p20` | 1280×720 at 20 fps | 80° | 4 / 6 / 12 Mbit/s |

Select a complete parameter group with:

```bash
roslaunch gazebo_sim_camera static_camera.launch \
  camera_profile:=world_ultrawide_4k30_130 gui:=true

roslaunch gazebo_sim_camera static_camera.launch \
  camera_profile:=calibration_standard_720p20 gui:=true
```

For direct developer launches, the historical `width`, `height`, `fps`,
`hfov`, clipping, noise, bitrate, VBV, and JPEG-quality arguments remain
available as explicit overrides. Their default value is the sentinel
`profile`; the managed ProcessDefinition intentionally exposes only
`cameraProfile` so production workflows cannot assemble an incoherent partial
profile.

Gazebo Classic interprets `horizontal_fov` as radians, so the profile keeps FOV
in human-readable degrees and xacro converts it. The 130° profile is an ideal
rectilinear pinhole camera with zero distortion, not a fisheye lens model. At
16:9 its vertical FOV is approximately 104.25°, and edge stretching is
expected.

The configured frame rate is the sensor target rate. It does not by itself
prove that Gazebo, RTP, WebRTC, and the browser sustain that cadence; effective
frame timing must be measured end to end.

## Instance placement

Identity, endpoints, and placement can be changed without creating another
optics profile:

```bash
roslaunch gazebo_sim_camera static_camera.launch \
  camera_profile:=world_ultrawide_4k30_130 \
  model_name:=yard_camera \
  media_source_id:=yard_cam \
  media_rtp_port:=5010 \
  media_control_socket:=/tmp/xgc2/media/yard_cam.sock \
  camera_link_frame:=yard_cam_link \
  optical_frame:=yard_cam_optical_frame \
  x:=-3 y:=2 z:=2 yaw:=-0.3
```

Changing a profile requires the managed camera process to stop, delete, and
respawn its Gazebo model. Profiles are not a hot dynamic-reconfigure interface.

## TF ownership modes

- `mode:=truth publish_truth_tf:=true` publishes the exact Gazebo pose as
  `map -> <camera_link> -> <optical_frame>`.
- `mode:=calibration` publishes no camera TF. A calibration result can own the
  official transform without a second authority.
- `mode:=validation` publishes truth only on the separate
  `<camera_link>_gt -> <optical_frame>_gt` chain.

## Calibration scenes and camera movement

The intrinsic and extrinsic launch files pass the selected camera profile
through the same `static_camera.launch` workflow:

```bash
roslaunch gazebo_sim_camera intrinsic_calibration_world.launch \
  camera_profile:=calibration_standard_720p20 gui:=true

roslaunch gazebo_sim_camera extrinsic_calibration_world.launch \
  camera_profile:=calibration_standard_720p20 \
  mode:=calibration publish_truth_tf:=false
```

The intrinsic scene composes `model://checkerboard_8x6`; the extrinsic scene
composes the six `model://cal_marker_*` assets and can start the shared VRPN
bridge. The intrinsic launch spawns the camera with `static:=false` and gravity
disabled, allowing `/gazebo/set_model_state` and the keyboard teleop to move it.
The standalone launch defaults to `static:=true`.

Keyboard controls use a drone Mode 2 layout:

```text
W / S          altitude up / down
A / D          yaw left / right
Arrow keys     forward-back / strafe
Q / E          pitch
Z / C          roll
+ / -          movement step
Space          restore launch pose
H              show controls
Esc            stop keyboard control
```

Set `keyboard_teleop:=false` if another calibration controller owns the camera
pose.

## Managed process

ProcessDefinition `gazebo-static-camera` version 0.7 exposes
`cameraProfile` as an enum synchronized with the checked-in YAML. Model/frame
identity, source ID, RTP port, control socket, pose, and TF mode remain ordinary
process parameters. Readiness is the presence of the private control socket,
not a ROS image topic.

## Test and package

```bash
source /opt/ros/noetic/setup.bash
python3 test/test_world_camera_profiles.py
python3 test/static_product_contract.py
.xgc2/scripts/check_package_compliance.sh

catkin_make run_tests_gazebo_sim_camera
catkin_test_results
```

The profile unit test validates schema bounds and expands every named profile
through xacro. The Gazebo contract requests one explicit snapshot through the
control socket and validates dimensions, JPEG/RGB payloads, pinhole intrinsics,
and TF without requiring NVENC video encoding in the test.

The Debian package installs the active ProcessDefinition at
`/usr/share/xgc2/process-definitions/gazebo-static-camera.json` and keeps a
package-share copy for ROS tooling.
