# XGC2 Gazebo Sim Camera

Independent world-camera product for Gazebo Classic 11 and ROS Noetic. It owns
the simulated camera model and GPU H264 source, but contains no vehicle model
and does not modify FS150, Scout, UAV, or other onboard camera definitions.

## Media contract

The managed live-video path performs one source encode:

```text
Gazebo render texture -> OpenGL texture -> NVENC H264 Annex-B
  |-> loopback RTP -> XGC media edge -> WebRTC -> WebUI
  `-> asynchronous ROS publisher -> source-quality recording
```

The plugin publishes each complete encoded access unit as
`foxglove_msgs/CompressedVideo`, with `xgc_camera_msgs/FrameTiming` and a
latched `xgc_camera_msgs/StreamInfo`. This reuses the NVENC output; it does not
render or encode a second copy. The exact Gazebo sensor measurement time is
preserved for every frame. See
[`docs/encoded_camera_recording.md`](docs/encoded_camera_recording.md) for
topics, rosbag commands, epoch semantics, and replay guidance.

The live WebUI path does not publish raw or periodic JPEG video through ROS. The camera
plugin exposes a XRPC HTTP source service in the granted private runtime directory; the media
edge activates the sensor only while a consumer needs live video. An explicit
snapshot request renders one fresh frame and returns its JPEG, optional RGB
pixels, source timestamp, pinhole camera matrix, zero-distortion vector, and the exact
optical-frame render pose with its declared pose frame. The pose and pixels are
sampled from the same Gazebo render transaction rather than joined later
through TF.

Snapshot JPEG policy is `auto`, strict `hardware`, or `cpu`. The portable path
issues a full-resolution OpenGL PBO readback, freezes metadata on that render,
and encodes through a depth-one libjpeg-turbo worker without blocking the
render callback. If PBO preflight or mapping fails it recaptures through the
legacy synchronous CPU readback. `auto` may use a separately packaged GPU
backend only after full runtime preflight and otherwise falls back; `hardware`
never silently falls back; `cpu` never probes an accelerator. NVENC remains the
H264 backend and is never reported as a JPEG encoder.
The mapped Gazebo/Ogre RenderTexture bytes already use the row order consumed
by the Live H264 path. The PBO helper therefore preserves that order; it must
not apply a generic OpenGL vertical flip. Runtime acceptance compares one Live
frame and one fresh snapshot at the same pose, including horizon, shadows, and
AprilTag chirality.
The optional module is loaded through the versioned C ABI in
`src/snapshot_jpeg_hardware_abi.h`. The main plugin has no CUDA linkage; a
vendor package owns device/runtime preflight and returns ordinary baseline
JPEG bytes. Runtime failure in `auto` fuses that module off and retries the
same captured RGB through libjpeg-turbo, while strict `hardware` fails the
snapshot.

Build the x86-64 CUDA 11.8 backend as a small relocatable runtime bundle, then
run the RTX gate:

```bash
bundle="$(mktemp -d /tmp/xgc-nvjpeg-bundle.XXXXXX)"
.xgc2/scripts/build_nvjpeg_backend_in_docker.sh --output-dir "$bundle"
.xgc2/scripts/test_nvjpeg_backend.sh "$bundle"
```

The pinned CUDA image is only a build stage. The bundle contains the module,
`libnvjpeg.so.11`, `libcudart.so.11.0`, and a digest manifest (about 6 MiB in
the current profile); the main plugin and CPU/Mesa package keep zero CUDA
linkage. RTX uses the CUDA implementation and is reported as `nvjpeg-cuda`,
not fixed-function JPEG hardware. Jetson hardware encode requires a separate
JetPack/Thor backend gate.
ROS consumers use the encoded H264 and CameraInfo topics. Full-resolution still
images are explicit source-control snapshot transactions; there is no periodic
ROS JPEG polling path.

The default instance uses:

- media source ID `usb_cam`;
- RTP destination `127.0.0.1:5004`;
- shared XRPC endpoint `/tmp/xgc2/media/camera-source.sock`;
- frames `usb_cam_link -> usb_cam_optical_frame`.

The optical-frame joint uses the REP-103 rotation
`rpy=(-pi/2, 0, -pi/2)`. A second instance must use a distinct model name,
source ID, RTP port, camera-link frame, and optical frame. All sensors in one world share the same XRPC endpoint.

The native source service uses the official XRPC SDK. Discover the bounded
source registry with `GET /v1/media/sources`, then use its fenced ServiceRef
for source status, start/stop, configuration, keyframe and same-frame capture.
The exact routes, revision/receipt semantics and resource/storage limits are
specified in [source_control.md](docs/source_control.md). The old JSON-line
socket operations have been removed.

## World camera profiles

`config/world_camera_profiles.yaml` is the canonical source for simulated
optics, render cadence, encoder budgets, and snapshot quality. Endpoint
identity, network ports, Gazebo pose, and TF ownership deliberately remain
per-instance launch parameters.

| Profile | Image | Horizontal FOV | H264 average / max / pacing |
| --- | --- | --- | --- |
| `world_wide_4k30_110` (default) | 3840×2160 at 30 fps | 110° | 24 / 36 / 72 Mbit/s |

The workflow owner explicitly starts the prepared native world and supplies its
complete simulation ServiceRef and target grant. Select a complete parameter
group with:

```bash
roslaunch gazebo_sim_camera static_camera.launch \
  world:=/absolute/prepared-camera.world \
  simulation_service_ref_json:="$SIMULATION_SERVICE_REF_JSON" target_id:="$SIMULATION_TARGET_ID" \
  media_control_endpoint:="$MEDIA_CONTROL_ENDPOINT" media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" \
  python_executable:="$CAMERA_PYTHON_EXECUTABLE" camera_profile:=world_wide_4k30_110 gui:=false

roslaunch gazebo_sim_camera static_camera.launch \
  world:=/absolute/prepared-camera.world \
  simulation_service_ref_json:="$SIMULATION_SERVICE_REF_JSON" target_id:="$SIMULATION_TARGET_ID" \
  media_control_endpoint:="$MEDIA_CONTROL_ENDPOINT" media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" \
  python_executable:="$CAMERA_PYTHON_EXECUTABLE" camera_profile:=world_wide_4k30_110 gui:=false
```

For direct developer launches, `width`, `height`, `fps`, `hfov_degrees`,
clipping, noise, bitrate, VBV, JPEG-quality, and snapshot JPEG policy arguments are explicit
overrides. Their default value is the sentinel `profile`; `hfov_degrees` is
converted to radians inside the xacro contract and there is no radians-based
alias. The managed ProcessDefinition exposes only `cameraProfile`, so
production workflows cannot assemble an incoherent partial profile. The
intrinsic Experiment selects the 90° field profile, whose ideal fx=640 px at
1280 width closely reproduces the focused station camera (measured fx≈638 px).

Gazebo Classic interprets `horizontal_fov` as radians, so the profile keeps FOV
in human-readable degrees and xacro converts it. The 110° profile is an ideal
rectilinear pinhole camera with zero distortion, not a fisheye lens model.

The configured frame rate is the sensor target rate. It does not by itself
prove that Gazebo, RTP, WebRTC, and the browser sustain that cadence; effective
frame timing must be measured end to end.

## Instance placement

Identity, endpoints, and placement can be changed without creating another
optics profile:

```bash
roslaunch gazebo_sim_camera static_camera.launch \
  camera_profile:=world_wide_4k30_110 \
  model_name:=yard_camera \
  world:=/absolute/prepared-camera.world \
  simulation_service_ref_json:="$SIMULATION_SERVICE_REF_JSON" target_id:="$SIMULATION_TARGET_ID" \
  media_source_id:=yard_cam \
  media_rtp_port:=5010 \
  media_control_endpoint:=/explicit/owned/runtime/camera-source.sock \
  media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" python_executable:="$CAMERA_PYTHON_EXECUTABLE" \
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
  world:=/absolute/prepared-camera.world \
  simulation_service_ref_json:="$SIMULATION_SERVICE_REF_JSON" target_id:="$SIMULATION_TARGET_ID" \
  media_control_endpoint:="$MEDIA_CONTROL_ENDPOINT" media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" \
  python_executable:="$CAMERA_PYTHON_EXECUTABLE" camera_profile:=world_wide_4k30_110 gui:=false

roslaunch gazebo_sim_camera extrinsic_calibration_world.launch \
  camera_profile:=world_wide_4k30_110 \
  world:=/absolute/prepared-extrinsic.world \
  simulation_service_ref_json:="$SIMULATION_SERVICE_REF_JSON" target_id:="$SIMULATION_TARGET_ID" \
  vrpn_config:=/absolute/extrinsic_markers_vrpn.yaml \
  media_control_endpoint:="$MEDIA_CONTROL_ENDPOINT" media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" \
  python_executable:="$CAMERA_PYTHON_EXECUTABLE" mode:=calibration publish_truth_tf:=false
```

The intrinsic scene composes the field-matched
`model://aprilgrid_6x6_tag36h11_88mm`: IDs 0–35, 88 mm tags and 26.4 mm gaps,
generated from Kalibr's official AprilGrid exporter. The extrinsic scene
composes the six `model://cal_marker_*` assets and can start the shared VRPN
source through the explicit native_world launcher vrpn_config. The prepared
world must declare required_component vrpn. Optional start_marker_data_client
exports only the six calibration markers as ROS algorithm input; Adapter owns
canonical vehicle localization and PX4 vision. The intrinsic launch spawns the camera with `static:=false` and gravity
disabled, allowing simulation-v1 state operations and the keyboard teleop to move it.
The standalone launch defaults to `static:=true`.

This is a separate world from the retained 8×6 checkerboard scene. Use
`camera_calibration_intrinsic_aprilgrid_6x6` for the station's field-matched
experiment and `camera_calibration_intrinsic` when explicitly testing the older
checkerboard target.

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

The XGC2 central process catalog, not this product, owns ProcessDefinition
`gazebo-static-camera`. It exposes `cameraProfile` as an enum synchronized with
the checked-in YAML. Model/frame identity, source ID, RTP port, control endpoint,
pose, and TF mode remain ordinary process parameters. Readiness uses native
source lifecycle and applied receipts on its private XRPC endpoint. This Debian package does
not install files under `/usr/share/xgc2/process-definitions`.

## Test and package

```bash
source /opt/ros/noetic/setup.bash
python3 test/test_world_camera_profiles.py
python3 test/static_product_contract.py
.xgc2/scripts/check_package_compliance.sh

catkin_make run_tests_gazebo_sim_camera
catkin_test_results
```

The camera calibration product owns calibration sessions, saved candidates and
their UI. This product supplies native camera capture, camera movement and
revision-fenced calibration metadata application. Applying estimates publishes
CameraInfo while preserving native optical intrinsics and capture truth.

The profile unit test validates schema bounds and expands every named profile
through xacro. While the source is inactive, the Gazebo contract first calls
`describe` and validates the actual source identity, H264/RTP contract and
loopback endpoint, dimensions, frame rate, frame ID, and capabilities. It then
requests both a same-frame JPEG+RGB capture and a fresh JPEG-only transaction;
it validates dimensions, increasing source timestamps, backend/readback
diagnostics, JPEG/RGB payloads, pinhole intrinsics, render pose, pose-frame
identity, and TF without requiring NVENC video encoding in the test.
The multi-architecture Docker build sources the just-built Catkin overlay and
runs this snapshot contract through Xvfb with Mesa software rendering. It is
therefore valid on an arm64 runner without a GPU; hardware NVENC remains a
target integration check rather than a packaging-CI prerequisite.

After building and sourcing a Catkin workspace, the Gazebo lifecycle regression
starts and stops the full contract twice and rejects an otherwise easy-to-miss
`gzserver` exit crash:

```bash
DISPLAY=:1 .xgc2/scripts/check_gazebo_shutdown.sh
```

The Debian package owns the ROS package share and executable directories plus
`/opt/ros/noetic/lib/libxgc_gazebo_media_camera.so`. It declares the plugin's
linked OpenGL, GLEW, and JPEG runtime libraries. `libnvidia-encode.so.1` is
loaded dynamically and must come from the target's matching NVIDIA driver
installation; the product does not pin or install a particular driver series.
