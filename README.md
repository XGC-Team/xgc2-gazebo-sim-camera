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

The camera uses:

- media source ID `usb_cam`;
- RTP host `127.0.0.1` and the port allocated in its inventory row;
- the opaque camera-source socket path allocated by Core;
- frames `usb_cam_link -> usb_cam_optical_frame`.

The optical-frame joint uses the REP-103 rotation
`rpy=(-pi/2, 0, -pi/2)`. At most one independently bound world camera is
supported per gzserver: the process-wide camera-source host refuses a second
endpoint/target binding. This existing limit is unchanged. Separate gzserver
processes require distinct model/source IDs, endpoints, RTP ports and frames.

The native `camera-source` service uses the official XRPC 0.2.0 SDK. Unbound
`GET /v1/describe` returns
`{"service":"camera-source","api_version":"v1","instance_id":"<hex>","ready":true,"facts":{...}}`.
The optional `wait_ready_ms=<0..30000>` query is also unbound and completes
asynchronously when ready or when the requested wait elapses. This shared host
is ready whenever it serves, including with no sources, so valid waits answer
immediately. Only this query with a canonical decimal integer is accepted;
other queries and out-of-range values return 400 `invalid_argument`.
`facts` contains `target_id`, the registered source IDs in `sources`, `limits`
and the complete `service_ref`. Limits are 16 sources, one concurrent capture,
eight connections, four in-flight calls, 65536 request bytes and 67108864
response bytes. Bind the returned ServiceRef for `GET /v1/media/sources` and
source status, start/stop, configuration, keyframe and same-frame capture.
The exact routes, revision/receipt semantics and resource/storage limits are
specified in [source_control.md](docs/source_control.md). The old JSON-line
socket operations have been removed.

## Placement and runtime

Core allocates the world and camera rendezvous endpoints and media ports before
starting the camera action. It starts `gazebo-static-camera` (the ROS runtime)
and media-edge in parallel at the action start. After `await world`, it runs
`/opt/ros/noetic/lib/gazebo_sim_camera/place_world_camera` as a finite
`command.run` with JSON output and a 60-second outer bound. The CLI renders
`fixed_rgb_camera.urdf.xacro` using the Python xacro module and places one
`role: sensor` URDF entity through `xgc2_gazebo_scene.placement`. It needs no
sourced ROS environment, `HOME`, ROS master, ROS parameters or stdin.

```bash
/opt/ros/noetic/lib/gazebo_sim_camera/place_world_camera \
  --world-endpoint "$SIMULATION_ENDPOINT" --target-id "$SIMULATION_TARGET_ID" \
  --model-name yard_camera --media-source-id yard_cam \
  --media-control-endpoint "$MEDIA_CONTROL_ENDPOINT" \
  --media-control-target-id "$MEDIA_CONTROL_TARGET_ID" \
  --media-rtp-port "$MEDIA_RTP_PORT" --camera-profile world_wide_4k30_110 \
  --camera-link-frame yard_cam_link --optical-frame yard_cam_optical_frame \
  --x -3 --y 2 --z 2 --yaw -0.3
```

The shared helper watches a briefly absent world socket, performs one unbound
held `GET /v1/describe?wait_ready_ms=...`, checks the simulation-v1 HTTP service
and target grant, pins the discovered instance, submits one creation and waits
once for completion. Rendering, discovery and creation share `--timeout`.
There is no placement polling, retry, replay, deletion or replacement. A second
placement of the same model ID fails with the world's conflict. A world restart
ends the Session. Stopping the runtime does not remove the model.

Success writes exactly one stdout line:

```json
{"schema":"xgc2.placement.v1","kind":"camera","id":"yard_camera","entity":{"entities":[{"ref":{"id":"yard_camera","generation":1}}]},"instance":"<world instance>"}
```

`entity` preserves the native result returned by the helper. Exit codes are
0 for success, 2 for usage/validation or rendering/import failure before
placement, and 1 for world discovery/creation failure. Failures write one
`place_world_camera: <reason>` line to stderr without a traceback.

The complete CLI argument list is:

| Argument | Default / constraint |
| --- | --- |
| `--world-endpoint` | Required; canonical absolute Unix socket path, at most 107 bytes |
| `--target-id` | Required; nonempty canonical world target grant |
| `--model-name` | `gazebo_static_camera`; bounded entity identifier |
| `--media-control-endpoint` | Required; canonical absolute camera-source Unix socket path, at most 107 bytes |
| `--media-control-target-id` | Required; bounded source target identifier |
| `--media-rtp-port` | Required, no default; 1–65535 |
| `--media-rtp-host` | `127.0.0.1`; also accepts `localhost` |
| `--media-source-id` | `usb_cam`; bounded source identifier |
| `--media-control-instance-id` | Empty; plugin assigns the source instance |
| `--camera-profile` | `world_wide_4k30_110` |
| `--camera-profiles-file` | Installed `share/gazebo_sim_camera/config/world_camera_profiles.yaml` |
| `--media-plugin-filename` | `libxgc_gazebo_media_camera.so`; an absolute plugin path is also accepted |
| `--camera-link-frame` | `usb_cam_link` |
| `--optical-frame` | `usb_cam_optical_frame` |
| `--snapshot-pose-frame-id` | `world`; declares the native capture pose domain |
| `--width` | `profile`; explicit 16–4096 pixels |
| `--height` | `profile`; explicit 16–2160 pixels |
| `--fps` | `profile`; explicit 0.1–240 Hz |
| `--hfov-degrees` | `profile`; explicit (0, 180) degrees, converted to radians |
| `--near-clip` | `profile`; finite and positive |
| `--far-clip` | `profile`; finite and greater than near clip |
| `--noise-stddev` | `profile`; finite and nonnegative |
| `--media-bitrate` | `profile`; 128000–2147483647 bits/s |
| `--media-max-bitrate` | `profile`; at least average bitrate, at most 2147483647 bits/s |
| `--media-pacing-bitrate` | `profile`; at least maximum bitrate, at most 2147483647 bits/s |
| `--media-vbv-buffer-milliseconds` | `profile`; 50–2000 ms |
| `--snapshot-jpeg-quality` | `profile`; 50–100 |
| `--snapshot-jpeg-backend` | `auto`; `auto`, `hardware`, or `cpu` |
| `--publish-encoded-video` | `true`; `true` or `false` |
| `--encoded-video-topic` | `/xgc/camera/world/video_h264` |
| `--frame-timing-topic` | `/xgc/camera/world/frame_timing` |
| `--stream-info-topic` | `/xgc/camera/world/stream_info` |
| `--xgc-camera-info-topic` | `/xgc/camera/world/camera_info`; embedded as the plugin CameraInfo topic |
| `--encoded-publisher-queue-capacity` | `8`; 1–128 |
| `--static` | `true`; `false` retains physics with link gravity disabled |
| `--x`, `--y`, `--z` | `-4.0`, `0.0`, `1.5` metres; finite |
| `--roll`, `--pitch`, `--yaw` | `0.0`, `0.0`, `0.0` radians; finite |
| `--timeout` | `50` seconds; finite (0, 55] |
| `--help` | Prints usage without importing the placement module or contacting a world |

`config/world_camera_profiles.yaml` remains the canonical optics, render
cadence, encoder and snapshot configuration. Its shipped
`world_wide_4k30_110` profile selects 3840×2160 at 30 fps, a 110° ideal
rectilinear pinhole with zero distortion, 24/36/72 Mbit/s average/maximum/pacing
budgets, 500 ms VBV and JPEG quality 90. Numeric overrides default to `profile`.
Configured cadence is a sensor target, not proof of end-to-end throughput.

`static_camera.launch` now runs only the contract publisher and the per-camera
TF nodes. It starts neither Gazebo nor placement and publishes no description
parameter. The installed definition retains readiness `none`, no services and
no `publishAs`. The plugin inside gzserver is the camera-source authority and
binds the supplied endpoint with `ReclaimUnreachable`, reclaiming unreachable
stale sockets and refusing a reachable owner.

Start the matching runtime separately at the action start:

```bash
roslaunch --wait gazebo_sim_camera static_camera.launch \
  model_name:=yard_camera camera_name:=yard_cam media_source_id:=yard_cam \
  media_control_endpoint:="$MEDIA_CONTROL_ENDPOINT" \
  media_control_target_id:="$MEDIA_CONTROL_TARGET_ID" python_executable:="$SELECTED_PYTHON" \
  camera_link_frame:=yard_cam_link optical_frame:=yard_cam_optical_frame \
  camera_profile:=world_wide_4k30_110 x:=-3 y:=2 z:=2 yaw:=-0.3
```

Shared runtime/model inputs (profile file, profile, dimensions/FOV overrides,
source ID, link/optical frames and truth pose) must match. The publisher alone
uses `camera_name`, `intrinsic_file`, `parent_frame`, `mode`, `publish_truth_tf`,
`gt_camera_link_frame`, `gt_optical_frame`, `calibration_root`,
`resolved_extrinsic_json`, `frame_roles_json`, `publish_xgc_contract`,
`xgc_transform_topic`, `use_sim_time` and `python_executable`. These are not
placement options. The six pose values are inputs of BOTH sides: the placement command puts the
model there, and the runtime definition (parameters `x`, `y`, `z`, `roll`, `pitch`, `yaw`) feeds the
truth transform. Core binds the same authored values to the command and to the definition; the
runtime never creates or moves a model.

The publisher starts before the source exists. Its existing off-thread
`CameraPeer` watches the source directory, applies CameraInfo metadata once
per source instance and reconciles lost replies. TF publication remains
available while the source is absent. The plugin's shared scene master watcher
lets native source control work before ROS appears; a replaced master ends the
bound Gazebo process. Consumers can join late or restart independently.

## TF and calibration

- `mode:=truth publish_truth_tf:=true` publishes the supplied truth pose under
  `parent_frame -> camera_link_frame -> optical_frame` on ROS TF.
- `mode:=calibration` adds no truth TF nodes.
- `mode:=validation` publishes truth on the separate `*_gt` chain.

The XGC contract transform topic remains the estimated camera chain in all
modes. Explicit intrinsic selection and frozen extrinsic application remain
owned by the runtime publisher. Applying estimates preserves native optics and
capture truth. Use `--static false` when a calibration helper will move the
camera through the world's simulation-v1 operations.

The combined intrinsic/extrinsic world launches, camera entity lifecycle node,
keyboard teleop, intrinsic sweep and combined Gazebo shutdown script were
removed. Core owns world startup and placement; the calibration product owns
interactive movement and capture. Existing calibration world/model assets
remain with the scenes product. `camera_ar_rviz.launch` is a view-only RViz
launch for an already running camera.

## Tests and packaging

```bash
source /opt/ros/noetic/setup.bash
nice -n 19 python3 test/test_place_world_camera.py
nice -n 19 python3 test/test_world_camera_profiles.py
nice -n 19 python3 test/static_product_contract.py
nice -n 19 bash .xgc2/scripts/check_package_compliance.sh
catkin_make -j2 run_tests_gazebo_sim_camera
catkin_test_results
```

The CLI tests inject a fake shared placement module, test validation and JSON
output, and compare six rendered URDFs to base-commit `roslaunch --dump-params`
references after XML normalization. They cover profiles, static/dynamic
models, encoded video on/off, endpoints, RTP ports, frame/topic overrides and
encoder/snapshot options. Publisher transport tests retain source-late/source-
restart and metadata reconciliation cases. The C++ source-control tests retain
stale socket reclaim and live owner refusal.

The live ROS contracts (`test/static_camera_contract.test` and the hardware
variant) require a camera already placed by Core or a station fixture. Supply
their source endpoint, target, RTP allocation and interpreter; they start only
the matching runtime and assertions. The fixture owns world startup, finite
placement and complete teardown. The hardware variant additionally requires
its model to use `--snapshot-jpeg-backend hardware` and an available backend.
Live Gazebo/rendering/NVENC checks are separate from these local unit tests.

The Debian package installs the process definition under
`/usr/share/xgc2/process-definitions`, the ROS package share/executable
directories, the placement CLI with mode 0755, and
`/opt/ros/noetic/lib/libxgc_gazebo_media_camera.so`. OpenGL, GLEW and JPEG
runtime dependencies remain explicit. `libnvidia-encode.so.1` is dynamically
loaded from the target's matching NVIDIA driver; no driver or CUDA image is
installed by the main plugin.
