# Encoded camera recording contract

The Gazebo source performs one NVENC H264 encode and fans each complete
Annex-B access unit into two independent branches:

```text
Gazebo render target -> NVENC Annex-B AU
                         |-> bounded RTP pacer -> Media Edge/WebRTC
                         `-> bounded ROS worker -> CompressedVideo + FrameTiming
```

No ROS message is constructed or serialized in Gazebo's render-target
callback. The callback copies the already encoded AU into an eight-frame queue
(configurable from 1 to 128); a worker thread owns ROS serialization and
publication.

## Topics

The default source publishes:

| Topic | Type | Behavior |
| --- | --- | --- |
| `/xgc/camera/world/video_h264` | `foxglove_msgs/CompressedVideo` | One complete H264 Annex-B AU per message |
| `/xgc/camera/world/frame_timing` | `xgc_camera_msgs/FrameTiming` | One timing/identity sidecar per video message |
| `/xgc/camera/world/stream_info` | `xgc_camera_msgs/StreamInfo` | Latched stream/epoch metadata |
| `/xgc/camera/world/camera_info` | `sensor_msgs/CameraInfo` | Latched simulation-truth intrinsics |

`CompressedVideo.timestamp` and `FrameTiming.source_time` are the exact
`CameraSensor::LastMeasurementTime()` for the rendered frame. They are
simulation time, not wall time. `FrameTiming.native_source_time_ns` preserves
the same native simulation value, while `host_publish_realtime_ns` records when
the asynchronous worker began publication.

Record source-quality video without browser screen capture:

```bash
rosbag record \
  /xgc/camera/world/video_h264 \
  /xgc/camera/world/frame_timing \
  /xgc/camera/world/stream_info \
  /xgc/camera/world/camera_info \
  /xgc/camera/world/tf \
  /clock
```

Record all three encoded-stream topics together. A video message has no epoch
field of its own; its sidecar carries the epoch and frame sequence needed to
detect resets and dropped intervals during replay.

## Epoch and overload behavior

An epoch is an opaque non-zero token. It changes when:

- simulation time moves backwards;
- the NVENC session is recreated;
- the encoded ROS branch resumes after having no subscribers;
- the ROS publisher queue overflows.

Every new epoch starts on an IDR. After overload, the source discards queued
frames and all following delta frames, requests an IDR, and resumes publication
only at that IDR. `dropped_frames_before` on the first published frame reports
the discarded count. The RTP branch remains available and has its own bounded
whole-access-unit pacer.

When the ROS branch resumes from zero subscribers, Gazebo can invoke the
render-target listener once with the texture and measurement timestamp from
before the sensor was deactivated. The source explicitly discards that callback
and all old NVENC pending state. Epoch sequence zero is the forced IDR from the
first later render whose simulation source time differs from the dormant
sample (normally it has advanced; a simulation reset may move it backwards).

An additional subscriber joining an already active epoch also causes the next
frame to be an IDR, because ROS1 does not replay the current GOP to late
subscribers. That IDR does not change the epoch for existing recorders.

The overload guarantee above covers the plugin's own worker queue. ROS1 then
places video and timing messages in two independent publisher transport queues;
slow TCP subscribers can make roscpp discard from either queue without
back-pressure or a drop callback to the plugin. For a scientific recording,
match the two topics by their identical source timestamp and check
`FrameTiming.frame_sequence` for gaps after recording. An orphan message or a
sequence gap is a transport-loss indication even when
`dropped_frames_before == 0`. The recorder host must sustain the encoded
bitrate; a dedicated recording process with observable storage back-pressure
is the stronger choice when loss cannot be tolerated.

## Configuration

`static_camera.launch` exposes:

- `publish_encoded_video` (default `true`);
- `encoded_video_topic`, `frame_timing_topic`, and `stream_info_topic`;
- `encoded_publisher_queue_capacity` (default `8`).

Set unique topics, source ID, frames, RTP port, and control socket for every
additional camera.

## Still images

The private source-control `snapshot` transaction remains the authoritative
full-resolution still-image path. It renders on demand and returns JPEG, an
optional RGB payload, source timestamp, intrinsics, and the exact optical-frame
world pose captured by that render transaction. `poseFrameId` is independently
configured and defaults to `world`; it does not borrow the ROS camera parent
frame because the value comes from Gazebo `WorldPose()`. It works even when no
live video consumer is active. Both `describe` and every successful snapshot
declare `timestampClockDomain: "simulation"`; `timestampNanoseconds` is
therefore directly comparable with the encoded-frame source timestamp and ROS
simulation observations, not with Unix wall time.

The historical timer that repeatedly calls this transaction to populate
`/xgc/camera/world/image_raw/compressed` is disabled by default because it
forces GPU readback and JPEG encoding. Temporarily enable it for an older
consumer with:

```bash
roslaunch gazebo_sim_camera static_camera.launch \
  enable_continuous_jpeg_preview:=true \
  xgc_image_publish_rate:=10
```

Automated capture workflows should invoke explicit `snapshot` transactions at
their event or sampling boundary instead of enabling periodic JPEG polling.
