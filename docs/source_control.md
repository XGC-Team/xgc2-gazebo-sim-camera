# Native camera source control

The authoritative shared contract is `products/common/media-edge/contracts/source-control-v1.md`.
The Gazebo SensorPlugin supplies one bounded SDK HTTP host per process for up
to 16 sources. Every sensor must provide the same explicit `controlEndpoint`
and `controlTargetId`, and the same optional `controlInstanceId`. There is no
endpoint or hostname identity fallback. The endpoint's owned parent directory
must already exist with mode 0700; the SDK owns the 0600 socket and lease.

GET `/v1/describe` is the sole unfenced service discovery route. It returns the
complete `camera-source` ServiceRef and source roster. GET `/v1/media/sources`
is a bound metadata roster. All domain calls use the SDK instance fence.
The final source prefix is `/v1/media/sources/{sourceId}`:

| Method and suffix | Body | Result |
| --- | --- | --- |
| GET `/describe` | None | CamelCase descriptor, RTP binding, capabilities and native force-IDR policy |
| GET `/status` | None | Snake_case lifecycle state, desired/applied revisions and native health |
| GET `/config` | None | Flat desired/applied configuration with top-level revisions |
| POST `/start` | `{}` | Applied only after a rendered frame is accepted by native NVENC encoding |
| POST `/stop` | `{}` | Applied after RTP queue and in-flight sends drain, and native capture/encoding demand stops |
| POST `/request-keyframe` | `{}` | Applied only after a native NV_ENC_PIC_FLAG_FORCEIDR submission is accepted; requires an active encoder |
| POST `/capture` | `{snapshotId,includeRgb,requireFresh,requestKeyframe}` | Immutable same-frame multipart metadata/JPEG/optional exact RGB8 |
| PATCH `/config` | `{expected_revision,persist:false,config:{jpeg_quality?,calibration?}}` | Real render-owner application and optional CameraInfo publication acknowledgement |

Bare GET of the prefix is 404. Capture has exactly one camelCase parser, with
includeRgb default true and requireFresh/requestKeyframe default false. Every
capture uses a fresh rendered frame accepted after admission. MIME disposition
is `inline`, with ordered metadata, JPEG and optional RGB parts. Metadata has a
positive frameSequence, the native Scene::SimTime timestamp, and that frame's
optical pose. `calibrationState` is `available` or `unavailable`. Available K
comes from the camera's actual ProjectionMatrix, D from its LensDistortion;
width alone never invents calibration. `calibrationSource:"native-projection"`
is a diagnostic. If the native projection is unsuitable, K/D are omitted.

Descriptor fields preserve camelCase: protocolVersion, sourceId, codec,
rtpPayloadType, rtpClockRate, rtpHost, rtpPort, width, height, fps, frameId,
capabilities, keyframeRequestSupported and keyframePolicy. This NVENC source
implements native-force-idr. A missing device or native encoder failure returns
an explicit failure; it cannot produce an applied start/keyframe receipt.
CPU JPEG capture works independently while the H264 source is idle.

Lifecycle success has ok, source_id, state, active, completion:"applied" and
configuration_revision. Status uses desired_active/applied_active and native
idle/starting/active/stopping/faulted states. Desired/applied divergence remains
visible after failure. Cancellation does not relinquish a consumed native
transition. Stop preserves independent ROS data and capture consumers.

Configuration desired/applied are flat configuration objects. Revisions are
only at top-level desired_revision/applied_revision/persisted_revision.
persistence is ephemeral, persisted_revision is null, and mutable_fields
advertises jpeg_quality (50–100) and calibration when ROS CameraInfo publication
is enabled. Authored RTP destination, bitrate, dimensions, cadence and backend
are startup bindings; attempts to change them report restart_required conflict.
Unknown fields and invalid types fail; stale expected_revision conflicts.

Calibration is a source-specific real metadata extension. Its exact body is:

```json
{"expected_revision":1,"persist":false,"config":{"calibration":{"scope":"calibration-metadata","model":"plumb_bob","width":320,"height":180,"camera_matrix":[123,0,159.5,0,124,89.5,0,0,1],"distortion":[0.01,0,0,0,0]}}}
```

Supported model/coefficient counts are plumb_bob/5,
rational_polynomial/8 and equidistant/4. Dimensions must match the native source,
coefficients must be finite, focal lengths positive and the bottom K row [0,0,1].
The native render owner applies the typed record and the existing ROS data worker
publishes a latched CameraInfo. Completion requires actual publication:
published_calibration_revision equals applied_revision and receipt.stage is
completed. This changes estimated CameraInfo metadata; native optics and capture
K remain the actual renderer values. Disabled ROS publication removes this
capability. No generic world sensor calibration support is advertised.

Bound source extensions retain GET `/receipts/{request_id}` (32 terminal entries)
and GET `/observe/{event_revision}` (four held observers). Receipt expiry is
404 and does not prove absence of effects. The receipt records request identity,
desired/applied/persisted revisions, stage and effects.applied. Failed native
work never returns a completed receipt.

The formal SDK RuntimePolicy is resolved once from XGC2_XRPC_ settings and
passed to the HTTP host. GET `/v1/runtime-policy` reports values, sources and
ceilings. Defaults/ceilings are eight connections, four in-flight calls, 64 KiB
requests and 64 MiB responses; smaller explicit values apply. One global capture
reservation stays owned through cancellation/removal until submitted work ends.
Capture geometry is at most 4096×2160, JPEG 32 MiB, RGB 26,542,080 bytes and the
capture budget four seconds. Native render callbacks receive typed records;
JSON and HTTP run on the one SDK IO executor.

Source membership follows actual native sensor lifecycle. Create/remove/reset
belongs to the existing simulation-v1 world host. Camera helpers require the
complete simulation_service_ref_json and target_id; Client.from_service applies
both target and instance fencing. The process owner explicitly starts/stops the
prepared native world. Camera launch also requires media_control_endpoint,
media_control_target_id and a selected python_executable with Python >=3.10,
the official SDK wheel and matching ROS dependencies. It does not use XRPC to
launch processes. Source effects remain ephemeral; captures are retained by
media/archive owners rather than written by this plugin.

The engine compilation unit stays C++17; the SDK facade is C++20. Existing
isolated evidence covers fresh native JPEG/RGB, actual ROS CameraInfo and
RuntimePolicy application under Classic 11.15.1/GCC13. The final contract and
new NVENC start/IDR/drain gates have only static compilation since the user's
native-test hard stop. Hardware NVENC, final cross-product interoperability,
Focal ABI/packages and deployed workflows have not been certified.
