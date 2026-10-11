# Native camera source control

The authoritative shared contract is `products/common/media-edge/contracts/source-control-v1.md`.
The Gazebo SensorPlugin supplies one bounded SDK HTTP host per process for up
to 16 sources. Every sensor must provide the same explicit `controlEndpoint`
and `controlTargetId`, and the same optional `controlInstanceId`. There is no
endpoint or hostname identity fallback. The endpoint's owned parent directory
must already exist with mode 0700; the SDK owns the 0600 socket and lease.

GET `/v1/describe` is the sole unbound service discovery route. It returns:

```json
{
  "service": "camera-source",
  "api_version": "v1",
  "instance_id": "<hex>",
  "ready": true,
  "facts": {
    "target_id": "<target>",
    "sources": ["<sourceId>"],
    "limits": {
      "sources": 16,
      "concurrent_captures": 1,
      "connections": 8,
      "inflight": 4,
      "request_bytes": 65536,
      "response_bytes": 67108864
    },
    "service_ref": {
      "target_id": "<target>",
      "service": "camera-source",
      "api_version": "v1",
      "instance_id": "<hex>",
      "profile": "http.v1",
      "endpoint": {"kind": "unix", "address": "<granted-socket-path>"}
    }
  }
}
```

The optional query `wait_ready_ms=<0..30000>` holds an asynchronous reply until
ready or the requested wait elapses, then returns the current envelope. This
shared host is ready whenever it serves, even before any source registers or
after all sources unregister. Its valid readiness waits are therefore already
satisfied and complete immediately on the single HTTP executor, without a
readiness polling loop. The value must be a canonical decimal integer; empty,
signed, fractional, duplicate, extra or out-of-range query values return 400
`invalid_argument`.

XRPC matches discovery on the GET target's path before `?`, and preserves the
query for host validation. A readiness query never requires an instance header.
Consumers read the existing ServiceRef from `facts.service_ref` and bind it for
domain calls. GET `/v1/media/sources` keeps its bound `{sources, limits}` metadata
roster. All domain calls use the SDK instance fence.
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

The XRPC 0.2.0 host receives an explicit `HttpLimits` struct: eight connections,
four in-flight calls, 8192 header bytes, 64 KiB requests, 64 MiB responses, a
10-second call timeout, 5-second header and idle timeouts, and a 1-second shutdown
timeout. These bounds are fixed by the host; environment variables do not change
them. The envelope's transport limits come from that same struct. One global capture
reservation stays owned through cancellation/removal until submitted work ends.
Capture geometry is at most 4096×2160, JPEG 32 MiB, RGB 26,542,080 bytes and the
capture budget four seconds. Native render callbacks receive typed records;
JSON and HTTP run on the one SDK IO executor.

Source membership follows actual native sensor lifecycle. Create/remove/reset
belongs to the existing simulation-v1 world host. Camera helpers take the opaque
simulation_endpoint and target_id, watch the endpoint's existing parent directory,
and discover the current world through one unbound held describe. The discovered
ServiceRef supplies target and instance fencing. The spawner replays the same
authored camera entity once per world instance, including after restart, without
blocking its initialization on the world. Its process readiness is none, with no
services or publishAs; the plugin inside gzserver is the camera-source authority.
Camera launch also requires media_control_endpoint, media_control_target_id, the
inventory's allocated media_rtp_port and a selected python_executable with Python >=3.8,
the official SDK wheel and matching ROS dependencies. It does not use XRPC to
launch processes. The plugin binds its allocated camera path with XRPC
ReclaimUnreachable: a stale unreachable socket is reclaimed, while a live owner
makes binding fail. Paths are accepted as given and require only an existing
parent; the product does not derive service names or parse hashed filenames.
Source effects remain ephemeral; captures are retained by
media/archive owners rather than written by this plugin.

The engine compilation unit stays C++17; the SDK facade is C++20. Existing
isolated evidence covers fresh native JPEG/RGB and actual ROS CameraInfo
under Classic 11.15.1/GCC13. The standalone control-host unit test covers the
XRPC 0.2.0 envelope, unbound readiness queries, fixed limits and source registry.
The final native contract and
new NVENC start/IDR/drain gates have only static compilation since the user's
native-test hard stop. Hardware NVENC, final cross-product interoperability,
Focal ABI/packages and deployed workflows have not been certified.
