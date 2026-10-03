# manus-examples

Stream Manus glove data out of the Manus SDK and visualize it in a browser.

```
Manus gloves (USB) -> docker: manus_data_publisher (ROS2) -> glove_bridge (TCP :9090)
                   -> web/glove_web.py (WebSocket :8765, HTTP :8000) -> browser
```

Requires [uv](https://docs.astral.sh/uv/) and `python3`. The browser loads three.js
from jsdelivr, so it needs internet access.

## Web glove viewer

On the Linux machine the gloves are plugged into:

```bash
./run.sh web
```

This starts the docker bridge if nothing is listening on :9090, then serves the
viewer. Open http://localhost:8000.

Without gloves (any OS):

```bash
./run.sh web --fake
```

Gloves on another machine (e.g. viewing from a Mac; Docker Desktop can't pass
USB through to the container):

```bash
# on the glove machine
./run.sh bridge
# locally
./run.sh web <glove-machine-hostname>
```

## Glove bridge

```bash
./run.sh bridge          # docker compose up -d, wait for :9090
./run.sh bridge logs
./run.sh bridge down
./run.sh bridge build    # ~30 min, ~10 GB; see docker/README.md to reuse an existing image
```

Wire format: each line of `ros2 topic echo /manus_glove_{0,1}` (YAML `ManusGlove`
messages) prefixed with `GLOVE0:` / `GLOVE1:`, messages terminated by `GLOVEn:---`.
`./run.sh fake` serves the same format with synthetic hands.

Raw node layout (25 nodes): `0` wrist, thumb `1-4`, index `5-9`, middle `10-14`,
ring `15-19`, pinky `20-24`, each finger ordered base to tip.

## Python client

```python
from manus_glove import ManusGloveBridgeClient

client = ManusGloveBridgeClient("127.0.0.1", 9090)
client.start()
frame = client.wait_for_first_frame("right", timeout_s=5.0)
if frame is not None:
    print(frame.keypoints.shape)  # (25, 7): x, y, z, qw, qx, qy, qz
    print(frame.ergonomics)       # e.g. {"IndexMCPStretch": 42.0, ...}
client.close()
```

`manus_glove.start_manus_bridge()` brings the docker bridge up from Python and
tears it down at exit.

## Capture synchronization telemetry (first implementation)

`tools/record_capture_sync.py` subscribes directly to ROS topics and preserves
**every received message** as base64 CDR, plus subscriber callback receipt
telemetry. Run it alongside `ros2 bag record` to retain normal bag playback.
This is not a rosbag replacement and does not recover sensor capture times.
It does not use the TCP viewer or its latest-frame cache.

The installed MANUS 3.2.1 publisher is not in this repository. Until that source
is instrumented or its existing fields are mapped, SDK timestamps, frame IDs,
device IDs and SDK callback times remain null. Recorder sequences count received
messages; they cannot prove there were no upstream drops. A recorder epoch is
not a device reconnect epoch. No clock synchronization is claimed by this tool.

### Run inside the existing Ubuntu ROS container

After pulling this fork in Ubuntu, copy the recorder into the running container
(no image rebuild is needed for this Python-only addition):

```bash
cd ~/manus-examples
sudo docker exec manus-bridge-arm64 mkdir -p /opt/manus_capture/tools /opt/manus_capture/manus_glove
sudo docker cp tools/record_capture_sync.py manus-bridge-arm64:/opt/manus_capture/tools/
sudo docker cp manus_glove/capture_sync.py manus-bridge-arm64:/opt/manus_capture/manus_glove/
sudo docker exec -it manus-bridge-arm64 bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/ros2_ws/install/setup.bash
  python3 /opt/manus_capture/tools/record_capture_sync.py \
    --output /root/recordings/take_001_telemetry \
    --session-id session_001 --take-id take_001
'
```

Start before the motion; wait for the recording message. Stop with Ctrl+C.
The output directory must be new. Verify both printed topic counts are nonzero.
Use the same session/take IDs for the RGB-D capture. Copy the result to Ubuntu:

```bash
mkdir -p ~/manus-recordings
sudo docker cp manus-bridge-arm64:/root/recordings/take_001_telemetry ~/manus-recordings/
```

The copied scripts persist in this container, but must be copied again after a
container is recreated or after updating the scripts. Run on live acquisition;
recording bag playback timestamps measures replay receipt, not original capture.

The default best-effort subscriber can receive reliable and best-effort
publishers, but delivery is not guaranteed. `--reliability reliable` requires a
compatible publisher. Check native bag counts and SDK frame gaps when available.
DDS queues and synchronous disk I/O can affect receipt latency; this first
implementation does not measure all upstream drops.

### Files and explicit source mappings

- `samples.jsonl`: topic, local sequence/epoch, clock pair at subscriber callback
  entry, optional source fields, and complete serialized message bytes.
- `clock_pairs.jsonl`: periodic monotonic/Unix clock correspondence with the
  duration bracketing each wall-clock read. VM and host monotonic clocks are
  distinct; these pairs alone do not resolve device or network latency.
- `manifest.json`: session/take, host/boot, ROS types and expanded interface
  definitions, mapping configuration, recorder hashes, QoS, and counts. Status
  `complete` means orderly close, not verified synchronization. Abrupt termination
  leaves `recording`; handled errors leave `failed`. Buffered tail data can be
  lost on abrupt termination.

After inspecting `ros2 interface show` and the installed publisher, provide a
`--source-config /path/in/container/config.json` with this structure:

```json
{
  "metadata": {
    "manus_sdk_version": "3.2.1",
    "calibration_ref": "calibration/session_001.json"
  },
  "topics": {
    "/manus_glove_0": {
      "fields": {},
      "constants": {"source_timestamp_kind": "unknown"}
    },
    "/manus_glove_1": {"fields": {}, "constants": {}}
  }
}
```

`fields` maps an output field such as `device_id` to its **verified** dotted ROS
field path. Absent configured fields cause an error instead of being guessed.
Supported fields are listed in `SOURCE_DEFAULTS` in `capture_sync.py`. Raw ROS
stamp objects retain their component fields. Do not label `header.stamp` as
sensor time without checking how the publisher constructs it. Do not assign
left/right based only on topic index. Include SDK/firmware versions, calibration,
skeleton topology, units and axes in metadata when known.

To finish SDK instrumentation, obtain the installed publisher sources from
`/opt/ros2_ws/src` in the actual 3.2.1 container. At each SDK callback entry,
record host monotonic/Unix times; preserve original SDK timestamps, their meaning
and units, source IDs and reset epochs through the ROS message. Skeleton and
ergonomics need separate source times if delivered by separate callbacks. This
requires reviewing and rebuilding that installed ROS package, not guessing
fields based on this fork's older Dockerfile.

Offline body/hand alignment should produce an explicit shared timeline with
clock-mapping provenance. The markerless-mocap SOMA exporter accepts that timeline
via `--timeline ... --require-timing`; neither tool fits a clock mapping yet.
