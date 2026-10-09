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

## Publisher telemetry: explaining gaps and finger twitches

The stock MANUS publisher re-sends its latest cached pose on a fixed 120 Hz
timer, and never publishes connection state, signal strength or SDK timestamps.
`docker/publisher_telemetry/` patches it to also publish **`/manus_telemetry`**
(batched JSON in `std_msgs/String`, every 50 ms). The glove topics are unchanged,
so the viewer and existing bags/tools keep working. Tested against the ROS2
package in MANUS SDK 3.1.1 (this Dockerfile) and 3.2.1.

What it records (clocks: `mono` = steady_clock, `unix` = system clock, ns):

| record | when | contents |
|---|---|---|
| `skel` / `ergo` / `raw` | every SDK callback, per glove | SDK `publishTime` (raw and Unix ms), per-glove sequence |
| `land` | device list change, and ≥1 Hz | per glove: paired state, `tx_strength` (signal strength, raw SDK units), battery, `lastSeen`, IMU calibration; per dongle: radio channel, firmware |
| `sys` | SDK system message | e.g. `LibDebugRxStall`, `LibDebugTxStall`, `LibDebugReplugDongle` |
| `log`, `connect`, `disconnect` | SDK log/connection callbacks | text and host |
| `tick` | every publish-timer run | duration, which gloves were published |
| `pub` | every glove message | topic sequence, SDK `publishTime`, `fresh` (new SDK skeleton vs re-sent cache), SDK updates since last publish, pose hash |

Every batch header also carries the publisher pid/host/boot, patch hash and
callback registration codes, and an empty batch is still sent as a heartbeat.

### On the glove machine

```bash
git pull                                              # this branch
docker/publisher_telemetry/install_in_container.sh manus-bridge-arm64
tools/record_glove_take.sh take_001 manus-bridge-arm64   # Ctrl+C to stop
python3 tools/diagnose_glove_telemetry.py ~/manus-recordings/take_001/bag \
    --kernel-log ~/manus-recordings/take_001/kernel.log
```

`install_in_container.sh` copies the patch into the running container, applies
it to `/opt/ros2_ws/src/manus_ros2` (set `WS` if elsewhere), rebuilds that
package, restarts `manus_data_publisher` (log: `/tmp/manus_data_publisher.log`
in the container) and waits for `/manus_telemetry`. Re-run it after pulling
updates or recreating the container; `REVERT=1` restores the stock publisher.
New images built from `docker/Dockerfile` include the patch.

`record_glove_take.sh` records both glove topics and `/manus_telemetry` into one
bag and saves, next to it, the kernel log (`sudo dmesg`: USB/dongle disconnects),
publisher and container logs, `ros2 bag info`, and a host snapshot (USB tree,
processes such as `glove_bridge` that add CPU load). Output goes to
`~/manus-recordings/<take>/`. `CAPTURE_SYNC=<session_id>` also runs
`tools/record_capture_sync.py` on the same topics.

`diagnose_glove_telemetry.py` (numpy only) pairs every bag message with its
publish record by pose hash and writes `diagnosis.json` and `events.csv`:

- **gap**: no message for a glove for >100 ms; **stale**: messages kept arriving
  but the SDK pose did not change for >100 ms; **twitch**: a fingertip moved
  >0.03 native units between messages.
- labels: `radio_disconnect`, `sdk_stalled`, `publisher_stalled`,
  `transport_recorder_stalled`, `host_stalled`, `in_stream_pose` (a twitch the
  SDK itself produced with clean delivery: sensing, solver, fit or
  interference), or `unclassified`. The decision rules are at the top of the
  script. Bags without telemetry are still scanned, with every event
  `unclassified`.

Signal strength has no documented unit in the SDK, so compare it across
conditions (dongle close vs far, wired vs wireless) rather than against an
absolute threshold. `tests/test_publisher_telemetry.py` drives the real
telemetry code through one injected fault of each kind; it needs
`MANUS_SDK_INCLUDE=<SDK>/ROS2/ManusSDK/include`.

## Capture synchronization telemetry (first implementation)

`tools/record_capture_sync.py` subscribes directly to ROS topics and preserves
**every received message** as base64 CDR, plus subscriber callback receipt
telemetry. Run it alongside `ros2 bag record` to retain normal bag playback.
This is not a rosbag replacement and does not recover sensor capture times.
It does not use the TCP viewer or its latest-frame cache.

`ManusGlove` carries no timestamps, frame IDs or device IDs, so those fields in
this recorder's output stay null; the SDK-side values are on `/manus_telemetry`
(see Publisher telemetry above). Recorder sequences count received
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

SDK-side times, device IDs and connection state now come from
`/manus_telemetry` (see Publisher telemetry above) rather than from fields in
`ManusGlove`, which has none. Record that topic too (`--topics /manus_glove_0
/manus_glove_1 /manus_telemetry`); its `pub` records carry the pose hash that
pairs them with glove messages.

Offline body/hand alignment should produce an explicit shared timeline with
clock-mapping provenance. The markerless-mocap SOMA exporter accepts that timeline
via `--timeline ... --require-timing`; neither tool fits a clock mapping yet.
