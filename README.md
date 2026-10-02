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
