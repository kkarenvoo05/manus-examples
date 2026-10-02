# Manus glove bridge container

Runs the Manus SDK + `glove_bridge` TCP server on host port 9090. Stream is
consumed by `manus_glove.ManusGloveBridgeClient` and the `web/glove_web.py`
visualizer. Start it with `./run.sh bridge` from the repo root.

## Use the pre-built image (recommended)

If you already have a `docker-dex-dev:latest` image from an earlier glove
setup, tag it once and skip the 30+ min rebuild:

```bash
docker tag docker-dex-dev:latest rllg2-manus-bridge:latest
```

`docker compose up -d` will then use the tagged image directly.

## Build from scratch

If the image isn't present, `docker compose up -d` will build it from the
included `Dockerfile`. That installs ROS2 Humble, builds gRPC v1.28.1 from
source, downloads the Manus C++ SDK, and compiles `glove_bridge.c`. Takes
~30 min and produces a ~10 GB image.

## What runs inside

`entrypoint.sh` starts two background processes:
1. `ros2 run manus_ros2 manus_data_publisher` — Manus SDK reading USB gloves.
2. `glove_bridge` — C TCP server on :9090 that re-publishes the ROS2 topic as
   line-delimited YAML.

The container itself just sleeps; everything useful happens via the host
network on port 9090.
