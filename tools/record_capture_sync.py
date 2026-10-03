#!/usr/bin/env python3
"""Record native ROS messages with explicitly labelled receipt telemetry."""
from __future__ import annotations

import argparse
import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import time

# Avoid importing the viewer package and its numerical dependencies in ROS.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'manus_glove'))
from capture_sync import CaptureSession, SOURCE_DEFAULTS, clock_pair, extract_fields


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--take-id', required=True)
    parser.add_argument('--topics', nargs='+', default=['/manus_glove_0', '/manus_glove_1'])
    parser.add_argument('--source-config', type=Path, help='Explicit topic field mappings and metadata JSON')
    parser.add_argument('--discovery-timeout', type=float, default=20)
    parser.add_argument('--qos-depth', type=int, default=1000)
    parser.add_argument('--reliability', choices=['best_effort', 'reliable'], default='best_effort')
    args = parser.parse_args(argv)
    if args.qos_depth < 1 or args.discovery_timeout <= 0:
        parser.error('depth and timeout must be positive')
    config = json.loads(args.source_config.read_text()) if args.source_config else {}
    for topic, spec in config.get('topics', {}).items():
        if topic not in args.topics:
            parser.error(f'configured topic is not selected: {topic}')
        unknown = (set(spec.get('fields', {})) | set(spec.get('constants', {}))) - SOURCE_DEFAULTS.keys()
        if unknown:
            parser.error(f'unsupported telemetry keys: {sorted(unknown)}')
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('manus_capture_sync')
    try:
        deadline = time.monotonic() + args.discovery_timeout
        while True:
            available = dict(node.get_topic_names_and_types())
            if all(topic in available for topic in args.topics):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f'topics not discovered: {set(args.topics) - available.keys()}')
            rclpy.spin_once(node, timeout_sec=0.1)
        streams = {}
        classes = {}
        for topic in args.topics:
            if len(available[topic]) != 1:
                raise ValueError(f'ambiguous topic type: {topic}')
            ros_type = available[topic][0]
            classes[topic] = get_message(ros_type)
            definition = subprocess.run(['ros2', 'interface', 'show', ros_type],
                                        check=True, capture_output=True, text=True, timeout=20).stdout
            streams[topic] = {'type': ros_type, 'interface_definition': definition,
                              'source_mapping': config.get('topics', {}).get(topic, {})}
        qos = QoSProfile(depth=args.qos_depth,
                         reliability=(ReliabilityPolicy.RELIABLE if args.reliability == 'reliable'
                                      else ReliabilityPolicy.BEST_EFFORT),
                         durability=DurabilityPolicy.VOLATILE)
        metadata = {**config.get('metadata', {}),
                    'ros_distro': os.environ.get('ROS_DISTRO'),
                    'python_version': sys.version,
                    'recorder_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    'telemetry_module_sha256': hashlib.sha256((Path(__file__).resolve().parents[1] / 'manus_glove/capture_sync.py').read_bytes()).hexdigest(),
                    'recorder_qos': {'depth': args.qos_depth, 'reliability': args.reliability},
                    'source_config': config}
        with CaptureSession(args.output, session_id=args.session_id, take_id=args.take_id,
                            streams=streams, metadata=metadata) as session:
            def callback(topic):
                spec = config.get('topics', {}).get(topic, {})
                def receive(message):
                    receipt = clock_pair()  # First work at callback entry, before serialization.
                    source = {**spec.get('constants', {}),
                              **extract_fields(message, spec.get('fields', {}))}
                    session.record(topic, serialize_message(message), receipt=receipt, source=source)
                return receive
            subscriptions = [node.create_subscription(classes[t], t, callback(t), qos) for t in args.topics]
            timer = node.create_timer(1.0, session.checkpoint)
            print(f'Recording to {args.output}. Timestamps are ROS receipt times, not sensor capture times.', flush=True)
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            print(f'Received counts: {session.counts}', flush=True)
            if any(count == 0 for count in session.counts.values()):
                raise RuntimeError('One or more selected topics received zero messages; inspect publishers and QoS.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
