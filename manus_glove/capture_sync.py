"""Lossless ROS payload telemetry. Receipt time is never called capture time.

This module uses only the standard library and can run in the ROS container.
"""
from __future__ import annotations

import base64
import json
import os
import platform
import time
import uuid
from pathlib import Path

SOURCE_DEFAULTS = {
    'source_timestamp_raw': None,
    'source_timestamp_kind': 'unknown',
    'source_clock_id': None,
    'source_timestamp_units': None,
    'source_frame_id': None,
    'publisher_sequence': None,
    'device_id': None,
    'hand_side': None,
    'source_stream_epoch': None,
    'sdk_callback_monotonic_ns': None,
    'sdk_callback_unix_ns': None,
    'validity': None,
}


def clock_pair():
    """Bracket a wall-clock read; midpoint is an estimate, span is retained."""
    before = time.monotonic_ns()
    wall = time.time_ns()
    after = time.monotonic_ns()
    return {'monotonic_ns': (before + after) // 2,
            'unix_ns': wall, 'pair_span_ns': after - before}


def _plain(value):
    if hasattr(value, 'get_fields_and_field_types'):
        return {key: _plain(getattr(value, key)) for key in value.get_fields_and_field_types()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise ValueError(f'unsupported mapped field type: {type(value).__name__}')


def extract_fields(message, mapping):
    """Extract explicitly configured ROS field paths; fail on absent fields."""
    result = {}
    for name, path in mapping.items():
        if name not in SOURCE_DEFAULTS:
            raise ValueError(f'unsupported telemetry field: {name}')
        value = message
        if not isinstance(path, str) or not path:
            raise ValueError(f'invalid field path for {name}')
        for part in path.split('.'):
            if not hasattr(value, part):
                raise ValueError(f'missing source field: {path}')
            value = getattr(value, part)
        result[name] = _plain(value)
    return result


class CaptureSession:
    """Append-only evidence directory. Abrupt termination leaves status recording.

    Use from a single callback thread. Counts are received samples, not proof
    that the publisher, DDS or device dropped no frames.
    """
    def __init__(self, root, *, session_id, take_id, streams, metadata=None):
        if not session_id or not take_id or not isinstance(streams, dict):
            raise ValueError('session_id, take_id and a streams mapping are required')
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.counts = {topic: 0 for topic in streams}
        self.epoch = str(uuid.uuid4())
        boot_path = Path('/proc/sys/kernel/random/boot_id')
        self.manifest = {
            'schema': 'manus.capture_sync.v1', 'session_id': session_id,
            'take_id': take_id, 'recorder_epoch': self.epoch,
            'host': platform.node(),
            'boot_id': boot_path.read_text().strip() if boot_path.exists() else None,
            'status': 'recording', 'streams': streams, 'metadata': metadata or {},
            'timestamp_stage': 'ros_subscriber_callback_entry',
            'clock': {'monotonic': 'python.time.monotonic_ns',
                      'wall': 'unix_ns', 'mapping_to_session': None},
            'payload_encoding': 'base64 ROS 2 CDR',
            'capture_time_verified': False,
        }
        self._write_manifest()
        self.samples = (self.root / 'samples.jsonl').open('x')
        self.clocks = (self.root / 'clock_pairs.jsonl').open('x')
        self.closed = False
        self.checkpoint()

    @staticmethod
    def _line(handle, row):
        handle.write(json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n')

    def _write_manifest(self):
        self.manifest['counts'] = dict(self.counts)
        temp = self.root / '.manifest.tmp'
        with temp.open('w') as handle:
            json.dump(self.manifest, handle, indent=2, allow_nan=False)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp, self.root / 'manifest.json')

    def record(self, topic, payload, *, receipt, source=None):
        if self.closed:
            raise ValueError('session is closed')
        if topic not in self.counts:
            raise ValueError(f'unknown topic: {topic}')
        source = source or {}
        if set(source) - SOURCE_DEFAULTS.keys():
            raise ValueError('unknown source telemetry fields')
        row = {**SOURCE_DEFAULTS, **source,
               'topic': topic, 'recorder_epoch': self.epoch,
               'recorder_sequence': self.counts[topic],
               'timestamp_stage': 'ros_subscriber_callback_entry',
               'receipt': receipt,
               'cdr_base64': base64.b64encode(payload).decode('ascii')}
        self._line(self.samples, row)
        self.counts[topic] += 1

    def checkpoint(self):
        self._line(self.clocks, {**clock_pair(), 'recorder_epoch': self.epoch})
        for handle in (self.samples, self.clocks):
            handle.flush()
        self._write_manifest()

    def close(self, *, failed=False):
        if self.closed:
            return
        try:
            self.checkpoint()
            for handle in (self.samples, self.clocks):
                os.fsync(handle.fileno())
            self.manifest['status'] = 'failed' if failed else 'complete'
            self._write_manifest()
        finally:
            self.samples.close()
            self.clocks.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, kind, value, traceback):
        self.close(failed=kind is not None)
