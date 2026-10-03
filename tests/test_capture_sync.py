import base64
import json

import pytest

from manus_glove.capture_sync import CaptureSession, clock_pair, extract_fields


def test_session_preserves_payload_and_unknown_source_timing(tmp_path):
    root = tmp_path / 'take'
    with CaptureSession(root, session_id='session', take_id='take', streams={'/hand': {'type': 'test/msg/Hand'}}) as session:
        session.record('/hand', b'\x00\xff', receipt={'monotonic_ns': 100, 'unix_ns': 200, 'pair_span_ns': 2})
        session.record('/hand', b'ab', receipt={'monotonic_ns': 110, 'unix_ns': 210, 'pair_span_ns': 2}, source={'source_frame_id': 8})
    manifest = json.loads((root / 'manifest.json').read_text())
    rows = [json.loads(s) for s in (root / 'samples.jsonl').read_text().splitlines()]
    assert manifest['status'] == 'complete'
    assert manifest['counts'] == {'/hand': 2}
    assert rows[0]['source_timestamp_raw'] is None
    assert rows[0]['source_timestamp_kind'] == 'unknown'
    assert rows[0]['timestamp_stage'] == 'ros_subscriber_callback_entry'
    assert rows[0]['recorder_sequence'] == 0
    assert rows[1]['source_frame_id'] == 8
    assert base64.b64decode(rows[0]['cdr_base64']) == b'\x00\xff'
    assert len((root / 'clock_pairs.jsonl').read_text().splitlines()) >= 2
    with pytest.raises(FileExistsError):
        CaptureSession(root, session_id='session', take_id='take', streams={})


def test_failure_leaves_failed_manifest(tmp_path):
    with pytest.raises(RuntimeError):
        with CaptureSession(tmp_path / 'take', session_id='s', take_id='t', streams={}) as session:
            raise RuntimeError('disk or callback failure')
    assert json.loads((tmp_path / 'take/manifest.json').read_text())['status'] == 'failed'


def test_explicit_source_mapping_never_guesses():
    from types import SimpleNamespace
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12)), glove_id=42)
    assert extract_fields(msg, {'device_id': 'glove_id'}) == {'device_id': 42}
    with pytest.raises(ValueError, match='missing'):
        extract_fields(msg, {'source_frame_id': 'missing'})
    with pytest.raises(ValueError):
        extract_fields(msg, {'receipt': 'glove_id'})


def test_clock_pair_brackets_wall_read():
    pair = clock_pair()
    assert all(isinstance(v, int) for v in pair.values())
    assert pair['pair_span_ns'] >= 0
