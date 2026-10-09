import json
import os
import shutil
import sqlite3
import struct
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'docker' / 'publisher_telemetry'))

import apply_patch  # noqa: E402
from diagnose_glove_telemetry import analyse, decode_glove, pose_hash  # noqa: E402

# The hook anchors as they appear in the MANUS 3.2.1 publisher (CRLF, as shipped).
PUBLISHER_FIXTURE = '''#include "ManusDataPublisher.hpp"
ManusDataPublisher *ManusDataPublisher::s_Instance = nullptr;
ManusDataPublisher::ManusDataPublisher() : Node("manus_data_publisher")
{
    s_Instance = this;
}
ManusDataPublisher::~ManusDataPublisher()
{
    s_Instance = nullptr;
}
ClientReturnCode ManusDataPublisher::InitializeSDK()
{
    const ClientReturnCode t_CallBackResults = RegisterAllCallbacks();
}
void ManusDataPublisher::PublishCallback()
{
        if (t_Publisher->second)
        {
            t_Publisher->second->publish(t_Msg);
        }
}
void ManusDataPublisher::OnRawSkeletonStreamCallback(const SkeletonStreamInfo *const p_RawSkeletonStreamInfo)
{
            s_Instance->m_GloveDataMap.insert_or_assign(t_NxtClientRawSkeleton.info.gloveId, t_NxtClientRawSkeleton);
}
void ManusDataPublisher::OnRawDeviceDataStreamCallback(const RawDeviceDataInfo *const p_RawDeviceDataInfo)
{
            s_Instance->m_RawSensorDataMap.insert_or_assign(t_NewData[i].id, t_NewData[i]);
}
void ManusDataPublisher::OnErgonomicsStreamCallback(const ErgonomicsStream* const p_Ergo)
{
            s_Instance->m_ErgonomicsDataMap.insert_or_assign(p_Ergo->data[i].id, t_Ergo);
}
void ManusDataPublisher::OnLandscapeCallback(const Landscape* const p_Landscape)
{
    Landscape* t_Landscape = new Landscape(*p_Landscape);
}
'''.replace('\n', '\r\n')
CMAKE_FIXTURE = 'add_executable(manus_data_publisher \n  src/manus_data_publisher.cpp\n  src/ManusDataPublisher.cpp)\n'


def make_package(root):
    (root / 'src').mkdir(parents=True)
    (root / 'src' / 'ManusDataPublisher.cpp').write_bytes(PUBLISHER_FIXTURE.encode())
    (root / 'CMakeLists.txt').write_text(CMAKE_FIXTURE)
    return root


def test_patch_applies_reapplies_and_reverts(tmp_path):
    package = make_package(tmp_path / 'manus_ros2')
    source = package / 'src' / 'ManusDataPublisher.cpp'
    assert apply_patch.main([str(package), '--check']) == 0
    assert source.read_bytes() == PUBLISHER_FIXTURE.encode()
    assert apply_patch.main([str(package)]) == 0
    first = source.read_bytes()
    assert first.count(b'manus-examples telemetry') == 10
    assert b'\n' not in first.replace(b'\r\n', b'')  # line endings preserved
    assert (package / 'src' / 'ManusTelemetry.cpp').exists()
    assert 'src/ManusTelemetry.cpp)' in (package / 'CMakeLists.txt').read_text()
    assert apply_patch.main([str(package)]) == 0  # re-applies from the original
    assert source.read_bytes() == first
    assert apply_patch.main([str(package), '--revert']) == 0
    assert source.read_bytes() == PUBLISHER_FIXTURE.encode()
    assert (package / 'CMakeLists.txt').read_text() == CMAKE_FIXTURE
    assert not (package / 'src' / 'ManusTelemetry.cpp').exists()


def test_patch_refuses_unknown_source(tmp_path, capsys):
    package = make_package(tmp_path / 'manus_ros2')
    source = package / 'src' / 'ManusDataPublisher.cpp'
    source.write_text(PUBLISHER_FIXTURE.replace('PublishCallback', 'PublishLoop'))
    before = source.read_bytes()
    assert apply_patch.main([str(package)]) == 1
    assert 'publish timer entry' in capsys.readouterr().err
    assert source.read_bytes() == before
    assert not (package / 'src' / 'ManusTelemetry.cpp').exists()


# ------------------------------------------------------------ bag helpers

class CdrWriter:
    def __init__(self):
        self.buf = bytearray(b'\x00\x01\x00\x00')

    def put(self, fmt, align, *values):
        self.buf += b'\x00' * ((-(len(self.buf) - 4)) % align)
        self.buf += struct.pack('<' + fmt, *values)

    def string(self, text):
        raw = text.encode() + b'\x00'
        self.put('I', 4, len(raw))
        self.buf += raw


def encode_glove(glove_id, side, poses):
    w = CdrWriter()
    w.put('i', 4, glove_id)
    w.string(side)
    w.put('i', 4, len(poses))
    w.put('I', 4, len(poses))
    for index, pose in enumerate(poses):
        w.put('ii', 4, index, 0)
        w.string('Joint')
        w.string('Chain')
        w.put('7d', 8, *pose)
    w.put('i', 4, 1)
    w.put('I', 4, 1)
    w.string('IndexMCPStretch')
    w.put('f', 4, 1.5)
    w.put('4d', 8, 0, 0, 0, 1)
    w.put('i', 4, 0)
    w.put('I', 4, 0)
    return bytes(w.buf)


def encode_string(text):
    w = CdrWriter()
    w.string(text)
    return bytes(w.buf)


def write_bag(path, rows):
    """rows: (topic, type, recv_ns, cdr_bytes)."""
    path.mkdir()
    con = sqlite3.connect(path / 'bag_0.db3')
    con.execute('create table topics(id integer primary key, name text, type text, '
                'serialization_format text, offered_qos_profiles text)')
    con.execute('create table messages(id integer primary key, topic_id integer, timestamp integer, data blob)')
    ids = {}
    for topic, kind, _, _ in rows:
        if topic not in ids:
            ids[topic] = len(ids) + 1
            con.execute('insert into topics values (?, ?, ?, "cdr", "")', (ids[topic], topic, kind))
    for topic, _, recv, data in sorted(rows, key=lambda r: r[2]):
        con.execute('insert into messages(topic_id, timestamp, data) values (?, ?, ?)', (ids[topic], recv, data))
    con.commit()
    con.close()
    return path


def test_glove_cdr_round_trip_and_hash():
    poses = [(0.01 * i, 0.5, -0.25, 0, 0, 0, 1) for i in range(25)]
    glove_id, side, nodes, ergonomics = decode_glove(encode_glove(7, 'Left', poses))
    assert (glove_id, side, ergonomics) == (7, 'Left', {'IndexMCPStretch': 1.5})
    assert [tuple(p) for _, p in nodes] == poses
    assert pose_hash([]) == 14695981039346656037
    assert pose_hash(nodes) != pose_hash(nodes[::-1])


def test_everything_stopping_is_host_stall(tmp_path):
    """Synthetic: SDK, timer and heartbeat all go quiet from 0.4 s to 0.7 s."""
    rows, records, t0 = [], [], 1_800_000_000 * 10**9
    pose = [(0.01 * i, 0.0, 0.0, 0, 0, 0, 1) for i in range(25)]
    quiet = lambda ns: 0.4e9 <= ns - t0 < 0.7e9
    for k in range(120):
        unix = t0 + int(k * 8.333e6)
        if quiet(unix):
            continue
        moved = [(x + 1e-5 * k, y, z, *q) for x, y, z, *q in pose]
        _, _, nodes, _ = decode_glove(encode_glove(1, 'Left', moved))
        records += [{'t': 'skel', 'unix': unix, 'mono': unix, 'glove': 1, 'seq': k + 1, 'pt': k},
                    {'t': 'tick', 'unix': unix, 'mono': unix, 'seq': k, 'dur_ns': 1, 'pub': [1]},
                    {'t': 'pub', 'unix': unix, 'mono': unix, 'glove': 1, 'topic': '/manus_glove_0', 'topic_seq': k,
                     'fresh': True, 'new_skel': 1, 'hash': f'{pose_hash(nodes):016x}', 'pt': k}]
        rows.append(('/manus_glove_0', 'manus_ros2_msgs/msg/ManusGlove', unix + 300_000,
                     encode_glove(1, 'Left', moved)))
    flushed = t0 - 1
    for b in range(21):
        unix = t0 + b * 50_000_000
        if quiet(unix):
            continue  # the heartbeat stops too; queued records go out with the next batch
        batch = {'schema': 'manus.publisher_telemetry.v1', 'session': 's', 'batch': b, 'unix': unix, 'mono': unix,
                 'dropped_records': 0, 'records': [r for r in records if flushed < r['unix'] <= unix]}
        flushed = unix
        rows.append(('/manus_telemetry', 'std_msgs/msg/String', unix + 200_000, encode_string(json.dumps(batch))))
    result = analyse(write_bag(tmp_path / 'bag', rows))
    gaps = [e for e in result['events'] if e['kind'] == 'gap']
    assert [e['label'] for e in gaps] == ['host_stalled']


# ------------------------------------------------- C++ end-to-end harness

def sdk_include():
    path = os.environ.get('MANUS_SDK_INCLUDE')
    return Path(path) if path and (Path(path) / 'ManusSDKTypes.h').exists() else None


@pytest.mark.skipif(sdk_include() is None or shutil.which('g++') is None,
                    reason='set MANUS_SDK_INCLUDE to the MANUS SDK include directory to run')
def test_real_telemetry_code_end_to_end(tmp_path):
    binary = tmp_path / 'harness'
    telemetry = ROOT / 'docker' / 'publisher_telemetry'
    subprocess.run(['g++', '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror', '-pthread',
                    '-I', str(telemetry), '-I', str(sdk_include()),
                    str(ROOT / 'tests' / 'telemetry_harness.cpp'), str(telemetry / 'ManusTelemetry.cpp'),
                    '-o', str(binary)], check=True)
    lines = subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout.splitlines()
    rows = []
    for line in map(json.loads, lines):
        if 'telemetry' in line:
            rows.append(('/manus_telemetry', 'std_msgs/msg/String', line['recv'],
                         encode_string(json.dumps(line['telemetry']))))
        else:
            g = line['glove']
            rows.append((g['topic'], 'manus_ros2_msgs/msg/ManusGlove', line['recv'],
                         encode_glove(g['id'], g['side'], [tuple(p) for p in g['poses']])))
    result = analyse(write_bag(tmp_path / 'bag', rows))
    for report in result['gloves'].values():
        assert report['matched_to_publish_records'] == report['messages']  # every message paired by hash
    assert result['telemetry']['batches_missing_in_bag'] == 0

    events = {(e['kind'], e['label']) for e in result['events']}
    # Each injected fault produces exactly its label (times are approximate, so match by label set).
    assert ('gap', 'publisher_stalled') in events
    assert ('gap', 'sdk_stalled') in events
    assert ('stale', 'radio_disconnect') in events
    assert ('gap', 'transport_recorder_stalled') in events
    assert ('twitch', 'in_stream_pose') in events
    assert {label for kind, label in events if kind == 'gap'} == {
        'publisher_stalled', 'sdk_stalled', 'transport_recorder_stalled'}
    radio = [e for e in result['events'] if e['label'] == 'radio_disconnect']
    assert {e['topic'] for e in radio} == {'/manus_glove_1'}
    assert any('pairing state unpaired' in item for e in radio for item in e['evidence'])
    lost = [e for e in result['events'] if 'never reached the bag' in e['reason']]
    assert {e['topic'] for e in lost} == {'/manus_glove_0', '/manus_glove_1'}
