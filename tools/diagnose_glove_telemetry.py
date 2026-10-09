#!/usr/bin/env python3
"""Label every glove gap, frozen-pose run and finger twitch in a Manus bag.

    python tools/diagnose_glove_telemetry.py ~/manus-recordings/take_001/bag \
        --kernel-log ~/manus-recordings/take_001/kernel.log

Needs a bag recorded with /manus_telemetry (docker/publisher_telemetry). Bags
without it still get their events listed, all labelled unclassified.

Events
  gap     no message for a glove for longer than --gap-ms (bag receipt time)
  stale   messages kept arriving but the SDK skeleton did not change for
          longer than --stale-ms (the 120 Hz timer re-sending a cached pose)
  twitch  a fingertip moved more than --twitch-step native units between
          consecutive messages

Labels (how each one is decided is in classify_gap / classify_stale)
  radio_disconnect            SDK reported the glove unpaired/absent, a radio
                              stall, a disconnect, a USB disconnect in the
                              kernel log, or only this glove's SDK stream stopped
  sdk_stalled                 SDK callbacks stopped for every glove, no
                              disconnect reported (includes dongle/USB stalls
                              the SDK does not report)
  publisher_stalled           SDK kept delivering but the publish timer did not
                              run, or ran and skipped this glove
  transport_recorder_stalled  published on time, reached the bag late or never
  host_stalled                every thread in the publisher process stopped
                              (SDK, timer and the telemetry heartbeat)
  in_stream_pose              twitch with clean delivery: the SDK produced that
                              pose (glove sensing, solver, fit, interference)
  unclassified                not enough telemetry to decide
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import re
import sqlite3
import struct
import sys
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

TELEMETRY_TOPIC = '/manus_telemetry'
GLOVE_TYPE = 'manus_ros2_msgs/msg/ManusGlove'
TIP_NODES = (4, 9, 14, 19, 24)
FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
RADIO_SYSTEM_TYPES = {'LibDebugReplugDongle', 'LibDebugRxStall', 'LibDebugTxStall'}
RADIO_LOG = re.compile(r'disconnect|reconnect|dongle|stall|unpair|lost', re.I)
KERNEL_USB = re.compile(r'usb \S+: (USB disconnect|new .*USB device|reset .*USB device)', re.I)
MATCH_LOOKAHEAD = 5000
NS = 1_000_000_000


# ---------------------------------------------------------------- decoding

class Cdr:
    """Little-endian CDR reader; alignment is relative to the 4-byte header."""
    def __init__(self, data):
        if data[:2] != b'\x00\x01':
            raise ValueError('not little-endian CDR')
        self.data, self.pos = data, 4

    def read(self, fmt, align):
        self.pos += (-(self.pos - 4)) % align
        values = struct.unpack_from('<' + fmt, self.data, self.pos)
        self.pos += struct.calcsize('<' + fmt)
        return values

    def u32(self):
        return self.read('I', 4)[0]

    def i32(self):
        return self.read('i', 4)[0]

    def string(self):
        size = self.u32()
        raw = self.data[self.pos:self.pos + size]
        self.pos += size
        return raw[:-1].decode('utf-8', 'replace')


def decode_glove(data):
    """manus_ros2_msgs/ManusGlove (identical in SDK 3.1.1 and 3.2.1)."""
    r = Cdr(data)
    glove_id = r.i32()
    side = r.string()
    r.i32()  # raw_node_count
    nodes = []
    for _ in range(r.u32()):
        node_id, _parent = r.i32(), r.i32()
        r.string(); r.string()  # joint_type, chain_type
        nodes.append((node_id, r.read('7d', 8)))  # position xyz, orientation xyzw
    r.i32()  # ergonomics_count
    ergonomics = {}
    for _ in range(r.u32()):
        name = r.string()
        ergonomics[name] = r.read('f', 4)[0]
    return glove_id, side, nodes, ergonomics


def pose_hash(nodes):
    """Same FNV-1a 64 as ManusTelemetry::PoseHash, over poses in message order."""
    value = 14695981039346656037
    for _, pose in nodes:
        for byte in struct.pack('<7d', *pose):
            value = ((value ^ byte) * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def decode_string(data):
    r = Cdr(data)
    return r.string()


def bag_files(path):
    path = Path(path)
    if path.is_file():
        return [path]
    files = sorted(path.glob('*.db3'))
    if not files:
        if list(path.glob('*.mcap')):
            raise SystemExit('MCAP bags are not supported; record with the default sqlite3 storage.')
        raise SystemExit(f'no .db3 files in {path}')
    return files


def read_bag(path):
    gloves = defaultdict(lambda: {'recv': [], 'hash': [], 'pos': [], 'glove_id': None, 'side': None})
    batches = []
    for file in bag_files(path):
        con = sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True)
        topics = {tid: (name, kind) for tid, name, kind in con.execute('select id, name, type from topics')}
        for tid, stamp, data in con.execute('select topic_id, timestamp, data from messages order by timestamp'):
            name, kind = topics[tid]
            if kind == GLOVE_TYPE:
                glove_id, side, nodes, _ = decode_glove(data)
                g = gloves[name]
                g['glove_id'], g['side'] = glove_id, side
                g['recv'].append(stamp)
                g['hash'].append(pose_hash(nodes))
                ordered = sorted(nodes)
                g['pos'].append([pose[:3] for _, pose in ordered])
            elif name == TELEMETRY_TOPIC:
                batch = json.loads(decode_string(data))
                batch['recv'] = stamp
                batches.append(batch)
        con.close()
    for g in gloves.values():
        g['recv'] = np.array(g['recv'], dtype=np.int64)
        g['pos'] = np.array(g['pos'], dtype=float)
    return dict(gloves), batches


def read_kernel_log(path):
    """USB events from `dmesg --time-format iso` output."""
    events = []
    for line in Path(path).read_text(errors='replace').splitlines():
        match = KERNEL_USB.search(line)
        if not match:
            continue
        stamp = line.split(' ', 1)[0].replace(',', '.')
        try:
            events.append((int(datetime.fromisoformat(stamp).timestamp() * NS), line.strip()))
        except ValueError:
            continue
    return events


# --------------------------------------------------------------- telemetry

class Telemetry:
    def __init__(self, batches):
        self.batches = sorted(batches, key=lambda b: (b['session'], b['batch']))
        self.records = defaultdict(list)
        for batch in self.batches:
            for record in batch['records']:
                record['_session'] = batch['session']
                self.records[record['t']].append(record)
        for rows in self.records.values():
            rows.sort(key=lambda r: r['unix'])
        self.flush_unix = sorted(b['unix'] for b in self.batches)
        self.tick_unix = [r['unix'] for r in self.records['tick']]
        sdk = self.records['skel'] + self.records['ergo'] + self.records['raw']
        self.sdk_unix = sorted(r['unix'] for r in sdk)
        self.skel_by_glove = defaultdict(list)
        for r in self.records['skel']:
            self.skel_by_glove[r['glove']].append(r['unix'])
        self.landscapes = self.records['land']
        self.land_unix = [r['unix'] for r in self.landscapes]

    @property
    def present(self):
        return bool(self.batches)

    def pubs_for(self, topic):
        return [r for r in self.records['pub'] if r['topic'] == topic]  # already in time order

    def health(self):
        sessions = defaultdict(list)
        for batch in self.batches:
            sessions[batch['session']].append(batch['batch'])
        missing = sum(max(seq) - min(seq) + 1 - len(seq) for seq in sessions.values())
        latency = sorted((b['recv'] - b['unix']) / 1e6 for b in self.batches)
        return {
            'batches': len(self.batches),
            'publisher_sessions': len(sessions),
            'batches_missing_in_bag': missing,
            'dropped_records_in_publisher': max((b['dropped_records'] for b in self.batches), default=0),
            'source_sha': sorted({b.get('source_sha', '') for b in self.batches}),
            'callback_registrations': self.batches[-1].get('registrations') if self.batches else None,
            'batch_delivery_latency_ms_median': latency[len(latency) // 2] if latency else None,
        }


def max_spacing(times, start, end):
    """Largest interval in [start, end] not containing any of the sorted times."""
    lo = bisect.bisect_right(times, start)
    hi = bisect.bisect_left(times, end)
    points = [start, *times[lo:hi], end]
    gaps = [(b - a, a, b) for a, b in zip(points, points[1:])]
    return max(gaps) if gaps else (0, start, end)


def glove_state(tel, glove_id, start, end):
    """Landscape view of one glove over [start, end]: list of (paired, present)."""
    if not tel.landscapes:
        return []
    lo = max(bisect.bisect_right(tel.land_unix, start) - 1, 0)
    hi = bisect.bisect_right(tel.land_unix, end)
    states = []
    for record in tel.landscapes[lo:hi]:
        gloves = {g['id']: g for g in record['devices']['gloves']}
        glove = gloves.get(glove_id)
        states.append({'present': glove is not None,
                       'paired': glove['paired'] if glove else None,
                       'tx_strength': glove['tx_strength'] if glove else None})
    return states


def radio_evidence(tel, kernel, glove_id, start, end, margin=NS // 4):
    start, end = start - margin, end + margin
    found = []
    for record in tel.records['sys']:
        if start <= record['unix'] <= end and record['type'] in RADIO_SYSTEM_TYPES:
            found.append(f"SDK system message {record['type']}: {record['info']}".strip())
    for kind in ('disconnect', 'connect'):
        for record in tel.records[kind]:
            if start <= record['unix'] <= end:
                found.append(f'SDK {kind} event')
    for record in tel.records['log']:
        if start <= record['unix'] <= end and RADIO_LOG.search(record['text']):
            found.append(f"SDK log ({record['sev']}): {record['text'][:160]}")
    for state in glove_state(tel, glove_id, start, end):
        if not state['present']:
            found.append('glove absent from SDK device list')
        elif state['paired'] != 'paired':
            found.append(f"glove pairing state {state['paired']}")
    for stamp, line in kernel:
        if start <= stamp <= end:
            found.append(f'kernel: {line[:160]}')
    return sorted(set(found))


# ---------------------------------------------------------- classification

def match_publishes(glove, pubs):
    """Pair each bag message with its publish record by pose hash, in order."""
    positions = defaultdict(list)
    for index, pub in enumerate(pubs):
        positions[int(pub['hash'], 16)].append(index)
    matches, cursor = [], 0
    for value in glove['hash']:
        candidates = positions.get(value, [])
        k = bisect.bisect_left(candidates, cursor)
        if k < len(candidates) and candidates[k] < cursor + MATCH_LOOKAHEAD:
            matches.append(candidates[k])
            cursor = candidates[k] + 1
        else:
            matches.append(None)
    return matches


def classify_gap(tel, kernel, glove, pubs, matches, i):
    gap = int(glove['recv'][i + 1] - glove['recv'][i])
    m0, m1 = matches[i], matches[i + 1]
    if not tel.present:
        return 'unclassified', 'bag has no /manus_telemetry', []
    if m0 is None or m1 is None:
        return 'unclassified', 'message could not be paired with a publish record', []
    p0, p1 = pubs[m0]['unix'], pubs[m1]['unix']
    evidence = radio_evidence(tel, kernel, glove['glove_id'], p0, p1)
    lost = m1 - m0 - 1
    if lost > 0:
        return ('transport_recorder_stalled',
                f'{lost} messages were published during the gap but never reached the bag', evidence)
    if p1 - p0 < gap / 2:
        return ('transport_recorder_stalled',
                f'published {(p1 - p0) / 1e6:.0f} ms apart but received {gap / 1e6:.0f} ms apart', evidence)
    # The publisher itself did not publish this glove during [p0, p1].
    tick_gap, t0, t1 = max_spacing(tel.tick_unix, p0, p1)
    if tick_gap < (p1 - p0) / 2:
        states = glove_state(tel, glove['glove_id'], p0, p1)
        if any(not s['present'] or s['paired'] != 'paired' for s in states):
            return ('radio_disconnect',
                    'publish timer kept running but the SDK listed this glove as absent/unpaired', evidence)
        return ('publisher_stalled',
                'publish timer kept running but skipped this glove (no skeleton/ergonomics in its cache)', evidence)
    sdk_gap, _, _ = max_spacing(tel.sdk_unix, t0, t1)
    flush_gap, _, _ = max_spacing(tel.flush_unix, t0, t1)
    if sdk_gap < tick_gap / 2:
        return ('publisher_stalled',
                f'publish timer stopped for {tick_gap / 1e6:.0f} ms while SDK callbacks continued', evidence)
    if flush_gap < tick_gap / 2:
        label = 'radio_disconnect' if evidence else 'sdk_stalled'
        return (label, f'SDK callbacks stopped for {sdk_gap / 1e6:.0f} ms and the publish timer stopped with '
                       'them (it shares a mutex with the SDK callback)', evidence)
    return ('host_stalled', f'SDK, publish timer and telemetry heartbeat all stopped for '
                            f'{min(tick_gap, sdk_gap, flush_gap) / 1e6:.0f} ms', evidence)


def classify_stale(tel, kernel, glove, pubs, first_pub, last_pub, other_gloves):
    p0, p1 = pubs[first_pub]['unix'], pubs[last_pub]['unix']
    evidence = radio_evidence(tel, kernel, glove['glove_id'], p0, p1)
    own = tel.skel_by_glove.get(glove['glove_id'], [])
    own_gap, g0, g1 = max_spacing(own, p0, p1)
    if own_gap < (p1 - p0) / 2:
        return ('publisher_stalled',
                'SDK delivered new skeletons that the publisher did not pick up', evidence)
    if evidence:
        return 'radio_disconnect', 'no new SDK skeletons for this glove; radio/connection evidence present', evidence
    others = [max_spacing(tel.skel_by_glove.get(g, []), g0, g1)[0] for g in other_gloves]
    if others and min(others) < own_gap / 2:
        return ('radio_disconnect',
                'only this glove stopped producing SDK skeletons while the other glove continued', evidence)
    return 'sdk_stalled', 'no new SDK skeletons for any glove; no disconnect reported', evidence


def tip_steps(pos):
    delta = np.diff(pos[:, TIP_NODES, :], axis=0)
    return delta, np.linalg.norm(delta, axis=2)


def analyse(bag, kernel_log=None, gap_ms=100, stale_ms=100, twitch_step=0.03):
    gloves, batches = read_bag(bag)
    if not gloves:
        raise SystemExit('no ManusGlove messages in bag')
    tel = Telemetry(batches)
    kernel = read_kernel_log(kernel_log) if kernel_log else []
    origin = min(int(g['recv'][0]) for g in gloves.values())
    rel = lambda ns: round((ns - origin) / NS, 4)
    events, glove_reports = [], {}
    gap_ns, stale_ns = gap_ms * 1e6, stale_ms * 1e6
    all_ids = {g['glove_id'] for g in gloves.values()}

    gap_windows = {}
    for topic, g in gloves.items():
        idx = np.flatnonzero(np.diff(g['recv']) > gap_ns)
        gap_windows[topic] = [(int(g['recv'][i]), int(g['recv'][i + 1])) for i in idx]

    for topic, g in sorted(gloves.items()):
        pubs = tel.pubs_for(topic)
        matches = match_publishes(g, pubs) if pubs else [None] * len(g['recv'])
        matched = [m for m in matches if m is not None]
        report = {'glove_id': g['glove_id'], 'side': g['side'], 'messages': len(g['recv']),
                  'matched_to_publish_records': len(matched)}
        if matched:
            span = pubs[matched[0]:matched[-1] + 1]
            report['published_while_recording'] = len(span)
            report['published_but_missing_from_bag'] = len(span) - len(matched)
            report['fresh_publish_fraction'] = round(float(np.mean([p['fresh'] for p in span])), 4)
            latency = sorted((int(g['recv'][i]) - pubs[m]['unix']) / 1e6 for i, m in enumerate(matches) if m is not None)
            report['publish_to_bag_latency_ms_quantiles_50_99_100'] = [
                round(latency[int(q * (len(latency) - 1))], 3) for q in (.5, .99, 1)]
            states = [s for s in glove_state(tel, g['glove_id'], span[0]['unix'], span[-1]['unix']) if s['present']]
            strengths = [s['tx_strength'] for s in states]
            if strengths:
                report['tx_strength_min_median_max'] = [min(strengths), sorted(strengths)[len(strengths) // 2],
                                                        max(strengths)]
            report['pairing_states_seen'] = sorted({str(s['paired']) for s in states})
        glove_reports[topic] = report
        others = sorted(all_ids - {g['glove_id']})
        base = {'topic': topic, 'glove_id': g['glove_id'], 'side': g['side']}
        missing = []  # (start, end, label) of gaps and frozen poses

        for i in np.flatnonzero(np.diff(g['recv']) > gap_ns):
            start, end = int(g['recv'][i]), int(g['recv'][i + 1])
            shared = any(min(end, b) - max(start, a) > 0.5 * (end - start)
                         for t, windows in gap_windows.items() if t != topic for a, b in windows)
            label, reason, evidence = classify_gap(tel, kernel, g, pubs, matches, int(i))
            events.append({**base, 'kind': 'gap', 'start_s': rel(start), 'end_s': rel(end),
                           'duration_ms': round((end - start) / 1e6, 1), 'label': label, 'reason': reason,
                           'evidence': evidence, 'shared_with_other_glove': shared})
            missing.append((start, end, label))

        # Frozen pose with continuing delivery.
        if pubs and tel.present:
            fresh = [pubs[m]['fresh'] if m is not None else None for m in matches]
            stale = [f is False for f in fresh]
        else:
            stale = [False] + [g['hash'][k] == g['hash'][k - 1] for k in range(1, len(g['hash']))]
        k = 1
        while k < len(stale):
            if not stale[k]:
                k += 1
                continue
            first = k
            while k < len(stale) and stale[k]:
                k += 1
            held_from, held_to = int(g['recv'][first - 1]), int(g['recv'][k - 1])
            paired = pubs and tel.present and matches[first - 1] is not None and matches[k - 1] is not None
            # Measure how long the pose was held at publish time when known: a
            # late-delivered burst must not look like a frozen pose.
            held = (pubs[matches[k - 1]]['unix'] - pubs[matches[first - 1]]['unix'] if paired
                    else held_to - held_from)
            if held <= stale_ns:
                continue
            if paired:
                label, reason, evidence = classify_stale(tel, kernel, g, pubs, matches[first - 1], matches[k - 1],
                                                         others)
            else:
                label, reason, evidence = 'unclassified', 'identical poses; no telemetry to explain them', []
            events.append({**base, 'kind': 'stale', 'start_s': rel(held_from), 'end_s': rel(held_to),
                           'duration_ms': round(held / 1e6, 1), 'label': label, 'reason': reason,
                           'evidence': evidence, 'messages': k - first + 1})
            missing.append((held_from, held_to, label))

        # Twitches: merge steps within 50 ms into one event. A step whose
        # starting sample lies inside a gap or frozen pose spans missing data.
        delta, steps = tip_steps(g['pos'])
        big = np.flatnonzero(steps.max(axis=1) > twitch_step)
        last_end = None
        for j in big:
            at = int(g['recv'][j + 1])
            if last_end is not None and at - last_end < 50e6:
                events[-1]['end_s'] = rel(at)
                events[-1]['max_step'] = max(events[-1]['max_step'], round(float(steps[j].max()), 5))
                last_end = at
                continue
            fingers = [FINGERS[f] for f in np.flatnonzero(steps[j] > twitch_step)]
            reversal = False
            if j + 1 < len(steps):
                a, b = delta[j], delta[j + 1]
                cos = (a * b).sum(1) / np.maximum(np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1), 1e-20)
                reversal = bool(((cos < -0.8) & (steps[j + 1] > twitch_step / 2)).any())
            step_from = int(g['recv'][j])
            spanned = [label for a, b, label in missing if a <= step_from < b]
            m = matches[j + 1]
            if spanned:
                label, reason = spanned[0], 'jump across a gap/frozen pose: the hand moved while data was missing'
            elif m is not None:
                pub = pubs[m]
                label, reason = 'in_stream_pose', 'delivered on time from a fresh SDK skeleton'
                if pub['new_skel'] > 1:
                    reason += f"; {pub['new_skel'] - 1} intermediate SDK skeletons were never published"
                if not pub['fresh']:
                    label, reason = 'unclassified', 'pose changed without a new SDK skeleton (unexpected)'
            else:
                label, reason = 'unclassified', 'no telemetry for this message'
            events.append({**base, 'kind': 'twitch', 'start_s': rel(int(g['recv'][j])), 'end_s': rel(at),
                           'duration_ms': None, 'label': label, 'reason': reason, 'evidence': [],
                           'fingers': fingers, 'max_step': round(float(steps[j].max()), 5),
                           'brief_reversal': reversal})
            last_end = at

    events.sort(key=lambda e: (e['start_s'], e['topic']))
    counts = defaultdict(Counter)
    for event in events:
        counts[event['kind']][event['label']] += 1
    system = Counter(r['type'] for r in tel.records['sys'])
    logs = Counter(r['sev'] for r in tel.records['log'])
    return {
        'bag': str(bag),
        'thresholds': {'gap_ms': gap_ms, 'stale_ms': stale_ms, 'twitch_step_native': twitch_step},
        'telemetry': tel.health() if tel.present else None,
        'kernel_usb_events': len(kernel),
        'gloves': glove_reports,
        'sdk_system_messages': dict(system),
        'sdk_log_severities': dict(logs),
        'event_counts': {kind: dict(c) for kind, c in counts.items()},
        'events': events,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path, help='rosbag2 directory or .db3 file')
    parser.add_argument('--out', type=Path, help='output directory (default: <bag>_diagnosis)')
    parser.add_argument('--kernel-log', type=Path, help='dmesg --time-format iso output saved during the take')
    parser.add_argument('--gap-ms', type=float, default=100)
    parser.add_argument('--stale-ms', type=float, default=100)
    parser.add_argument('--twitch-step', type=float, default=0.03, help='fingertip step, native units')
    args = parser.parse_args(argv)
    result = analyse(args.bag, args.kernel_log, args.gap_ms, args.stale_ms, args.twitch_step)
    out = args.out or args.bag.with_name(args.bag.name + '_diagnosis')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'diagnosis.json').write_text(json.dumps(result, indent=2) + '\n')
    fields = ['kind', 'label', 'topic', 'side', 'start_s', 'end_s', 'duration_ms', 'reason', 'evidence',
              'shared_with_other_glove', 'fingers', 'max_step', 'brief_reversal', 'messages']
    with (out / 'events.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for event in result['events']:
            writer.writerow({**event, 'evidence': ' | '.join(event['evidence']),
                             'fingers': ' '.join(event.get('fingers', []))})

    print(f"Telemetry: {'present' if result['telemetry'] else 'MISSING (labels will be unclassified)'}")
    if result['telemetry']:
        health = result['telemetry']
        print(f"  {health['batches']} batches, {health['batches_missing_in_bag']} missing, "
              f"{health['publisher_sessions']} publisher session(s), source {', '.join(health['source_sha'])}")
    for topic, report in result['gloves'].items():
        print(f"{topic} ({report['side']}): {report['messages']} messages, "
              f"{report['matched_to_publish_records']} paired with publish records"
              + (f", {report['published_but_missing_from_bag']} published but missing from bag"
                 if 'published_but_missing_from_bag' in report else ''))
    for kind in ('gap', 'stale', 'twitch'):
        labels = result['event_counts'].get(kind, {})
        print(f"{kind:7s} {sum(labels.values()):4d}  " + ', '.join(f'{k}: {v}' for k, v in sorted(labels.items())))
    print(f'Wrote {out / "diagnosis.json"} and {out / "events.csv"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
