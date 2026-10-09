#!/usr/bin/env python3
"""Add /manus_telemetry to an installed MANUS manus_ros2 package.

Usage: apply_patch.py /opt/ros2_ws/src/manus_ros2 [--check | --revert]

Copies ManusTelemetry.{hpp,cpp} into the package and inserts small hooks into
ManusDataPublisher.cpp and CMakeLists.txt. Every anchor must match exactly once
or nothing is written. Originals are kept as *.telemetry-orig and re-applying
always starts from them, so updating this repo and re-running is safe.
Tested against the ROS2 package in MANUS SDK 3.1.1 and 3.2.1.
"""
import argparse
import hashlib
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MARKER = 'manus-examples telemetry'
BACKUP_SUFFIX = '.telemetry-orig'
SOURCES = ('ManusTelemetry.hpp', 'ManusTelemetry.cpp')


def source_sha():
    digest = hashlib.sha256()
    for name in (*SOURCES, Path(__file__).name):
        digest.update((HERE / name).read_bytes())
    return digest.hexdigest()[:16]


# (description, pattern, replacement). Patterns tolerate the whitespace
# differences between SDK releases; \g<0> keeps the matched anchor.
def publisher_edits(sha):
    return [
        ('include', r'#include\s+"ManusDataPublisher\.hpp"[^\n]*\n',
         '\\g<0>// ' + MARKER + '\n#include "ManusTelemetry.hpp"\n#include "std_msgs/msg/string.hpp"\n'
         '#define MANUS_TELEMETRY_SOURCE_SHA "' + sha + '"\n'),
        ('start telemetry in constructor', r'\n([ \t]+)s_Instance\s*=\s*this\s*;[^\n]*\n',
         '\\g<0>\n\\1// ' + MARKER + ': separate /manus_telemetry topic; glove topics are unchanged.\n'
         '\\1{\n'
         '\\1    auto t_TelemetryPublisher = this->create_publisher<std_msgs::msg::String>(\n'
         '\\1        "manus_telemetry", rclcpp::QoS(rclcpp::KeepLast(1000)).reliable());\n'
         '\\1    ManusTelemetry::Instance().Start(\n'
         '\\1        [t_TelemetryPublisher](const std::string &p_Batch) {\n'
         '\\1            std_msgs::msg::String t_TelemetryMsg;\n'
         '\\1            t_TelemetryMsg.data = p_Batch;\n'
         '\\1            t_TelemetryPublisher->publish(t_TelemetryMsg);\n'
         '\\1        },\n'
         '\\1        &CoreSdk_GetTimestampInfo, MANUS_TELEMETRY_SOURCE_SHA);\n'
         '\\1}\n'),
        ('stop telemetry in destructor', r'\n([ \t]+)s_Instance\s*=\s*nullptr\s*;[^\n]*\n',
         '\n\\1ManusTelemetry::Instance().Stop(); // ' + MARKER + '\n\\g<0>'),
        ('register extra SDK callbacks',
         r'\n([ \t]+)const\s+ClientReturnCode\s+t_CallBackResults\s*=\s*RegisterAllCallbacks\(\)\s*;[^\n]*\n',
         '\\g<0>\\1// ' + MARKER + ': connection, log and system-message callbacks.\n'
         '\\1ManusTelemetry::Instance().NoteRegistration("OnConnect", '
         '(int)CoreSdk_RegisterCallbackForOnConnect(&ManusTelemetry::OnConnect));\n'
         '\\1ManusTelemetry::Instance().NoteRegistration("OnDisconnect", '
         '(int)CoreSdk_RegisterCallbackForOnDisconnect(&ManusTelemetry::OnDisconnect));\n'
         '\\1ManusTelemetry::Instance().NoteRegistration("OnLog", '
         '(int)CoreSdk_RegisterCallbackForOnLog(&ManusTelemetry::OnLog));\n'
         '\\1ManusTelemetry::Instance().NoteRegistration("SystemStream", '
         '(int)CoreSdk_RegisterCallbackForSystemStream(&ManusTelemetry::OnSystem));\n'),
        ('raw skeleton callback',
         r'\n([ \t]+)s_Instance->m_GloveDataMap\.insert_or_assign\(\s*t_NxtClientRawSkeleton\.info\.gloveId\s*,'
         r'\s*t_NxtClientRawSkeleton\s*\)\s*;[^\n]*\n',
         '\\g<0>\\1ManusTelemetry::Instance().NoteSkeleton(t_NxtClientRawSkeleton.info.gloveId, '
         't_NxtClientRawSkeleton.info.nodesCount, p_RawSkeletonStreamInfo->publishTime); // ' + MARKER + '\n'),
        ('raw device callback',
         r'\n([ \t]+)s_Instance->m_RawSensorDataMap\.insert_or_assign\(\s*t_NewData\[i\]\.id\s*,'
         r'\s*t_NewData\[i\]\s*\)\s*;[^\n]*\n',
         '\\g<0>\\1ManusTelemetry::Instance().NoteRawDevice(t_NewData[i].id, t_NewData[i].sensorCount, '
         'p_RawDeviceDataInfo->publishTime); // ' + MARKER + '\n'),
        ('ergonomics callback',
         r'\n([ \t]+)s_Instance->m_ErgonomicsDataMap\.insert_or_assign\(\s*p_Ergo->data\[i\]\.id\s*,'
         r'\s*t_Ergo\s*\)\s*;[^\n]*\n',
         '\\g<0>\\1ManusTelemetry::Instance().NoteErgonomics(p_Ergo->data[i].id, p_Ergo->publishTime); // '
         + MARKER + '\n'),
        ('landscape callback',
         r'\n([ \t]+)Landscape\s*\*\s*t_Landscape\s*=\s*new\s+Landscape\(\s*\*p_Landscape\s*\)\s*;[^\n]*\n',
         '\\g<0>\\1ManusTelemetry::Instance().NoteLandscape(*p_Landscape); // ' + MARKER + '\n'),
        ('publish timer entry',
         r'void\s+ManusDataPublisher::PublishCallback\(\)\s*\{[^\n]*\n',
         '\\g<0>    ManusTelemetry::TickScope t_TelemetryTick(ManusTelemetry::Instance()); // ' + MARKER + '\n'),
        ('glove publish',
         r'\n([ \t]+)t_Publisher->second->publish\(\s*t_Msg\s*\)\s*;[^\n]*\n',
         '\n\\1const int64_t t_TelemetryPreMono = ManusTelemetry::MonoNs(); // ' + MARKER + '\n'
         '\\1const int64_t t_TelemetryPreUnix = ManusTelemetry::UnixNs();\n'
         '\\1t_Publisher->second->publish(t_Msg);\n'
         '\\1ManusTelemetry::Instance().NotePublish(t_Msg.glove_id, t_Publisher->second->get_topic_name(),\n'
         '\\1    t_RawSkel.info.publishTime, ManusTelemetry::PoseHash(t_Msg),\n'
         '\\1    t_TelemetryPreMono, t_TelemetryPreUnix, ManusTelemetry::MonoNs());\n'),
    ]


CMAKE_EDITS = [
    ('add ManusTelemetry.cpp to the executable',
     r'(add_executable\(\s*manus_data_publisher[^)]*?)(src/ManusDataPublisher\.cpp)',
     '\\1\\2\n  # ' + MARKER + '\n  src/ManusTelemetry.cpp'),
]


def apply_edits(text, edits, name):
    errors = []
    for description, pattern, replacement in edits:
        matches = len(re.findall(pattern, text))
        if matches != 1:
            errors.append(f'{name}: anchor "{description}" matched {matches} times (expected 1)')
            continue
        text = re.sub(pattern, replacement, text, count=1)
    return text, errors


def read(path):
    """Return (text with \\n newlines, original newline) so CRLF sources stay CRLF."""
    with open(path, newline='') as handle:
        text = handle.read()
    newline = '\r\n' if '\r\n' in text else '\n'
    return text.replace('\r\n', '\n'), newline


def write(path, text, newline):
    with open(path, 'w', newline='') as handle:
        handle.write(text.replace('\n', newline))


def pristine(path):
    backup = path.with_name(path.name + BACKUP_SUFFIX)
    text, newline = read(backup if backup.exists() else path)
    if not backup.exists() and MARKER in text:
        raise SystemExit(f'{path} is already patched but {backup.name} is missing; restore the original first.')
    return text, newline


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('package', type=Path, help='manus_ros2 package directory (contains CMakeLists.txt and src/)')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='verify every anchor matches, write nothing')
    mode.add_argument('--revert', action='store_true', help='restore the original files')
    args = parser.parse_args(argv)

    targets = {'publisher': args.package / 'src' / 'ManusDataPublisher.cpp',
               'cmake': args.package / 'CMakeLists.txt'}
    for path in targets.values():
        if not path.exists():
            parser.error(f'not a manus_ros2 package: {path} is missing')

    if args.revert:
        for path in targets.values():
            backup = path.with_name(path.name + BACKUP_SUFFIX)
            if backup.exists():
                shutil.copy2(backup, path)
                backup.unlink()
        for name in SOURCES:
            (args.package / 'src' / name).unlink(missing_ok=True)
        print(f'Reverted {args.package}')
        return 0

    sha = source_sha()
    publisher_text, publisher_newline = pristine(targets['publisher'])
    cmake_text, cmake_newline = pristine(targets['cmake'])
    publisher, errors = apply_edits(publisher_text, publisher_edits(sha), 'ManusDataPublisher.cpp')
    cmake, cmake_errors = apply_edits(cmake_text, CMAKE_EDITS, 'CMakeLists.txt')
    errors += cmake_errors
    if errors:
        print('Patch not applied; this publisher source differs from the tested SDK releases:', file=sys.stderr)
        for error in errors:
            print('  ' + error, file=sys.stderr)
        return 1
    if args.check:
        print(f'All anchors match in {args.package} (telemetry source {sha}). Nothing written.')
        return 0

    for path in targets.values():
        backup = path.with_name(path.name + BACKUP_SUFFIX)
        if not backup.exists():
            shutil.copy2(path, backup)
    write(targets['publisher'], publisher, publisher_newline)
    write(targets['cmake'], cmake, cmake_newline)
    for name in SOURCES:
        shutil.copy2(HERE / name, args.package / 'src' / name)
    print(f'Patched {args.package} (telemetry source {sha}). Rebuild with: '
          'colcon build --packages-select manus_ros2')
    return 0


if __name__ == '__main__':
    sys.exit(main())
