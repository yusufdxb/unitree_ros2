"""ROS RMW against unitree_sdk2's own generated C++ types and bundled Cyclone DDS.

Run through scripts/test_sdk2_peer.sh, which builds the peer from a pinned
unitree_sdk2 revision. Each type is checked in three ways: a sample the SDK2
peer filled reaches ROS with every field equal to what the peer reports; a ROS
sample reaches the peer with the named fields it prints; and the peer's echo of
the ROS sample comes back unchanged. The peer fills and prints a fixed set of
named fields (array ends, every scalar); the rest stay at their defaults.
"""
import numbers
import os
import re
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from test_interfaces import fields, populate

TYPES = {
    'go_LowState': 'unitree_go/msg/LowState',
    'go_SportModeState': 'unitree_go/msg/SportModeState',
    'go_LidarState': 'unitree_go/msg/LidarState',
    'go_WirelessController': 'unitree_go/msg/WirelessController',
    'go_LowCmd': 'unitree_go/msg/LowCmd',
    'go_MotorCmds': 'unitree_go/msg/MotorCmds',
    'hg_LowState': 'unitree_hg/msg/LowState',
    'hg_LowCmd': 'unitree_hg/msg/LowCmd',
    'hg_IMUState': 'unitree_hg/msg/IMUState',
    'hg_MotorState': 'unitree_hg/msg/MotorState',
    'hg_MotorCmd': 'unitree_hg/msg/MotorCmd',
    'hg_HandState': 'unitree_hg/msg/HandState',
    'hg_HandCmd': 'unitree_hg/msg/HandCmd',
    'hg_BmsState': 'unitree_hg/msg/BmsState',
    'hg_MainBoardState': 'unitree_hg/msg/MainBoardState',
    'hg_PressSensorState': 'unitree_hg/msg/PressSensorState',
    'geometry_Point32': 'geometry_msgs/msg/Point32',
}
# Fields unitree_sdk2 defines that the ROS definitions do not. dirty_percentage
# occupies what is alignment padding in the ROS layout; path_point[10] trails
# the ROS layout (recorded GO2 data carries it as a 280-byte zero trailer).
SDK2_ONLY = {
    'go_SportModeState': ('path_point',),
    'go_LidarState': ('dirty_percentage',),
}
# Same width, different signedness: int8 in ROS, uint8 in unitree_sdk2.
SIGNEDNESS = {
    'go_LowState': {
        'imu_state.temperature', 'motor_state[0].temperature',
        'motor_state[19].temperature', 'bms_state.bq_ntc[1]', 'bms_state.mcu_ntc[0]',
        'temperature_ntc1', 'temperature_ntc2'},
    'go_SportModeState': {'imu_state.temperature'},
}
# ROS cannot publish a SportModeState that an SDK2 reader accepts: the SDK2 type
# expects the trailing path_point array the ROS type does not serialize.
NOT_DELIVERED_TO_SDK2 = {'go_SportModeState'}
TOKEN = re.compile(r'(\w+)(?:\[(\d+)\])?$')
BITS = {'u8': 8, 'u16': 16, 'u32': 32, 'u64': 64}


def parse(line):
    """'KIND key path=value:kind ...' -> (key, [(path, value, kind)])."""
    words = line.split()
    entries = []
    for word in words[2:]:
        path, rest = word.split('=', 1)
        value, kind = rest.rsplit(':', 1)
        if kind == 'str':
            pass
        elif kind.startswith('f'):
            value = float(value)
        else:
            value = int(value)
        entries.append((path, value, kind))
    return words[1], entries


def walk(message, path):
    """Return (parent, name, index) for a dotted path such as motor_state[3].q."""
    parts = path.split('.')
    target = message
    for part in parts[:-1]:
        name, index = TOKEN.match(part).groups()
        target = getattr(target, name)
        if index is not None:
            target = target[int(index)]
    name, index = TOKEN.match(parts[-1]).groups()
    if not hasattr(target, name):
        raise AttributeError(path)
    return target, name, None if index is None else int(index)


def get(message, path):
    if path.endswith('@size'):
        return len(getattr(*walk(message, path[:-5])[:2]))
    parent, name, index = walk(message, path)
    value = getattr(parent, name)
    return value if index is None else value[index]


def put(message, path, value):
    if path.endswith('@size'):
        parent, name, _ = walk(message, path[:-5])
        element = parent.get_fields_and_field_types()[name]
        element = element[len('sequence<'):-1].replace('/', '/msg/', 1)
        setattr(parent, name, [get_message(element)() for _ in range(value)])
        return
    parent, name, index = walk(message, path)
    if index is None:
        setattr(parent, name, value)
    else:
        getattr(parent, name)[index] = value


def sdk2_only(key, path):
    return path.split('.')[0].split('[')[0] in SDK2_ONLY.get(key, ())


def same(key, path, ros_value, sdk_value, kind):
    if ros_value == sdk_value:
        return True
    if kind in BITS and isinstance(ros_value, numbers.Integral) and ros_value < 0:
        return (path in SIGNEDNESS.get(key, ())
                and int(ros_value) % (1 << BITS[kind]) == sdk_value)
    return False


@pytest.fixture(scope='module')
def exchange():
    peer = os.environ.get('UNITREE_SDK2_PEER')
    if not peer:
        pytest.fail('UNITREE_SDK2_PEER is not set; run scripts/test_sdk2_peer.sh')
    namespace = 'unitree_test/sdk2_' + uuid.uuid4().hex
    env = os.environ.copy()
    env['UNITREE_DDS_TEST_NAMESPACE'] = namespace
    env['LD_LIBRARY_PATH'] = os.environ.get('UNITREE_SDK2_PEER_LIBRARY_PATH', '')
    process = subprocess.Popen([peer], env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    lines = []
    reader = threading.Thread(target=lambda: lines.extend(process.stdout), daemon=True)
    reader.start()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    rclpy.init()
    node = rclpy.create_node('sdk2_peer_regression')
    sent = {key: populate(get_message(name)()) for key, name in TYPES.items()}
    echoes = {key: [] for key in TYPES}
    origins = {key: [] for key in TYPES}
    publishers = {}
    for key, name in TYPES.items():
        cls = get_message(name)
        publishers[key] = node.create_publisher(cls, '/{}/in/{}'.format(namespace, key), qos)
        node.create_subscription(cls, '/{}/out/{}'.format(namespace, key),
                                 echoes[key].append, qos)
        node.create_subscription(cls, '/{}/origin/{}'.format(namespace, key),
                                 origins[key].append, qos)
    try:
        expected = set(TYPES) - NOT_DELIVERED_TO_SDK2
        deadline = time.monotonic() + 30
        settle = None
        while time.monotonic() < deadline:
            assert process.poll() is None, ''.join(lines)
            for key, publisher in publishers.items():
                if not echoes[key] and publisher.get_subscription_count():
                    publisher.publish(sent[key])
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.01)
            if settle is None and all(origins.values()) and all(echoes[k] for k in expected):
                # Keep offering the undeliverable samples for a further 3 s.
                settle = time.monotonic() + 3
            if settle is not None and time.monotonic() > settle:
                break
        matched = {key: publisher.get_subscription_count()
                   for key, publisher in publishers.items()}
    finally:
        process.terminate()
        process.wait(timeout=10)
        reader.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    origin_lines, received = {}, {}
    for line in lines:
        if line.startswith('ORIGIN '):
            key, entries = parse(line)
            origin_lines[key] = entries
        elif line.startswith('RECV '):
            key, entries = parse(line)
            received.setdefault(key, entries)
    return {'sent': sent, 'echoes': echoes, 'origins': origins, 'origin_lines': origin_lines,
            'received': received, 'output': ''.join(lines), 'returncode': process.returncode,
            'matched': matched}


def test_peer_ran_and_stopped_cleanly(exchange):
    assert exchange['returncode'] == 0, exchange['output']
    assert 'READY {}'.format(len(TYPES)) in exchange['output'], exchange['output']


@pytest.mark.parametrize('key', list(TYPES))
def test_sdk2_sample_reaches_ros_field_for_field(key, exchange):
    assert exchange['origins'][key], 'no SDK2 sample of ' + key
    received = exchange['origins'][key][0]
    expected = get_message(TYPES[key])()
    skipped = set()
    for path, value, _ in exchange['origin_lines'][key]:
        if sdk2_only(key, path):
            skipped.add(path.split('.')[0].split('[')[0])
            continue
        put(expected, path, value)
    # Every field ROS knows equals the SDK2 value or its default; nothing shifted.
    assert fields(received) == fields(expected)
    assert skipped == set(SDK2_ONLY.get(key, ()))


@pytest.mark.parametrize('key', list(TYPES))
def test_ros_sample_reaches_sdk2_by_field_name(key, exchange):
    if key in NOT_DELIVERED_TO_SDK2:
        # Matched, offered for at least 3 s, and rejected by SDK2's deserializer.
        assert exchange['matched'][key], 'SDK2 reader never matched ' + key
        assert any('/in/' + key + '/' in line and 'failed' in line
                   for line in exchange['output'].splitlines()), exchange['output']
        assert key not in exchange['received'], exchange['received'].get(key)
        assert not exchange['echoes'][key]
        return
    assert key in exchange['received'], 'SDK2 peer received no ' + key
    sent = exchange['sent'][key]
    mismatched = []
    for path, value, kind in exchange['received'][key]:
        if sdk2_only(key, path):
            continue
        if not same(key, path, get(sent, path), value, kind):
            mismatched.append((path, get(sent, path), value))
    assert not mismatched


@pytest.mark.parametrize('key', sorted(set(TYPES) - NOT_DELIVERED_TO_SDK2))
def test_sdk2_echo_of_ros_sample_is_unchanged(key, exchange):
    assert exchange['echoes'][key], 'no echo of ' + key
    assert fields(exchange['echoes'][key][0]) == fields(exchange['sent'][key])
