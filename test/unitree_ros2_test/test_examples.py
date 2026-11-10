"""Exercise the existing telemetry subscribers and recorder on private topics."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from test_interfaces import fields, populate


@pytest.mark.parametrize('executable,type_name,topic,marker', [
    ('read_low_state', 'unitree_go/msg/LowState', 'lowstate', 'Motor state'),
    ('read_low_state_hg', 'unitree_hg/msg/LowState', 'lowstate', 'Motor state'),
    ('read_motion_state', 'unitree_go/msg/SportModeState', 'lf/sportmodestate', 'Position'),
    ('read_wireless_controller', 'unitree_go/msg/WirelessController', '/wirelesscontroller',
     'Wireless controller'),
    ('record_bag', 'unitree_go/msg/SportModeState', 'lf/sportmodestate', 'Position'),
])
# GO2 telemetry publishers have been observed both RELIABLE (/lowstate,
# /sportmodestate) and BEST_EFFORT (/wirelesscontroller); cover both.
@pytest.mark.parametrize('publisher_qos', [
    qos_profile_sensor_data,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
], ids=['best-effort', 'reliable-like-firmware'])
def test_telemetry_examples(executable, type_name, topic, marker, publisher_qos, tmp_path):
    cls = get_message(type_name)
    message = populate(cls())
    safe_topic = '/unitree_test/examples/' + executable
    prefix = Path(get_package_prefix('unitree_ros2_example'))
    binary = prefix / 'lib/unitree_ros2_example' / executable
    args = [str(binary), '--ros-args', '-r', topic + ':=' + safe_topic]
    bag_path = tmp_path / 'recording'
    if executable == 'record_bag':
        args += ['-p', 'output:=' + str(bag_path)]
    rclpy.init()
    node = rclpy.create_node('example_regression_' + executable)
    publisher = node.create_publisher(cls, safe_topic, publisher_qos)
    logfile = tmp_path / 'output.log'
    with logfile.open('w') as output:
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not publisher.get_subscription_count() and time.monotonic() < deadline:
                assert process.poll() is None, logfile.read_text()
                rclpy.spin_once(node, timeout_sec=0.05)
            assert publisher.get_subscription_count(), logfile.read_text()
            for _ in range(5):
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            assert process.returncode == 0, logfile.read_text()
            assert marker in logfile.read_text(), logfile.read_text()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
    if executable == 'record_bag':
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        types = reader.get_all_topics_and_types()
        assert len(types) == 1 and types[0].type == type_name
        assert types[0].name == safe_topic
        assert reader.has_next()
        name, data, timestamp = reader.read_next()
        assert name == safe_topic
        assert timestamp > 0
        assert fields(deserialize_message(data, cls)) == fields(message)


def run_subscriber(executable, arguments, publications, tmp_path, until=None):
    """Start a subscriber example on private topics, publish, stop it with SIGINT.

    publications maps a private topic to (message, qos). until is a callable on the
    running ROS node that must become true before publishing (default: matched).
    Returns (returncode, log text, value returned by until).
    """
    prefix = Path(get_package_prefix('unitree_ros2_example'))
    args = [str(prefix / 'lib/unitree_ros2_example' / executable), '--ros-args'] + arguments
    rclpy.init()
    node = rclpy.create_node('example_models_' + executable)
    publishers = [(node.create_publisher(type(message), topic, qos), message)
                  for topic, (message, qos) in publications.items()]
    logfile = tmp_path / (executable + '.log')
    observed = None
    with logfile.open('w') as output:
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and process.poll() is None:
                if all(p.get_subscription_count() for p, _ in publishers):
                    observed = until(node) if until else True
                    if observed:
                        break
                rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is None:
                assert observed, logfile.read_text()
                for _ in range(5):
                    for publisher, message in publishers:
                        publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)
                process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
    return process.returncode, logfile.read_text(), observed


@pytest.mark.parametrize('executable,robot,motors', [
    ('read_low_state', 'go2', 12),
    ('read_low_state', 'go2w', 16),
    ('read_low_state', 'b2', 12),
    ('read_low_state', 'b2w', 16),
    ('read_low_state', 'h1', 20),
    ('read_low_state_hg', 'g1', 29),
    ('read_low_state_hg', 'h1_2', 27),
    ('read_low_state_hg', 'h2', 31),
    ('read_low_state_hg', 'r1', 26),
])
def test_low_state_readers_report_each_models_motor_slots(executable, robot, motors, tmp_path):
    family = 'unitree_hg' if executable.endswith('_hg') else 'unitree_go'
    message = populate(get_message(family + '/msg/LowState')())
    topic = '/unitree_test/models/' + robot
    code, log, _ = run_subscriber(
        executable, ['-p', 'robot:=' + robot, '-r', 'lowstate:=' + topic],
        {topic: (message, qos_profile_sensor_data)}, tmp_path)
    assert code == 0, log
    assert 'Motor state -- num: {};'.format(motors - 1) in log, log
    assert 'Motor state -- num: {};'.format(motors) not in log, log
    if family == 'unitree_hg':
        assert 'mode_machine: {}'.format(message.mode_machine) in log, log


@pytest.mark.parametrize('executable,robot', [
    ('read_low_state', 'g1'),
    ('read_low_state', 'unknown'),
    ('read_low_state_hg', 'go2'),
    ('read_low_state_hg', 'h1'),
    ('read_low_state_hg', 'G1'),
])
def test_low_state_readers_reject_other_families_and_unknown_models(executable, robot, tmp_path):
    code, log, _ = run_subscriber(executable, ['-p', 'robot:=' + robot], {}, tmp_path,
                                  until=lambda node: False)
    assert code == 1, log
    assert 'does not publish' in log, log
    assert 'Motor state' not in log, log


def test_hg_reader_reads_secondary_imu_on_request(tmp_path):
    imu = populate(get_message('unitree_hg/msg/IMUState')())
    topic = '/unitree_test/models/secondary_imu'
    code, log, _ = run_subscriber(
        'read_low_state_hg', ['-p', 'secondary_imu:=true', '-r', 'secondary_imu:=' + topic],
        {topic: (imu, qos_profile_sensor_data)}, tmp_path)
    assert code == 0, log
    assert 'Secondary IMU -- roll: {:f}'.format(imu.rpy[0]) in log, log


@pytest.mark.parametrize('executable,arguments,subscribed', [
    ('read_low_state', ['-p', 'robot:=b2'], {'lowstate'}),
    ('read_low_state_hg', ['-p', 'robot:=h2', '-p', 'secondary_imu:=true'],
     {'lowstate', 'secondary_imu'}),
    ('read_motion_state', [], {'lf/sportmodestate'}),
    ('read_wireless_controller', [], {'wirelesscontroller'}),
])
def test_telemetry_examples_are_subscriber_only(executable, arguments, subscribed, tmp_path):
    # Robot topics are remapped away; the graph is inspected before anything is sent.
    name = 'endpoint_probe_' + executable
    remaps = []
    for topic in sorted(subscribed):
        remaps += ['-r', topic + ':=/unitree_test/endpoints/' + topic]
    expected = {'/unitree_test/endpoints/' + topic for topic in subscribed}

    def endpoints(node):
        if name not in node.get_node_names():
            return None
        # Every rclcpp node also follows /parameter_events.
        topics = {t for t, _ in node.get_subscriber_names_and_types_by_node(name, '/')}
        if topics - {'/parameter_events'} != expected:
            return None
        return (topics,
                {t for t, _ in node.get_publisher_names_and_types_by_node(name, '/')},
                node.get_client_names_and_types_by_node(name, '/'))

    code, log, observed = run_subscriber(
        executable, arguments + remaps + ['-r', '__node:=' + name], {}, tmp_path,
        until=endpoints)
    assert code == 0, log
    assert observed, log
    _, publishers, clients = observed
    assert publishers <= {'/rosout', '/parameter_events'}, publishers
    assert not clients, clients


EXPECTED_EXAMPLES = 24


def test_every_example_is_installed_and_fully_linked():
    # Static check only: control examples are never executed by the test suite.
    prefix = Path(get_package_prefix('unitree_ros2_example'))
    executables = sorted(p for p in (prefix / 'lib/unitree_ros2_example').iterdir()
                         if p.is_file() and os.access(p, os.X_OK))
    assert len(executables) == EXPECTED_EXAMPLES, [p.name for p in executables]
    distro = os.environ['ROS_DISTRO']
    for executable in executables:
        assert (prefix / 'bin' / executable.name).is_file(), executable.name
        linked = subprocess.run(['ldd', str(executable)], capture_output=True, text=True,
                                check=True).stdout
        assert 'not found' not in linked, executable.name + '\n' + linked
        foreign = [line for line in linked.splitlines()
                   if '/opt/ros/' in line and '/opt/ros/' + distro + '/' not in line]
        assert not foreign, executable.name + '\n' + '\n'.join(foreign)
        assert '/opt/ros/' + distro + '/' in linked, executable.name
