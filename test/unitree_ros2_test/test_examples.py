"""Exercise the existing telemetry subscribers and recorder on private topics."""
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
# Recorded GO2 firmware offers /lowstate, /sportmodestate and /wirelesscontroller
# as RELIABLE; cover that as well as best-effort publishers.
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
