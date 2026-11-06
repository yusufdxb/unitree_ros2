"""Isolation and timed playback checks using generated SQLite bags."""
import importlib.util
from pathlib import Path

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

SPEC = importlib.util.spec_from_file_location(
    'replay_bags', Path(__file__).with_name('replay_bags.py'))
replay_bags = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay_bags)

VALID_CONFIG = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo"/>'
    '</Interfaces><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery>'
    '</Domain></CycloneDDS>')


def test_valid_isolation_reuses_bag_guard(monkeypatch):
    calls = []
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('CYCLONEDDS_URI', VALID_CONFIG)
    monkeypatch.setattr(
        replay_bags.check_bags, 'validate_ros',
        lambda samples, resolver: calls.append(samples))
    replay_bags.verify_isolation(get_message)
    assert calls == [{}]


@pytest.mark.parametrize('config', [
    VALID_CONFIG.replace('<NetworkInterface name="lo"/>',
                         '<NetworkInterface name="lo"/>'
                         '<NetworkInterface name="eth0"/>'),
    VALID_CONFIG.replace('127.0.0.1', '192.0.2.1'),
    VALID_CONFIG.replace('<AllowMulticast>false', '<AllowMulticast>true'),
    VALID_CONFIG.replace('name="lo"', 'name="eth0"'),
])
def test_unsafe_network_configuration_rejected(monkeypatch, config):
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('CYCLONEDDS_URI', config)
    with pytest.raises(RuntimeError):
        replay_bags.verify_isolation(get_message)


def test_wrong_domain_rejected(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('CYCLONEDDS_URI', VALID_CONFIG)
    with pytest.raises(RuntimeError, match='ROS_DOMAIN_ID=173'):
        replay_bags.verify_isolation(get_message)


def test_player_command_whitelists_and_remaps_every_topic(tmp_path):
    topics = {
        '/source/lowstate': {
            'type': 'unitree_go/msg/LowState',
            'remapped_name': '/unitree_regression/replay/run_test/topic_0'},
        '/source/sportstate': {
            'type': 'unitree_go/msg/SportModeState',
            'remapped_name': '/unitree_regression/replay/run_test/topic_1'},
    }
    command = replay_bags.build_player_command(
        tmp_path, 0.25, tmp_path / 'qos.yaml', topics)
    selected = command[command.index('--topics') + 1:command.index('--remap')]
    assert selected == list(topics)
    remaps = command[command.index('--remap') + 1:]
    assert remaps == [name + ':=' + item['remapped_name']
                      for name, item in topics.items()]
    assert '--clock' not in command
    assert '--publish-service-requests' not in command


def test_player_command_rejects_command_message(tmp_path):
    topics = {'/source/request': {
        'type': 'unitree_api/msg/Request',
        'remapped_name': '/unitree_regression/replay/run_test/topic_0'}}
    with pytest.raises(ValueError, match='non-telemetry'):
        replay_bags.build_player_command(
            tmp_path, 0.25, tmp_path / 'qos.yaml', topics)


def synthetic_bag(directory):
    import rosbag2_py
    types = (
        'unitree_go/msg/LowState',
        'unitree_go/msg/SportModeState',
        'unitree_go/msg/WirelessController',
        'unitree_api/msg/Request',
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    for index, typename in enumerate(types):
        attributes = {
            'name': '/synthetic/topic_{}'.format(index), 'type': typename,
            'serialization_format': 'cdr'}
        try:
            topic = rosbag2_py.TopicMetadata(id=index, **attributes)
        except TypeError:
            topic = rosbag2_py.TopicMetadata(**attributes)
        writer.create_topic(topic)
    for sequence in range(3):
        for index, typename in enumerate(types):
            message = get_message(typename)()
            if hasattr(message, 'tick'):
                message.tick = 1234 + sequence
            if hasattr(message, 'stamp'):
                message.stamp.sec = 17
                message.stamp.nanosec = sequence
            if hasattr(message, 'keys'):
                message.keys = sequence
            writer.write('/synthetic/topic_{}'.format(index),
                         serialize_message(message),
                         1234567890000000000 + sequence * 100000000
                         + index * 10000000)
    del writer
    return directory


def test_generated_bag_inventory_excludes_commands(tmp_path):
    directory = synthetic_bag(tmp_path / 'bag')
    _, files = replay_bags.source_files(directory)
    topics, _ = replay_bags.inventory(
        files, get_message, deserialize_message)
    assert len(topics) == 3
    assert all(topic['count'] == 3 for topic in topics.values())
    assert '/synthetic/topic_3' not in topics
    assert all('/unitree_regression/replay/run_' in topic['remapped_name']
               for topic in topics.values())


def test_generated_bag_timed_playback(tmp_path):
    directory = synthetic_bag(tmp_path / 'bag')
    workdir = tmp_path / 'results'
    workdir.mkdir()
    report = replay_bags.replay(
        directory, 1.0, workdir, get_message, deserialize_message)
    assert report['passed'], report['errors']
    assert report['player_exit_code'] == 0
    assert (report['source_duration_seconds']
            > report['selected_telemetry_duration_seconds'])
    assert len(report['topics']) == 3
    assert report['original_sha256_before'] == report['original_sha256_after']
    for topic in report['topics']:
        assert topic['received_count'] == topic['count'] == 3
        assert not topic['errors']
        assert (topic['received_fields_sha256']
                == topic['expected_fields_sha256'])
