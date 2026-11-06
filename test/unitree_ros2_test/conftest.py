"""Keep automated communication tests confined to loopback."""
import os
import xml.etree.ElementTree as ET

import pytest


@pytest.fixture(scope='session', autouse=True)
def isolated_communication(request):
    communicating = {'test_native_dds.py', 'test_examples.py', 'test_api_client.py',
                     'test_replay_tool.py'}
    if not any(os.path.basename(item.location[0]) in communicating
               for item in request.session.items):
        return
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        pytest.fail('Communication tests require rmw_cyclonedds_cpp')
    if os.environ.get('ROS_DOMAIN_ID') != '173':
        pytest.fail('Communication tests require dedicated ROS_DOMAIN_ID=173')
    try:
        config = ET.fromstring(os.environ.get('CYCLONEDDS_URI', ''))
    except ET.ParseError:
        pytest.fail('Communication tests require inline loopback Cyclone DDS XML')
    interfaces = list(config.iter('NetworkInterface'))
    if len(interfaces) != 1 or interfaces[0].get('name') != 'lo':
        pytest.fail('Communication tests require only the lo interface')
    if any(peer.get('Address') != '127.0.0.1' for peer in config.iter('Peer')):
        pytest.fail('Communication tests refuse non-loopback peers')
