"""BaseClient against a separate plain Cyclone DDS API server (no ROS on the server)."""
import os
import subprocess
import uuid


def test_api_client_against_plain_dds_server():
    namespace = 'unitree_test/api_' + uuid.uuid4().hex
    server_env = os.environ.copy()
    server_env['UNITREE_DDS_TEST_NAMESPACE'] = namespace
    if os.environ.get('UNITREE_DDS_PEER_LIBRARY_PATH'):
        server_env['LD_LIBRARY_PATH'] = os.environ['UNITREE_DDS_PEER_LIBRARY_PATH']
    client_env = os.environ.copy()
    client_env['UNITREE_DDS_TEST_NAMESPACE'] = namespace
    server = subprocess.Popen(
        [os.environ['UNITREE_DDS_API_SERVER']], env=server_env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        # Started together on purpose: the client must not depend on settle time.
        client = subprocess.run(
            [os.environ['UNITREE_API_CLIENT_PEER_TEST']], env=client_env,
            capture_output=True, text=True, timeout=30)
        server_output, server_error = server.communicate(timeout=30)
    finally:
        if server.poll() is None:
            server.kill()
            server.communicate()
    assert client.returncode == 0, client.stdout + client.stderr + server_error
    assert server.returncode == 0, (server_output, server_error)
