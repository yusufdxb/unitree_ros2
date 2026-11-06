"""Source setup scripts in clean subprocesses, including error cases."""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = Path(get_package_prefix('unitree_go')).parents[1]


def run(command, **extra):
    env = {key: os.environ[key] for key in ('HOME', 'PATH', 'LANG') if key in os.environ}
    env.update(UNITREE_ROS_WS=str(WORKSPACE), **extra)
    return subprocess.run(['bash', '--noprofile', '--norc', '-c', command],
                          env=env, text=True, capture_output=True)


def test_local_source_from_other_directory():
    result = run('cd /; source "' + str(ROOT / 'setup_local.sh') +
                 '" ' + os.environ['ROS_DISTRO'] +
                 '; test "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp && '
                 'test "$ROS_DOMAIN_ID" = 173 && [[ "$CYCLONEDDS_URI" == *127.0.0.1* ]]')
    assert result.returncode == 0, result.stderr


def test_reject_mixed_distribution():
    result = run('source "' + str(ROOT / 'setup_local.sh') + '" jazzy', ROS_DISTRO='humble')
    assert result.returncode != 0
    assert 'different ROS distribution' in result.stderr


def test_reject_missing_interface():
    result = run('source "' + str(ROOT / 'setup.sh') + '" missing-interface ' +
                 os.environ['ROS_DISTRO'])
    assert result.returncode != 0
    assert 'existing robot-facing interface' in result.stderr


def test_default_clears_stale_dds_configuration():
    result = run('source "' + str(ROOT / 'setup_default.sh') + '" ' +
                 os.environ['ROS_DISTRO'] + ' && test -z "$CYCLONEDDS_URI"',
                 CYCLONEDDS_URI='file:///nonexistent')
    assert result.returncode == 0, result.stderr
