#!/usr/bin/env python3
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
"""Read-only gate to pass before running a GO2 telemetry example against a robot.

Exit status is 0 only when every check passed. Any failed or inconclusive check
exits 1. The script observes the ROS graph of the current environment and writes
no topic, service, parameter or configuration; see docs/jazzy.md for the session
procedure and for what discovery itself puts on the network.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import time

# The only examples a telemetry session may start, with what each subscribes to.
APPROVED = {
    'read_low_state': ('/lowstate', 'unitree_go/msg/LowState'),
    'read_motion_state': ('/lf/sportmodestate', 'unitree_go/msg/SportModeState'),
    'read_wireless_controller': ('/wirelesscontroller', 'unitree_go/msg/WirelessController'),
}
COMMAND_TYPE = re.compile(r'^unitree_\w+/msg/\w*(Cmds?|Switch|Req|Request)$')
COMMAND_TOPIC = re.compile(r'lowcmd|cmd_vel|uwbswitch|/request$', re.IGNORECASE)
# What the ROS graph reports for a plain DDS endpoint. _NODE_NAME_UNKNOWN_ means
# the owner could not be resolved, so it is not accepted as plain DDS.
NON_ROS_OWNER = '_CREATED_BY_BARE_DDS_APP_'
# Message types an approved binary may import type support for.
IMPORTED_TYPE = re.compile(r'\b(unitree_\w+)::msg::(\w+?)_<')
PUBLISHER_TYPE = re.compile(r'rclcpp::Publisher<([\w:]+)')
# rclcpp instantiates this writer for optional subscription statistics, which
# the telemetry examples leave disabled.
STATISTICS_PUBLISHER = 'statistics_msgs::msg::MetricsMessage_'
GUARD_SECONDS = 5


class Report:

    def __init__(self):
        self.checks = []
        self.data = {}

    def check(self, name, passed, detail=''):
        self.checks.append({'name': name, 'passed': bool(passed), 'detail': str(detail)})
        print('[%s] %s%s' % ('ok' if passed else 'FAIL', name, ': %s' % detail if detail else ''),
              flush=True)
        return bool(passed)

    def passed(self):
        return bool(self.checks) and all(item['passed'] for item in self.checks)


def parse_acknowledgements(values):
    acknowledged = {}
    for value in values:
        topic, separator, count = value.partition('=')
        if not separator or not topic.startswith('/') or not count.isdecimal() or int(count) < 1:
            raise ValueError('expected /TOPIC=COUNT, got ' + value)
        acknowledged[topic] = int(count)
    return acknowledged


def check_examples(report, examples):
    rejected = [name for name in examples if name not in APPROVED]
    report.check('only approved telemetry examples', examples and not rejected,
                 'rejected: ' + ', '.join(rejected) if rejected else ', '.join(examples))
    return not rejected


def check_environment(report, args):
    env = os.environ
    domain = env.get('ROS_DOMAIN_ID')
    ok = report.check('ROS_DOMAIN_ID set explicitly and as expected', domain == str(args.domain),
                      'environment=%s expected=%d' % (domain, args.domain))
    ok &= report.check('RMW is rmw_cyclonedds_cpp',
                       env.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp',
                       env.get('RMW_IMPLEMENTATION'))
    localhost = env.get('ROS_LOCALHOST_ONLY') == '1' or \
        env.get('ROS_AUTOMATIC_DISCOVERY_RANGE') in ('LOCALHOST', 'OFF')
    ok &= report.check('discovery is not restricted to this host', not localhost)
    interfaces = re.findall(r'<NetworkInterface\s+name="([^"]+)"', env.get('CYCLONEDDS_URI', ''))
    ok &= report.check('Cyclone DDS is bound to exactly the named interface',
                       interfaces == [args.interface], 'CYCLONEDDS_URI names %s' % interfaces)
    try:
        state = Path('/sys/class/net', args.interface, 'operstate').read_text().strip()
    except OSError as error:
        state = str(error)
    ok &= report.check('interface is up', state == 'up' or (args.interface == 'lo' and
                                                            state == 'unknown'), state)
    return ok


def check_binaries(report, examples):
    from ament_index_python.packages import get_package_prefix
    directory = Path(get_package_prefix('unitree_ros2_example')) / 'lib/unitree_ros2_example'
    ok = True
    report.data['binaries'] = {}
    for name in examples:
        binary = directory / name
        if binary.is_symlink() or not binary.is_file():
            ok &= report.check(name + ' is an installed file', False, binary)
            continue
        report.data['binaries'][name] = hashlib.sha256(binary.read_bytes()).hexdigest()
        symbols = subprocess.run(['nm', '-DC', str(binary)], capture_output=True, text=True,
                                 check=True, timeout=10).stdout
        imported = sorted({package + '/msg/' + message
                           for package, message in IMPORTED_TYPE.findall(symbols)})
        ok &= report.check(name + ' imports only its telemetry type',
                           imported == [APPROVED[name][1]], imported)
        publishers = sorted(set(PUBLISHER_TYPE.findall(symbols)) - {STATISTICS_PUBLISHER})
        ok &= report.check(name + ' instantiates no message publisher', not publishers, publishers)
    return ok


def publisher_record(info):
    return {
        'node': info.node_name,
        'type': info.topic_type,
        'gid': bytes(info.endpoint_gid).hex(),
        'reliability': getattr(info.qos_profile.reliability, 'name',
                               str(info.qos_profile.reliability)),
        'durability': getattr(info.qos_profile.durability, 'name',
                              str(info.qos_profile.durability)),
    }


def is_command_topic(topic, types):
    return COMMAND_TOPIC.search(topic) is not None or \
        any(COMMAND_TYPE.match(name) for name in types)


def telemetry_problem(topic, expected_type, publishers):
    if not publishers:
        return 'no publisher discovered'
    if any(item['type'] != expected_type for item in publishers):
        return 'type mismatch: %s' % sorted({item['type'] for item in publishers})
    # Firmware endpoints are plain DDS. A named ROS node here is a bridge, a bag
    # replay or a local process, not evidence that the robot is on this domain.
    named = sorted({item['node'] for item in publishers if item['node'] != NON_ROS_OWNER})
    if named:
        return 'published by ROS node(s) %s' % named
    return ''


def observe_graph(report, args, examples, acknowledged):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    class InertPublisher:
        """Stands in for the writer rclpy would create and use during construction."""

        def publish(self, _message):
            pass

    class GraphObserver(Node):
        # rclpy always creates a /parameter_events writer and publishes the
        # use_sim_time declaration on it from the constructor. This node has no
        # writer, so it cannot publish that or anything else.
        def create_publisher(self, *_args, **_kwargs):
            return InertPublisher()

    context = Context()
    rclpy.init(args=[], context=context)
    node = None
    try:
        name = 'go2_telemetry_preflight_%d_%s' % (os.getpid(), secrets.token_hex(4))
        node = GraphObserver(
            name, context=context, use_global_arguments=False, enable_rosout=False,
            start_parameter_services=False,
            parameter_overrides=[
                Parameter('start_type_description_service', Parameter.Type.BOOL, False)])
        if hasattr(context, 'get_domain_id'):
            report.check('DDS domain in use', context.get_domain_id() == args.domain,
                         context.get_domain_id())
        own = {
            'publishers': node.get_publisher_names_and_types_by_node(name, '/'),
            'services': node.get_service_names_and_types_by_node(name, '/'),
        }
        report.check('this check owns no publisher or service', not any(own.values()), own)
        commands, telemetry = {}, {}
        telemetry_deadline = time.monotonic() + args.timeout
        settled_at = None
        while True:
            now = time.monotonic()
            for topic, types in node.get_topic_names_and_types():
                if is_command_topic(topic, types):
                    for info in node.get_publishers_info_by_topic(topic):
                        record = publisher_record(info)
                        commands[(topic, record['gid'])] = dict(record, topic=topic)
            telemetry = {}
            for example in examples:
                topic = APPROVED[example][0]
                telemetry[topic] = [publisher_record(info)
                                    for info in node.get_publishers_info_by_topic(topic)]
            found = not any(telemetry_problem(topic, APPROVED[example][1], telemetry[topic])
                            for example in examples for topic in [APPROVED[example][0]])
            if found and settled_at is None:
                # Keep looking: a command writer can be discovered after telemetry.
                settled_at = now + args.settle
            if not found:
                settled_at = None
            if (found and now >= settled_at) or (not found and now >= telemetry_deadline):
                break
            time.sleep(0.05)
        report.data['telemetry_publishers'] = telemetry
        for example in examples:
            topic, expected_type = APPROVED[example]
            problem = telemetry_problem(topic, expected_type, telemetry[topic])
            report.check('telemetry %s (%s)' % (topic, expected_type), not problem,
                         problem or '%d publisher(s)' % len(telemetry[topic]))
        check_command_writers(report, sorted(commands.values(), key=lambda r: r['topic']),
                              acknowledged)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)


def check_command_writers(report, writers, acknowledged):
    report.data['command_writers'] = writers
    named = [item for item in writers if item['node'] != NON_ROS_OWNER]
    report.check('no ROS node publishes a command topic', not named,
                 ['%s by %s' % (item['topic'], item['node']) for item in named])
    counts = {}
    for item in writers:
        if item['node'] == NON_ROS_OWNER:
            counts[item['topic']] = counts.get(item['topic'], 0) + 1
    report.data['acknowledged_nonros_writers'] = acknowledged
    report.check('no unacknowledged non-ROS writer on a command topic', counts == acknowledged,
                 'discovered %s, acknowledged %s' % (counts, acknowledged))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('examples', nargs='*', default=sorted(APPROVED),
                        help='telemetry examples to clear (default: all approved)')
    parser.add_argument('--domain', type=int, required=True, choices=range(0, 233),
                        metavar='ID', help='DDS domain the firmware uses')
    parser.add_argument('--interface', required=True, help='robot-facing network interface')
    parser.add_argument('--timeout', type=float, default=15.0, help='discovery limit, seconds')
    parser.add_argument('--settle', type=float, default=5.0,
                        help='seconds to keep watching for command writers after telemetry')
    parser.add_argument('--allow-nonros-writer', action='append', default=[], metavar='TOPIC=N',
                        help='accept exactly N plain-DDS writers on this command topic; '
                        'a ROS node on the topic always fails')
    parser.add_argument('--report', type=Path, help='write the result as JSON')
    args = parser.parse_args()
    if not 0 < args.timeout <= 60 or not 0 < args.settle <= 60:
        parser.error('--timeout and --settle must be in (0, 60]')
    try:
        acknowledged = parse_acknowledgements(args.allow_nonros_writer)
    except ValueError as error:
        parser.error(str(error))

    def expired(_signum, _frame):
        print('[FAIL] bounded run limit exceeded', flush=True)
        os._exit(1)

    signal.signal(signal.SIGALRM, expired)
    signal.alarm(int(args.timeout + args.settle) + GUARD_SECONDS)

    report = Report()
    try:
        # DDS starts only after every local precondition has passed.
        if (check_examples(report, args.examples) and
                check_environment(report, args) and
                check_binaries(report, args.examples)):
            observe_graph(report, args, args.examples, acknowledged)
    except Exception as error:  # An unexpected condition is not a pass.
        report.check('completed without error', False, repr(error))
    result = 'PASS' if report.passed() else 'FAIL'
    if args.report:
        args.report.write_text(json.dumps(
            dict(report.data, result=result, checks=report.checks,
                 domain=args.domain, interface=args.interface,
                 ros_distro=os.environ.get('ROS_DISTRO')), indent=2) + '\n')
    print('PREFLIGHT ' + result, flush=True)
    return 0 if result == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
