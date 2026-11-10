#!/usr/bin/env python3
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
"""Copy GO2 discovery multicast to the telemetry host, and log other participants.

Runs unprivileged on the payload Jetson. It only receives on the robot network:
SPDP datagrams from the GO2 address are sent to the host's unicast discovery
port, and nothing is sent toward the robot. SPDP from any other address on the
robot network (a stack on the Jetson itself, for example) is not forwarded; each
such participant is written once to the log as FOREIGN, because the host cannot
discover it over the routed path and its writers stay invisible to the preflight.
"""
import argparse
import json
import socket
import struct
import time

# Linux: deliver only the groups this socket joined, not those of other sockets.
IP_MULTICAST_ALL = getattr(socket, 'IP_MULTICAST_ALL', 49)


def spdp_identity(data):
    """Return (guid prefix hex, process name) of an RTPS datagram, or None."""
    if len(data) < 20 or data[:4] != b'RTPS':
        return None
    prefix = data[8:20].hex()
    name = ''
    index = data.find(b'__ProcessName\x00')
    if index >= 0:
        # PL_CDR_LE property: the name string, padded to 4, then a length-prefixed value.
        end = index + len('__ProcessName') + 1
        end += -end % 4
        if end + 4 <= len(data):
            length, = struct.unpack_from('<I', data, end)
            name = data[end + 4:end + 4 + length].rstrip(b'\x00').decode(errors='replace')
    return prefix, name


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--group', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--local', required=True, help='Jetson address on the robot network')
    parser.add_argument('--source', required=True, help='GO2 address')
    parser.add_argument('--dest', required=True, help='host LAN address')
    parser.add_argument('--dest-port', type=int, required=True)
    parser.add_argument('--max-seconds', type=float, required=True)
    args = parser.parse_args()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bound to the group address, so unicast to the Jetson's own DDS port 7400
    # is never delivered here instead of to the Jetson's participants.
    receiver.bind((args.group, args.port))
    receiver.setsockopt(socket.IPPROTO_IP, IP_MULTICAST_ALL, 0)
    receiver.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack(
        '4s4s', socket.inet_aton(args.group), socket.inet_aton(args.local)))
    receiver.settimeout(1.0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + args.max_seconds
    relayed, foreign, last = 0, {}, time.monotonic()
    print(json.dumps({'event': 'start', 'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                      **vars(args)}), flush=True)
    while time.monotonic() < deadline:
        try:
            data, (address, _) = receiver.recvfrom(65535)
        except socket.timeout:
            data, address = None, None
        if data is not None and address == args.source:
            try:
                sender.sendto(data, (args.dest, args.dest_port))
                relayed += 1
            except OSError:
                pass
        elif data is not None:
            identity = spdp_identity(data)
            if identity and identity[0] not in foreign:
                foreign[identity[0]] = (address, identity[1])
                print(json.dumps({'event': 'FOREIGN', 'address': address, 'prefix': identity[0],
                                  'process': identity[1]}), flush=True)
        if time.monotonic() - last >= 10:
            print(json.dumps({'event': 'count', 'utc': time.strftime('%H:%M:%S', time.gmtime()),
                              'relayed': relayed, 'foreign': len(foreign)}), flush=True)
            last = time.monotonic()
    print(json.dumps({'event': 'stop', 'reason': 'max-seconds', 'relayed': relayed,
                      'foreign': len(foreign)}), flush=True)


if __name__ == '__main__':
    main()
