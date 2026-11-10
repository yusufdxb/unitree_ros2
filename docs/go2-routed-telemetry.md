# Read-only GO2 telemetry over a routed path

This procedure connects a native Jazzy host that is not wired to the robot to a
GO2's DDS network through the payload Jetson, runs only the approved telemetry
examples, and removes every change afterwards. It complements the session in
[jazzy.md](jazzy.md#telemetry-only-go2-session) and uses the same preflight.
The scripts are in `scripts/go2_routed/`; every site value is in `site.env`.

Status: the path was first built by hand on 2026-11-10 (results below). The
scripts here reproduce that path with recorded prior state and rollback checks.
They have been syntax-checked, but have not yet run against the robot.

## Topology

```text
 lab PC (Jazzy, Cyclone 0.10.5)            payload Jetson                GO2 (Cyclone 0.10.2)
 wlxc01c303625d6 192.168.0.137  -- Wi-Fi --  wlx3c64cf2d28e2 192.168.0.70
 go2fwd (veth)   192.168.123.137/32          enP8p1s0 192.168.123.18 -- wired -- 192.168.123.161
 route 192.168.123.161/32                    route 192.168.123.137/32 via 192.168.0.137
   via 192.168.0.70 src .137                 proxy ARP for .137 on enP8p1s0
                                             forwarding on the two interfaces only
                                             FORWARD: .137 <-> .161 accepted, all else between
                                             the two interfaces dropped (tagged rules)
                                             spdp_relay.py: GO2 SPDP multicast -> 192.168.0.137:7410
```

Why each part is needed:

- **Robot-subnet address on the host.** The firmware sends to the locators that
  a participant advertises. The host advertises `192.168.123.137`, carried by a
  veth end so that Cyclone DDS can bind to it by name, and routes the GO2 address
  through the Jetson with that source address.
- **Jetson routing.** The Jetson answers ARP for `.137` on the robot network and
  routes it back to the host over Wi-Fi. Forwarding is enabled on the two
  interfaces only; IPv4 decides per input interface, so other interfaces keep
  their setting. On 2026-11-10 the global `ip_forward` was used with the FORWARD
  policy `ACCEPT`, which briefly allowed forwarding between any interface pair
  that was not covered by the drop rules.
- **Discovery relay.** Firmware participants announce themselves only by
  multicast to `239.255.0.1:7400`, and use ephemeral unicast ports, so listing the
  GO2 as a peer is not enough: the host's announcements reach the robot, but the
  host never learns the firmware locators. The relay receives the GO2's SPDP on the
  robot network and copies it to the host's unicast discovery port (7410 for
  participant index 0). It binds to the group address, so unicast traffic for
  the Jetson's own participants is never delivered to it, and it sends nothing
  toward the robot.
- **Host firewall.** firewalld zone `public` silently drops unsolicited UDP.
  Runtime rich rules accept traffic to `.137` from the GO2 (any port) and from the
  Jetson LAN address (relayed SPDP). Nothing is added permanently; `down` compares
  both the runtime and permanent zone with the recorded state.

## What the preflight can and cannot see here

The host discovers only participants that are reachable over this path, which
are the firmware participants at `192.168.123.161`. A DDS stack on the Jetson
itself (MOSAIC, a GO2 bridge) is on the same robot network but is not discovered
by the host, so its command writers never appear in the preflight report. On
2026-11-10 a MOSAIC live run held motion authority on the Jetson while the
examples ran, and the preflight could not see it. `jetson_relay.sh check`
closes that gap: it fails while any Jetson process holds a domain-0 DDS port, or
while the relay has logged SPDP from any robot-network address other than the
GO2 (`FOREIGN`). Run it as root so it sees every user's sockets.

## Procedure

Each step that changes a machine or connects to the robot needs its own approval.
Stop at the first failure; teardown is always safe to run.

1. **Supervision.** Robot at rest, operator present with the remote, no other
   session using the robot or the Jetson, and the owners of other stacks informed.
2. **Jetson quiet (read-only).** `sudo scripts/go2_routed/jetson_relay.sh check`
   must print `JETSON QUIET`. Any listed process is a reason to stop, not to
   acknowledge.
3. **Paths up (approval: network change).** On the Jetson
   `sudo scripts/go2_routed/jetson_path.sh up`, then on the host
   `sudo scripts/go2_routed/host_path.sh up`. Each prints its prior state and ends
   with `PATH UP`. A second run changes nothing.
4. **Relay (approval: connects to the robot network).** On the Jetson
   `scripts/go2_routed/jetson_relay.sh start`. It stops by itself after
   `RELAY_MAX_SECONDS`. Repeat `sudo jetson_relay.sh check`: `FOREIGN` stops the session.
5. **Survey.** On the host `scripts/go2_routed/session.sh survey OUT-survey`. It
   fails while firmware writers are unacknowledged. The supervisor reviews the
   listed writers (all of them must be plain DDS on firmware participants, see
   `participant`) and writes the exact `/TOPIC=COUNT` lines to an ACKFILE.
6. **Session.** `scripts/go2_routed/session.sh run OUT ACKFILE` runs the before
   check, a `--monitor` check alongside the three examples, and the after check
   with `--baseline`. It prints `SESSION CLEAN` only when all three exit 0.
7. **Teardown, always.** `jetson_relay.sh stop`, `sudo host_path.sh down`,
   `sudo jetson_path.sh down`. Each ends with `ROLLBACK OK` after comparing the
   result with the recorded prior state; `ROLLBACK MISMATCH` keeps the state file
   and needs inspection by hand. `status` on either side reports `PATH DOWN`.

Reading the after check: exit 3 means that the only difference from the
baseline is acknowledged firmware writers that disappeared, with no writer
added. The report lists each one with its participant prefix and whether that
participant is still in the graph. It remains a failed check. A writer that
appeared since the baseline always exits 1, with its attribution: an approved
example's node, another ROS node, a new writer of a firmware participant from
the baseline, or an unidentified participant. The count acknowledgement alone
would not notice a writer replaced by another on the same topic.

## Results of 2026-11-10

Revision: preflight at `a6fb2e7` (clean tree). The example binaries were built
on 2026-11-06 from a tree whose example sources equal `a6fb2e7`; their SHA-256
digests are in the reports.

| Item | Observed |
| --- | --- |
| Discovery | 23 firmware participants, host `Unitree`, all at `192.168.123.161`, vendor Eclipse, `Unitree/0.10.2/Linux` |
| `/lowstate` | `unitree_go/msg/LowState`, 1 plain-DDS writer, RELIABLE/VOLATILE; 4053 messages in 8.39 s, about 483 Hz after the first sample (about 7 s after start) |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState`, 1 writer, RELIABLE/VOLATILE; 231 messages in 11.5 s, 20 Hz |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController`, 2 writers, RELIABLE/VOLATILE; no messages (remote not transmitting) |
| Command writers | 50 plain-DDS writers on 21 command/request topics, all on 13 firmware participants; acknowledged per exact count |
| After check | failed: `/api/vui/request` writer `01103c36…:a03` of firmware process `python3` (pid 1071 on the GO2) was gone; no writer was added |

Earlier summaries quoted about 270 Hz and 15 Hz; those divided by the full 15 s
run, including discovery time.

The robot was not at rest. A MOSAIC live run started on the Jetson at 19:50 UTC
and held motion authority from 19:50:58. While `read_motion_state` ran, the
robot moved 1.27 m at up to 1.05 m/s. The telemetry session published nothing,
but it did not run under the conditions in step 1.
