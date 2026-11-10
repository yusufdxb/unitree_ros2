#!/bin/bash
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
# Supervised read-only telemetry session over the routed path. Unprivileged, on the host.
#   survey OUT       one preflight without acknowledgements, for the supervisor to review
#   run OUT ACKFILE  preflight before, a monitor alongside the examples, preflight after
# OUT must not exist: every session writes a new directory and never overwrites evidence.
# ACKFILE holds one reviewed /TOPIC=COUNT per line, taken from the survey report.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# shellcheck source=site.env
source "$HERE/site.env"
mode=${1:-} OUT=${2:-}
[ -n "$OUT" ] && [ "$mode" = survey -o "$mode" = run ] || { echo "usage: $0 survey OUT | run OUT ACKFILE" >&2; exit 2; }
[ -e "$OUT" ] && { echo "$OUT exists; choose a new directory" >&2; exit 2; }
ACK=()
if [ "$mode" = run ]; then
  [ -f "${3:-}" ] || { echo "run needs a reviewed ACKFILE" >&2; exit 2; }
  while IFS= read -r line; do [ -n "$line" ] && ACK+=(--allow-nonros-writer "$line"); done < "$3"
fi
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
cd "$REPO" || exit 2
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export UNITREE_ROS_WS=${UNITREE_ROS_WS:-$(cd "$REPO/../.." && pwd)}
# shellcheck disable=SC1091
source setup.sh "$HOST_ROBOT_IF" jazzy >/dev/null || exit 2
export ROS_DOMAIN_ID=0
# No multicast on the routed path: the GO2 is a unicast peer, and its discovery
# multicast reaches this host through the Jetson relay.
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$HOST_ROBOT_IF\" multicast=\"false\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>0</ParticipantIndex><Peers><Peer Address=\"$GO2_IP\"/><Peer Address=\"$GO2_IP:$DDS_PORT\"/></Peers></Discovery></Domain></CycloneDDS>"
PRE=(python3 scripts/go2_telemetry_preflight.py --domain 0 --interface "$HOST_ROBOT_IF" --timeout 45)
bin="$(ros2 pkg prefix unitree_ros2_example)/lib/unitree_ros2_example"
{
  echo "utc_start $(date -u +%FT%TZ)"; echo "mode $mode"; echo "host $(hostname)"
  echo "revision $(git rev-parse HEAD) dirty=$(git status --porcelain | grep -c .)"
  sha256sum scripts/go2_telemetry_preflight.py "$HERE/site.env" "$bin"/read_low_state \
    "$bin"/read_motion_state "$bin"/read_wireless_controller
  env | grep -E '^(ROS_|RMW_|CYCLONEDDS_URI)' | sort
  [ "$mode" = run ] && sed 's/^/ack /' "$3"
} > "$OUT/manifest.txt"
if [ "$mode" = survey ]; then
  "${PRE[@]}" --report "$OUT/preflight-survey.json" > "$OUT/preflight-survey.log"; rc=$?
  echo "survey exit=$rc"; echo "survey exit=$rc" >> "$OUT/manifest.txt"; exit $rc
fi
"${PRE[@]}" "${ACK[@]}" --report "$OUT/preflight-before.json" > "$OUT/preflight-before.log"
rc=$?; echo "preflight-before exit=$rc" | tee -a "$OUT/manifest.txt"
[ $rc = 0 ] || exit $rc
# The monitor covers all three examples, so a writer an example creates and
# removes while it runs is still recorded.
"${PRE[@]}" "${ACK[@]}" --monitor 60 --baseline "$OUT/preflight-before.json" \
  --report "$OUT/preflight-during.json" > "$OUT/preflight-during.log" &
monitor=$!
sleep 8
for example in read_low_state read_motion_state read_wireless_controller; do
  timeout --preserve-status -s INT 15 "$bin/$example" > "$OUT/$example.log" 2>&1
  echo "$example exit=$? lines=$(wc -l < "$OUT/$example.log")" | tee -a "$OUT/manifest.txt"
done
wait $monitor; during=$?
"${PRE[@]}" "${ACK[@]}" --baseline "$OUT/preflight-before.json" \
  --report "$OUT/preflight-after.json" > "$OUT/preflight-after.log"; after=$?
{ echo "preflight-during exit=$during"; echo "preflight-after exit=$after"
  echo "utc_end $(date -u +%FT%TZ)"; } | tee -a "$OUT/manifest.txt"
# 3 means only acknowledged firmware writers were withdrawn; it is reported, never accepted here.
[ $during = 0 ] && [ $after = 0 ] && { echo "SESSION CLEAN"; exit 0; }
echo "SESSION NOT CLEAN: review preflight-during.json and preflight-after.json"; exit 1
