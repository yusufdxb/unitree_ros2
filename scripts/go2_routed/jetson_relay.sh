#!/bin/bash
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
# Discovery relay and robot-network quiet check on the payload Jetson. Unprivileged.
#   check   read-only: fail if any process on the Jetson holds a DDS domain port, or
#           the relay has logged a FOREIGN participant; neither is visible to the host
#   start   start the relay once (it stops by itself after RELAY_MAX_SECONDS)
#   status  print the relay state
#   stop    stop the relay started here, identified by its pid file and command line
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=site.env
source ./site.env
RUN=$HOME/go2-routed
PID=$RUN/relay.pid LOG=$RUN/relay.log

relay_pid() {
  [ -f "$PID" ] || return 1
  local p; p=$(cat "$PID")
  tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q 'spdp_relay.py' || return 1
  echo "$p"
}

case "${1:-}" in
check)
  ok=0; mine=$(relay_pid || true)
  # Run as root to see the sockets of every user; unprivileged, only this user's.
  holders=$(ss -uanp 2>/dev/null | awk -v lo="$DDS_PORT" -v hi="$DDS_PORT_MAX" '{
      n = split($4, a, ":"); p = a[n] + 0
      if (p >= lo && p <= hi) { while (match($0, /pid=[0-9]+/)) {
        print substr($0, RSTART + 4, RLENGTH - 4); $0 = substr($0, RSTART + RLENGTH) } } }' |
    sort -un | grep -vx "${mine:-none}" || true)
  for p in $holders; do
    echo "DDS PROCESS $p $(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-160)"; ok=1
  done
  [ "$(id -u)" = 0 ] || echo "note: unprivileged, only $(id -un)'s processes were inspected"
  if [ -f "$LOG" ] && grep -q '"FOREIGN"' "$LOG"; then grep '"FOREIGN"' "$LOG"; ok=1; fi
  [ $ok = 0 ] && echo "JETSON QUIET" || { echo "JETSON NOT QUIET"; exit 1; } ;;
start)
  if p=$(relay_pid); then echo "relay already running: $p"; exit 0; fi
  mkdir -p "$RUN"
  # Each relay run gets its own log, so check reports only this run's participants.
  [ -f "$LOG" ] && mv "$LOG" "$LOG.$(date -u +%Y%m%dT%H%M%SZ)"
  setsid python3 -I ./spdp_relay.py --group "$DDS_GROUP" --port "$DDS_PORT" \
    --local "$JETSON_ROBOT_IP" --source "$GO2_IP" --dest "$HOST_LAN_IP" \
    --dest-port "$HOST_SPDP_PORT" --max-seconds "$RELAY_MAX_SECONDS" \
    >> "$LOG" 2>&1 < /dev/null &
  echo $! > "$PID"; sleep 1
  p=$(relay_pid) || { tail -5 "$LOG"; echo "relay did not start"; exit 1; }
  echo "relay started: $p" ;;
status)
  if p=$(relay_pid); then echo "relay running: $p"; else echo "relay not running"; fi
  [ -f "$LOG" ] && tail -3 "$LOG" || true ;;
stop)
  if p=$(relay_pid); then
    kill -TERM "$p"
    for _ in 1 2 3 4 5; do kill -0 "$p" 2>/dev/null || break; sleep 1; done
    kill -0 "$p" 2>/dev/null && { echo "relay $p still running"; exit 1; }
  fi
  rm -f "$PID"; echo "relay stopped" ;;
*) echo "usage: $0 check|start|status|stop" >&2; exit 2 ;;
esac
