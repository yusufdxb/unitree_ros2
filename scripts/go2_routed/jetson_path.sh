#!/bin/bash
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
# Routed path on the payload Jetson between the telemetry host and the GO2 only.
# Runtime state only (nothing persists across a reboot). Run as root on the Jetson.
#   up      record the prior state, then add the path; a second run verifies and changes nothing
#   status  print the path state and check it; changes nothing
#   down    remove the path and check that the prior state is back
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=site.env
source ./site.env
W=$JETSON_LAN_IF E=$JETSON_ROBOT_IF L=$HOST_ROBOT_IP G=$GO2_IP
PRIOR=$STATE_DIR/jetson.prior
# Each rule is matched by its tag, so only rules added here are ever removed.
RULES=(
  "-i $W -o $E -s $L/32 -d $G/32"
  "-i $E -o $W -s $G/32 -d $L/32"
)
DROPS=("-i $W -o $E" "-i $E -o $W" "-i $W -o $W" "-i $E -o $E")

die() { echo "jetson_path: $*" >&2; exit 1; }
fwd() { cat "/proc/sys/net/ipv4/conf/$1/forwarding"; }
tagged() { iptables -S FORWARD | grep -c -- "--comment $TAG" || true; }
proxied() { ip neigh show proxy dev "$E" | grep -qw "$L"; }
routed() { ip route show "$L/32" | grep -q "via $HOST_LAN_IP dev $W"; }

preconditions() {
  [ "$(id -u)" = 0 ] || die "run as root"
  ip -4 -br addr show dev "$W" | grep -qw "$JETSON_LAN_IP/24" || die "$W is not $JETSON_LAN_IP"
  ip -4 -br addr show dev "$E" | grep -qw "$JETSON_ROBOT_IP/24" || die "$E is not $JETSON_ROBOT_IP"
}

check_up() {
  local ok=0
  [ "$(tagged)" = $(( ${#RULES[@]} + ${#DROPS[@]} )) ] || { echo "FAIL tagged rules: $(tagged)"; ok=1; }
  # The accepts must come before every other rule, or a prior rule decides first.
  iptables -S FORWARD | sed -n 2,3p | grep -q -- "--comment $TAG" || { echo "FAIL accept rules not first"; ok=1; }
  [ "$(fwd "$W")$(fwd "$E")" = 11 ] || { echo "FAIL forwarding $W=$(fwd "$W") $E=$(fwd "$E")"; ok=1; }
  proxied || { echo "FAIL no proxy ARP entry for $L on $E"; ok=1; }
  routed || { echo "FAIL no route $L via $HOST_LAN_IP"; ok=1; }
  return $ok
}

show() {
  echo "forwarding all=$(fwd all) $W=$(fwd "$W") $E=$(fwd "$E")"
  iptables -S FORWARD
  ip neigh show proxy dev "$E"; ip route show "$L/32"
}

case "${1:-}" in
up)
  preconditions
  if [ -f "$PRIOR" ]; then
    check_up && { echo "path already up, unchanged"; exit 0; }
    die "partial path from an earlier run; run down first"
  fi
  [ "$(tagged)" = 0 ] || die "tagged rules exist without a recorded prior state; inspect by hand"
  proxied && die "a proxy ARP entry for $L already exists"
  # The host address must be free on the robot network before the Jetson answers for it.
  command -v arping >/dev/null || die "arping is required to check that $L is unused"
  if arping -D -q -c 2 -w 3 -I "$E" "$L"; then :; else die "$L answers on $E; choose another HOST_ROBOT_IP"; fi
  mkdir -p "$STATE_DIR"; umask 077
  { echo "FWD_W=$(fwd "$W")"; echo "FWD_E=$(fwd "$E")"; echo "FWD_ALL=$(fwd all)"
    echo "ROUTE=$(ip route show "$L/32" | head -1)"
    echo "FORWARD_SHA=$(iptables -S FORWARD | sha256sum | cut -d' ' -f1)"; } > "$PRIOR"
  echo "prior state:"; cat "$PRIOR"
  ip route replace "$L/32" via "$HOST_LAN_IP" dev "$W"
  ip neigh replace proxy "$L" dev "$E"
  n=1
  for r in "${RULES[@]}"; do
    # shellcheck disable=SC2086
    iptables -I FORWARD $n $r -m comment --comment "$TAG" -j ACCEPT; n=$((n + 1))
  done
  for r in "${DROPS[@]}"; do
    # shellcheck disable=SC2086
    iptables -I FORWARD $n $r -m comment --comment "$TAG" -j DROP; n=$((n + 1))
  done
  # Forwarding on the two interfaces only; IPv4 decides per input interface,
  # so other interfaces keep their prior setting.
  echo 1 > "/proc/sys/net/ipv4/conf/$W/forwarding"
  echo 1 > "/proc/sys/net/ipv4/conf/$E/forwarding"
  show; check_up && echo "PATH UP" ;;
status)
  show
  if [ -f "$PRIOR" ]; then check_up && echo "PATH UP"; else
    [ "$(tagged)" = 0 ] && ! proxied && echo "PATH DOWN" || { echo "PATH INCONSISTENT"; exit 1; }
  fi ;;
down)
  [ "$(id -u)" = 0 ] || die "run as root"
  [ -f "$PRIOR" ] || { [ "$(tagged)" = 0 ] && ! proxied && { echo "path already down"; exit 0; }
                       die "no recorded prior state but path pieces exist; inspect by hand"; }
  # shellcheck disable=SC1090
  source "$PRIOR"
  while iptables -S FORWARD | grep -q -- "--comment $TAG"; do
    spec=$(iptables -S FORWARD | grep -m1 -- "--comment $TAG" | sed 's/^-A /-D /')
    # shellcheck disable=SC2086
    eval iptables $spec
  done
  echo "$FWD_W" > "/proc/sys/net/ipv4/conf/$W/forwarding"
  echo "$FWD_E" > "/proc/sys/net/ipv4/conf/$E/forwarding"
  ip neigh del proxy "$L" dev "$E" 2>/dev/null || true
  [ -n "$ROUTE" ] || ip route del "$L/32" 2>/dev/null || true
  bad=0
  [ "$(fwd "$W")$(fwd "$E")$(fwd all)" = "$FWD_W$FWD_E$FWD_ALL" ] || { echo "MISMATCH forwarding"; bad=1; }
  [ "$(iptables -S FORWARD | sha256sum | cut -d' ' -f1)" = "$FORWARD_SHA" ] || { echo "MISMATCH FORWARD chain"; bad=1; }
  [ "$(ip route show "$L/32" | head -1)" = "$ROUTE" ] || { echo "MISMATCH route"; bad=1; }
  proxied && { echo "MISMATCH proxy entry"; bad=1; }
  show
  [ $bad = 0 ] || { echo "ROLLBACK MISMATCH: prior state kept in $PRIOR"; exit 1; }
  rm -f "$PRIOR"; echo "ROLLBACK OK" ;;
*) echo "usage: $0 up|status|down" >&2; exit 2 ;;
esac
