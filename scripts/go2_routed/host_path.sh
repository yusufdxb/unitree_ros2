#!/bin/bash
# Copyright 2026 Yusuf Guenena
# SPDX-License-Identifier: BSD-3-Clause
# Routed path on the telemetry host (lab PC). Runtime state only. Run as root.
#   up      record the prior state, add the robot-subnet address, the GO2 route and
#           runtime firewalld rules; a second run verifies and changes nothing
#   status  print the path state and check it; changes nothing
#   down    remove the path and check that the prior state is back
set -euo pipefail
cd "$(dirname "$0")"
# shellcheck source=site.env
source ./site.env
PRIOR=$STATE_DIR/host.prior
# firewalld zone "public" drops unsolicited UDP. The GO2 sends from ephemeral
# ports, and the relay from the Jetson's LAN address, so both sources are
# accepted, for the robot-subnet address only.
RULES=(
  "rule family=\"ipv4\" source address=\"$GO2_IP/32\" destination address=\"$HOST_ROBOT_IP/32\" accept"
  "rule family=\"ipv4\" source address=\"$JETSON_LAN_IP/32\" destination address=\"$HOST_ROBOT_IP/32\" accept"
)

die() { echo "host_path: $*" >&2; exit 1; }
zone() { firewall-cmd --get-zone-of-interface="$HOST_LAN_IF" 2>/dev/null || firewall-cmd --get-default-zone; }
rules_present() { local n=0 r; for r in "${RULES[@]}"; do
  firewall-cmd --zone="$1" --query-rich-rule="$r" >/dev/null 2>&1 && n=$((n + 1)); done; echo $n; }
routed() { ip route show "$GO2_IP/32" | grep -q "via $JETSON_LAN_IP dev $HOST_LAN_IF src $HOST_ROBOT_IP"; }

check_up() {
  local ok=0 z; z=$(zone)
  ip -4 -br addr show dev "$HOST_ROBOT_IF" 2>/dev/null | grep -qw "$HOST_ROBOT_IP/32" ||
    { echo "FAIL $HOST_ROBOT_IF lacks $HOST_ROBOT_IP"; ok=1; }
  [ "$(cat "/sys/class/net/$HOST_ROBOT_IF/operstate" 2>/dev/null)" = up ] || { echo "FAIL $HOST_ROBOT_IF not up"; ok=1; }
  routed || { echo "FAIL no route to $GO2_IP via $JETSON_LAN_IP"; ok=1; }
  [ "$(rules_present "$z")" = ${#RULES[@]} ] || { echo "FAIL firewalld rules in zone $z"; ok=1; }
  return $ok
}

show() {
  ip -br addr show dev "$HOST_ROBOT_IF" 2>/dev/null || echo "$HOST_ROBOT_IF absent"
  ip route show "$GO2_IP/32"; echo "zone $(zone):"; firewall-cmd --zone="$(zone)" --list-rich-rules
}

case "${1:-}" in
up)
  [ "$(id -u)" = 0 ] || die "run as root"
  ip -4 -br addr show dev "$HOST_LAN_IF" | grep -qw "$HOST_LAN_IP/24" || die "$HOST_LAN_IF is not $HOST_LAN_IP"
  if [ -f "$PRIOR" ]; then
    check_up && { echo "path already up, unchanged"; exit 0; }
    die "partial path from an earlier run; run down first"
  fi
  ip link show "$HOST_ROBOT_IF" >/dev/null 2>&1 && die "$HOST_ROBOT_IF already exists"
  ip route show "$GO2_IP/32" | grep -q . && die "a route to $GO2_IP already exists"
  z=$(zone); [ "$(rules_present "$z")" = 0 ] || die "rich rules already present in zone $z"
  mkdir -p "$STATE_DIR"; umask 077
  { echo "ZONE=$z"
    echo "RUNTIME_SHA=$(firewall-cmd --zone="$z" --list-all | sha256sum | cut -d' ' -f1)"
    echo "PERMANENT_SHA=$(firewall-cmd --permanent --zone="$z" --list-all | sha256sum | cut -d' ' -f1)"
  } > "$PRIOR"
  # A veth end gives Cyclone DDS a named interface that carries only the robot-subnet address.
  ip link add "$HOST_ROBOT_IF" type veth peer name "$HOST_ROBOT_IF-p"
  ip addr add "$HOST_ROBOT_IP/32" dev "$HOST_ROBOT_IF"
  ip link set "$HOST_ROBOT_IF-p" up; ip link set "$HOST_ROBOT_IF" up
  ip route add "$GO2_IP/32" via "$JETSON_LAN_IP" dev "$HOST_LAN_IF" src "$HOST_ROBOT_IP"
  for r in "${RULES[@]}"; do firewall-cmd --zone="$z" --add-rich-rule="$r" >/dev/null; done
  show; check_up && echo "PATH UP" ;;
status)
  show
  if [ -f "$PRIOR" ]; then check_up && echo "PATH UP"; else
    ! ip link show "$HOST_ROBOT_IF" >/dev/null 2>&1 && [ "$(rules_present "$(zone)")" = 0 ] &&
      echo "PATH DOWN" || { echo "PATH INCONSISTENT"; exit 1; }
  fi ;;
down)
  [ "$(id -u)" = 0 ] || die "run as root"
  [ -f "$PRIOR" ] || { ! ip link show "$HOST_ROBOT_IF" >/dev/null 2>&1 && { echo "path already down"; exit 0; }
                       die "no recorded prior state but $HOST_ROBOT_IF exists; inspect by hand"; }
  # shellcheck disable=SC1090
  source "$PRIOR"
  ip route del "$GO2_IP/32" 2>/dev/null || true
  ip link del "$HOST_ROBOT_IF" 2>/dev/null || true
  for r in "${RULES[@]}"; do firewall-cmd --zone="$ZONE" --remove-rich-rule="$r" >/dev/null 2>&1 || true; done
  bad=0
  ip link show "$HOST_ROBOT_IF" >/dev/null 2>&1 && { echo "MISMATCH $HOST_ROBOT_IF exists"; bad=1; }
  ip route show "$GO2_IP/32" | grep -q . && { echo "MISMATCH route"; bad=1; }
  [ "$(firewall-cmd --zone="$ZONE" --list-all | sha256sum | cut -d' ' -f1)" = "$RUNTIME_SHA" ] ||
    { echo "MISMATCH runtime firewall zone $ZONE"; bad=1; }
  [ "$(firewall-cmd --permanent --zone="$ZONE" --list-all | sha256sum | cut -d' ' -f1)" = "$PERMANENT_SHA" ] ||
    { echo "MISMATCH permanent firewall zone $ZONE"; bad=1; }
  show
  [ $bad = 0 ] || { echo "ROLLBACK MISMATCH: prior state kept in $PRIOR"; exit 1; }
  rm -f "$PRIOR"; echo "ROLLBACK OK" ;;
*) echo "usage: $0 up|status|down" >&2; exit 2 ;;
esac
