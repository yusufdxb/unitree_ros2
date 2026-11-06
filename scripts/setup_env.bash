# Shared implementation for the sourceable setup scripts.
_unitree_setup() {
  local mode="$1" interface="$2" distro="$3"
  local root prefix workspace variable value other
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  distro="${distro:-${ROS_DISTRO:-}}"
  if [[ -z "$distro" ]]; then
    echo "Select a ROS distribution: source setup_local.sh jazzy (or source ROS first)." >&2
    return 1
  fi
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$distro" ]]; then
    echo "A different ROS distribution is already sourced. Start a clean shell." >&2
    return 1
  fi
  for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
    value="${!variable}"
    for other in foxy humble jazzy; do
      if [[ "$other" != "$distro" && ( "$value" == *"/opt/ros/$other/"* || ":$value:" == *"/opt/ros/$other:"* ) ]]; then
        echo "$variable contains another ROS distribution. Start a clean shell." >&2
        return 1
      fi
    done
  done
  prefix="${UNITREE_ROS_PREFIX:-/opt/ros/$distro}"
  workspace="${UNITREE_ROS_WS:-$root/cyclonedds_ws}"
  if [[ ! -f "$prefix/setup.bash" || ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "ROS setup or Unitree install missing. Build first; set UNITREE_ROS_WS for an external workspace." >&2
    return 1
  fi
  if [[ "$mode" == network ]]; then
    if [[ ! "$interface" =~ ^[a-zA-Z0-9_.:-]+$ || ! -d "/sys/class/net/$interface" ]]; then
      echo "Specify an existing robot-facing interface: source setup.sh INTERFACE [DISTRO]." >&2
      return 1
    fi
  fi
  source "$prefix/setup.bash" || return 1
  source "$workspace/install/local_setup.bash" || return 1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  case "$mode" in
    local)
      unset ROS_LOCALHOST_ONLY
      export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-173}"
      export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
      ;;
    network)
      unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
      export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$interface\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
      ;;
    default)
      unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
      ;;
  esac
}
