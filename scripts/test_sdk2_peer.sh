#!/bin/bash
# Build a peer from unitree_sdk2's own generated types and bundled Cyclone DDS,
# then test both Unitree message families against the current ROS RMW.
set -eo pipefail
if [[ -z "${ROS_DISTRO:-}" || -z "${1:-}" ]]; then
  echo "Usage: source ROS and the Unitree install, then $0 SCRATCH_DIRECTORY" >&2
  exit 2
fi
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$1"
scratch="$(cd "$1" && pwd)"
touch "$scratch/COLCON_IGNORE"
revision=aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1
sdk="$scratch/unitree_sdk2"
if [[ "$(git -C "$sdk" rev-parse -q --verify HEAD 2>/dev/null)" != "$revision" ]]; then
  rm -rf "$sdk"
  git init -q "$sdk"
  git -C "$sdk" fetch -q --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git "$revision"
  git -C "$sdk" checkout -q FETCH_HEAD
fi
test "$(git -C "$sdk" rev-parse HEAD)" = "$revision"
arch="$(uname -m)"
libraries="$sdk/thirdparty/lib/$arch"
if [[ ! -f "$libraries/libddscxx.so" ]]; then
  echo "unitree_sdk2 ships no Cyclone DDS libraries for $arch" >&2
  exit 2
fi
# Only the peer sees the SDK's headers and libraries; the ROS build never does.
# libunitree_sdk2.a holds the type-support definitions of the generated headers.
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  g++ -std=c++17 -O1 -Wall -Wextra \
    -isystem "$sdk/include" -isystem "$sdk/thirdparty/include" \
    -isystem "$sdk/thirdparty/include/ddscxx" \
    "$root/test/unitree_ros2_test/sdk2/sdk2_peer.cpp" \
    "$sdk/lib/$arch/libunitree_sdk2.a" \
    -L"$libraries" -lddscxx -lddsc -lpthread -o "$scratch/sdk2_peer"
UNITREE_SDK2_PEER="$scratch/sdk2_peer" \
UNITREE_SDK2_PEER_LIBRARY_PATH="$libraries" \
  python3 -m pytest "$root/test/unitree_ros2_test/test_sdk2_peer.py" \
    --junitxml="$scratch/sdk2-peer.xml" -v
