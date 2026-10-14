#!/usr/bin/env bash
set -eo pipefail
native_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
native_livox_setup="${SIMPLE_RBT_LIVOX_SETUP:-$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash}"
source "$native_livox_setup"
native_livox_prefix="${SIMPLE_RBT_LIVOX_PREFIX:-$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2}"
export CMAKE_PREFIX_PATH="$native_livox_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cmake -S "$native_root" -B "$native_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$native_root/build" --parallel "${SIMPLE_RBT_BUILD_JOBS:-2}"
