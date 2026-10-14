#!/usr/bin/env bash
# Source the native Noetic installation or the repository-local exported SDK.
ros1_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMPLE_RBT_ROS1_PREFIX="${SIMPLE_RBT_ROS1_PREFIX:-$ros1_project_root/build/ros1-sdk}"
if [[ -f /opt/ros/noetic/setup.bash && ! -d "$SIMPLE_RBT_ROS1_PREFIX/lib" ]]; then
  source /opt/ros/noetic/setup.bash
elif [[ -f "$SIMPLE_RBT_ROS1_PREFIX/lib/libroscpp.so" ]]; then
  export LD_LIBRARY_PATH="$SIMPLE_RBT_ROS1_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export ROS_VERSION=1 ROS_PYTHON_VERSION=3 ROS_DISTRO=noetic
else
  echo "Prepare the Noetic C++ SDK with ./setup.sh --ros1 --native-only." >&2
  return 1
fi
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
unset ros1_project_root
