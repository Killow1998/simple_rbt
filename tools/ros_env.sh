#!/usr/bin/env bash
# Source from repository launchers. Shell-only: native startup does not run Python.
ros_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export SIMPLE_RBT_LIVOX_PREFIX="${SIMPLE_RBT_LIVOX_PREFIX:-$ros_project_root/build/interfaces/ros2/install/livox_ros_driver2}"
if [[ ! -f "$SIMPLE_RBT_LIVOX_PREFIX/share/livox_ros_driver2/package.xml" ]]; then
  echo "Local Livox messages are missing; run ./setup.sh (or ./setup.sh --native-only with prepared SDK/assets)." >&2
  return 1
fi
export AMENT_PREFIX_PATH="$SIMPLE_RBT_LIVOX_PREFIX:/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$SIMPLE_RBT_LIVOX_PREFIX:/opt/ros/humble${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$SIMPLE_RBT_LIVOX_PREFIX/lib:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="/opt/ros/humble/bin:$PATH"
# Humble uses Python 3.10 on Ubuntu 22.04; include both ROS installation layouts.
export PYTHONPATH="$SIMPLE_RBT_LIVOX_PREFIX/local/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_VERSION=2 ROS_PYTHON_VERSION=3 ROS_DISTRO=humble
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
unset ros_project_root
