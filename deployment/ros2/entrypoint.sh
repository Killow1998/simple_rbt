#!/usr/bin/env bash
set -eo pipefail
if [[ "${ROS_DISTRO:-humble}" != humble || "${ROS_VERSION:-2}" != 2 ]]; then
  echo "simple-rbt-ros2 supports ROS2 Humble only; use the Ubuntu 22.04 Humble container." >&2; exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo "The Humble container requires Ubuntu 22.04." >&2; exit 1
fi
# ROS setup scripts intentionally inspect optional, unset environment variables.
source /opt/ros/humble/setup.bash
if [[ "${ROS_DISTRO:-}" != humble || "${ROS_VERSION:-}" != 2 ]]; then
  echo "Expected the actual ROS2 Humble setup, found ROS_DISTRO=${ROS_DISTRO:-unset}." >&2; exit 1
fi
source /opt/simple-rbt/interfaces/share/livox_ros_driver2/local_setup.bash
exec /opt/simple-rbt/bin/simple-rbt-ros-session "$@"
