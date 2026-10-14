#!/usr/bin/env bash
set -eo pipefail
native_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$native_root/.." && pwd)"
cd "$project_root"
native_livox_prefix="${SIMPLE_RBT_LIVOX_PREFIX:-$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2}"
export AMENT_PREFIX_PATH="$native_livox_prefix:/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$native_root/build/lib:$native_livox_prefix/lib:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export OMP_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
if [[ ! -x "$native_root/build/simple-rbt-native" ]]; then
  echo "Build the C++ runtime with ./mujoco_sim_cpp/setup.sh first." >&2
  exit 1
fi
exec "$native_root/build/simple-rbt-native" --project-root "$project_root" "$@"
