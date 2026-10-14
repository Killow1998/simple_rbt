#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export OMP_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export MKL_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
if [[ ! -x .venv/bin/simple-rbt-mujoco ]]; then
  echo "Run ./mujoco_sim/setup.sh first." >&2
  exit 1
fi
use_ros=0
use_viewer=0
for arg in "$@"; do
  [[ "$arg" != --ros2 ]] || use_ros=1
  [[ "$arg" != --viewer ]] || use_viewer=1
done
if [[ "$use_ros" == 1 ]]; then
  source /opt/ros/humble/setup.bash
  livox_setup="${SIMPLE_RBT_LIVOX_SETUP:-$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash}"
  if [[ ! -f "$livox_setup" ]]; then
    echo "Set SIMPLE_RBT_LIVOX_SETUP to your built livox_ros_driver2 local_setup.bash." >&2
    exit 1
  fi
  source "$livox_setup"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
fi
if [[ "$use_viewer" == 0 ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
fi
exec .venv/bin/simple-rbt-mujoco "$@"
