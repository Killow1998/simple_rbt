#!/usr/bin/env bash
# Explicit Python oracle; the default repository launcher runs C++.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_root"
export OMP_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export MKL_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
if [[ ! -x .venv/bin/python ]]; then
  echo "Run ./setup.sh first." >&2
  exit 1
fi
use_ros=0
use_viewer=0
for arg in "$@"; do
  [[ "$arg" != --ros2 ]] || use_ros=1
  [[ "$arg" != --viewer ]] || use_viewer=1
done
if [[ "$use_ros" == 1 ]]; then
  source "$project_root/tools/ros_env.sh"
fi
export PYTHONPATH="$project_root/reference/python${PYTHONPATH:+:$PYTHONPATH}"
if [[ "$use_viewer" == 0 ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
fi
exec .venv/bin/python -m simple_rbt_mujoco.cli "$@"
