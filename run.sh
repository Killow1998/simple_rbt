#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
build_directory="${SIMPLE_RBT_BUILD_DIR:-$project_root/build/native}"
if [[ -z "${SIMPLE_RBT_BUILD_DIR:-}" ]]; then
  for argument in "$@"; do
    if [[ "$argument" == --ros1 ]]; then build_directory="$project_root/build/ros1"; fi
  done
fi
if [[ ! -f "$build_directory/ros_backend.txt" ]]; then
  echo "Build the requested C++ runtime with ./setup.sh first." >&2
  exit 1
fi
read -r ros_backend < "$build_directory/ros_backend.txt"
case "$ros_backend" in
  ROS2) source "$project_root/tools/ros_env.sh" ;;
  ROS1) source "$project_root/tools/ros1_env.sh" ;;
  NONE) ;;
  *) echo "Unknown compiled transport: $ros_backend" >&2; exit 1 ;;
esac
export LD_LIBRARY_PATH="$build_directory/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export OMP_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
if [[ ! -x "$build_directory/simple-rbt-native" ]]; then
  echo "Build the C++ runtime with ./setup.sh first." >&2
  exit 1
fi
exec "$build_directory/simple-rbt-native" --project-root "$project_root" "$@"
