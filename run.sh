#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
source "$project_root/tools/ros_env.sh"
export LD_LIBRARY_PATH="$project_root/build/native/lib:$LD_LIBRARY_PATH"
export OPENBLAS_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
export OMP_NUM_THREADS="${SIMPLE_RBT_CPU_THREADS:-1}"
if [[ ! -x "$project_root/build/native/simple-rbt-native" ]]; then
  echo "Build the C++ runtime with ./setup.sh first." >&2
  exit 1
fi
exec "$project_root/build/native/simple-rbt-native" --project-root "$project_root" "$@"
