#!/usr/bin/env bash
# Prepare dependencies and build the repository-local ROS messages and C++ runtime.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
case "${1:-}" in
  --native-only) [[ $# == 1 ]] || { echo "Usage: ./setup.sh [--native-only]" >&2; exit 2; } ;;
  --help|-h) echo "Usage: ./setup.sh [--native-only]"; echo "--native-only: reuse existing SDK, Python environment and assets; build messages and native targets."; exit 0 ;;
  "") ./tools/setup-sdk.sh ;;
  *) echo "Usage: ./setup.sh [--native-only]" >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
interface_build="$project_root/build/ros_interfaces"
colcon --log-base "$interface_build/log" build \
  --base-paths "$project_root/ros2_interfaces/livox_ros_driver2" \
  --packages-select livox_ros_driver2 \
  --build-base "$interface_build/build" \
  --install-base "$interface_build/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source "$project_root/tools/ros_env.sh"
cmake -S "$project_root" -B "$project_root/build/native" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dlivox_ros_driver2_DIR="$SIMPLE_RBT_LIVOX_PREFIX/share/livox_ros_driver2/cmake"
cmake --build "$project_root/build/native" --parallel "${SIMPLE_RBT_BUILD_JOBS:-2}"
echo "Ready: ./run.sh --robot go2 --cmd .3 0 0 --duration 10"
