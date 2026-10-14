#!/usr/bin/env bash
# Prepare dependencies and build the repository-local ROS messages and C++ runtime.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
native_only=false
ros_backend=ROS2
for argument in "$@"; do
  case "$argument" in
    --native-only) native_only=true ;;
    --ros1) ros_backend=ROS1 ;;
    --ros2) ros_backend=ROS2 ;;
    --no-ros) ros_backend=NONE ;;
    --help|-h) echo "Usage: ./setup.sh [--native-only] [--ros1|--ros2|--no-ros]"; exit 0 ;;
    *) echo "Unknown setup option: $argument" >&2; exit 2 ;;
  esac
done
if [[ "$native_only" == false ]]; then ./tools/setup-sdk.sh; fi
native_build="$project_root/build/native"
cmake_options=(-DSIMPLE_RBT_ROS="$ros_backend")
if [[ "$ros_backend" == ROS2 ]]; then
  source /opt/ros/humble/setup.bash
  interface_build="$project_root/build/interfaces/ros2"
  colcon --log-base "$interface_build/log" build \
    --base-paths "$project_root/interfaces/livox_ros_driver2" \
    --packages-select livox_ros_driver2 \
    --build-base "$interface_build/build" \
    --install-base "$interface_build/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
  source "$project_root/tools/ros_env.sh"
  cmake_options+=(-Dlivox_ros_driver2_DIR="$SIMPLE_RBT_LIVOX_PREFIX/share/livox_ros_driver2/cmake")
elif [[ "$ros_backend" == ROS1 ]]; then
  native_build="$project_root/build/ros1"
  noetic_image="${SIMPLE_RBT_NOETIC_IMAGE:-simple-rbt-ros1-test:local}"
  if [[ ! -f "$project_root/build/ros1-sdk/source.json" ]]; then
    .venv/bin/python tools/prepare_ros1.py --image "$noetic_image"
  fi
  mkdir -p "$project_root/build/interfaces/ros1"
  docker run --rm --user "$(id -u):$(id -g)" --entrypoint /bin/bash \
    --mount "type=bind,src=$project_root/interfaces,dst=/source,readonly" \
    --mount "type=bind,src=$project_root/build/interfaces/ros1,dst=/build" \
    "$noetic_image" -c 'set -e; source /opt/ros/noetic/setup.bash; cmake -S /source/livox_ros_driver2 -B /build/cmake -DCMAKE_INSTALL_PREFIX=/build/install; cmake --build /build/cmake --target install --parallel 2'
  source "$project_root/tools/ros1_env.sh"
  cmake_options+=(-DSIMPLE_RBT_ROS1_PREFIX="$SIMPLE_RBT_ROS1_PREFIX")
else
  native_build="$project_root/build/standalone"
fi
cmake -S "$project_root" -B "$native_build" \
  -DCMAKE_BUILD_TYPE=Release \
  "${cmake_options[@]}"
cmake --build "$native_build" --parallel "${SIMPLE_RBT_BUILD_JOBS:-2}"
echo "Ready: ./run.sh --robot go2 --cmd .3 0 0 --duration 10"
