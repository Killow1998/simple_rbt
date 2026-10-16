#!/usr/bin/env bash
# Development preparation only. Installed users launch the native desktop app.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_root"
native_only=false
desktop=ON
cuda=ON
adapters=()
for argument in "$@"; do
  case "$argument" in
    --native-only) native_only=true ;;
    --ros1) adapters+=(ros1) ;;
    --ros2) adapters+=(ros2) ;;
    --no-ros) adapters=() ;;
    --no-desktop) desktop=OFF ;;
    --cpu-only) cuda=OFF ;;
    --help|-h) echo "Usage: ./setup.sh [--native-only] [--no-desktop] [--cpu-only] [--ros1] [--ros2]"; exit 0 ;;
    *) echo "Unknown setup option: $argument" >&2; exit 2 ;;
  esac
done
if [[ "$native_only" == false ]]; then ./tools/setup-sdk.sh; fi
python3 tools/prepare_visual_assets.py
python3 tools/prepare_sensor_assets.py
if [[ "$desktop" == ON ]]; then python3 tools/setup-gui.py; fi
cmake -S "$project_root" -B "$project_root/build/desktop" \
  -DCMAKE_BUILD_TYPE=Release -DSIMPLE_RBT_ROS=NONE -DSIMPLE_RBT_DESKTOP="$desktop" -DSIMPLE_RBT_CUDA="$cuda"
cmake --build "$project_root/build/desktop" --parallel "${SIMPLE_RBT_BUILD_JOBS:-2}"
for adapter in "${adapters[@]}"; do
  python3 tools/test_ros_session.py --ros "$adapter" --build-only
done
echo "Ready: build/desktop/simple-rbt-sim and build/desktop/simple-rbt-console"
