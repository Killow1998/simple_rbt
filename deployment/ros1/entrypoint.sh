#!/usr/bin/env bash
set -eo pipefail
if [[ "${ROS_DISTRO:-noetic}" != noetic || "${ROS_VERSION:-1}" != 1 ]]; then
  echo "simple-rbt-ros1 supports ROS1 Noetic only; use the Ubuntu 20.04 Noetic container." >&2; exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 20.04 ]]; then
  echo "The Noetic container requires Ubuntu 20.04; an Ubuntu 22.04 host can run it through Docker." >&2; exit 1
fi
source /opt/ros/noetic/setup.bash
if [[ "${ROS_DISTRO:-}" != noetic || "${ROS_VERSION:-}" != 1 ]]; then
  echo "Expected the actual ROS1 Noetic setup, found ROS_DISTRO=${ROS_DISTRO:-unset}." >&2; exit 1
fi
source /opt/simple-rbt/interfaces/setup.bash
master_pid=""
cleanup() {
  if [[ -n "$master_pid" ]]; then kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
# Reuse a reachable master. Only create a local one when none is running.
if ! timeout 3s rosparam list >/dev/null 2>&1; then
  master_uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  if [[ "$master_uri" =~ ^http://(127\.0\.0\.1|localhost):([0-9]+)$ ]]; then
    master_port="${BASH_REMATCH[2]}"
    if [[ "$master_port" != 11311 && "${SIMPLE_RBT_START_MASTER:-0}" != 1 ]]; then
      echo "External ROS master is unreachable: $master_uri" >&2; exit 1
    fi
    if (( master_port < 1024 || master_port > 65535 )); then echo "Invalid master port" >&2; exit 1; fi
    roscore -p "$master_port" >/tmp/simple-rbt-roscore.log 2>&1 & master_pid=$!
    ready=0
    for attempt in {1..50}; do
      if timeout 1s rosparam list >/dev/null 2>&1; then ready=1; break; fi
      sleep .1
    done
    if [[ "$ready" != 1 ]]; then cat /tmp/simple-rbt-roscore.log >&2; exit 1; fi
  else
    echo "Configured ROS_MASTER_URI is unreachable: $master_uri" >&2; exit 1
  fi
fi
/opt/simple-rbt/bin/simple-rbt-ros-session "$@" & node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true; cleanup' INT TERM
wait "$node_pid"
