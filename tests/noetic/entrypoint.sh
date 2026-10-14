#!/usr/bin/env bash
set -eo pipefail
export ROS_HOME=/tmp/simple-rbt-ros1
source /opt/ros/noetic/setup.bash
source /opt/simple-rbt/ros1/setup.bash
roscore -p "${SIMPLE_RBT_MASTER_PORT:-11383}" > /tmp/roscore.log 2>&1 &
master_pid=$!
python3 - <<'WAIT'
import os, time, xmlrpc.client
master = os.environ["ROS_MASTER_URI"]
for attempt in range(100):
    try:
        if xmlrpc.client.ServerProxy(master).getPid("/simple_rbt_noetic_startup")[0] == 1:
            break
    except OSError:
        pass
    time.sleep(.1)
else:
    raise RuntimeError("Noetic master did not start")
WAIT
/bin/bash --noprofile --norc -c '
  source /opt/ros/noetic/setup.bash
  source /opt/simple-rbt/ros1/setup.bash
  source /opt/ros/foxy/setup.bash
  source /opt/simple-rbt/ros2/local_setup.bash
  source /opt/simple-rbt/bridge/local_setup.bash
  exec /opt/simple-rbt/bridge/lib/ros1_bridge/dynamic_bridge --bridge-all-2to1-topics
' > /tmp/bridge.log 2>&1 &
bridge_pid=$!
trap 'kill "$bridge_pid" "$master_pid" 2>/dev/null || true' EXIT TERM INT
"$@" &
client_pid=$!
wait "$client_pid"
