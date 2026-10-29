#!/usr/bin/env bash
set -eo pipefail
algorithm=${1:?Select fast-lio or fast-livo2}
case "$algorithm" in
  fast-lio) package=fast_lio; node=fastlio_mapping; odom=/Odometry;;
  fast-livo2) package=fast_livo; node=fastlivo_mapping; odom=/n3m_input/odometry;;
  *) echo 'Unknown tutorial algorithm' >&2; exit 2;;
esac
source /opt/ros/"$ROS_DISTRO"/setup.bash
export LD_LIBRARY_PATH=/opt/frontends/lib:/opt/messages/lib:${LD_LIBRARY_PATH:-}
export FAST_LIO_LOG_DIR=/run/fast-lio-log FAST_LIVO2_LOG_DIR=/run/fast-livo2-log
mkdir -p "$ROS_HOME"
if [[ "$ROS_VERSION" == 1 ]]; then
  source /opt/messages/setup.bash
  export ROS_PACKAGE_PATH=/opt/frontends/share:$ROS_PACKAGE_PATH
  rosparam set /use_sim_time true
  rosparam load /run/frontend.json /laserMapping
  /opt/frontends/lib/$package/$node __name:=laserMapping &
else
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-83} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tutorial/udp.xml
  source /opt/messages/share/livox_ros_driver2/local_setup.bash
  export AMENT_PREFIX_PATH=/opt/frontends:/opt/messages:$AMENT_PREFIX_PATH
  /opt/frontends/lib/$package/$node --ros-args -r __node:=laserMapping \
    --params-file /run/frontend.json &
fi
frontend_pid=$!
/opt/tutorial/bin/simple-rbt-tutorial-observer "$odom" "$algorithm" &
observer_pid=$!
cleanup() {
  kill -INT "$frontend_pid" "$observer_pid" 2>/dev/null || true
  wait "$frontend_pid" "$observer_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM
wait -n "$frontend_pid" "$observer_pid"
