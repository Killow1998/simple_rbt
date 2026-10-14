ARG NOETIC_IMAGE=ros:noetic-ros-base
FROM ${NOETIC_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS1_BRIDGE_COMMIT=689a932499befbd1ec3cb273a1054430e55a43c3
RUN --mount=type=secret,id=session_ca,required=true \
    export SSL_CERT_FILE=/run/secrets/session_ca CURL_CA_BUNDLE=/run/secrets/session_ca \
      GIT_SSL_CAINFO=/run/secrets/session_ca http_proxy="$HTTP_PROXY" https_proxy="$HTTPS_PROXY" && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo 'deb http://packages.ros.org/ros2/ubuntu focal main' > /etc/apt/sources.list.d/simple-rbt-ros2.list && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-foxy-ros1-bridge python3-colcon-common-extensions build-essential cmake git && \
    git init /tmp/ros1_bridge && git -C /tmp/ros1_bridge fetch --depth 1 \
      https://github.com/ros2/ros1_bridge.git "$ROS1_BRIDGE_COMMIT" && \
    git -C /tmp/ros1_bridge checkout --detach FETCH_HEAD && \
    rm -rf /var/lib/apt/lists/*
RUN --mount=type=secret,id=session_ca,required=true \
    export http_proxy="$HTTP_PROXY" https_proxy="$HTTPS_PROXY" && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-foxy-rosidl-default-generators && rm -rf /var/lib/apt/lists/*
COPY ros1/ /tmp/ros1-src/livox_ros_driver2/
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make --source /tmp/ros1-src --build /tmp/ros1-build \
      -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt/ros1 install -j2
COPY ros2/ /tmp/ros2-src/livox_ros_driver2/
RUN source /opt/ros/foxy/setup.bash && \
    colcon --log-base /tmp/ros2-log build --base-paths /tmp/ros2-src \
      --build-base /tmp/ros2-build --install-base /opt/simple-rbt/ros2 \
      --merge-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
COPY tf_static.patch /tmp/tf_static.patch
RUN git -C /tmp/ros1_bridge apply /tmp/tf_static.patch && \
    source /opt/ros/noetic/setup.bash && source /opt/simple-rbt/ros1/setup.bash && \
    source /opt/ros/foxy/setup.bash && source /opt/simple-rbt/ros2/local_setup.bash && \
    MAKEFLAGS=-j2 colcon --log-base /tmp/bridge-log build --base-paths /tmp/ros1_bridge \
      --build-base /tmp/bridge-build --install-base /opt/simple-rbt/bridge \
      --merge-install --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/ros1-src /tmp/ros1-build /tmp/ros2-src /tmp/ros2-build /tmp/ros2-log \
      /tmp/ros1_bridge /tmp/bridge-build /tmp/bridge-log
COPY entrypoint.sh /opt/simple-rbt/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/opt/simple-rbt/entrypoint.sh"]
CMD ["sleep", "infinity"]
