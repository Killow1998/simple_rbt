# Public Docker Official Image, Linux amd64 manifest; no workstation SDK export.
ARG ROS_IMAGE=ros:noetic-ros-base-focal@sha256:4f01fa743be43da0e14ddf2ac29b5118d9c0eb4057c9761c13303b8a52fa9c4f
FROM ${ROS_IMAGE} AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ros-noetic-message-generation ros-noetic-tf2-msgs && \
    rm -rf /var/lib/apt/lists/*
COPY interfaces/livox_ros_driver2 /tmp/messages
RUN source /opt/ros/noetic/setup.bash && cmake -S /tmp/messages -B /tmp/message-build \
    -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt/interfaces && \
    cmake --build /tmp/message-build --target install --parallel 2
COPY include /tmp/source/include
COPY third_party/nlohmann /tmp/source/third_party/nlohmann
COPY src/robots.cpp /tmp/source/src/robots.cpp
COPY ipc /tmp/source/ipc
COPY wrappers /tmp/source/wrappers
RUN source /opt/ros/noetic/setup.bash && \
    CMAKE_PREFIX_PATH=/opt/simple-rbt/interfaces:$CMAKE_PREFIX_PATH \
    cmake -S /tmp/source/wrappers -B /tmp/build -DSIMPLE_RBT_ROS=ROS1 \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt && \
    cmake --build /tmp/build --target install --parallel 2
FROM ${ROS_IMAGE}
COPY --from=builder /opt/simple-rbt /opt/simple-rbt
# TFMessage is a message-only package: expose its definitions and Python types.
COPY --from=builder /opt/ros/noetic/share/tf2_msgs /opt/ros/noetic/share/tf2_msgs
COPY --from=builder /opt/ros/noetic/lib/python3/dist-packages/tf2_msgs /opt/ros/noetic/lib/python3/dist-packages/tf2_msgs
COPY --from=builder /usr/share/doc/ros-noetic-tf2-msgs /usr/share/doc/ros-noetic-tf2-msgs
COPY deployment/ros1/entrypoint.sh /usr/local/bin/simple-rbt-ros-entrypoint
ENV ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
ENTRYPOINT ["/usr/local/bin/simple-rbt-ros-entrypoint"]
CMD ["--session-dir", "/session"]
