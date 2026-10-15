# Public Docker Official Image, Linux amd64 manifest.
ARG ROS_IMAGE=ros:humble-ros-base-jammy@sha256:6b4abb5340e430c632bcf4710fb34996df4f80e502993e5755f8fc59a39c7b2e
FROM ${ROS_IMAGE} AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY interfaces/livox_ros_driver2 /tmp/messages
RUN source /opt/ros/humble/setup.bash && cmake -S /tmp/messages -B /tmp/message-build \
    -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt/interfaces && \
    cmake --build /tmp/message-build --target install --parallel 2
COPY include /tmp/source/include
COPY third_party/nlohmann /tmp/source/third_party/nlohmann
COPY src/robots.cpp /tmp/source/src/robots.cpp
COPY ipc /tmp/source/ipc
COPY wrappers /tmp/source/wrappers
RUN source /opt/ros/humble/setup.bash && \
    CMAKE_PREFIX_PATH=/opt/simple-rbt/interfaces:$CMAKE_PREFIX_PATH \
    cmake -S /tmp/source/wrappers -B /tmp/build -DSIMPLE_RBT_ROS=ROS2 \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt && \
    cmake --build /tmp/build --target install --parallel 2
FROM ${ROS_IMAGE}
COPY --from=builder /opt/simple-rbt /opt/simple-rbt
COPY deployment/ros2/entrypoint.sh /usr/local/bin/simple-rbt-ros-entrypoint
COPY deployment/ros2/udp.xml /opt/simple-rbt/udp.xml
ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83 \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/simple-rbt/udp.xml
ENTRYPOINT ["/usr/local/bin/simple-rbt-ros-entrypoint"]
CMD ["--session-dir", "/session"]
