ARG NOETIC_IMAGE=ros:noetic-ros-base
FROM ${NOETIC_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY messages/ /tmp/simple-rbt-messages/
RUN source /opt/ros/noetic/setup.bash && \
    cmake -S /tmp/simple-rbt-messages -B /tmp/simple-rbt-build \
      -DCMAKE_INSTALL_PREFIX=/opt/simple-rbt/ros1 && \
    cmake --build /tmp/simple-rbt-build --target install --parallel 2 && \
    rm -rf /tmp/simple-rbt-build /tmp/simple-rbt-messages
CMD ["sleep", "infinity"]
