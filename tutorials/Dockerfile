ARG ROS_IMAGE=ros:noetic-ros-base-focal@sha256:4f01fa743be43da0e14ddf2ac29b5118d9c0eb4057c9761c13303b8a52fa9c4f
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ENTRY=noetic
ARG BUILD_CPUS
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libpcl-dev libopencv-dev libeigen3-dev \
    ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-visualization-msgs && \
    rm -rf /var/lib/apt/lists/*
COPY interfaces /tmp/interfaces
RUN if [[ -n "$BUILD_CPUS" ]]; then taskset -pc "$BUILD_CPUS" $$ >/dev/null; fi; \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cmake -S /tmp/interfaces -B /tmp/interfaces-build -DCMAKE_INSTALL_PREFIX=/opt/messages && \
    cmake --build /tmp/interfaces-build --target install -j4 && \
    if [[ "$ROS_VERSION" == 1 ]]; then \
      cp -a /tmp/interfaces /tmp/compat && \
      sed -i 's/livox_ros_driver2/livox_ros_driver/g' /tmp/compat/{CMakeLists.txt,package.xml} && \
      cmake -S /tmp/compat -B /tmp/compat-build -DCMAKE_INSTALL_PREFIX=/opt/messages && \
      cmake --build /tmp/compat-build --target install -j4; fi
COPY sources /src
RUN if [[ -n "$BUILD_CPUS" ]]; then taskset -pc "$BUILD_CPUS" $$ >/dev/null; fi; \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    export CMAKE_PREFIX_PATH=/opt/messages:$CMAKE_PREFIX_PATH && \
    for source in FAST_LIO FAST_LIVO2; do \
      cmake -S /src/$source/$ENTRY -B /tmp/build-$source -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/frontends -DBUILD_TESTING=OFF -DCATKIN_ENABLE_TESTING=OFF && \
      cmake --build /tmp/build-$source --target install -j4 || exit; done && \
    rm -rf /tmp/build-FAST_LIO /tmp/build-FAST_LIVO2
COPY observer /tmp/observer
COPY nlohmann /tmp/observer/nlohmann
RUN if [[ -n "$BUILD_CPUS" ]]; then taskset -pc "$BUILD_CPUS" $$ >/dev/null; fi; \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    CMAKE_PREFIX_PATH=/opt/messages:$CMAKE_PREFIX_PATH cmake -S /tmp/observer -B /tmp/observer-build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/tutorial && \
    cmake --build /tmp/observer-build --target install -j4
COPY entrypoint.sh /opt/tutorial/entrypoint.sh
COPY udp.xml /opt/tutorial/udp.xml
COPY sources.json /opt/tutorial/sources.json
LABEL org.simple-rbt.tutorial.contract="1"
ENV OMP_WAIT_POLICY=PASSIVE OMP_NUM_THREADS=4 ROS_HOME=/run/ros
ENTRYPOINT ["/bin/bash", "/opt/tutorial/entrypoint.sh"]
