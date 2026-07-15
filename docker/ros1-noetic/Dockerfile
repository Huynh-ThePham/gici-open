# ROS 1 Noetic environment for GICI UrbanNav RRR (author-native rosbag replay).
# Ubuntu 20.04 base — matches GICI upstream target.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/workspace/gici/ros_wrapper

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgflags-dev \
    libsuitesparse-dev \
    liblapack-dev \
    libblas-dev \
    python3-yaml \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir rosbags

# GICI requires find_package(glog) with CMake config. Ubuntu 20.04 apt glog (0.4.0)
# lacks glogConfig.cmake — build 0.6.0 from source per upstream README.
ARG GLOG_VERSION=v0.6.0
RUN git clone --depth 1 --branch "${GLOG_VERSION}" https://github.com/google/glog.git /tmp/glog \
    && cmake -S /tmp/glog -B /tmp/glog/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/glog/build -j"$(nproc)" \
    && cmake --install /tmp/glog/build \
    && ldconfig \
    && rm -rf /tmp/glog

# Ubuntu 20.04 ships Ceres 1.14; GICI needs 2.1+ (StaticParameterDims, EvaluateResidualBlock).
ARG CERES_VERSION=2.1.0
RUN git clone --depth 1 --branch "${CERES_VERSION}" https://github.com/ceres-solver/ceres-solver.git /tmp/ceres \
    && cmake -S /tmp/ceres -B /tmp/ceres/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/usr/local \
    && cmake --build /tmp/ceres/build -j"$(nproc)" \
    && cmake --install /tmp/ceres/build \
    && ldconfig \
    && rm -rf /tmp/ceres

ENV CMAKE_PREFIX_PATH=/opt/ros/noetic:/usr/local

# ROS wrapper deps (pcl, cv_bridge, …) are in desktop-full.
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

WORKDIR /workspace/gici

COPY scripts/ros1/docker_entrypoint.sh /usr/local/bin/gici-ros1-entrypoint.sh
RUN chmod +x /usr/local/bin/gici-ros1-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/gici-ros1-entrypoint.sh"]
CMD ["bash"]
