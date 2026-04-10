FROM osrf/ros:melodic-desktop-full-bionic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    xauth \
    libeigen3-dev \
    libopencv-dev \
    libceres-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libboost-all-dev \
    python-catkin-tools \
    ros-melodic-cv-bridge \
    ros-melodic-tf \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions \
    ros-melodic-image-transport \
    ros-melodic-message-filters \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/VINS-RGBD

CMD ["bash"]
