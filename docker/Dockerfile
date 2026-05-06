FROM osrf/ros:melodic-desktop-full-bionic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    rsync \
    xauth \
    libeigen3-dev \
    libopencv-dev \
    libceres-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libboost-all-dev \
    python-catkin-tools \
    python-pip \
    python-opencv \
    ros-melodic-cv-bridge \
    ros-melodic-tf \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions \
    ros-melodic-image-transport \
    ros-melodic-message-filters \
 && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir --upgrade "pip<21" setuptools wheel future \
 && python -m pip install --no-cache-dir torch==1.4.0+cpu \
      -f https://download.pytorch.org/whl/torch_stable.html

WORKDIR /workspace/VINS-RGBD

CMD ["bash"]
