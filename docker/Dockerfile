FROM osrf/ros:melodic-desktop-full-bionic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    rsync \
    xauth \
    autoconf \
    libtool \
    rsync \
    protobuf-compiler \
    libssl-dev \
    zlib1g-dev \
    libeigen3-dev \
    libopencv-dev \
    libceres-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libboost-all-dev \
    python-catkin-tools \
    python-numpy \
    python-wstool \
    python-pip \
    python-opencv \
    ros-melodic-cv-bridge \
    ros-melodic-cmake-modules \
    ros-melodic-tf \
    ros-melodic-pcl-ros \
    ros-melodic-pcl-conversions \
    ros-melodic-image-transport \
    ros-melodic-message-filters \
 && rm -rf /var/lib/apt/lists/*

ENV ONNXRUNTIME_VERSION=1.8.1
RUN curl -L -o /tmp/onnxruntime.tgz \
      "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
 && tar -xzf /tmp/onnxruntime.tgz -C /opt \
 && mv "/opt/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}" /opt/onnxruntime \
 && echo "/opt/onnxruntime/lib" > /etc/ld.so.conf.d/onnxruntime.conf \
 && ldconfig \
 && rm -f /tmp/onnxruntime.tgz

RUN python -m pip install --no-cache-dir --upgrade "pip<21" setuptools wheel future \
 && python -m pip install --no-cache-dir torch==1.4.0+cpu \
      -f https://download.pytorch.org/whl/torch_stable.html \
 && python -m pip install --no-cache-dir onnx==1.6.0 protobuf==3.17.3

WORKDIR /workspace/VINS-RGBD

CMD ["bash"]
