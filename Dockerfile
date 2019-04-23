FROM ubuntu:16.04
LABEL maintainer "e-Lab Purdue <tmolnar@purdue.edu>"

RUN apt-get update
RUN apt-get install --no-install-recommends -y tmux vim wget unzip

# Install CUDA 10.0.130
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \
    rm -rf /var/lib/apt/lists/* && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION \
        cuda-compat-10-0 && \
    ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=410,driver<411"

# Install ROS Kinetic
# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN cd \
    && rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    ros-kinetic-tf \
    ros-kinetic-image-transport \
    ros-kinetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Install ORB-SLAM2
WORKDIR /opt
RUN cd /opt \
    && git clone https://github.com/tmolnar18/ORB-SLAM.git ORB_SLAM \
    && cd ORB_SLAM && chmod +x build.sh && chmod +x build_ros.sh

RUN echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/opt/ORB_SLAM/Examples/ROS" >> ~/.bashrc
#RUN echo "source ~/.bashrc" >> ~/.bashrc
RUN apt-get update

# Install Vizdoom Dependencies
# ViZdoom dependencies
RUN apt-get update
RUN apt-get install -y \
    build-essential \
    bzip2 \
    cmake \
    curl \
    git \
    libboost-all-dev \
    libbz2-dev \
    libfluidsynth-dev \
    libfreetype6-dev \
    libgme-dev \
    libgtk2.0-dev \
    libjpeg-dev \
    libopenal-dev \
    libpng12-dev \
    libsdl2-dev \
    libwildmidi-dev \
    libzmq3-dev \
    nano \
    nasm \
    pkg-config \
    rsync \
    software-properties-common \
    sudo \
    tar \
    timidity \
    zlib1g-dev \
    libeigen3-dev \
    python-pil \
    python3-pil

# Install Pangolin
RUN apt-get install -y libglew-dev
RUN cd \
    && git clone https://github.com/stevenlovegrove/Pangolin.git \
    && cd Pangolin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && cmake --build .

RUN apt-get update
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev \
    libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
    libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev unzip

# Get and build OpenCV 2.4.11
RUN cd \
    && wget https://github.com/opencv/opencv/archive/2.4.11.zip \
    && unzip 2.4.11.zip \
    && cd opencv-2.4.11 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8 \
    && make install \
    && cd \
    && rm 2.4.11.zip

# Python2
RUN apt-get install -y python-dev python python-pip
RUN pip install pip --upgrade
RUN pip install numpy
RUN apt-get update

# Python3
RUN apt-get install -y python3-dev python3 python3-pip
RUN pip3 install pip3 --upgrade
RUN pip3 install numpy
RUN apt-get update

# Install Vizdoom
RUN cd \
    && git clone https://github.com/mwydmuch/ViZDoom vizdoom
RUN pip install vizdoom
