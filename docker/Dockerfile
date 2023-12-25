# Use NVIDIA CUDA 12.3.1 on Ubuntu 20.04 as the base image
FROM docker.io/nvidia/cuda:12.3.1-devel-ubuntu20.04

# Set non-interactive installation environment variable
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages including Git, SSH, and other common tools
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    git \
    openssh-client \
    openssh-server \
    curl \
    wget \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS repository and import the key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic, python3-rosdep, and initialize rosdep
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

# Source ROS setup script in bashrc for interactive sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Set up SSH, configure to allow root login with password
RUN mkdir /var/run/sshd \
    && echo 'root:12345' | chpasswd \
    && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
    && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Install additional packages for ROS and other dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    libglpk-dev \
    ros-noetic-catkin \
    ros-noetic-pybind11-catkin \
    python3-catkin-tools \
    doxygen \
    doxygen-latex \
    liburdfdom-dev \
    liboctomap-dev \
    libassimp-dev \
    libopencv-dev \
    libgmp-dev \
    libmpfr-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone necessary repositories for ocs2, pinocchio, hpp-fcl, elevation mapping, and grid map
RUN mkdir -p /root/ocs2_ws/src && cd /root/ocs2_ws/src \
    && git clone https://github.com/leggedrobotics/ocs2.git \
    && git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git \
    && git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git \
    && git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git \
    && git clone https://github.com/ANYbotics/grid_map.git \
    && git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git \
    && git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01 /root/raisimLib \
    && rm -rf /root/ocs2_ws/src/ocs2/ocs2_doc

# Install
RUN apt-get update && apt-get install -y \
    checkinstall \
    rsync \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-costmap-2d \
    ros-noetic-rqt-multiplot \
    && rm -rf /var/lib/apt/lists/*

# Build and install RaiSim
RUN cd /root/raisimLib \
    && mkdir build && cd build \
    && cmake .. \
    && make -j \
    && checkinstall --pkgname=raisim --pkgversion=1.1.01 --backup=no \
       --deldoc=yes --fstrans=no --default

# Install onnxruntime
RUN cd /tmp \
    && wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz \
    && tar xf onnxruntime-linux-x64-1.7.0.tgz \
    && mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime \
    && rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime \
    && rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib \
    && rsync -a /root/ocs2_ws/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime \
    && echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/root/.local/" >> ~/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/root/.local" >> ~/.bashrc

# rosdep install
RUN apt-get update && cd /root/ocs2_ws \
    && rosdep install --from-paths src --ignore-src -r -y

# Set the CMAKE_PREFIX_PATH to include /root/.local
ENV CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/root/.local"

# Build ocs2 workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/ocs2_ws; catkin build'

# Source ocs2 workspace in bashrc
RUN echo "source /root/ocs2_ws/devel/setup.bash" >> ~/.bashrc

# Expose the SSH port
EXPOSE 22

# Start SSH service when the container launches
CMD ["/usr/sbin/sshd", "-D"]