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

# Add the ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Import the ROS repository key
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic along with python3-rosdep
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

# Initialize rosdep
RUN rosdep init \
    && rosdep update

# Set up environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Set up SSH. Configure SSH to allow root login with password
RUN mkdir /var/run/sshd
RUN echo 'root:12345' | chpasswd # Replace YOUR_PASSWORD with a secure password

# Allow root login via SSH and enable password authentication
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# Install additional packages: Eigen, Boost, GLPK, catkin, pybind11_catkin, catkin-pkg, Doxygen
RUN apt-get update && apt-get install -y \
    libeigen3-dev \             
    libboost-all-dev \           
    libglpk-dev \                
    ros-noetic-catkin \          
    ros-noetic-pybind11-catkin \ 
    python3-catkin-tools \       
    doxygen \                    
    doxygen-latex \              
    && rm -rf /var/lib/apt/lists/*

# Expose the SSH port
EXPOSE 22

# Start SSH service when the container launches
CMD ["/usr/sbin/sshd", "-D"]