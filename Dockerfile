# Use a specific version tag for better reproducibility
FROM ros:humble-ros-core-jammy

# set the shell to bash
SHELL ["/bin/bash", "-c"]

# set environment variables
ENV ROS_DISTRO humble
ENV HOME=/root
ENV TEMOTO_WS=/root/temoto_ws

ENV NUM_CORES=8

# install required apt packages
RUN apt update && apt upgrade -y \
    && apt install -y \
    apt-utils \
    git \
    curl \
    nano \
    vim \
    tmux \
    wget \
    jq \
    sudo \
    x11-apps \
    python3-pip \
    python-is-python3 \
    python3-colcon-common-extensions \
    libqt5websockets5-dev \
    qtbase5-dev \  
    postgresql-client \ 
    libyaml-cpp-dev \
    libogre-1.12-dev \
    libclass-loader-dev \
    libboost-all-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*
   

# install required python packages
RUN pip3 install \
    inflection

# setup ssh to clone private repos
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# switch to home dir
WORKDIR ${HOME}

# setup
RUN mkdir -p ${TEMOTO_WS}/src

# Change the working directory
WORKDIR ${TEMOTO_WS}/src

# install ros deps
RUN apt update && apt upgrade -y \
    && apt install -y \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cyclonedds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # << add more ros message packages here >>
    && rm -rf /var/lib/apt/lists/*

# clone robofleet client 
RUN git clone --recursive https://github.com/temoto-framework/temoto_action_engine_ros2

# Change directory to temoto_action_engine_ros2
WORKDIR ${TEMOTO_WS}/src/temoto_action_engine_ros2/temoto_action_engine_ros2/temoto_action_engine

# Create build directory, run cmake and make install
RUN mkdir -p build \
    && cd build \
    && cmake .. \
    && make install

# build base catkin workspace
WORKDIR ${TEMOTO_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build 

# source temoto workspace
RUN source install/setup.bash

# Export library path 
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Copy the ROS2 cyclonedds configuration file
COPY ./cyclonedds.xml /root/cyclonedds.xml

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/temoto_ws/install/setup.bash" >> ~/.bashrc

# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# run this script on startup
ENTRYPOINT /entrypoint.sh