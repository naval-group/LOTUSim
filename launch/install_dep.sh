#!/bin/bash

echo "Installing ROS and GZ"
apt update

apt-get -y install locales tzdata lsb-release

cp ${LOTUSIM_PATH}/ci/* /usr/share/keyrings/

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null
apt-get update
apt-get -y install gz-harmonic \
    libstdc++-12-dev \
    ros-humble-desktop \
    python3-rosdep \
    ros-dev-tools \
    clang \
    libwebsocketpp-dev \
    nlohmann-json3-dev \
    ros-humble-backward-ros
update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100
