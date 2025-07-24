#!/bin/bash

echo "Installing ROS and GZ"

apt update
# Enabling Ubuntu Universe repository.
apt install -y software-properties-common
add-apt-repository -y universe

# Install base tools
apt-get -y install locales tzdata lsb-release curl gnupg2 ca-certificates
update-ca-certificates

# Fetch latest ROS apt source release
ROS_APT_SOURCE_VERSION=$(curl -v -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
  echo "Error: Could not fetch ROS_APT_SOURCE_VERSION" >&2
  ROS_APT_SOURCE_VERSION=1.1.0
else
  echo "ROS_APT_SOURCE_VERSION = $ROS_APT_SOURCE_VERSION"
fi
VERSION_CODENAME=$(source /etc/os-release && echo "$VERSION_CODENAME")
if [[ -z "$VERSION_CODENAME" ]]; then
  echo "Error: Could not detect Ubuntu VERSION_CODENAME" >&2
  exit 1
else
  echo "VERSION_CODENAME = $VERSION_CODENAME"
fi

# Download and install the ROS apt source package
ROS2_DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
echo "Downloading ROS2 apt source package from: $ROS2_DEB_URL"
curl -fsSL -o /tmp/ros2-apt-source.deb "$ROS2_DEB_URL"
if [ ! -f /tmp/ros2-apt-source.deb ]; then
  echo "Error: Failed to download ros2-apt-source package" >&2
  exit 1
else
  echo "Downloaded ros2-apt-source package"
fi
apt install -y /tmp/ros2-apt-source.deb

# Add Gazebo key and source
echo "Installing Gazebo APT source and key"
curl -fsSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
  tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

# Install ROS, Gazebo, and dependencies
apt-get update
apt-get -y install gz-harmonic \
  libstdc++-12-dev \
  ros-humble-ros-core \
  ros-humble-backward-ros \
  ros-humble-geographic-msgs \
  python3-colcon-common-extensions \
  clang \
  libyaml-cpp-dev \
  libwebsocketpp-dev \
  nlohmann-json3-dev \
  libreadline-dev \
  libcli11-dev

update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100
