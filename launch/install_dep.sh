#!/bin/bash
##
## Copyright (c) 2025 Naval Group
##
## This program and the accompanying materials are made available under the
## terms of the Eclipse Public License 2.0 which is available at
## https://www.eclipse.org/legal/epl-2.0.
##
## SPDX-License-Identifier: EPL-2.0
##
set -e

# Color definitions
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "\n🔧 Starting install of ROS and Gazebo..." 

apt-get -qq update  >/dev/nul 2>&1
apt-get install -y -qq software-properties-common  >/dev/nul 2>&1
add-apt-repository -y universe  >/dev/nul 2>&1
apt-get -y -qq install locales tzdata lsb-release curl gnupg2 ca-certificates >/dev/nul 2>&1 
update-ca-certificates >/dev/nul 2>&1 || echo "Warning: CA certificate update had issues"

# Detect Ubuntu codename
VERSION_CODENAME=$(source /etc/os-release && echo "$VERSION_CODENAME")
if [[ -z "$VERSION_CODENAME" ]]; then
  echo -e "\n❌ Error: Could not detect Ubuntu VERSION_CODENAME" >&2
  exit 1
fi

# Install ROS2
ROS2_LIST_FILE="/etc/apt/sources.list.d/ros2.list"
if [[ -f "$ROS2_LIST_FILE" ]]; then
  echo -e "\n✅ ROS2 repo already exists at: $ROS2_LIST_FILE"
else
  echo -e "\n🌐 ROS2 repo not found — downloading ros2-apt-source package..."
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
    ROS_APT_SOURCE_VERSION=1.1.0
  fi

  ROS_APT_SOURCE_VERSION=$(echo "$ROS_APT_SOURCE_VERSION" | tr -d '\n' | xargs)
  VERSION_CODENAME=$(echo "$VERSION_CODENAME" | tr -d '\n' | xargs)
  ROS2_PKG_NAME="ros2-apt-source"
  ROS2_PKG_VERSION="${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}"
  DOWNLOAD_ROS2_DEB=0

  if dpkg -s "$ROS2_PKG_NAME" &> /dev/null; then
    INSTALLED_VERSION=$(dpkg -s "$ROS2_PKG_NAME" | grep '^Version:' | awk '{print $2}')
    if [[ "$INSTALLED_VERSION" != "$ROS2_PKG_VERSION" ]]; then
      DOWNLOAD_ROS2_DEB=1
    fi
  else
    DOWNLOAD_ROS2_DEB=1
  fi

  if [ "$DOWNLOAD_ROS2_DEB" -eq 1 ]; then
    ROS2_DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
    ROS2_DEB_PATH="/tmp/ros2-apt-source.deb"
    if ! touch "$ROS2_DEB_PATH" &>/dev/null; then
      ROS2_DEB_PATH="$HOME/ros2-apt-source.deb"
    fi
    curl -fsSL -o "$ROS2_DEB_PATH" "$ROS2_DEB_URL" 2>/dev/null
    if [ ! -f "$ROS2_DEB_PATH" ]; then
      echo -e "\n❌ Error: Failed to download ros2-apt-source package" >&2
      exit 1
    fi
    apt-get install -y -qq "$ROS2_DEB_PATH" >/dev/null 2>&1
  fi
fi

# Install GZ
GAZEBO_LIST_FILE="/etc/apt/sources.list.d/gazebo-stable.list"
GAZEBO_KEYRING="/usr/share/keyrings/gazebo-archive-keyring.gpg"
GAZEBO_SOURCE_LINE="deb [arch=$(dpkg --print-architecture) signed-by=${GAZEBO_KEYRING}] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main"
if [[ -f "$GAZEBO_LIST_FILE" ]]; then
    echo -e "\n✅ Gazebo repo already exists at: $GAZEBO_LIST_FILE"
else
  echo -e "\n🌐 Gazebo repo not found — adding new source..."
  # Add key if missing
  if [ ! -f "$GAZEBO_KEYRING" ]; then
    curl -fsSL https://packages.osrfoundation.org/gazebo.key 2>/dev/null | gpg --dearmor -o "$GAZEBO_KEYRING" 2>/dev/null
  fi
  echo "$GAZEBO_SOURCE_LINE" | tee "$GAZEBO_LIST_FILE" >/dev/null
fi

apt-get update -qq >/dev/null 
apt-get -y install \
  gz-harmonic \
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
  libcli11-dev \
  >/dev/null || { echo -e "\n❌ Error installing packages." >&2; exit 1; }
  
# Set Clang as default C++
update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100 >/dev/null

echo -e "
********************************************************************************************
******************    ${GREEN}✅ ROS and Gazebo installation complete!${NC}    *********************
********************************************************************************************
"