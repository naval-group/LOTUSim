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
set -euo pipefail

# Color definitions
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "\n🔧 Starting install of ROS and Gazebo..."

# Derive VERSION_CODENAME from /etc/os-release if not already set
if [[ -z "${VERSION_CODENAME:-}" ]]; then
  VERSION_CODENAME=$(. /etc/os-release 2>/dev/null && echo "${VERSION_CODENAME:-}")
fi
# Last resort: use UBUNTU_VERSION from CI matrix
if [[ -z "${VERSION_CODENAME:-}" ]]; then
  VERSION_CODENAME=${UBUNTU_VERSION:-}
fi
if [[ -z "${VERSION_CODENAME:-}" ]]; then
  echo -e "${RED}❌ Cannot determine VERSION_CODENAME. Set it or run on a supported Ubuntu.${NC}" >&2
  exit 1
fi

echo -e "${YELLOW}[ENV]${NC} ROS_DISTRO=${ROS_DISTRO:-unset}, GAZEBO_VERSION=${GAZEBO_VERSION:-unset}, VERSION_CODENAME=${VERSION_CODENAME}"

echo -e "\n🔧 [1/5] Bootstrapping apt prerequisites..."
apt-get -qq update
apt-get install -y -qq software-properties-common build-essential
add-apt-repository -y universe
apt-get -y -qq install locales tzdata lsb-release curl gnupg2 ca-certificates
update-ca-certificates || echo -e "${YELLOW}Warning:${NC} CA certificate update had issues"
echo -e "${GREEN}✅ Prerequisites installed${NC}"

# Install ROS2 repository
echo -e "\n🔧 [2/5] Setting up ROS2 repository..."
if ls /etc/apt/sources.list.d/*ros2* 1>/dev/null 2>&1; then
  echo -e "${GREEN}✅ ROS2 repo already exists.${NC}"
else
  echo -e "\n🌐 ROS2 repo not found — downloading ros2-apt-source package..."
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
    ROS_APT_SOURCE_VERSION=1.1.0
  fi

  ROS_APT_SOURCE_VERSION=$(echo "$ROS_APT_SOURCE_VERSION" | tr -d '\n' | xargs)
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
    echo -e "  Downloading: ${ROS2_DEB_URL}"
    ROS2_DEB_PATH="/tmp/ros2-apt-source.deb"
    if ! touch "$ROS2_DEB_PATH" &>/dev/null; then
      ROS2_DEB_PATH="$HOME/ros2-apt-source.deb"
    fi
    curl -fsSL -o "$ROS2_DEB_PATH" "$ROS2_DEB_URL" || { echo -e "${RED}❌ curl failed (exit $?)${NC}" >&2; exit 1; }
    if [ ! -f "$ROS2_DEB_PATH" ]; then
      echo -e "${RED}❌ Error: Failed to download ros2-apt-source package${NC}" >&2
      exit 1
    fi
    echo -e "  Installing ${ROS2_DEB_PATH}..."
    apt-get install -y -qq "$ROS2_DEB_PATH" >/dev/null 2>&1
  fi
  echo -e "${GREEN}✅ ROS2 repo added${NC}"
fi

# Install GZ
echo -e "\n🔧 [3/5] Setting up Gazebo repository..."
GAZEBO_LIST_FILE="/etc/apt/sources.list.d/gazebo-stable.list"
GAZEBO_KEYRING="/usr/share/keyrings/gazebo-archive-keyring.gpg"
GAZEBO_SOURCE_LINE="deb [arch=$(dpkg --print-architecture) signed-by=${GAZEBO_KEYRING}] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main"
if [[ -f "$GAZEBO_LIST_FILE" ]]; then
    echo -e "${GREEN}✅ Gazebo repo already exists.${NC}"
else
  echo -e "\n🌐 Gazebo repo not found — adding new source..."
  # Add key if missing
  if [ ! -f "$GAZEBO_KEYRING" ]; then
    echo -e "  Fetching Gazebo GPG key..."
    curl -fsSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o "$GAZEBO_KEYRING" \
      || { echo -e "${RED}❌ Failed to fetch Gazebo GPG key${NC}" >&2; exit 1; }
  fi
  echo "$GAZEBO_SOURCE_LINE" | tee "$GAZEBO_LIST_FILE" >/dev/null
  echo -e "${GREEN}✅ Gazebo repo added${NC}"
fi

echo -e "\n🔧 [4/5] Updating apt..."
apt-get update -qq >/dev/null
echo -e "${GREEN}✅ Apt updated${NC}"

echo -e "\n🔧 [5/5] Installing dependencies..."
echo -e "  gz-${GAZEBO_VERSION}, ros-${ROS_DISTRO}-ros-core, clang, and other libs"
apt-get -y install \
  gz-$GAZEBO_VERSION \
  libstdc++-12-dev \
  ros-$ROS_DISTRO-ros-core \
  ros-$ROS_DISTRO-backward-ros \
  ros-$ROS_DISTRO-geographic-msgs \
  ros-$ROS_DISTRO-radar-msgs \
  python3-colcon-common-extensions \
  clang \
  libyaml-cpp-dev \
  libwebsocketpp-dev \
  nlohmann-json3-dev \
  libreadline-dev \
  libcli11-dev \
  || { echo -e "${RED}❌ apt-get install failed (exit $?). Check above for the offending package.${NC}" >&2; exit 1; }

# Set Clang as default C++
echo -e "  Setting clang++ as default c++ compiler..."
update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100 >/dev/null

echo -e "
********************************************************************************************
******************    ${GREEN}✅ ROS and Gazebo installation complete!${NC}    *********************
********************************************************************************************
"