#!/bin/bash
set -e

echo -e "\n🔧 Starting install of ROS and Gazebo..."

echo -e "\n🔄 Updating package index..."
apt-get update -qq >/dev/null

echo -e "\n📦 Installing software-properties-common..."
apt-get install -y -qq software-properties-common >/dev/null

echo -e "\n🗂 Enabling Ubuntu Universe repository..."
add-apt-repository -y universe >/dev/null

echo -e "\n📦 Installing base tools..."
apt-get -y -qq install locales tzdata lsb-release curl gnupg2 ca-certificates >/dev/null
update-ca-certificates >/dev/null

# Fetch latest ROS apt source release
echo -e "\n🌐 Fetching latest ROS apt source release..."
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
  echo -e "\n⚠️  Error: Could not fetch ROS_APT_SOURCE_VERSION" >&2
  ROS_APT_SOURCE_VERSION=1.1.0
else
  echo -e "\n✅ ROS_APT_SOURCE_VERSION = $ROS_APT_SOURCE_VERSION"
fi

VERSION_CODENAME=$(source /etc/os-release && echo -e "\n$VERSION_CODENAME")
if [[ -z "$VERSION_CODENAME" ]]; then
  echo -e "\n❌ Error: Could not detect Ubuntu VERSION_CODENAME" >&2
  exit 1
else
  echo -e "\n✅ Ubuntu codename: $VERSION_CODENAME"
fi

ROS2_PKG_NAME="ros2-apt-source"
ROS2_PKG_VERSION="${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}"
DOWNLOAD_ROS2_DEB=0

# Check if ROS2 apt source package is already installed
echo -e "\n🔍 Checking if $ROS2_PKG_NAME is already installed..."
if dpkg -s "$ROS2_PKG_NAME" &> /dev/null; then
  INSTALLED_VERSION=$(dpkg -s "$ROS2_PKG_NAME" | grep '^Version:' | awk '{print $2}')
  if [[ "$INSTALLED_VERSION" == "$ROS2_PKG_VERSION" ]]; then
    echo -e "\n✅ $ROS2_PKG_NAME version $INSTALLED_VERSION already installed, skipping download."
  else
    echo -e "\n🔁 $ROS2_PKG_NAME installed but version differs ($INSTALLED_VERSION), updating..."
    DOWNLOAD_ROS2_DEB=1
  fi
else
  echo -e "\n⬇️  $ROS2_PKG_NAME not installed, downloading..."
  DOWNLOAD_ROS2_DEB=1
fi

if [ "$DOWNLOAD_ROS2_DEB" -eq 1 ]; then
  ROS2_DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  echo -e "\n🌐 Downloading ROS2 apt source from: $ROS2_DEB_URL"
  
  # Try downloading to $HOME if /tmp is not writable
  ROS2_DEB_PATH="/tmp/ros2-apt-source.deb"
  if ! touch "$ROS2_DEB_PATH" &>/dev/null; then
    ROS2_DEB_PATH="$HOME/ros2-apt-source.deb"
    echo -e "\n⚠️  /tmp not writable, using $HOME instead."
  fi

  curl -fsSL -o "$ROS2_DEB_PATH" "$ROS2_DEB_URL"
  if [ ! -f "$ROS2_DEB_PATH" ]; then
    echo -e "\n❌ Error: Failed to download ros2-apt-source package" >&2
    exit 1
  fi

  echo -e "\n📦 Installing $ROS2_PKG_NAME..."
  apt-get install -y -qq "$ROS2_DEB_PATH" >/dev/null
else
  echo -e "\n✅ No need to download or install $ROS2_PKG_NAME."
fi

# Add Gazebo GPG keyring
echo -e "\n🔐 Checking for Gazebo GPG keyring..."
if [ ! -f /usr/share/keyrings/gazebo-archive-keyring.gpg ]; then
  echo -e "\n🔑 Installing Gazebo GPG keyring..."
  curl -fsSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
else
  echo -e "\n✅ Gazebo GPG keyring already installed."
fi

# Add Gazebo APT source
GAZEBO_LIST_FILE="/etc/apt/sources.list.d/gazebo-stable.list"
GAZEBO_SOURCE_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"

echo -e "\n📄 Checking for Gazebo APT source list..."
if ! grep -Fxq "$GAZEBO_SOURCE_LINE" "$GAZEBO_LIST_FILE" 2>/dev/null; then
  echo -e "\n➕ Adding Gazebo APT source list..."
  echo -e "\n$GAZEBO_SOURCE_LINE" | tee "$GAZEBO_LIST_FILE" >/dev/null
else
  echo -e "\n✅ Gazebo APT source list already present."
fi

# Install packages
echo -e "\n🔄 Final update of package index..."
apt-get update -qq >/dev/null

echo -e "\n📦 Installing ROS, Gazebo, and dependencies..."
apt-get -y install -qq \
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
  || { echo -e "\n❌ Error installing packages." >&2; exit 1; }

# Set Clang as default C++
echo -e "\n🔧 Setting Clang++ as default C++ compiler..."
update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100

echo -e "\n✅ Installation complete."
