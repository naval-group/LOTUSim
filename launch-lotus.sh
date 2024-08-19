#!/bin/bash
# Color definition
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

export ROS_DISTRO=humble
export GZ_VERSION=harmonic

# Définition des chemins
ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
LOTUSIM_PATH="$(pwd)"
ASSETS_MODELS_PATH="assets/models"
OLD_GZ_GUI_LIB_PATH="/lib/x86_64-linux-gnu"
GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/lib"
GZ_SIM_RESOURCE_PATH="$(pwd)/${ASSETS_MODELS_PATH}"

cd ../..

source "$ROS_SETUP_PATH"
source "$(pwd)/install/setup.bash"

export GZ_SIM_SYSTEM_PLUGIN_PATH
echo -e "GZ_SIM_SYSTEM_PLUGIN_PATH : ${GREEN}$GZ_SIM_SYSTEM_PLUGIN_PATH${NC}"

export GZ_SIM_RESOURCE_PATH
echo -e "GZ_SIM_RESOURCE_PATH : ${GREEN}$GZ_SIM_RESOURCE_PATH${NC}"

echo -e "GZ_GUI_PLUGIN_PATH : ${GREEN}$GZ_GUI_PLUGIN_PATH${NC}"

# Dirty fix of common issue
unset GTK_PATH

cd "$LOTUSIM_PATH" || return 
echo -e "${GREEN}Launching LOTUSim...${NC}"

ros2 launch bringup mas.launch.py