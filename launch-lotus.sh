#!/bin/bash
# Color definition
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Arg 1 verif
if [ $# -eq 0 ]; then
    echo -e "${RED}Error: You must provide the path to the world file.${NC}"
    # Affichage des fichiers .world disponibles
    echo -e "${GREEN}Available world files:${NC}"
    for file in assets/worlds/*.world; do
        echo -e "${GREEN}${file}${NC}"
    done
    return 1
fi

WORLD_FILE="$1"

if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}Error: File '$WORLD_FILE' not found.${NC}"
    return 1
fi

# Default config file 
# CONFIG_FILE="${2:-"gui/design.config"}"
echo -e "$CONFIG_FILE"

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

# gz sim "$WORLD_FILE"
ros2 launch bringup mas.launch.py & rqt