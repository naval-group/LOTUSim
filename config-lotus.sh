#!/bin/bash
# Color definition
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color


ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
LOTUSIM_PATH="src/lotusim"
ASSETS_MODELS_PATH="assets/models"

NEW_GZ_GUI_LIB_PATH="gui/lib"
OLD_GZ_GUI_LIB_PATH="/lib/x86_64-linux-gnu"

export ROS_DISTRO=humble
export GZ_VERSION=harmonic

if [ ! -f "$ROS_SETUP_PATH" ]; then
    echo -e "${RED}Erreur:${NC} Le fichier ROS setup n'existe pas à $ROS_SETUP_PATH."
    return 1
fi

if [ ! -d "../../${LOTUSIM_PATH}" ]; then
    echo -e "${RED}Erreur:${NC} Le répertoire lotusim n'existe pas à $LOTUSIM_PATH."
    return 1
fi

if [ ! -d "${ASSETS_MODELS_PATH}" ]; then
    echo -e "${RED}Erreur:${NC} Le répertoire des modèles assets n'existe pas à $ASSETS_MODELS_PATH."
    return 1
fi

if ! command -v colcon clean &>/dev/null; then
    echo -e "${RED}Erreur:${NC} Le package colcon-clean n'est pas installé."
    return 1
fi

# Move to root directory
cd ../..

source "$ROS_SETUP_PATH"

if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    read -rp "The build, install and/or log directories already exist in the workspace. Do you want to clean the workspace ? (y/n) " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        colcon clean workspace
    fi
    read -rp "Do you want to rebuild those files ? (y/n) " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        colcon build --merge-install --allow-overriding ros_gz_bridge
    fi
else
    # Construct environment
    colcon build --merge-install --allow-overriding ros_gz_bridge
fi

source "$(pwd)/install/setup.bash"

GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH
echo -e "GZ_SIM_SYSTEM_PLUGIN_PATH : ${GREEN}$GZ_SIM_SYSTEM_PLUGIN_PATH${NC}"

cd "$LOTUSIM_PATH" || return 1

GZ_SIM_RESOURCE_PATH="$(pwd)/${ASSETS_MODELS_PATH}"
export GZ_SIM_RESOURCE_PATH
echo -e "GZ_SIM_RESOURCE_PATH : ${GREEN}$GZ_SIM_RESOURCE_PATH${NC}"
