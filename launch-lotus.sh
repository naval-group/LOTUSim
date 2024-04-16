#!/bin/bash

# Définition des couleurs
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Définition des chemins
ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
LOTUSIM_PATH="src/lotusim"
ASSETS_MODELS_PATH="assets/models"
ASV_WAVE_SIM_PATH="asv_wave_sim/gz-waves-models/world_models"
OLD_GZ_GUI_LIB_PATH="/lib/x86_64-linux-gnu"

cd ../..

# Source de la configuration ROS
# shellcheck source=/dev/null
source "$ROS_SETUP_PATH"

# Source de l'environnement installé
# shellcheck source=/dev/null
source "$(pwd)/install/setup.bash"

# Définition de la variable d'environnement GZ_SIM_SYSTEM_PLUGIN_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH
echo -e "GZ_SIM_SYSTEM_PLUGIN_PATH : ${GREEN}$GZ_SIM_SYSTEM_PLUGIN_PATH${NC}"

# Se déplacer vers le répertoire lotusim
cd "$LOTUSIM_PATH" || return 

# Définition de la variable d'environnement GZ_SIM_RESOURCE_PATH
GZ_SIM_RESOURCE_PATH="$(pwd)/${ASSETS_MODELS_PATH}:$(pwd)/$ASV_WAVE_SIM_PATH"
export GZ_SIM_RESOURCE_PATH
echo -e "GZ_SIM_RESOURCE_PATH : ${GREEN}$GZ_SIM_RESOURCE_PATH${NC}"

# Variable d'environnement des autres lib de GZ GUI (plugins)
GZ_GUI_PLUGIN_PATH="${OLD_GZ_GUI_LIB_PATH}/gz-gui-7/plugins"
export GZ_GUI_PLUGIN_PATH
echo -e "GZ_GUI_PLUGIN_PATH : ${GREEN}$GZ_GUI_PLUGIN_PATH${NC}"
echo -e "${GREEN}Launching LOTUSim...${NC}"
gz sim 'assets/worlds/poc_test.world' --gui-config='gui/design.config'
