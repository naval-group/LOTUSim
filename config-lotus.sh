#!/bin/bash

# Définition des couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Définition des chemins
ROS_SETUP_PATH="/opt/ros/humble/setup.bash"
LOTUSIM_PATH="src/lotusim"
ASSETS_MODELS_PATH="assets/models"
ASV_WAVE_SIM_PATH="asv_wave_sim/gz-waves-models/world_models"
NEW_GZ_GUI_LIB_PATH="gui/lib"
OLD_GZ_GUI_LIB_PATH="/lib/x86_64-linux-gnu"

# Vérification de l'existence des répertoires
if [ ! -f "$ROS_SETUP_PATH" ]; then
    echo -e "${RED}Erreur:${NC} Le fichier ROS setup n'existe pas à $ROS_SETUP_PATH."
    exit 1
fi

if [ ! -d "../../${LOTUSIM_PATH}" ]; then
    echo -e "${RED}Erreur:${NC} Le répertoire lotusim n'existe pas à $LOTUSIM_PATH."
    exit 1
fi

if [ ! -d "${ASSETS_MODELS_PATH}" ]; then
    echo -e "${RED}Erreur:${NC} Le répertoire des modèles assets n'existe pas à $ASSETS_MODELS_PATH."
    exit 1
fi

if [ ! -d "${ASV_WAVE_SIM_PATH}" ]; then
    echo -e "${RED}Erreur:${NC} Le répertoire des modèles de simulation ASV n'existe pas à $ASV_WAVE_SIM_PATH."
    exit 1
fi

# Vérification de l'installation de colcon-clean
if ! command -v colcon clean &> /dev/null; then
    echo -e "${RED}Erreur:${NC} Le package colcon-clean n'est pas installé."
    exit 1
fi



# Se déplacer vers le répertoire racine
cd ../..

# Source de la configuration ROS
# shellcheck source=/dev/null
source "$ROS_SETUP_PATH"

# Vérification de la présence des dossiers build, install et log
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    read -rp "Les dossiers build, install et/ou log existent déjà dans le workspace. Voulez-vous nettoyer l'espace de travail ? (y/n) " response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        colcon clean workspace
    fi
fi

# Construction de l'environnement
colcon build --merge-install

# Source de l'environnement installé
# shellcheck source=/dev/null
source "$(pwd)/install/setup.bash"

# Définition de la variable d'environnement GZ_SIM_SYSTEM_PLUGIN_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH
echo -e "GZ_SIM_SYSTEM_PLUGIN_PATH : ${GREEN}$GZ_SIM_SYSTEM_PLUGIN_PATH${NC}"

# Se déplacer vers le répertoire lotusim
cd "$LOTUSIM_PATH" || exit

# Définition de la variable d'environnement GZ_SIM_RESOURCE_PATH
GZ_SIM_RESOURCE_PATH="$(pwd)/${ASSETS_MODELS_PATH}:$(pwd)/$ASV_WAVE_SIM_PATH"
export GZ_SIM_RESOURCE_PATH
echo -e "GZ_SIM_RESOURCE_PATH : ${GREEN}$GZ_SIM_RESOURCE_PATH${NC}"


# Remplacement de certaines librairies de GZ GUI
# La modification de ces libraires permet d'afficher le logo NG + corporate sensitivity 
sudo mv "$NEW_GZ_GUI_LIB_PATH"/* "$OLD_GZ_GUI_LIB_PATH"

# Variable d'environnement des autres lib de GZ GUI (plugins) 
GZ_GUI_PLUGIN_PATH="${OLD_GZ_GUI_LIB_PATH}/gz-gui-7/plugins"
export GZ_GUI_PLUGIN_PATH
echo -e "GZ_GUI_PLUGIN_PATH : ${GREEN}$GZ_GUI_PLUGIN_PATH${NC}"




