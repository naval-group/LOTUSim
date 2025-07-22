#!/bin/bash

cat <<EOF >>~/.bashrc
export PATH=\$HOME/lotusim_ws/src/lotusim/physics/:\$HOME/lotusim_ws/src/lotusim/launch:\$PATH
export LOTUSIM_WS=\$HOME/lotusim_ws
export LOTUSIM_PATH=\$LOTUSIM_WS/src/lotusim
export LD_LIBRARY_PATH=\$LOTUSIM_PATH/physics
export LOTUSIM_MODELS_PATH=\$LOTUSIM_PATH/assets/models/
source \$LOTUSIM_PATH/launch/bash_completion.sh
EOF

source ~/.bashrc
chmod -R +x "$LOTUSIM_PATH/launch/"*
chmod -R +x "$LOTUSIM_PATH/physics/"*
lotusim install
