# Installing
1. Copy the following into the bashrc

```
export LOTUSIM_WS=\$HOME/lotusim_ws
export LOTUSIM_PATH=\$LOTUSIM_WS/install/share/lotusim/lotusim
export PATH=\$LOTUSIM_WS/install/share/lotusim/physics/:\$LOTUSIM_WS/install/share/lotusim/launch:\$PATH
export LD_LIBRARY_PATH=\$LOTUSIM_WS/install/physics:\$LD_LIBRARY_PATH
source \$LOTUSIM_PATH/bash_completion.sh
```

2. Source the environment
```
source ~/.bashrc
```