# Guide of xdyn

This is a developer guide for [xdyn](https://github.com/sirehna/xdyn).
Some of the content will be a copy of xdyn doc as the original is in french and is not translated to english.

## Installation
### Docker
```
docker image pull sirehna/xdyn:v6-4-2
```
### HDF5Viewer
To see xdyn results
```
sudo apt install python3-pyqt5
sudo pip3 install hdf5view
```

## XDyn Demo

xdyn has airy waves only.

Airy waves are the propagation of gravity waves

```
 docker run --rm -u `id -u`:`id -g` -v `pwd`:/xdyn_demos -w /xdyn_demos sirehna/xdyn:v6-4-2 tutorial_03_waves.yml --dt 1 --tend 10 -w tutorial_03_results.h5 -d

 hdf5view -f waves.h5
```



## Xdyn integration 

Using xdyn-for-cs websocket

```
# Terminal 1
export LD_LIBRARY_PATH=/home/malcom/garden_ws/src/liquidai
./xdyn-for-cs ./assets/models/5415_hull/xdyn_test.yml -v -p 12345 -d --dt 0.01

# Terminal 2
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models:$(pwd)/asv_wave_sim/gz-waves-models/world_models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/malcom/garden_ws/install/lib
clear; gz sim -v4 -s -r assets/worlds/vessel_wave.world
```

