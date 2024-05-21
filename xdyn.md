# Guide of xdyn

This is a developer guide for [xdyn](https://github.com/sirehna/xdyn).
Some of the content will be a copy of xdyn doc as the original is in french and is not translated to english.

### HDF5Viewer
To see xdyn results
```
sudo apt install python3-pyqt5
sudo pip3 install hdf5view
```



## Xdyn integration 

Using xdyn-for-cs websocket

```
# Terminal 1
export LD_LIBRARY_PATH=<path to repo>/physics/xdynSurface/filter09
cd <path to repo>/physics/xdynSurface
./xdyn-for-cs ../../assets/models/dtmb_hull/dtmb-wave-propeller-PID.yml -v -p 12345 -d --dt 0.1

# Terminal 2
cd <path to repo>
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=<path to ws>/install/lib
clear; gz sim -v4 -s -r assets/worlds/xdyn_test.world
```
