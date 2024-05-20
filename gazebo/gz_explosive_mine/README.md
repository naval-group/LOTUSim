<div align="center">

# Explosive Mine

Gazebo plugin forked from the TouchPlugin. The plugin has been changed so that it can receive multiple targets.
</div>

## Flowchart

![Flowchart](ExplodingMine.drawio.svg)

## Build
In the project root folder execute :
```bash
mkdir build
cd build
cmake ..
make
```
## Run
In the project root folder execute :
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build/lib
gz sim -v 4 test_world.sdf
```
