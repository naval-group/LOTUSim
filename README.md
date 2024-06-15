# Lotusim

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn) and [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim).

This simulation is built on Gazebo Harmonic and ROS Humble

## Installing

1. Install [gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. Libraries needed

```
# ROS libraries
sudo apt install -y libogre-next-2.3-dev libcgal-dev libfftw3-dev ros-humble-robot-localization ros-humble-gps-tools ros-humble-ros-gz ros-humble-xacro libignition-transport11-dev python3-colcon-common-extensions ros-humble-nav2-common ros-humble-navigation2

# Other libraries
sudo apt install -y libwebsocketpp-dev nlohmann-json3-dev libxlsxwriter-dev
```

3. Creating workspace

```
cd; mkdir -p lotusim_ws/src; cd lotusim_ws/src;

<clone repo>
```

4. Building

```
cd lotusim_ws
git submodule init
git submodule update

# Building
cd ~/lotusim_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install
```

## Tutorial

This is an example launch for a surface and underwater vessel

and

Using xdyn-for-cs websocket

```
# Terminal 1 launching surface xdyn
export LD_LIBRARY_PATH=/home/malcom/release_ws/src/lotusim/physics/xdynSurface/
cd src/liquidai/physics/xdynSurface
clear;./xdyn-for-cs ../../assets/models/dtmb_hull/dtmb-wave-propeller-PID.yml -v -a 127.0.0.1 -p 12345 -d --dt 0.2

# Terminal 2 launching underwater xdyn
export LD_LIBRARY_PATH=/home/malcom/release_ws/src/lotusim/physics/xdynUnderwater
cd src/lotusim/physics/xdynUnderwater/
clear;./xdyn-for-cs ../../assets/models/lrauv_xdyn/lrauv.yml -v -a 127.0.0.1 -p 12345 -d --dt 0.2

# Terminal 3 Launch gz sim
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models:$(pwd)/asv_wave_sim/gz-waves-models/world_models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/malcom/release_ws/install/lib
clear; gz sim -v4 -s -r assets/worlds/xdyn_underwater.world

# Terminal 4 bridge ROS2 and gz stuff
ros2 launch dtmb_description ros_bridge.launch.py

# Terminal 5 keyboard control
ros2 run keyboard_control keyboard_control --ros-args -p vessel_name:=test_ship_vessel
```
