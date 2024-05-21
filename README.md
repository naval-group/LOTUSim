# LiquidAi

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn) and [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim).

This simulation is built on Gazebo Harmonic and ROS Humble

## Installing

1. Install [gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. Libraries needed

```
# ROS libraries
sudo apt install -y libogre-next-2.3-dev libcgal-dev libfftw3-dev ros-humble-robot-localization ros-humble-gps-tools ros-humble-ros-gz ros-humble-xacro libignition-transport11-dev python3-colcon-common-extensions  ros-humble-nav2-common ros-humble-navigation2

# Other libraries
sudo apt install -y libwebsocketpp-dev nlohmann-json3-dev
```

3. Creating workspace
```
cd; mkdir -p liquidai_ws/src; cd liquidai_ws/src; 

<clone repo>
```

4. Building
```
cd liquidai
git submodule init
git submodule update

# Building
cd ~/liqduiai_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install 
```

## Tutorial

### Setting the environment variable

```
# Source lib and binaries
source <workshop_dir>/install/setup.bash

# navigate into liquidAi repo
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/../../install/lib
```

### Defining the simulation

The world is defined in gazebo style as per usual. 

There are some plugins created to cater to the use case of marine shown below

#### AIS system

``` 
// In the overall world
<plugin filename="ais_plugin" name="liquidai::gazebo::AISPlugin">
      <period> 1 </period>
</plugin>

// In individual models
<publish_ais>false</publish_ais>

```

#### Rendering system

```
// In the overall world
<plugin filename="render_plugin" name="liquidai::gazebo::RenderPlugin">
    <connection_protocol>udp</connection_protocol>
    <ip>127.0.0.1</ip>
    <port>23456</port>
</plugin>

// In individual models
<publish_render>false</publish_render>
```



