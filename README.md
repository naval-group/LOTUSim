<div align="center">

# Lotusim

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn) and [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim).

This simulation is built on Gazebo Harmonic and ROS Humble

</div>

[[_TOC_]]
## Get Started
### Installing

1. Install [gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. Libraries needed

```bash
# ROS libraries
sudo apt install -y libogre-next-2.3-dev libcgal-dev libfftw3-dev ros-humble-robot-localization ros-humble-gps-tools ros-humble-ros-gz ros-humble-xacro libignition-transport11-dev python3-colcon-common-extensions ros-humble-nav2-common ros-humble-navigation2

# Other libraries
sudo apt install -y libwebsocketpp-dev nlohmann-json3-dev libxlsxwriter-dev
```

3. Creating workspace

```bash
mkdir -p lotusim_ws/src; 
cd lotusim_ws/src;
git clone --recurse-submodules https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotus/lotusim.git
```

4. Building

```bash
source config-lotus.bash
```

### Running

This is an example launch for a surface and underwater vessel

and

Using xdyn-for-cs websocket

```bash
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
## Contributing

### Workflow
We use the Gitflow collaborating workflow. You can find the explanation of this workflow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

We are using the default Gitflow branch naming like [here](https://www.gitkraken.com/blog/gitflow).

### Issues

When you open an issue, you need to put the correct label corresponding to the category of the issue e.g. ~bug or ~suggestion. It needs to be written in english.

**IMPORTANT** You have to put ~"project::development" or ~"project::management" label on each issue as they are used for filtering in the different issue boards

You can find the description of the labels [here](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim/-/labels).


