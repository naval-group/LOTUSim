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
sudo apt-get install libxlsxwriter-dev
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

This is an example launch for a test vehicle, wam-v.

This is an example launch for a test vehicle, wam-v.

### Setting the environment variable

```
# Source lib and binaries
source <workshop_dir>/install/setup.bash

# navigate into liquidAi repo
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models:$(pwd)/asv_wave_sim/gz-waves-models/world_models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/../../install/lib
```

### Defining the simulation

The world this tutorial will launch the defined world in `assets/worlds/vessel_wave.world`

The bridge between gazebo and Unity is currently a websocket in AIS plugin. It will be moved to ROS2.

During startup, the IP address of the Unity computer has to be defined as shown below and has to match the definition in Unity

```
<plugin filename="ais_plugin" name="liquidai::gazebo::AISPlugin">
    <period> 1 </period>
    <ip_address>127.0.0.1</ip_address>
    <port>23456</port>
</plugin>
```

The surface wave is defined in `assets/models/regular_waves/model.sdf`. Do change out these 2 lines throughout the file to change the wave

```
<amplitude>0.0</amplitude>
<period>6.0</period>
```

### Running the simulation

<details>
    <summary>
        Running gazebo without GUI
    </summary>

```
# Navigate to repo

# Running in server
gz sim -v4 -s -r assets/worlds/vessel_wave.world
# Running separate GUI in another window
gz sim -v4 -g
```

</details>

<details>
    <summary>
        Running gazebo with GUI
    </summary>

```
# Navigate to repo

clear;gz sim -v4 -r assets/worlds/vessel_wave.world
```

</details>

### Publishing gz msg

Publishing command in gazebo

```
# Gazebo
gz topic -t <topic_name> -m <msgTpye e.g gz.msgs.Double> -p 'data: 1000'
```

### Launching autonomous navigation stack

The following is a common launch to get the

1. msg bridge between gazebo and ROS
2. wam-v transform frames
3. transform GPS to odom frame for navigation stack
4. P controller for velocity to rpm of thruster

```
# Open another terminal
clear;ros2 launch wamv_description vessel_launch.xml

# Open another terminal
rviz2
---w
# Or for debugging
ros2 launch wamv_description ros_bridge.launch.py
ros2 launch wamv_description robot_description.launch.py
ros2 launch gps_to_odom gps_to_odom.launch.py
ros2 launch wamv_description vel_control.launch.py reset_tam:=false
```

And finally the navigation stack.

The navigation stack of **Land vehicle** is used instead to reduce time spent on testing on simulation. It still works on certain vessels but we aim to incorporate marine navigation stack in the future. Take note that the frame to publish waypoint is **utm**.

```
ros2 launch wamv_description nav2.launch.py
rviz2 -d <liquidai_dir>/rviz_config.rviz# GUI used to give waypoint

# To manually give waypoint:
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: utm}, pose: {position: {x: 2.716, y: -4.0763}, orientation: {x: 0.0, y: 0.0, z: -0.301924, w: 0.95333}}}"
```

For debugging, you can publish motor command in ROS for individual motor command

```
#ROS
ros2 topic pub /thrusters/id_0/input liquidai_msgs/msg/FloatStamped "{data: 250}"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

## Saving sensor data to file

```
ros2 bag record /front_camera /clock /tf /tf_static /cmd_accel /cmd_force /cmd_vel /global_costmap/costmap /global_costmap/costmap_raw /global_costmap/costmap_updates /global_costmap/footprint /global_costmap/global_costmap/transition_event /global_costmap/published_footprint /goal_pose /local_costmap/costmap /local_costmap/costmap_raw /local_costmap/costmap_updates /local_costmap/footprint /local_costmap/local_costmap/transition_event /local_costmap/published_footprint /local_plan /plan /plan_smoothed /planner_server/transition_event /received_global_plan /evaluation /cost_cloud
```

# Navigation system

A basic land vehicle stack is used to

```
ros2 launch wamv_description nav2.launch.py
```

# Map server interface

```
ros2 run map_interface map_interface  --ros-args ip_address:=<ip_address> ais_topic_name:=<topic_name>
```

## Xdyn integration

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
