# LiquidAi

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn) and [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim).

This simulation is built on Gazebo Garden, a non LTS version, as this is started when the simulator Ignition is renaming to Gazebo and libraries are being renamed too. Hence, for future proofing, Garden is used. The simulation will be moved to LTS after alpha release.

## Installing

1. Install [gazebo garden](https://gazebosim.org/docs/garden/install_ubuntu) and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

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

## Launching the simulation

This is an example launch for a test vehicle, wam-v.

### Setting the environment variable

```
# Source lib and binaries

cd /home/helene/Documents/LOTUSim/liquidAi/src/liquidai
source /home/helene/Documents/LOTUSim/liquidAi/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(pwd)/assets/models:$(pwd)/asv_wave_sim/gz-waves-models/world_models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/helene/Documents/LOTUSim/liquidAi/install/lib
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



### Publishing thruster commands

Publishing motor command in gazebo for individual motor command


```
# Gazebo
gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 1000'
gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 1000'
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
---
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
