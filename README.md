<div align="center">


# LOTUSim

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn) and [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim).

This simulation is built on Gazebo Garden, a non LTS version, as this is started when the simulator Ignition is renaming to Gazebo and libraries are being renamed too. Hence, for future proofing, Garden is used. The simulation will be moved to LTS after alpha release.
</div>

[[_TOC_]]

## Installing

1. Install [gazebo garden](https://gazebosim.org/docs/garden/install_ubuntu) and [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. Libraries needed

```bash
# ROS libraries
sudo apt install -y libogre-next-2.3-dev libcgal-dev libfftw3-dev ros-humble-robot-localization ros-humble-gps-tools ros-humble-ros-gz ros-humble-xacro libignition-transport11-dev python3-colcon-common-extensions  ros-humble-nav2-common ros-humble-navigation2

# Other libraries
sudo apt install -y libwebsocketpp-dev nlohmann-json3-dev
```

3. Creating workspace
```bash
cd; mkdir -p LOTUSim_ws/src; cd LOTUSim_ws/src; 
git clone https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim.git
```

4. Building
```bash
cd lotusim/
git submodule init
git submodule update --recursive

# Building
source config-lotus.sh
```

## Launching the simulation

This is an example launch for a test vehicle, wam-v.


### Running the simulation


```bash
source launch-lotus.sh <world e.g. assets/worlds/poc_test.world>
```


### Publishing thruster commands

Publishing motor command in gazebo for individual motor command

```bash
# Gazebo

# Pour le drone de surface 
gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 1000'
gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 1000'


# Pour le drone sous-marin
gz topic -t /model/lrauv/joint/propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 300'

```

### ROS commands

```bash
# Open another terminal

source /opt/ros/humble/setup.bash
source <workshop_dir>/install/setup.bash
ros2 launch wamv_description vessel_launch.xml

# Open another terminal
rviz2


# Open another terminal

source /opt/ros/humble/setup.bash

source <workshop_dir>/install/setup.bash


# Pour le drone de surface 

ros2 topic pub /wamv/thrusters/id_0/input liquidai_msgs/msg/FloatStamped "{data: 250}"

# Pour le drone sous-marin

ros2 topic pub /lrauv/propeller/input liquidai_msgs/msg/FloatStamped "{data: 250}"
```

## Contributing

### Workflow
We use the Gitflow collaborating workflow. You can find the explanation of this workflow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

### Issues

When you open an issue, you need to put the correct label corresponding to the category of the issue e.g. ~bug or ~suggestion. Ir need to be written in english.

**IMPORTANT** You have to put ~"project::development" or ~"project::management" label on each issue as they are used for filtering in the different issue boards

You can find the description of the labels [here](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim/-/labels).


