## Installations

- PX4-Autopilot
#### 1. Clone PX4 and run setup script

```bash
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

#### 2. Reboot your system before the next step

#### 3. Build PX4 SITL 
```bash
cd PX4-Autopilot
make px4_sitl
```

- QGroundControl 
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

- Micro XRCE-DDS Agent if you want to use ROS2 for the drones (optional)

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
## Basic Wind and Drone Demo
### Terminal 1

#### 1. Run lotusim aerial world

```bash
lotusim --gui run poc_aerial_jacob.world
```

### Terminal 2

#### 2. Launch PX4 instance to connect to the drone 

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

#### 3. Launch QGroundControl drone controller

#### 4. Get the drone to take off using the controls

#### 5. Locate the "Wind Adjuster" GUI plugin

#### 6. Adjust the wind velocity using the sliders. The turbines will spin when the wind direction is opposite to the direction the propeller is facing. In this world, because the turbines are facing the negative x direction, the x velocity needs to be positive for them to spin. This represents how real world turbines are effected by the forces of wind. 


## Multiple Drones

### Terminal 1

#### 1. Run lotusim multiple drone world

```bash
lotusim --gui run poc_multiple_drones.world
```

### Terminal 2

#### 2. Launch PX4 to spawn 1st drone model and server

```bash
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

### Terminal 3

#### 2. Launch PX4 to spawn 2nd drone model and server

```bash
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
```

### Terminal 4

#### 2. Launch PX4 to spawn 3rd drone model and server

```bash
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="0,-2" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3
```
On QGroundControl switch between the drones by selecting the drone on the dropdown above.

Note: If you use joystick control, you will only be able to fly one drone at a time. Switching between drones will activate a failsafe and force it to land.

## Time-based Wind Variation
### Terminal 1
#### 1. Run lotusim wind variation world

```bash
lotusim --gui run poc_aerial_time_variations_demo.world
```
### Terminal 2
#### 2. Launch PX4 to spawn drone model and server

```bash
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

The drone can be observed displaying variation in the wind forces applied which are time dependant. You can adjust the parameters to simulate different intensities, frequencies and the amound of noise.

The model has had some basic modifications and now includes wind gusts. This can only be applied using the modified windEffects plugin.

## Spatial Wind Regions
A world demonstrating the spatial wind dependencies that can be set. You can define multiple regions but they must not overlap. More information can be seen in the WindEffects plugin.

Note: It has been discovered that the wind regions will only effect objects that have the <enable_wind>true</enable_wind> tag set. This tag is needed for all objects that you want affected by the wind. Since the wind turbines rely on the "LiftDrag" plugin which utlises the wind in the world to simulate drag, it will not be registered in the regions and only impacted by the base wind velocity (the wind velocity ouside the regions). I think this could potentially be resolved if you implement the same logic as seen in the WindEffects plugin for how it registers objects and keeps track of what region they are in. Another point to note is that the PX4 drone model does not seem to be effect by the regions and only the base wind is applied. It may have something to do with how gazebo registers the model, though, the reason is not yet known. 

### Terminal 1
#### 1. Run lotusim wind variation world

```bash
lotusim --gui run poc_aerial_spatial_demo.world
```

#### 2. Use the wind adjuster gui plugin to adjust the wind speed. The wind region scaling can be observed. Each sphere is identical and the effects of the regions can be seen, particularly when increasing the z velocity. The red region has no wind, green is slightly increased, blue is stronger, and yellow is the strongest. 


# - Additional Notes -
## Aerial System

The aerial wind physics in the simulation is based off of gazebos "WindEffects" plugin. After research on air models was conducted, it was found that the wind plugin allows for more parameters and overall realism of simulating wind forces. It includes both time and spatial dependencies as demonstrated, allowing for various conditions to be simulated. For an object/entity to be effected by wind, the model needs to have the "enable_wind" tag set to true. For more complex models that contain multiple links such as the PX4 drone, you can choose to enable wind for all links which will allow for a much greater wind impact. By default I have only set enable wind in the scope of the model at a high level.

Regarding future developments, I think a way that you could connect the wind to xdyn would be to publish the x, y, z velocities to the models within xdyn. I have already made some progress on this in my modified version of the WindEffects plugin which prints out the velocities at each update cycle. I have also made some progress on creating a seperate aerial server, creating the publishing and subscribing system which can be seen in the "gazebo transport" repository. Additionally, the gui plugin "WindAdjuster" should always reference the world that is simulating wind. For integration within unity, you could create a similar gui wind adjuster that publishes the set wind velociies to the aerial world.

## Wind Turbine

As described earlier, the wind turbine utilises the "LiftDrag" plugin to allow for it to be physically impacted by the wind. It was a lengthy process to assemble the wind turbine from blender and configure all physical properties of the blades including inertia, center of mass etc. There is currently no other gazebo project known that simulates wind impacting a wind turbine. The turbine is not completely accurate in terms of the masses etc, however it serves as a proof of concept. The revolute joint for the hub can also be adjusted to limit the max rotational speed, friction etc to create more accuracy.

## Modified Wind Effects Plugin

In the process of analysing and testing Gazebos Wind Effects plugin, small modifications were made to test the flexibility of the plugin. An important property of wind is wind gusts, and this wasnt really simulated in the original plugin. A function for wind gusts was added and can be used by implementing the following as a child of the <wind> element. It simulates timed based-wind gusts which can be adjusted. When modifying the plugin, it was done through the source install of gazebo and building the plugin after each modification. A better way this could be done is to take the whole wind effects plugin from the gazebo source and make it your own plugin within the LOTUSim project, allowing easy access for modification. To apply the modified wind effects plugin, I have added the cc file in this branch called "ModifiedWindEffectsPlugin.cc", it just needs to be applied and built with the entire wind effects plugin. 

```bash
<wind>
  <gusts>
    <amplitude>5.0</amplitude>
    <period>10.0</period>
    <duration>3.0</duration>
  </gusts>
</wind>

```
The modified script now has message outputs to view the wind velocity and execution speed each update cycle. It can be seen by adding -v4 when running gazebo.

## PX4 Autopilot

PX4 provides well written documentation that covers a wide range of scenarios. Something to point out is how PX4 spawns in a model in the world. It was not really covered in the PX4 documentation, however, after some testing, it was discovered that you are required to have the x500 and x500_base model folders in the model directory of the project (you can spawn a PX4 model in a world without referencing it in the sdf), however, camera tracking for the drone only seems to work if you have the model referenced in the sdf. Normally, the camera tracking can be adjusted in the "Camera Tracking Config" gui plugin, but does no seem to work in lotusim. Refer to PX4's documentation for futher information.

Documentation: https://docs.px4.io/main/en/sim_gazebo_gz/

## QGroundControl

QGroundControl is full of useful features relating to drone control and analysis. You can create flight plans and control the drone using vurtual joysticks or a controller. I was able to easily set up a PS4 controller and control the drone. You can configure the drones sensors, parameters, flight behaviours, safety features and more. Also, it recognises the spherical coordinates defined in the gazebo world sdf and sets the world map to locate to that point which can be useful for demonstrations. Refer to the QGroundControl documentation for futher insight.


