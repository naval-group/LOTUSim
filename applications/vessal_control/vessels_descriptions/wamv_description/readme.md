## Xacro model linkage
```
wamv_gazebo.urdf.xacro
|── wamv_gazebo.xacro
    |── macros.xacro : Load all sensors macro
    |── wamv_base.urdf.xacro : Create the collision
    |── wamv_aft_thrusters.xacro : Create left right thruster
        |── engine.xacro : Create a single thruster
        |── wamv_gazebo_thruster_config.xacro : Create thruster plugin
```

## TODO
1. Unable to generate model in gazebo using robot_description topic. Currently fixed by generating the model by using sdf generated using xacro
