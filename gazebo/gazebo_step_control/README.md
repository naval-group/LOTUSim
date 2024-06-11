# gazebo_step_control

Gazebo Plugin to allow step control during ROS2 simulation.

### Services exposed:

```sh
/step_control_enable (bool)
  service to enable/disable stepping control

/step (num_of_steps, blocking)
  service to provide number of steps to execute. blocking=true will block service call until steps are executed
```

### SDF/URDF Usage:

```sh
<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
<world name="default">
...
<plugin name="gazebo_step_control" filename="libgazebo_step_control.so">
   <enable_control>false</enable_control>
</plugin>
...
</world>
</sdf>
```

Enable step control using service call:

```sh
ros2 service call /step_control_enable std_srvs/srv/SetBool "{data: True}"
```

## Execution:

### Blocking mode:

Execute 100 steps in blocking mode. Service call will return back after Gazebo executed 100 steps of simulation.

```sh
ros2 service call /step gazebo_step_control_interface/srv/StepControl "{steps: 100, block: 1}"
```

### Non Blocking mode:

Execute 100 steps in blocking mode. Service call will return immediately. Step control implementation will publish /step_completed topic once 100 steps are completed.

```sh
ros2 service call /step_control gazebo_step_control_interface/srv/StepControl "{steps: 100, block: 0}"
ros2 topic echo /step_completed
```

Step controller client implementation included in the repo.