# World plugin

These are the plugins used in liquidAi.

TODO:

- Currently the plugin is directly using ROS2 node. It shouldn't have direct link and should be seperated by ignition bridge. However, as the bridge does not support ignition services at time of development, ROS2 node is used instead.

```
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/malcom/liquidAi/install/uuv_world_ros_plugins_ignition/lib:/home/malcom/liquidAi/install/test_hello_world/lib

export IGN_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gui-6/plugins/gui/

ign service  --service /world/shapes/set_spherical_coordinates --reqtype ignition.msgs.SphericalCoordinates --reptype ignition.msgs.Boolean --timeout 2000 --req 'latitude_deg: 0, longitude_deg: 10' 

ign service -s /world/shapes/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 300 --req 'name: "box", position: {z: 5.0}'

ros2 service call /world/shapes/get_origin_spherical_coordinates/get_origin_spherical_coordinates liquidai_msgs/srv/GetOriginSphericalCoord "{}"

ros2 service call /world/shapes/get_origin_spherical_coordinates/set_origin_spherical_coordinates liquidai_msgs/srv/SetOriginSphericalCoord "{latitude_deg: 0.0, longitude_deg: 0.0, altitude: 0.0 }"
```

## Param nodes
Wave and wind params will be held using ROS nodes for dynamic configuration.