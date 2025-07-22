## TO BE RETIRED.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/AIS@liquidai_msgs/msg/AISArray@gz_liquidai_msgs.msgs.AISArray',
            '/frigate/cmd_vel@liquidai_msgs/msg/Xdyncmd@gz_liquidai_msgs.msgs.XdynCmd',
            '/lrauv/cmd_vel@liquidai_msgs/msg/Xdyncmd@gz_liquidai_msgs.msgs.XdynCmd',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            # '/magnetometer@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer',
            # '/gps@liquidai_msgs/msg/GPS@ignition.msgs.NavSat',
            # 'lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # '/model/wave_world/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                 '/model/bluerov2_heavy/joint/thruster1_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster3_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster4_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster5_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster6_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster7_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/joint/thruster8_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/bluerov2_heavy/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/wave_world/pose', '/tf'),
                ( '/model/bluerov2_heavy/joint/thruster1_joint/cmd_thrust',
             "/bluerov2_heavy/thruster1/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster2_joint/cmd_thrust',
             "/bluerov2_heavy/thruster2/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster3_joint/cmd_thrust',
             "/bluerov2_heavy/thruster3/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster4_joint/cmd_thrust',
             "/bluerov2_heavy/thruster4/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster5_joint/cmd_thrust',
             "/bluerov2_heavy/thruster5/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster6_joint/cmd_thrust',
             "/bluerov2_heavy/thruster6/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster7_joint/cmd_thrust',
             "/bluerov2_heavy/thruster7/cmd_thrust"),
            ( '/model/bluerov2_heavy/joint/thruster8_joint/cmd_thrust', "/bluerov2_heavy/thruster8/cmd_thrust")
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
    ])
