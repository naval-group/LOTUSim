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
        ],
        remappings=[
            ('/model/wave_world/pose', '/tf'),
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
    ])
