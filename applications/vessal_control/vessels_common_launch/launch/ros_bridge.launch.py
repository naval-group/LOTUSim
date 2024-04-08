import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/AIS@liquidai_msgs/msg/AISArray@gz_liquidai_plugins_msgs.msgs.AISArray',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
    ])
