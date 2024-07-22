import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument("model_name", default_value="wamv"),
    DeclareLaunchArgument("uuv_name", default_value="wamv"),
    DeclareLaunchArgument("base_link", default_value="base_link"),
    DeclareLaunchArgument("timeout", default_value="-1"),
    DeclareLaunchArgument("reset_tam", default_value="false"),
    DeclareLaunchArgument(
        "output_dir", default_value="/home/buche/LOTUSim_ws/src/wamv_description/config"),
    DeclareLaunchArgument(
        "config_file", default_value="/home/buche/LOTUSim_ws/src/wamv_description/config/thruster_manager.yaml"),
    DeclareLaunchArgument(
        "tam_file", default_value="/home/buche/LOTUSim_ws/src/wamv_description/config/TAM.yaml"),
    DeclareLaunchArgument("urdf_file", default_value=""),
]


def generate_launch_description():
    uuv_thruster_manager = get_package_share_directory('uuv_thruster_manager')
    
    if LaunchConfiguration('reset_tam'):
        thruster_allocator = Node(
            package='uuv_thruster_manager',
            executable='thruster_allocator.py',
            name='thruster_allocator',
            output='screen',
            parameters=[
                {'output_dir': LaunchConfiguration('output_dir')},
                {'urdf_file': LaunchConfiguration('urdf_file')},
                LaunchConfiguration('config_file')
            ],
            remappings=[
                ('tf', '/tf'),
            ]
        )
    else:
        taml_config = os.path.join(
            get_package_share_directory('wamv_description'),
            'config',
            'taml.yaml'
            )
        thruster_allocator = Node(
            package='uuv_thruster_manager',
            executable='thruster_allocator.py',
            name='thruster_allocator',
            output='screen',
            parameters=[
                {'output_dir': LaunchConfiguration('output_dir')},
                {'urdf_file': LaunchConfiguration('urdf_file')},
                LaunchConfiguration('config_file'),
                taml_config
            ],
            remappings=[
                ('tf', '/tf'),
            ]
        )
    
    ld=LaunchDescription(ARGUMENTS)
    ld.add_action(thruster_allocator)
    return ld

