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
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('pause', default_value='false',
                          description='pause simulation at start'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Ignition World'),
]

def generate_launch_description():
    asset_dir = get_package_share_directory(
        'assets')
    wamv_model_description_dir = get_package_share_directory('wamv_description')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(asset_dir, 'worlds'), ':' +
            os.path.join(asset_dir, 'models'), ':' +
            os.path.join(wamv_model_description_dir, 'models'), ':' +
            str(Path(wamv_model_description_dir).parent.resolve())
        ])
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'),
                ' -v 4'])
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(ignition_gazebo)
    return ld
