import os
import sys

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    asset_path = FindPackageShare("assets").perform(LaunchContext())
    env = {
        "GZ_SIM_RESOURCE_PATH": asset_path+"/models",
        "GZ_SIM_SYSTEM_PLUGIN_PATH": asset_path+"/../../lib"
    }

    world_name = LaunchConfiguration('world')
    world_file = PathJoinSubstitution(
        [
            asset_path,
            'worlds',
            world_name
        ])
    use_gui = LaunchConfiguration('use_gui')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='vessel_wave.world',
        description='Directory of gazebo world file'))
    ld.add_action(DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Whether to use Gazebo GUI'))

    launch_cmd = ['gz sim']
    launch_cmd.append(' -r ')
    if (not True):
        launch_cmd.append(' -s ')
    launch_cmd.append(world_file)

    gz_launch_action = ExecuteProcess(
        cmd=launch_cmd,
        name='gazebo sim',
        output='both',
        additional_env=env,
        shell=True
    )
    ld.add_action(gz_launch_action)
    return ld
