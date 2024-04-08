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


def generate_launch_description():
    uuv_thruster_manager_pkg = get_package_share_directory(
        'uuv_thruster_manager')
    thruster_manager_launch = PathJoinSubstitution(
        [uuv_thruster_manager_pkg, 'launch', 'thruster_manager.launch.py'])
    wamv_description_pkg = get_package_share_directory(
        'wamv_description')

    ARGUMENTS = [
        DeclareLaunchArgument("model_name", default_value="wamv"),
        DeclareLaunchArgument("uuv_name", default_value="$(arg model_name)"),
        DeclareLaunchArgument(
            "base_link", default_value="wamv/base_link"),
        DeclareLaunchArgument("timeout", default_value="-1"),
        DeclareLaunchArgument("reset_tam", default_value="false"),
        DeclareLaunchArgument(
            "output_dir", default_value="/home/buche/liquidAi/log"),
        DeclareLaunchArgument(
            "config_file", default_value=wamv_description_pkg + "/config/thruster_manager.yaml"),
        DeclareLaunchArgument(
            "tam_file", default_value=wamv_description_pkg + "/config/TAM.yaml"),
    ]

    launch_args = [
        ('model_name', LaunchConfiguration('model_name')),
        ("uuv_name", LaunchConfiguration("uuv_name")),
        ("base_link", LaunchConfiguration("base_link")),
        ("timeout", LaunchConfiguration("timeout")),
        ("reset_tam", LaunchConfiguration("reset_tam")),
        ("output_dir", LaunchConfiguration("output_dir")),
        ("config_file", LaunchConfiguration("config_file")),
        ("tam_file", LaunchConfiguration("tam_file")),
    ]

    accel_controller = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control.py',
        name="acceleration_control",
        parameters=[
            wamv_description_pkg+"/config/accel.yaml"
        ],
        output='screen'
    )

    vel_controller = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control.py',
        name="vel_controller",
        parameters=[
            wamv_description_pkg+"/config/vel_pid.yaml"
        ],
        output='screen'
    )

    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([thruster_manager_launch]),
        launch_arguments=launch_args)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(thruster_manager)
    ld.add_action(accel_controller)
    ld.add_action(vel_controller)
    return ld
