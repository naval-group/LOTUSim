#!/usr/bin/python3
import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    configure_on_startup = LaunchConfiguration('configure_on_startup')
    ns = LaunchConfiguration('ns')
    sdf_file  =  LaunchConfiguration('sdf_file')
    sdf_filename  =  LaunchConfiguration('sdf_filename')
    name  =  LaunchConfiguration('name')
    pose  =  LaunchConfiguration('pose')
    gazebo_id  =  LaunchConfiguration('gazebo_id')
    
    agent_node_component = ComposableNodeContainer(
    name='AgentContainer',
    namespace=ns,
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='agent_cpp',
            plugin='AgentEntity',
            name=name,
            namespace=ns,
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[{'use_sim_time': use_sim_time,
                'sdf_file': sdf_file,
                'sdf_filename': sdf_filename,
                'pose': pose,
                'configure_on_startup': configure_on_startup,
                }],
        ),
    ]
)
    
    agent_node = Node(
        package='agent_cpp',
        executable='agent_node',
        name=name,
        namespace=ns,
        parameters=[{'use_sim_time': True,
            'sdf_file': sdf_file,
            'sdf_filename': sdf_filename,
            'pose': pose,
            'configure_on_startup': configure_on_startup,
            'gazebo_id': gazebo_id
            }],
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='true',
        #     description='Use simulation (Gazebo) clock if true'),
        agent_node,
        # agent_node_component
    ])