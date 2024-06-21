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
    ns = LaunchConfiguration('ns')
    sdf_file  =  LaunchConfiguration('sdf_file')
    sdf_filename  =  LaunchConfiguration('sdf_filename')
    name  =  LaunchConfiguration('name')
    pose  =  LaunchConfiguration('pose')
    
    agent_node_component = ComposableNodeContainer(
    name='container',
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
                }],
        ),
    ]
)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument(
        #     'ns',
        #     default_value='ns_default',
        #     description='Name of the namespace'),
        # agent_node,
        agent_node_component
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        #     arguments=[urdf]),
    ])