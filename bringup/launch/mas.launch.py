import os
import sys

from sympy import true

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from pathlib import Path
import json


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_description = get_package_share_directory('assets')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    CONFIG_FILE_PATH = os.path.join(pkg_project_bringup, 'config', 'mas_config.json')

    # Open the JSON configuration file in read mode
    with open(CONFIG_FILE_PATH, "r") as file:
        json_data = json.load(file)  # Load JSON data from the file into a variable

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-v {} '.format(json_data['params']['verbose']) 
            + os.path.join(pkg_project_description,'worlds',json_data['params']['world_filename']) 
            + " --gui-config={}".format(json_data['params']['gui-config'])}.items(),
    )

    
    agent_launch = LaunchDescription()
    namespace_nb = 1

    for agent_config in json_data['agent_configs']:
        if json_data['params']['automatic_namespace'] == True:
            default_namespace = json_data['params']['default_namespace'] + str(namespace_nb)
        else:
            default_namespace = ""
        
        agent_launch.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_project_bringup, 'launch', 'multi_agent_spawn.launch.py')),
                launch_arguments={'agent_config': str(agent_config),
                                  'ns_base': default_namespace
                                }.items())
        )
        namespace_nb = namespace_nb + 1

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'expand_gz_topic_names': True, # Enables the Bridge to apply ROS2 namespace on the Gazebo topics,
            'use_sim_time': True
        }],
        output='screen'
    )

    scheduler = Node(
        package='agent_cpp',
        executable='scheduler',
        parameters=[{'use_sim_time': True,
            }],
    )
    
    agent_factory = Node(
        package='agent_cpp',
        executable='agent_factory',
        arguments=[{json_data['agent_configs'][0]}, {'main'}],
        parameters=[{'use_sim_time': True,
            }],
    )
    
    mas_components = ComposableNodeContainer(
    name='SimulationContainer',
    namespace='lotusim',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='simulation_control',
            plugin='SimulationControl',
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[{'use_sim_time': False,
                }],
        ),
    ])

    return LaunchDescription([
        # scheduler,
        # agent_factory,
        gz_sim,
        # agent_node_component,
        mas_components,
        agent_launch,
        bridge,
    ])
