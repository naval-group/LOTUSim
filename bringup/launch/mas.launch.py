# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from pathlib import Path  # Import the Path class for secure file path manipulation
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
        launch_arguments={'gz_args': '-v {} '.format(json_data['params']['verbose']) + os.path.join( # Reduce the verbose number to reduce the log messages
            pkg_project_description,
            'worlds',
            json_data['params']['world_name']
        )}.items(),
    )

    entity_management = Node(
        package='entity_management',
        executable='entity_management_service',
        parameters=[{'use_sim_time': True,
            }],
        output='screen'
    )

    simulation_control = Node(
        package='simulation_control',
        executable='simulation_control',
        parameters=[{'use_sim_time': True,
            }],
        output='screen'
    )

    # We create the list of spawn robots commands
    spawn_agents_cmds = []

    namespace_nb = 1

    for agent_config in json_data['agent_configs']:
        spawn_agents_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_project_bringup, 'launch', 'multi_agent_spawn.launch.py')),
                launch_arguments={'agent_config': str(agent_config),
                                  'ns_base': 'ns_main'
                                }.items())
        )
        namespace_nb = namespace_nb + 1

    # Create the launch description and populate
    agent_launch = LaunchDescription()
    
    for spawn_agent_cmd in spawn_agents_cmds:
        agent_launch.add_action(spawn_agent_cmd)

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc},
    #     ]
    # )

    # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'expand_gz_topic_names': True, # Enables the Bridge to apply ROS2 namespace on the Gazebo topics
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        simulation_control,
        entity_management,
        agent_launch,
        bridge,
        # robot_state_publisher,
        # rviz
    ])
