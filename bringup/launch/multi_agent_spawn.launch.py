#!/usr/bin/python
import os
import json
from pathlib import Path
from re import A
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# 
# This launch file is called by mas.launch.py and calls multiple agent_spawn.launch.py
# It takes in entry the name of an agent_config file and calls the launch file on every agent
# It is responsible of assigning incremental namespaces to each spawn of agents
# It parses the JSON from the config files
#

agent_launch = LaunchDescription()
agent_fullnames = []

def render_xacro(context: LaunchContext, agent_config, ns_base, configure_on_startup_default):
    # Transforms the LaunchConfiguration variables into str
    agent_config_str = context.perform_substitution(agent_config)
    ns_base_str:str = context.perform_substitution(ns_base)
    configure_on_startup_default_str:str = context.perform_substitution(configure_on_startup_default)

    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_description = get_package_share_directory('assets')

    CONFIG_FILE_PATH = os.path.join(pkg_project_bringup, 'config', agent_config_str)

    # Open the JSON configuration file in read mode
    with open(CONFIG_FILE_PATH, "r") as file:
        agent_config_json_data = json.load(file)
    
    ns_index = 1

    for agent_type in agent_config_json_data.keys():
        # Only the further end of the sdf file path is provided because the full path depends on the machine
        sdf_filename  =  os.path.join('models', agent_type, 'model.sdf')
        for agent_data in agent_config_json_data[agent_type]:
            namespace = ns_base_str + str("{:03d}".format(ns_index)) # Format as <ns>00X

            xacro_filepath = os.path.join(pkg_project_description, 'models', agent_type, 'model.xacro')
            sdf_file = ""
            if(os.path.isfile(xacro_filepath)):
                doc = xacro.process_file(xacro_filepath,
                                        mappings={
                                                'namespace': namespace,
                                                'name': agent_data['name']})
                sdf_file = doc.toxml()
                print("sdf_file generated: " + sdf_file)

            if(not "configure_on_startup" in agent_data):
                agent_data.update({'configure_on_startup': configure_on_startup_default_str})
            
            agent_data.update(
                {'sdf_file': sdf_file,
                 'sdf_filename': sdf_filename, 
                 'ns': namespace
                }
            )
            print('Agent data: ' + str(agent_data))
            agent_launch.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_project_bringup, 'launch', 'agent_spawn.launch.py')),
                    launch_arguments = agent_data.items()
                )
            )
            ns_index = ns_index + 1

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent_config', default_value='default_agent_config.json'),
        DeclareLaunchArgument('ns_base', default_value='ns_default'),
        DeclareLaunchArgument('configure_on_startup_default'),
        OpaqueFunction(function=render_xacro, args=[LaunchConfiguration('agent_config'),LaunchConfiguration('ns_base'), LaunchConfiguration('configure_on_startup_default')]),
        agent_launch,
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()