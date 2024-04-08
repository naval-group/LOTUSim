from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.descriptions import ParameterValue, ComposableNode

from launch_ros.actions import Node, ComposableNodeContainer

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('append_zone', default_value='false',
                          choices=['true', 'false'],
                          description='append utm zone to the child frame id in odom'),
    DeclareLaunchArgument('odom_child_frame_id', default_value='wamv/base_link',
                          description='odom child frame'),
    DeclareLaunchArgument('odom_parent_frame_id', default_value='wamv/odom',
                          description='odom parent frame'),
    DeclareLaunchArgument('tf_child_link_frame_id', default_value='wave_world/wamv_vessel',
                          description='tf child link to be used to get orientation'),
    DeclareLaunchArgument('tf_parent_link_frame_id', default_value='wave_world',
                          description='tf parent link to be used to get orientation'),
]


def generate_launch_description():
    container = ComposableNodeContainer(
        name='gps_to_odom_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='gps_to_odom',
                    plugin='gps_tools::UtmOdometryComponent',
                    name='gps_to_odom',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'append_zone': LaunchConfiguration('append_zone')},
                        {'odom_child_frame_id': LaunchConfiguration(
                            'odom_child_frame_id')},
                        {'odom_parent_frame_id': LaunchConfiguration(
                            'odom_parent_frame_id')},
                        {'tf_child_link_frame_id': LaunchConfiguration(
                            'tf_child_link_frame_id')},
                        {'tf_parent_link_frame_id': LaunchConfiguration(
                            'tf_parent_link_frame_id')},
                    ]
                ),
        ],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(container)
    return ld
