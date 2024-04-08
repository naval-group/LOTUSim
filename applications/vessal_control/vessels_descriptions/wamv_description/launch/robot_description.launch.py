
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.descriptions import ParameterValue

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]


def generate_launch_description():
    wamv_model_description = get_package_share_directory('wamv_description')
    xacro_file = PathJoinSubstitution([wamv_model_description,
                                       'urdf',
                                       'wamv_gazebo.urdf.xacro'])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ParameterValue(
                Command(['xacro', ' ', xacro_file, ' ', 'gazebo:=ignition']), value_type=str)},
            # {'frame_prefix': 'wave_world/'},
        ],
    )
    static_world_ned_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '1.57079', '--pitch', '0', '--roll', '3.14159',
                   '--frame-id', 'wave_world', '--child-frame-id', 'wave_world/world_ned_frame']
    )
    static_base_link_ned_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '1.57079', '--pitch', '0', '--roll',
                   '3.14159', '--frame-id', 'wamv/base_link', '--child-frame-id', 'wamv/ned_link']
    )
    # wamv_base_link_pub = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll',wq
    #                '0', '--frame-id', 'wave_world/wamv', '--child-frame-id', 'wave_world/wamv/base_link']
    # )
    utm_to_odom_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll',
                   '0', '--frame-id', 'utm', '--child-frame-id', 'wamv/odom']
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(static_world_ned_pub)
    ld.add_action(static_base_link_ned_pub)
    ld.add_action(utm_to_odom_pub)
    # ld.add_action(wamv_base_link_pub)
    return ld
