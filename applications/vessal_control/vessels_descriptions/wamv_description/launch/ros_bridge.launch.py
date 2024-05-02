import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/AIS@liquidai_msgs/msg/AISArray@gz_liquidai_plugins_msgs.msgs.AISArray',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/magnetometer@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer',
            '/gps@liquidai_msgs/msg/GPS@ignition.msgs.NavSat',
            'lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/wamv/joint/right_engine_propeller_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/wamv/joint/left_engine_propeller_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/lrauv/joint/propeller_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/lrauv/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState',
            '/model/rexrov/joint/propeller0_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/rexrov/joint/propeller1_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/rexrov/joint/propeller2_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/rexrov/joint/propeller3_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/rexrov/joint/propeller4_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/model/rexrov/joint/propeller5_joint/cmd_thrust@liquidai_msgs/msg/FloatStamped@ignition.msgs.Double',
            '/world/vessel_wave/model/wave_world/model/rexrov/link/front_camera/sensor/front_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/front_left_camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/front_right_camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/middle_right_camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/vessel_test/model/wamv/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/model/wave_world/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            
        ],
        remappings=[
            ('/model/wamv/joint/left_engine_propeller_joint/cmd_thrust',
             "/wamv/thrusters/id_0/input"),
            ('/model/wamv/joint/right_engine_propeller_joint/cmd_thrust',
             "/wamv/thrusters/id_1/input"),
            ('/model/lrauv/joint/propeller_joint/cmd_thrust',
             "/lrauv/propeller/input"),
            ('/model/rexrov/joint/propeller0_joint/cmd_thrust',
             "/rexrov/propeller0/input"),
            ('/model/rexrov/joint/propeller1_joint/cmd_thrust',
             "/rexrov/propeller1/input"),
            ('/model/rexrov/joint/propeller2_joint/cmd_thrust',
             "/rexrov/propeller2/input"),
            ('/model/rexrov/joint/propeller3_joint/cmd_thrust',
             "/rexrov/propeller3/input"),
            ('/model/rexrov/joint/propeller4_joint/cmd_thrust',
             "/rexrov/propeller4/input"),
            ( '/model/rexrov/joint/propeller5_joint/cmd_thrust',
             "/rexrov/propeller5/input"),
            ('/world/vessel_wave/model/wave_world/model/rexrov/link/front_camera/sensor/front_camera/image',
             "rexrov/camera/image"),
            ('/model/wave_world/pose', '/tf'),
            ('/world/vessel_test/model/wamv/joint_state', '/joint_state')
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
    ])
