# Unified launch: follower robot + leader teleop component + web teleop + rosbridge
#
# Single command replaces:
#   ros2 launch so101_bringup so101_robot.launch.py
#   ros2 launch so101_teleop  so101_leader_teleop.launch.py
#   ros2 launch so101_web_teleop so101_web_teleop.launch.py
#
# Usage:
#   ros2 launch so101_web_teleop so101_web_teleoperate.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = get_package_share_directory('so101_bringup')
    description_pkg = get_package_share_directory('so101_description')
    teleop_pkg = get_package_share_directory('so101_teleop')

    # ── arguments ──
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    http_port_arg = DeclareLaunchArgument('http_port', default_value='8080')
    ws_port_arg = DeclareLaunchArgument('ws_port', default_value='9090')
    ssl_cert_arg = DeclareLaunchArgument('ssl_cert', default_value='')
    ssl_key_arg = DeclareLaunchArgument('ssl_key', default_value='')

    model = LaunchConfiguration('model')
    http_port = LaunchConfiguration('http_port')
    ws_port = LaunchConfiguration('ws_port')
    ssl_cert = LaunchConfiguration('ssl_cert')
    ssl_key = LaunchConfiguration('ssl_key')

    # ── 1. Follower robot (hardware bridge + controllers) ──
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'follower.launch.py')
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': 'false',
        }.items(),
    )

    # ── 2. Leader teleop component (reads /leader/joint_states, commands follower) ──
    leader_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, 'launch', 'so101_leader_teleop.launch.py')
        ),
        launch_arguments={'mode': 'real'}.items(),
    )
    delayed_leader = TimerAction(period=10.0, actions=[leader_teleop_launch])

    # ── 3. rosbridge WebSocket ──
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': ws_port,
            'ssl': False,
            'certfile': ssl_cert,
            'keyfile': ssl_key,
            'authenticate': False,
            'max_message_size': 10000000,
        }],
    )

    # ── 4. Web teleop node (HTTP server + phone control) ──
    config = PathJoinSubstitution([
        FindPackageShare('so101_web_teleop'), 'config', 'so101_web_teleop.yaml',
    ])
    web_node = Node(
        package='so101_web_teleop',
        executable='web_teleop_node',
        name='web_teleop_node',
        output='screen',
        parameters=[config, {
            'http_port': http_port,
            'ssl_cert': ssl_cert,
            'ssl_key': ssl_key,
        }],
    )

    return LaunchDescription([
        model_arg,
        http_port_arg,
        ws_port_arg,
        ssl_cert_arg,
        ssl_key_arg,
        follower_launch,
        delayed_leader,
        rosbridge_node,
        web_node,
    ])
