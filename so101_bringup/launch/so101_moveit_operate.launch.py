# Copyright 2025
# SPDX-License-Identifier: MIT

"""Follower hardware + MoveIt move_group + RViz (full stack entry point in bringup)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('so101_bringup')
    description_pkg = get_package_share_directory('so101_description')
    moveit_pkg = get_package_share_directory('so101_moveit_config')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    tf_prefix_mode_arg = DeclareLaunchArgument(
        'tf_prefix_mode',
        default_value='none',
        description='none: unprefixed TF for MoveIt (default). prefixed: follower/...',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Launch RViz2 with MoveIt after move_group is up.',
    )

    model = LaunchConfiguration('model')
    tf_prefix_mode = LaunchConfiguration('tf_prefix_mode')
    start_rviz = LaunchConfiguration('start_rviz')

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'follower.launch.py')
        ),
        launch_arguments={
            'model': model,
            'tf_prefix_mode': tf_prefix_mode,
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        ),
    )
    delayed_move_group = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[so101_moveit_operate] Starting move_group (delayed).'),
            move_group_launch,
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'moveit_rviz.launch.py')
        ),
    )
    delayed_rviz = GroupAction(
        condition=IfCondition(start_rviz),
        actions=[
            TimerAction(period=9.0, actions=[rviz_launch]),
        ],
    )

    return LaunchDescription([
        model_arg,
        tf_prefix_mode_arg,
        start_rviz_arg,
        follower_launch,
        delayed_move_group,
        delayed_rviz,
    ])
