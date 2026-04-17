# Copyright 2025
# SPDX-License-Identifier: MIT

"""RViz2 with MoveIt parameters and a saved layout (world frame, RobotModel, TF, MotionPlanning /follower)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = Path(get_package_share_directory('so101_moveit_config'))
    rviz_config = str(moveit_share / 'config' / 'so101_moveit_demo.rviz')

    desc_share = Path(get_package_share_directory('so101_description'))
    urdf_path = str(desc_share / 'urdf' / 'so101_new_calib.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('so101_new_calib', package_name='so101_moveit_config')
        .robot_description(file_path=urdf_path, mappings={'mode': 'real'})
        .robot_description_semantic(file_path='config/so101_new_calib.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'], load_all=False)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .to_moveit_configs()
    )

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=rviz_parameters,
                arguments=['-d', rviz_config],
            ),
        ]
    )
