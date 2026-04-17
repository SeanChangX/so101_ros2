# Copyright 2025
# SPDX-License-Identifier: MIT

"""Run move_group in the /follower namespace (matches ros2_control + TF without prefix)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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

    # Use real booleans (not LaunchConfiguration strings) so move_group gets bool params.
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'capabilities': ParameterValue(LaunchConfiguration('capabilities'), value_type=str),
        'disable_capabilities': ParameterValue(LaunchConfiguration('disable_capabilities'), value_type=str),
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        # Absolute topic: namespaced move_group must not subscribe to /follower/move_group/joint_states.
        'planning_scene_monitor_options': {
            'joint_state_topic': '/follower/joint_states',
        },
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument('capabilities', default_value=''),
            DeclareLaunchArgument('disable_capabilities', default_value=''),
            DeclareLaunchArgument(
                'publish_monitored_planning_scene',
                default_value='true',
                description='Unused; publishing flags are fixed True in this launch.',
            ),
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                namespace='follower',
                output='screen',
                parameters=[moveit_config.to_dict(), move_group_configuration],
            ),
        ]
    )
