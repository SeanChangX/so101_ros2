# Copyright 2026
# SPDX-License-Identifier: MIT

"""Follower stack + latched /robot_description + SpesRobotics WebXR teleop (phone pose to arm IK)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    bringup_pkg = get_package_share_directory("so101_bringup")
    description_pkg = get_package_share_directory("so101_description")
    default_model = os.path.join(description_pkg, "urdf", "so101_new_calib.urdf.xacro")

    model = context.launch_configurations.get("model", default_model)
    webxr_host = context.launch_configurations.get("webxr_host", "0.0.0.0")
    webxr_port = context.launch_configurations.get("webxr_port", "4443")
    try:
        delay_s = float(context.launch_configurations.get("teleop_start_delay", "10.0"))
    except ValueError:
        delay_s = 10.0

    servo_dt = context.launch_configurations.get("servo_dt", "0.05")
    command_rate_hz = context.launch_configurations.get("command_rate_hz", "18.0")
    gripper_action_name = context.launch_configurations.get(
        "gripper_action_name", "/follower/gripper_controller/gripper_cmd"
    )
    gripper_min_position = context.launch_configurations.get("gripper_min_position", "0.0")
    gripper_max_position = context.launch_configurations.get("gripper_max_position", "3.141592653589793")
    gripper_max_effort = context.launch_configurations.get("gripper_max_effort", "10.0")
    gripper_deadband = context.launch_configurations.get("gripper_deadband", "0.01")
    gripper_rate_hz = context.launch_configurations.get("gripper_rate_hz", "12.0")

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "include", "follower.launch.py")
        ),
        launch_arguments={
            "model": model,
            "tf_prefix_mode": context.launch_configurations.get("tf_prefix_mode", "none"),
        }.items(),
    )

    robot_description_pub = Node(
        package="so101_bringup",
        executable="robot_description_latched_pub.py",
        name="robot_description_latched_pub",
        output="screen",
        parameters=[
            {
                "xacro_path": model,
                "topic": "/robot_description",
                "xacro_mappings": ["mode:=real"],
            }
        ],
    )

    webxr_node = Node(
        package="so101_bringup",
        executable="so101_webxr_teleop.py",
        name="so101_webxr_teleop",
        output="screen",
        arguments=[
            "--host",
            webxr_host,
            "--port",
            webxr_port,
            "--servo-dt",
            servo_dt,
            "--command-rate-hz",
            command_rate_hz,
            "--gripper-action-name",
            gripper_action_name,
            "--gripper-min-position",
            gripper_min_position,
            "--gripper-max-position",
            gripper_max_position,
            "--gripper-max-effort",
            gripper_max_effort,
            "--gripper-deadband",
            gripper_deadband,
            "--gripper-rate-hz",
            gripper_rate_hz,
        ],
    )

    delayed_webxr = TimerAction(
        period=delay_s,
        actions=[
            LogInfo(
                msg=(
                    "[so101_webxr_teleop] Starting WebXR node. "
                    "On a WebXR-capable phone: https://<this-pc-ip>:%s - "
                    "requires: pip install 'teleop[utils]' matplotlib in the ROS Python environment."
                )
                % webxr_port
            ),
            webxr_node,
        ],
    )

    return [follower_launch, robot_description_pub, delayed_webxr]


def generate_launch_description():
    description_pkg = get_package_share_directory("so101_description")
    default_model = os.path.join(description_pkg, "urdf", "so101_new_calib.urdf.xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument(
                "tf_prefix_mode",
                default_value="none",
                description="Match MoveIt follower stack: none = unprefixed TF link names.",
            ),
            DeclareLaunchArgument(
                "webxr_host",
                default_value="0.0.0.0",
                description="Bind address for teleop HTTPS server.",
            ),
            DeclareLaunchArgument("webxr_port", default_value="4443"),
            DeclareLaunchArgument(
                "teleop_start_delay",
                default_value="10.0",
                description="Seconds before starting WebXR node (allow ros2_control spawners).",
            ),
            DeclareLaunchArgument(
                "servo_dt",
                default_value="0.05",
                description="JointTrajectory point duration (seconds) for each servo_to_pose.",
            ),
            DeclareLaunchArgument(
                "command_rate_hz",
                default_value="18.0",
                description="Fixed ROS-side command send rate in Hz using the latest WebXR pose.",
            ),
            DeclareLaunchArgument(
                "gripper_action_name",
                default_value="/follower/gripper_controller/gripper_cmd",
                description="control_msgs/GripperCommand action name.",
            ),
            DeclareLaunchArgument(
                "gripper_min_position",
                default_value="0.0",
                description="Gripper min position in action units.",
            ),
            DeclareLaunchArgument(
                "gripper_max_position",
                default_value="3.141592653589793",
                description="Gripper max position in action units.",
            ),
            DeclareLaunchArgument(
                "gripper_max_effort",
                default_value="10.0",
                description="Gripper max_effort sent with goals.",
            ),
            DeclareLaunchArgument(
                "gripper_deadband",
                default_value="0.01",
                description="Minimum position delta before sending new gripper goal.",
            ),
            DeclareLaunchArgument(
                "gripper_rate_hz",
                default_value="12.0",
                description="Maximum gripper goal send rate.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
