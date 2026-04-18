#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: MIT

"""
WebXR phone teleoperation for SO-101 follower using SpesRobotics teleop + JacobiRobotROS.

Requires: pip install 'teleop[utils]' matplotlib (same Python as rclpy).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import numpy as np

try:
    import rclpy
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionClient
    from std_msgs.msg import String
except ImportError as e:
    raise SystemExit("ROS 2 is not available. Source your ROS 2 setup.bash before running.") from e

try:
    from teleop import Teleop
except ImportError as e:
    exe = getattr(sys, "executable", "python3")
    raise SystemExit(
        "Base package `teleop` failed to import (interpreter: %s).\n"
        "Install with: %s -m pip install teleop\n"
        "Use the same Python as ROS (see repo Docker image if applicable)."
        % (exe, exe)
    ) from e

try:
    from teleop.utils.jacobi_robot_ros import JacobiRobotROS
except ImportError as e:
    exe = getattr(sys, "executable", "python3")
    hint = ""
    if isinstance(e, ModuleNotFoundError) and getattr(e, "name", "") == "matplotlib":
        hint = "Try: %s -m pip install matplotlib\n" % (exe,)
    elif "matplotlib" in str(e):
        hint = "Try: %s -m pip install matplotlib\n" % (exe,)
    raise SystemExit(
        "WebXR IK failed loading JacobiRobotROS (interpreter: %s).\n"
        "Cause: %s: %s\n"
        "%s"
        "Typical fix: %s -m pip install 'teleop[utils]' matplotlib\n"
        "Note: `import teleop` can succeed while JacobiRobotROS still pulls extra deps."
        % (exe, type(e).__name__, e, hint, exe)
    ) from e

SO101_ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

DEFAULT_EE_LINK = "gripper_frame_link"
DEFAULT_RD_TOPIC = "/robot_description"
DEFAULT_JS_TOPIC = "/follower/joint_states"
DEFAULT_TRAJ_TOPIC = "/follower/arm_controller/joint_trajectory"
DEFAULT_GRIPPER_ACTION = "/follower/gripper_controller/gripper_cmd"


def to_pose_cmd(pose):
    try:
        pose_cmd = np.asarray(pose, dtype=np.float64, order="C").copy()
    except Exception:
        return None
    if pose_cmd.size == 0:
        return None
    return pose_cmd


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 WebXR teleop (SpesRobotics teleop + IK).")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="HTTPS server bind address")
    parser.add_argument("--port", type=int, default=4443, help="HTTPS server port")
    parser.add_argument(
        "--natural-position",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        help="Natural position of the phone",
    )
    parser.add_argument(
        "--natural-orientation",
        nargs=3,
        type=float,
        default=[0.0, -45.0, 0.0],
        help="Natural orientation of the phone (degrees)",
    )
    parser.add_argument("--ee-link", type=str, default=DEFAULT_EE_LINK, help="End-effector link in URDF")
    parser.add_argument(
        "--robot-description-topic",
        type=str,
        default=DEFAULT_RD_TOPIC,
        help="std_msgs/String topic carrying URDF XML",
    )
    parser.add_argument(
        "--joint-state-topic",
        type=str,
        default=DEFAULT_JS_TOPIC,
        help="sensor_msgs/JointState topic",
    )
    parser.add_argument(
        "--joint-trajectory-topic",
        type=str,
        default=DEFAULT_TRAJ_TOPIC,
        help="trajectory_msgs/JointTrajectory command topic for arm_controller",
    )
    parser.add_argument(
        "--joint-names",
        nargs="+",
        default=SO101_ARM_JOINTS,
        help="Arm joint names (must match URDF and controller)",
    )
    parser.add_argument(
        "--servo-dt",
        type=float,
        default=0.05,
        help="JointTrajectory point time_from_start (s).",
    )
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=18.0,
        help="Fixed ROS-side command send rate in Hz using the latest WebXR pose.",
    )
    parser.add_argument(
        "--gripper-action-name",
        type=str,
        default=DEFAULT_GRIPPER_ACTION,
        help="control_msgs/GripperCommand action name.",
    )
    parser.add_argument(
        "--gripper-min-position",
        type=float,
        default=0.0,
        help="Gripper min position in action units (typically radians).",
    )
    parser.add_argument(
        "--gripper-max-position",
        type=float,
        default=float(np.pi),
        help="Gripper max position in action units (typically radians).",
    )
    parser.add_argument(
        "--gripper-max-effort",
        type=float,
        default=10.0,
        help="Gripper max_effort sent with GripperCommand goals.",
    )
    parser.add_argument(
        "--gripper-deadband",
        type=float,
        default=0.01,
        help="Minimum change in gripper position before sending a new goal.",
    )
    parser.add_argument(
        "--gripper-rate-hz",
        type=float,
        default=12.0,
        help="Maximum gripper goal send rate.",
    )
    args, _unknown = parser.parse_known_args()
    rclpy.init()

    node = rclpy.create_node("so101_webxr_teleop")
    gripper_echo_publisher = node.create_publisher(String, "/gripper_command", 1)
    gripper_action_client = ActionClient(node, GripperCommand, args.gripper_action_name)

    teleop = Teleop(
        host=args.host,
        port=args.port,
        natural_phone_orientation_euler=args.natural_orientation,
        natural_phone_position=args.natural_position,
    )
    robot = JacobiRobotROS(
        node,
        robot_description_topic=args.robot_description_topic,
        ee_link=args.ee_link,
        joint_names=args.joint_names,
        joint_state_topic=args.joint_state_topic,
        position_command_topic=args.joint_trajectory_topic,
    )
    robot.reset_joint_states()

    startup_wait_s = 6.0
    wait_deadline = time.monotonic() + startup_wait_s
    while rclpy.ok() and not robot.are_joint_states_received() and time.monotonic() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not robot.are_joint_states_received():
        node.get_logger().warn(
            "Joint states not received after %.1fs; WebXR may start with unstable pose tracking." % startup_wait_s
        )

    servo_dt = args.servo_dt
    command_rate_hz = max(args.command_rate_hz, 1.0)
    command_period_s = 1.0 / command_rate_hz
    gripper_send_period_s = 1.0 / max(args.gripper_rate_hz, 1.0)
    gripper_min = min(args.gripper_min_position, args.gripper_max_position)
    gripper_max = max(args.gripper_min_position, args.gripper_max_position)
    gripper_span = max(gripper_max - gripper_min, 1e-6)
    gripper_deadband = max(args.gripper_deadband, 0.0)
    last_invalid_pose_log_mono = 0.0
    last_pose_resync_mono = 0.0
    last_gripper_server_warn_mono = 0.0
    last_gripper_send_mono = 0.0
    last_gripper_goal_position = None
    pose_lock = threading.Lock()
    latest_pose_cmd = None
    latest_gripper_norm = None
    move_enabled = False
    pending_gripper_goal_futures = []

    if robot.are_joint_states_received():
        ee_pose = robot.get_ee_pose()
        ee_pose_cmd = to_pose_cmd(ee_pose)
        if ee_pose_cmd is not None and np.isfinite(ee_pose_cmd).all():
            teleop.set_pose(ee_pose)
            latest_pose_cmd = ee_pose_cmd
        else:
            node.get_logger().warn("Initial EE pose is non-finite; waiting for first valid WebXR pose.")

    def parse_gripper_norm(raw):
        if raw is None:
            return None
        if isinstance(raw, bool):
            return 1.0 if raw else 0.0

        value = None
        if isinstance(raw, (int, float, np.integer, np.floating)):
            value = float(raw)
        elif isinstance(raw, str):
            lowered = raw.strip().lower()
            aliases = {
                "open": 1.0,
                "opened": 1.0,
                "release": 1.0,
                "close": 0.0,
                "closed": 0.0,
                "grab": 0.0,
            }
            if lowered in aliases:
                return aliases[lowered]
            try:
                value = float(lowered)
            except ValueError:
                return None
        else:
            return None

        if 0.0 <= value <= 1.0:
            return value
        if 0.0 <= value <= 100.0:
            return value / 100.0
        if gripper_min <= value <= gripper_max:
            return (value - gripper_min) / gripper_span
        return max(0.0, min(1.0, value))

    def teleop_pose_callback(pose, params):
        nonlocal last_invalid_pose_log_mono
        nonlocal last_pose_resync_mono
        nonlocal latest_pose_cmd
        nonlocal latest_gripper_norm
        nonlocal move_enabled

        gripper_raw = None
        for key in ("gripper", "gripper_slider", "grip"):
            if key in params:
                gripper_raw = params[key]
                break
        gripper_norm = parse_gripper_norm(gripper_raw)
        if gripper_norm is not None:
            with pose_lock:
                latest_gripper_norm = gripper_norm
            gripper_echo_publisher.publish(String(data=f"{gripper_norm:.4f}"))

        if not bool(params.get("move", False)):
            with pose_lock:
                move_enabled = False
                latest_pose_cmd = None
            return

        pose_cmd = to_pose_cmd(pose)
        if pose_cmd is None:
            now = time.monotonic()
            if now - last_invalid_pose_log_mono > 1.0:
                node.get_logger().warn("Skipping empty/invalid WebXR pose command")
                last_invalid_pose_log_mono = now
            return

        if not np.isfinite(pose_cmd).all():
            now = time.monotonic()
            if now - last_invalid_pose_log_mono > 1.0:
                node.get_logger().warn("Skipping non-finite WebXR pose command")
                last_invalid_pose_log_mono = now

            if now - last_pose_resync_mono > 1.0 and robot.are_joint_states_received():
                ee_pose_now = robot.get_ee_pose()
                ee_pose_now_cmd = to_pose_cmd(ee_pose_now)
                if ee_pose_now_cmd is not None and np.isfinite(ee_pose_now_cmd).all():
                    with pose_lock:
                        latest_pose_cmd = ee_pose_now_cmd
                    teleop.set_pose(ee_pose_now)
                    last_pose_resync_mono = now
                    node.get_logger().warn("WebXR pose was non-finite; re-synced from current EE pose")
            return

        with pose_lock:
            move_enabled = True
            latest_pose_cmd = pose_cmd

    def command_timer_callback():
        nonlocal last_gripper_goal_position
        nonlocal last_gripper_send_mono
        nonlocal last_gripper_server_warn_mono
        nonlocal latest_pose_cmd
        nonlocal latest_gripper_norm
        nonlocal move_enabled

        if not robot.are_joint_states_received():
            return

        with pose_lock:
            if not move_enabled or latest_pose_cmd is None:
                pose_to_send = None
            else:
                pose_to_send = latest_pose_cmd.copy()
            gripper_norm = latest_gripper_norm

        if pose_to_send is not None:
            robot.servo_to_pose(pose_to_send, servo_dt)

        if gripper_norm is None:
            return

        now = time.monotonic()
        if now - last_gripper_send_mono < gripper_send_period_s:
            return

        target_position = gripper_min + gripper_norm * gripper_span
        if (
            last_gripper_goal_position is not None
            and abs(target_position - last_gripper_goal_position) <= gripper_deadband
        ):
            return

        if not gripper_action_client.server_is_ready():
            if now - last_gripper_server_warn_mono > 2.0:
                node.get_logger().warn(f"Gripper action server '{args.gripper_action_name}' is not ready")
                last_gripper_server_warn_mono = now
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(target_position)
        goal_msg.command.max_effort = float(args.gripper_max_effort)
        goal_future = gripper_action_client.send_goal_async(goal_msg)
        pending_gripper_goal_futures.append(goal_future)
        pending_gripper_goal_futures[:] = [f for f in pending_gripper_goal_futures if not f.done()]
        last_gripper_goal_position = target_position
        last_gripper_send_mono = now

    teleop.subscribe(teleop_pose_callback)
    _command_timer = node.create_timer(command_period_s, command_timer_callback)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        teleop.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
