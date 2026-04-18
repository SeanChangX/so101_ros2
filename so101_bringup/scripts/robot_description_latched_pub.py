#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: MIT

"""Publish robot URDF as std_msgs/String for tools that expect /robot_description."""

import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    node = Node("robot_description_latched_pub")
    xacro_path = node.declare_parameter("xacro_path", "").get_parameter_value().string_value
    topic = node.declare_parameter("topic", "/robot_description").get_parameter_value().string_value
    xacro_mappings = node.declare_parameter("xacro_mappings", ["mode:=real"]).get_parameter_value().string_array_value

    if not xacro_path:
        node.get_logger().error("Parameter xacro_path is empty")
        rclpy.shutdown()
        return 1

    cmd = ["xacro", xacro_path]
    cmd.extend(xacro_mappings)
    try:
        proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError:
        node.get_logger().error("xacro not found; install ros-${ROS_DISTRO}-xacro")
        rclpy.shutdown()
        return 1
    except subprocess.CalledProcessError as e:
        node.get_logger().error(f"xacro failed: {e}\n{e.stderr}")
        rclpy.shutdown()
        return 1

    urdf = proc.stdout
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(String, topic, qos)
    pub.publish(String(data=urdf))
    node.get_logger().info("Published URDF (%d bytes) on %s (latched)" % (len(urdf), topic))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
