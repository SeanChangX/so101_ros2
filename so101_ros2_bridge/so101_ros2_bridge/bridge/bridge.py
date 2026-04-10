#!/usr/bin/env python3

# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import math
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils.core import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

from lerobot.robots.so101_follower import SO101Follower
from lerobot.teleoperators.so101_leader import SO101Leader

from so101_ros2_bridge import CALIBRATION_BASE_DIR, DEFAULT_URDF_CALIBRATION_FILE
from so101_ros2_bridge.bridge.registry import ROBOT_FACTORY_REGISTRY
from so101_ros2_bridge.utils.joint_calibration import (
    apply_raw_to_urdf,
    apply_urdf_to_raw,
    build_identity_calibration,
    load_joint_calibration,
)


class SO101ROS2Bridge(Node, ABC):
    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self, node_name='so101_ros2_bridge'):
        super().__init__(node_name)
        params = self.read_parameters()
        self.use_degrees = params['use_degrees']
        self.enable_urdf_calibration = params['enable_urdf_calibration']
        self.urdf_calibration_file = Path(params['urdf_calibration_file'])
        self.urdf_calibration = build_identity_calibration(self.JOINT_NAMES)
        if self.enable_urdf_calibration:
            self.urdf_calibration = load_joint_calibration(
                self.urdf_calibration_file,
                self.JOINT_NAMES,
                logger=self.get_logger(),
            )
            self.get_logger().info(
                'URDF calibration enabled from '
                f'"{self.urdf_calibration_file}".'
            )
        else:
            self.get_logger().info('URDF calibration disabled. Using raw joint mapping.')

        # Initialize watchdog at the background
        self._is_alive = True
        self._watchdog_interval = 0.01  # 100 hz whatchdog
        self._timeout = 5.0
        self._alive_thread = threading.Thread(target=self._alive, daemon=True)

        # Pre-allocate the message and its internal lists
        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self.JOINT_NAMES
        self._positions = [0.0] * len(self.JOINT_NAMES)
        self._velocities = [0.0] * len(self.JOINT_NAMES)

        rate = params.get('publish_rate', 30.0)

        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # Register the robot type in the factory registry
        if not ROBOT_FACTORY_REGISTRY:
            raise RuntimeError(
                'ROBOT_FACTORY_REGISTRY is empty. Ensure robot types are registered.'
            )

        self.declare_parameter('type', 'follower')
        robot_type = self.get_parameter('type').get_parameter_value().string_value
        self.get_logger().info(f'Initializing SO101 ROS2 Bridge for robot type: {robot_type}')
        factory_fn = ROBOT_FACTORY_REGISTRY.get(robot_type)
        if factory_fn is None:
            raise ValueError(f"Robot type '{robot_type}' not registered.")

        # Initialize robot
        self.robot = factory_fn(params)

        try:
            self.robot.connect(calibrate=False)
        except Exception as e:
            raise RuntimeError(f'Failed to connect to SO101 robot: {e}') from e

        self.last_positions = None
        self.last_time = self.get_clock().now()

        # Start alive
        self._alive_thread.start()

    @abstractmethod
    def read_parameters(self) -> dict:
        """Reads parameters from the ROS2 parameter server."""

    @abstractmethod
    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """

    # In your publish_joint_states method:
    def publish_joint_states(self):
        try:
            current_time = self.get_clock().now()
            obs = self.get_joints_states()

            # Update the pre-allocated lists instead of creating new ones
            for i, joint in enumerate(self.JOINT_NAMES):
                raw_pos = self.observation_to_radians(joint, obs.get(f'{joint}.pos', 0.0))
                if self.enable_urdf_calibration:
                    self._positions[i] = apply_raw_to_urdf(self.urdf_calibration, joint, raw_pos)
                else:
                    self._positions[i] = raw_pos

            if self.last_positions is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 1e-6:  # More robust check against zero
                    for i in range(len(self.JOINT_NAMES)):
                        self._velocities[i] = (self._positions[i] - self.last_positions[i]) / dt

            # Update and publish the pre-allocated message
            self._joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self._joint_state_msg.position = self._positions
            self._joint_state_msg.velocity = self._velocities
            self.joint_pub.publish(self._joint_state_msg)

            # Store a copy for the next velocity calculation
            self.last_positions = list(self._positions)  # or self._positions[:]
            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f'Failed to publish joint states: {e}')

    def _alive(self):
        """
        Watchdog thread running at a fixed rate. Monitors rclpy.ok() and ROS time.
        Triggers shutdown if ROS crashes or time stops progressing.
        """
        interval = self._watchdog_interval
        next_tick = time.monotonic()

        while self._is_alive:
            # Check ROS system state
            if not rclpy.ok():
                self.get_logger().error(
                    'ROS shutdown detected (rclpy.ok() == False). Triggering shutdown.'
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Check time progress
            now = self.get_clock().now()
            elapsed = (now - self.last_time).nanoseconds / 1e9
            if elapsed > self._timeout:
                self.get_logger().error(
                    f'ROS time not updating for {elapsed:.1f}s — triggering shutdown.'
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Sleep until next tick
            next_tick += interval
            sleep_time = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_time)

    def shutdown_hook(self):
        if not self._is_alive:
            return  # Prevent double cleanup

        self._is_alive = False
        try:
            self.get_logger().info('Shutting down, disconnecting robot...')
            if hasattr(self, 'robot'):
                (
                    self.robot.disconnect()
                    if self.robot.is_connected
                    else self.get_logger().warn('Robot is already disconnected')
                )
        except Exception as e:
            self.get_logger().warn(f'Exception during shutdown: {e}')

    def radians_to_normalized(self, joint_name: str, rad: float) -> float:
        """
        converts a command in radians from MoveIt to the format expected by the SO101 API.
        """
        if joint_name != 'gripper' and self.use_degrees:
            return math.degrees(rad)
        else:
            # Convert radians to normalized range [-100, 100] for other joints and gripper to normalized [0,100]
            normalized = (rad / math.pi) * 100.0
            return normalized

    def observation_to_radians(self, joint_name: str, observation_value: float) -> float:
        if joint_name == 'gripper':
            return (observation_value / 100.0) * math.pi
        if self.use_degrees:
            return math.radians(observation_value)
        # unormalized range [-100, 100] to radians
        return (observation_value / 100.0) * math.pi


class FollowerBridge(SO101ROS2Bridge):
    def __init__(self):
        super().__init__('so101_follower_ros2_bridge')

        # Type hint of the robot
        if not isinstance(self.robot, SO101Follower):
            raise TypeError(f'Expected robot type SO101Follower, got {type(self.robot).__name__}')

        # QoS for joint command subscriber
        qos_joint_cmds = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # only latest command matters
            reliability=QoSReliabilityPolicy.RELIABLE,  # ensure delivery
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe to commands from the ros2_control hardware interface bridge
        self.create_subscription(
            Float64MultiArray,
            'joint_commands',  # This topic should match the publisher in the C++ bridge
            self.command_callback,
            qos_joint_cmds,
        )
        self._servo_bus = self._resolve_servo_bus()
        self._torque_enabled = True
        self.create_service(SetBool, 'set_torque', self._set_torque_cb)
        self.get_logger().info('SO101 Follower ROS2 Bridge initialized.')

    def _resolve_servo_bus(self):
        """Find the low-level Feetech bus from the lerobot robot object."""
        for attr in ('bus', 'arm', 'arm_bus', 'follower_bus'):
            bus = getattr(self.robot, attr, None)
            if bus is not None:
                return bus
        for attr in ('follower_arms', 'arms'):
            d = getattr(self.robot, attr, None)
            if isinstance(d, dict) and d:
                return next(iter(d.values()))
        self.get_logger().warn(
            'Could not locate servo bus on robot object; set_torque will be unavailable')
        return None

    def _set_torque_cb(self, request: SetBool.Request, response: SetBool.Response):
        label = 'enabled' if request.data else 'disabled'
        if self._servo_bus is None:
            response.success = False
            response.message = 'Servo bus not found on robot object'
            return response
        try:
            if request.data:
                self._servo_bus.enable_torque()
                self._torque_enabled = True
            else:
                self._torque_enabled = False
                self._servo_bus.disable_torque()
            response.success = True
            response.message = f'Torque {label}'
            self.get_logger().info(f'Torque {label}')
        except Exception as e:
            self.get_logger().error(f'set_torque failed: {e}')
            response.success = False
            response.message = str(e)
        return response

    def read_parameters(self) -> dict:
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('id', 'Tzili')
        self.declare_parameter('calibration_dir', str(CALIBRATION_BASE_DIR))
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('enable_urdf_calibration', False)
        self.declare_parameter('urdf_calibration_file', str(DEFAULT_URDF_CALIBRATION_FILE))
        self.declare_parameter('max_relative_target', 0)
        self.declare_parameter('disable_torque_on_disconnect', True)
        self.declare_parameter('publish_rate', 30.0)

        max_relative_target_raw = self.get_parameter('max_relative_target').value
        if isinstance(max_relative_target_raw, bool):
            raise ValueError(
                "Invalid 'max_relative_target' type: bool. Use a numeric value (int/float)."
            )
        try:
            max_relative_target = float(max_relative_target_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"Invalid 'max_relative_target' value: {max_relative_target_raw!r}. "
                'Use 0 (disable) or a positive numeric value.'
            ) from exc
        max_relative_target = max_relative_target if max_relative_target != 0.0 else None

        return {
            'port': self.get_parameter('port').get_parameter_value().string_value,
            'id': self.get_parameter('id').get_parameter_value().string_value,
            'calibration_dir': Path(
                self.get_parameter('calibration_dir').get_parameter_value().string_value
            ),
            'use_degrees': self.get_parameter('use_degrees').get_parameter_value().bool_value,
            'enable_urdf_calibration': (
                self.get_parameter('enable_urdf_calibration').get_parameter_value().bool_value
            ),
            'urdf_calibration_file': (
                self.get_parameter('urdf_calibration_file').get_parameter_value().string_value
            ),
            'max_relative_target': max_relative_target,
            'disable_torque_on_disconnect': (
                self.get_parameter('disable_torque_on_disconnect').get_parameter_value().bool_value
            ),
            'publish_rate': self.get_parameter('publish_rate').get_parameter_value().double_value,
        }

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_observation()

    def command_callback(self, msg: Float64MultiArray):
        """
        Receives joint command goals in radians and sends them to the robot.
        """
        if not self._torque_enabled:
            return

        if len(msg.data) != len(self.JOINT_NAMES):
            self.get_logger().error(
                f'Received command with {len(msg.data)} joints, but expected {len(self.JOINT_NAMES)}.'
            )
            return

        target_positions = {}
        for i, joint in enumerate(self.JOINT_NAMES):
            desired_pos = msg.data[i]
            if self.enable_urdf_calibration:
                desired_pos = apply_urdf_to_raw(self.urdf_calibration, joint, desired_pos)
            target_positions[f'{joint}.pos'] = self.radians_to_normalized(joint, desired_pos)

        try:
            self.robot.send_action(target_positions)
        except Exception as e:
            self.get_logger().error(f'Failed to send commands to robot: {e}')


class LeaderBridge(SO101ROS2Bridge):
    def __init__(self):
        super().__init__('so101_leader_ros2_bridge')

        # Type hint of the robot
        if not isinstance(self.robot, SO101Leader):
            raise TypeError(f'Expected robot type SO101Leader, got {type(self.robot).__name__}')
        self.get_logger().info('SO101 Leader ROS2 Bridge initialized.')

    def read_parameters(self) -> dict:
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('id', 'Gili')
        self.declare_parameter('calibration_dir', str(CALIBRATION_BASE_DIR))
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('enable_urdf_calibration', False)
        self.declare_parameter('urdf_calibration_file', str(DEFAULT_URDF_CALIBRATION_FILE))
        self.declare_parameter('publish_rate', 30.0)

        return {
            'port': self.get_parameter('port').get_parameter_value().string_value,
            'id': self.get_parameter('id').get_parameter_value().string_value,
            'calibration_dir': Path(
                self.get_parameter('calibration_dir').get_parameter_value().string_value
            ),
            'use_degrees': self.get_parameter('use_degrees').get_parameter_value().bool_value,
            'enable_urdf_calibration': (
                self.get_parameter('enable_urdf_calibration').get_parameter_value().bool_value
            ),
            'urdf_calibration_file': (
                self.get_parameter('urdf_calibration_file').get_parameter_value().string_value
            ),
            'publish_rate': self.get_parameter('publish_rate').get_parameter_value().double_value,
        }

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_action()
