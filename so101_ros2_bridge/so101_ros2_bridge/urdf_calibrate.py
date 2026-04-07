#!/usr/bin/env python3

# Copyright 2026 nimiCurtis
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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import annotations

import argparse
import math
import threading
from pathlib import Path
from statistics import median
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_ros2_bridge.utils.joint_calibration import save_joint_calibration

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]

URDF_LIMITS_RAD = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}


def parse_joint_values(entries: List[str], value_name: str) -> Dict[str, float]:
    parsed: Dict[str, float] = {}
    for entry in entries:
        if '=' not in entry:
            raise ValueError(f'Invalid {value_name} "{entry}". Expected format: joint=value.')
        joint_name, raw_value = entry.split('=', 1)
        joint_name = joint_name.strip()
        if joint_name not in JOINT_NAMES:
            raise ValueError(f'Unknown joint "{joint_name}" in {value_name}.')
        parsed[joint_name] = float(raw_value.strip())
    return parsed


def wait_enter(prompt: str) -> bool:
    print('')
    print(prompt)
    try:
        input('Press ENTER to capture (Ctrl+C to cancel): ')
        return True
    except (EOFError, KeyboardInterrupt):
        return False


class JointStateCapture(Node):
    def __init__(self, topic: str):
        super().__init__('so101_urdf_calibrate')
        self.latest: Dict[str, float] = {}
        self.subscription = self.create_subscription(JointState, topic, self.callback, 10)

    def callback(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos


def wait_for_joint_state(node: JointStateCapture, timeout_sec: float) -> bool:
    end_time = node.get_clock().now().nanoseconds / 1e9 + timeout_sec
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(joint in node.latest for joint in JOINT_NAMES):
            return True
        now = node.get_clock().now().nanoseconds / 1e9
        if now >= end_time:
            return False
    return False


def collect_samples(
    node: JointStateCapture,
    sample_count: int,
    spin_timeout_sec: float,
) -> Dict[str, List[float]]:
    samples = {joint: [] for joint in JOINT_NAMES}
    while rclpy.ok() and len(samples[JOINT_NAMES[0]]) < sample_count:
        rclpy.spin_once(node, timeout_sec=spin_timeout_sec)
        if not all(joint in node.latest for joint in JOINT_NAMES):
            continue
        for joint in JOINT_NAMES:
            samples[joint].append(node.latest[joint])
    return samples


def collect_joint_ranges_until_enter(
    node: JointStateCapture,
    spin_timeout_sec: float,
) -> Tuple[Dict[str, float], Dict[str, float], int, bool]:
    ranges_min = {joint: float('inf') for joint in JOINT_NAMES}
    ranges_max = {joint: float('-inf') for joint in JOINT_NAMES}
    sample_count = 0

    stop_event = threading.Event()
    cancelled = {'value': False}

    def _wait_for_enter():
        try:
            input('Recording... Press ENTER to finish range capture: ')
        except (EOFError, KeyboardInterrupt):
            cancelled['value'] = True
        finally:
            stop_event.set()

    input_thread = threading.Thread(target=_wait_for_enter, daemon=True)
    input_thread.start()

    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=spin_timeout_sec)
        if not all(joint in node.latest for joint in JOINT_NAMES):
            continue
        sample_count += 1
        for joint in JOINT_NAMES:
            value = node.latest[joint]
            if value < ranges_min[joint]:
                ranges_min[joint] = value
            if value > ranges_max[joint]:
                ranges_max[joint] = value

    return ranges_min, ranges_max, sample_count, cancelled['value']


def capture_pose_median(
    node: JointStateCapture,
    sample_count: int,
    spin_timeout_sec: float,
) -> Dict[str, float]:
    samples = collect_samples(
        node=node,
        sample_count=sample_count,
        spin_timeout_sec=spin_timeout_sec,
    )
    if len(samples[JOINT_NAMES[0]]) < sample_count:
        raise RuntimeError('Sampling interrupted before enough samples were collected.')
    return {joint: median(values) for joint, values in samples.items()}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Interactive URDF calibration for SO101 ROS 2 bridge. '
            'Capture the current joint pose and generate per-joint scale/offset JSON.'
        )
    )
    parser.add_argument(
        '--method',
        choices=('range', 'zero'),
        default='range',
        help='Calibration method. "range" captures mid/min/max (recommended).',
    )
    parser.add_argument(
        '--topic',
        default='/follower/joint_states_raw',
        help='JointState topic to capture from (default: /follower/joint_states_raw).',
    )
    parser.add_argument(
        '--output',
        default='so101_urdf_calibration.json',
        help='Output JSON file path.',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=120,
        help='MID pose median samples and minimum recommended range samples (default: 120).',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=8.0,
        help='Timeout while waiting for first valid JointState message (default: 8.0s).',
    )
    parser.add_argument(
        '--spin-timeout',
        type=float,
        default=0.02,
        help='Spin timeout used while sampling messages (default: 0.02s).',
    )
    parser.add_argument(
        '--scale',
        action='append',
        default=[],
        metavar='JOINT=VALUE',
        help='Override scale for specific joint. Example: --scale shoulder_lift=-1.0',
    )
    parser.add_argument(
        '--target-deg',
        action='append',
        default=[],
        metavar='JOINT=DEG',
        help='Expected target angle in degrees for the calibration mid pose (default: all 0).',
    )
    return parser


def main(args: List[str] | None = None) -> int:
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    if parsed.samples <= 0:
        raise ValueError('--samples must be > 0')
    if parsed.spin_timeout <= 0.0:
        raise ValueError('--spin-timeout must be > 0')
    if parsed.timeout <= 0.0:
        raise ValueError('--timeout must be > 0')

    scale_override = parse_joint_values(parsed.scale, 'scale')
    target_deg_override = parse_joint_values(parsed.target_deg, 'target-deg')

    target_mid_rad = {
        joint: math.radians(target_deg_override.get(joint, 0.0)) for joint in JOINT_NAMES
    }

    for joint, value in scale_override.items():
        if abs(value) < 1e-9:
            raise ValueError(f'Joint "{joint}" has scale=0, which is invalid.')

    print(f'Waiting for joint states on topic: {parsed.topic}')
    print('Note: keep enable_urdf_calibration=false while capturing calibration samples.')
    print(f'Calibration method: {parsed.method}')
    rclpy.init(args=None)
    node = JointStateCapture(parsed.topic)

    try:
        if not wait_for_joint_state(node, timeout_sec=parsed.timeout):
            print(
                f'No complete JointState received within {parsed.timeout:.1f}s on {parsed.topic}.'
            )
            return 1

        if not wait_enter(
            'Step 1/3: Move robot to MID reference pose '
            '(the pose you want to map to target-deg), then capture.'
        ):
            print('Calibration cancelled by user.')
            return 1
        print(f'Capturing {parsed.samples} samples for MID pose...')
        mid_pose = capture_pose_median(
            node=node,
            sample_count=parsed.samples,
            spin_timeout_sec=parsed.spin_timeout,
        )

        calibration = {}
        if parsed.method == 'zero':
            for joint in JOINT_NAMES:
                scale = scale_override.get(joint, 1.0)
                offset_rad = target_mid_rad[joint] - scale * mid_pose[joint]
                calibration[joint] = {'scale': scale, 'offset_rad': offset_rad}
        else:
            print('')
            print(
                'Step 2/3: Start sweeping ALL joints across full range in both directions.'
            )
            print(
                'Keep moving joints until you cover full motion; then press ENTER once to finish.'
            )
            ranges_min, ranges_max, captured_samples, capture_cancelled = (
                collect_joint_ranges_until_enter(
                    node=node,
                    spin_timeout_sec=parsed.spin_timeout,
                )
            )
            if capture_cancelled:
                print('Calibration cancelled by user.')
                return 1
            if captured_samples < parsed.samples:
                print(
                    f'Not enough range samples ({captured_samples}). '
                    f'Collect at least {parsed.samples} before finishing.'
                )
                return 1

            print(f'Step 3/3: Range capture complete ({captured_samples} samples).')
            for joint in JOINT_NAMES:
                raw_min = ranges_min[joint]
                raw_max = ranges_max[joint]
                lower, upper = URDF_LIMITS_RAD[joint]
                span_raw = raw_max - raw_min
                if abs(span_raw) < 1e-6:
                    print(
                        f'Warning: "{joint}" min/max too close. '
                        'Falling back to scale=1.0 based on MID pose.'
                    )
                    scale = 1.0
                    offset_rad = target_mid_rad[joint] - scale * mid_pose[joint]
                else:
                    estimated_scale = (upper - lower) / span_raw
                    if joint in scale_override:
                        scale = scale_override[joint]
                        offset_rad = target_mid_rad[joint] - scale * mid_pose[joint]
                    else:
                        scale_pos = estimated_scale
                        offset_pos = lower - scale_pos * raw_min

                        scale_neg = -estimated_scale
                        offset_neg = upper - scale_neg * raw_min

                        mid_err_pos = abs(
                            (scale_pos * mid_pose[joint] + offset_pos) - target_mid_rad[joint]
                        )
                        mid_err_neg = abs(
                            (scale_neg * mid_pose[joint] + offset_neg) - target_mid_rad[joint]
                        )

                        if mid_err_pos <= mid_err_neg:
                            scale = scale_pos
                            offset_rad = offset_pos
                        else:
                            scale = scale_neg
                            offset_rad = offset_neg

                calibration[joint] = {'scale': scale, 'offset_rad': offset_rad}

        output_path = Path(parsed.output).expanduser().resolve()
        metadata = {
            'tool': 'so101_urdf_calibrate',
            'method': parsed.method,
            'topic': parsed.topic,
            'samples': parsed.samples,
            'target_mid_deg': {joint: target_deg_override.get(joint, 0.0) for joint in JOINT_NAMES},
        }
        save_joint_calibration(output_path, calibration, metadata=metadata)

        print('')
        print(f'Calibration saved: {output_path}')
        print('Joint offsets (degrees):')
        for joint in JOINT_NAMES:
            print(
                f'  - {joint}: scale={calibration[joint]["scale"]:.6f}, '
                f'offset_deg={math.degrees(calibration[joint]["offset_rad"]):.3f}'
            )
        print('')
        print('Set these bridge parameters to use it:')
        print('  enable_urdf_calibration: true')
        print(f'  urdf_calibration_file: "{output_path}"')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
