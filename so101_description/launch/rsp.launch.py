# MIT License
#
# Copyright (c) 2025 nimiCurtis
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

import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _rsp_setup(context, *_args, **_kwargs):
    model = context.launch_configurations['model']
    mode = context.launch_configurations['mode']
    robot_type = context.launch_configurations['type']
    tf_prefix_mode = context.launch_configurations.get('tf_prefix_mode', 'prefixed')

    proc = subprocess.run(
        ['xacro', model, f'mode:={mode}'],
        check=True,
        capture_output=True,
        text=True,
    )
    robot_description = proc.stdout
    use_sim = mode != 'real'

    params = [
        {'robot_description': robot_description},
        {'use_sim_time': use_sim},
    ]
    if tf_prefix_mode != 'none':
        params.append({'frame_prefix': f'{robot_type}/'})

    child_base = 'base_link' if tf_prefix_mode == 'none' else f'{robot_type}/base_link'

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=params,
        namespace=robot_type,
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        namespace=robot_type,
        arguments=['0', '0', '0', '0', '0', '0', 'world', child_base],
        output='log',
    )
    return [rsp, static_tf]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_rsp_setup),
        ]
    )
