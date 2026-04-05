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


import inspect
from pathlib import Path

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils.core import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig

from so101_ros2_bridge.bridge.registry import register_robot


def _build_config(config_cls, raw_kwargs: dict):
    """Build config object while handling minor API drift across lerobot forks."""
    signature = inspect.signature(config_cls)
    accepted = set(signature.parameters.keys())
    kwargs = dict(raw_kwargs)

    # Support either calibration_dir (nimiCurtis) or calibration_path (Seeed SOFollowerConfig).
    if 'calibration_dir' in kwargs and 'calibration_dir' not in accepted:
        if 'calibration_path' in accepted:
            kwargs['calibration_path'] = kwargs['calibration_dir']

    filtered_kwargs = {k: v for k, v in kwargs.items() if k in accepted}
    config = config_cls(**filtered_kwargs)

    # Backfill missing attrs expected by older/newer robot base classes.
    for key, value in raw_kwargs.items():
        if hasattr(config, key):
            continue
        try:
            setattr(config, key, value)
        except Exception:
            # Some config objects may be frozen/slots-only; best effort only.
            pass

    # Keep both names available when either side expects a specific calibration field.
    if not hasattr(config, 'calibration_dir') and hasattr(config, 'calibration_path'):
        try:
            setattr(config, 'calibration_dir', getattr(config, 'calibration_path'))
        except Exception:
            pass
    if not hasattr(config, 'calibration_path') and hasattr(config, 'calibration_dir'):
        try:
            setattr(config, 'calibration_path', getattr(config, 'calibration_dir'))
        except Exception:
            pass

    return config


@register_robot('follower')
def create_follower(params: dict):
    config = _build_config(
        SO101FollowerConfig,
        {
            'port': params['port'],
            'calibration_dir': Path(params['calibration_dir']),
            'id': params['id'],
            'use_degrees': params['use_degrees'],
            'max_relative_target': params['max_relative_target'],
            'disable_torque_on_disconnect': params['disable_torque_on_disconnect'],
        },
    )
    return SO101Follower(config)


@register_robot('leader')
def create_leader(params: dict):
    config = _build_config(
        SO101LeaderConfig,
        {
            'port': params['port'],
            'calibration_dir': Path(params['calibration_dir']),
            'id': params['id'],
            'use_degrees': params['use_degrees'],
        },
    )
    return SO101Leader(config)
