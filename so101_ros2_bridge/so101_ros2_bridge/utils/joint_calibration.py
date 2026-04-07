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

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

JointCalibrationMap = Dict[str, Dict[str, float]]


def build_identity_calibration(joint_names: Iterable[str]) -> JointCalibrationMap:
    return {name: {'scale': 1.0, 'offset_rad': 0.0} for name in joint_names}


def _coerce_float(value: Any, default_value: float) -> float:
    if value is None:
        return default_value
    try:
        return float(value)
    except (TypeError, ValueError):
        return default_value


def _log_warn(logger: Optional[Any], message: str) -> None:
    if logger is not None:
        logger.warn(message)


def load_joint_calibration(
    calibration_file: Path,
    joint_names: Iterable[str],
    logger: Optional[Any] = None,
) -> JointCalibrationMap:
    identity = build_identity_calibration(joint_names)
    path = Path(calibration_file)
    if not path.exists():
        _log_warn(
            logger,
            f'URDF calibration file "{path}" not found. Falling back to identity mapping.',
        )
        return identity

    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception as exc:
        _log_warn(
            logger,
            f'Failed to parse URDF calibration file "{path}": {exc}. '
            'Falling back to identity mapping.',
        )
        return identity

    joints_payload = payload.get('joints')
    if not isinstance(joints_payload, dict):
        _log_warn(
            logger,
            f'URDF calibration file "{path}" has no "joints" object. '
            'Falling back to identity mapping.',
        )
        return identity

    resolved: JointCalibrationMap = {}
    for joint_name in identity.keys():
        joint_cfg = joints_payload.get(joint_name, {})
        if not isinstance(joint_cfg, dict):
            joint_cfg = {}

        scale = _coerce_float(
            joint_cfg.get('scale', joint_cfg.get('position_scale', 1.0)),
            1.0,
        )
        if abs(scale) < 1e-9:
            _log_warn(
                logger,
                f'Invalid scale for joint "{joint_name}" in "{path}" (scale=0). '
                'Using scale=1.0.',
            )
            scale = 1.0
        offset_rad = _coerce_float(
            joint_cfg.get('offset_rad', joint_cfg.get('position_offset_rad', 0.0)),
            0.0,
        )

        resolved[joint_name] = {'scale': scale, 'offset_rad': offset_rad}

    return resolved


def apply_raw_to_urdf(calibration: JointCalibrationMap, joint_name: str, raw_pos: float) -> float:
    cfg = calibration[joint_name]
    return cfg['scale'] * raw_pos + cfg['offset_rad']


def apply_urdf_to_raw(calibration: JointCalibrationMap, joint_name: str, urdf_pos: float) -> float:
    cfg = calibration[joint_name]
    return (urdf_pos - cfg['offset_rad']) / cfg['scale']


def save_joint_calibration(
    output_file: Path,
    calibration: JointCalibrationMap,
    metadata: Optional[Dict[str, Any]] = None,
) -> None:
    data: Dict[str, Any] = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'joints': calibration,
    }
    if metadata:
        data['metadata'] = metadata

    path = Path(output_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
