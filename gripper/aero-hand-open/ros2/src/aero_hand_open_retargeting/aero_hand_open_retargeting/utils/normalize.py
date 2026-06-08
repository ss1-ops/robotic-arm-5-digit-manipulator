#!/usr/bin/env python3
import numpy as np
from typing import Union, Dict, Any
from aero_open_sdk.aero_hand_constants import AeroHandConstants
from sensor_msgs.msg import JointState

# Joint names from aero_hand_open_msgs/msg/JointControl.msg (source of truth)
JOINT_NAME_MAP = {
    0: "thumb_cmc_abd",
    1: "thumb_cmc_flex",
    2: "thumb_mcp",
    3: "thumb_ip",
    4: "index_mcp_flex",
    5: "index_pip",
    6: "index_dip",
    7: "middle_mcp_flex",
    8: "middle_pip",
    9: "middle_dip",
    10: "ring_mcp_flex",
    11: "ring_pip",
    12: "ring_dip",
    13: "pinky_mcp_flex",
    14: "pinky_pip",
    15: "pinky_dip",
}

__all__ = ["normalize_value", "normalize_joint_state", "JOINT_NAME_MAP"]


def _check_config(config: Dict[str, Any], joint_idx: int):
    if joint_idx not in JOINT_NAME_MAP:
        raise IndexError(f"joint_idx {joint_idx} is invalid")
    jname = JOINT_NAME_MAP[joint_idx]
    if jname not in config:
        raise KeyError(f"config is missing key: {jname}")
    if "valley" not in config[jname] or "peak" not in config[jname]:
        raise KeyError(f"config['{jname}'] must contain 'valley' and 'peak' fields")
    return jname


def normalize_value(raw_val: float, joint_idx: int, config: dict) -> float:
    """
    single joint scalar normalization: raw_val -> radians (float)
    valley -> mechanical lower limit, peak -> mechanical upper limit; clamped to range automatically.
    """
    jname = _check_config(config, joint_idx)
    valley = float(config[jname]["valley"])
    peak = float(config[jname]["peak"])
    denom = peak - valley
    if abs(denom) < 1e-9:
        denom = 1e-9  # avoid division by zero

    # mechanical angle limits (degrees -> radians)
    lower_rad = float(np.deg2rad(AeroHandConstants.joint_lower_limits[joint_idx]))
    upper_rad = float(np.deg2rad(AeroHandConstants.joint_upper_limits[joint_idx]))

    # normalize to [0,1], linearly map to radians and clamp
    norm01 = (float(raw_val) - valley) / denom
    mapped = lower_rad + norm01 * (upper_rad - lower_rad)
    return float(np.clip(mapped, min(lower_rad, upper_rad), max(lower_rad, upper_rad)))


def normalize_joint_state(
    arg: Union[JointState, float, int], joint_idx: int, config: dict
) -> Union[JointState, float]:
    """
    compatible with two calling methods:
      - normalize_joint_state(joint_state: JointState, joint_idx, config) -> JointState
        normalize position[joint_idx] to radians and write back.
      - normalize_joint_state(raw_val: float, joint_idx, config) -> float
        return normalized radians (float) directly.
    """
    # scalar path: return radians directly
    if not isinstance(arg, JointState):
        return normalize_value(float(arg), joint_idx, config)

    mapped = normalize_value(float(arg.position[joint_idx]), joint_idx, config)
    arg.position[joint_idx] = mapped
    return arg
