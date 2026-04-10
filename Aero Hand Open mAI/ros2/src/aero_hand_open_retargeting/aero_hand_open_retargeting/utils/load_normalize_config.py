from pathlib import Path
from typing import Dict, Any
import yaml
from ament_index_python.packages import get_package_share_directory


def load_normalize_config(user: str) -> Dict[str, Any]:
    """Load normalize_<user>.yaml from this ROS package."""
    pkg_share = Path(get_package_share_directory("aero_hand_open_retargeting"))
    config_path = pkg_share / "config" / f"normalize_{user}.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Normalize config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError(f"Invalid YAML format in {config_path}")

    return config
