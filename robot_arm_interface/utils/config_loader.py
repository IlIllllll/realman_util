"""
Configuration loader for robot arm interface.
"""

import json
import yaml
import os
from typing import Dict, Any, List
import numpy as np

from robot_arm_interface.core.base_arm import ArmConfig
from robot_arm_interface.core.arm_factory import RobotArmFactory


def load_config_file(config_path: str) -> Dict[str, Any]:
    """
    Load configuration from file.
    
    Supports JSON and YAML formats.
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    ext = os.path.splitext(config_path)[1].lower()
    
    with open(config_path, 'r') as f:
        if ext in ['.yaml', '.yml']:
            return yaml.safe_load(f)
        elif ext == '.json':
            return json.load(f)
        else:
            raise ValueError(f"Unsupported configuration format: {ext}")


def create_arm_config(config_dict: Dict[str, Any]) -> ArmConfig:
    """
    Create ArmConfig from configuration dictionary.
    
    Args:
        config_dict: Configuration dictionary
        
    Returns:
        ArmConfig instance
    """
    # Convert lists to numpy arrays
    def to_numpy(value):
        if isinstance(value, list):
            return np.array(value)
        return value
    
    return ArmConfig(
        name=config_dict.get("name", "robot_arm"),
        dof=config_dict.get("dof", 6),
        ip=config_dict.get("ip", ["192.168.1.18"]),
        max_joint_velocities=to_numpy(config_dict.get("max_joint_velocities", [1.0] * 6)),
        max_joint_accelerations=to_numpy(config_dict.get("max_joint_accelerations", [1.0] * 6)),
        joint_limits_lower=to_numpy(config_dict.get("joint_limits_lower", [-np.pi] * 6)),
        joint_limits_upper=to_numpy(config_dict.get("joint_limits_upper", [np.pi] * 6)),
        control_frequency=config_dict.get("control_frequency", 100.0),
        connection_params=config_dict.get("connection_params", {})
    )


def load_plugins_from_config(config_path: str):
    """
    Load plugins from configuration file.
    
    Args:
        config_path: Path to plugins configuration file
    """
    config = load_config_file(config_path)
    plugins = config.get("plugins", [])
    
    RobotArmFactory.load_plugins_from_config(plugins)


def create_arm_from_config_file(config_path: str):
    """
    Create robot arm instance from configuration file.
    
    The configuration file should have the following structure:
    {
        "type": "simulation",  # Robot arm type
        "config": {            # ArmConfig parameters
            "name": "my_robot",
            "dof": 6,
            ...
        }
    }
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Robot arm instance
    """
    config = load_config_file(config_path)
    return RobotArmFactory.create_from_config(config)


# Example configuration templates
SIMULATION_ARM_CONFIG = {
    "type": "simulation",
    "config": {
        "name": "simulation_arm",
        "dof": 6,
        "max_joint_velocities": [2.0, 2.0, 2.0, 3.0, 3.0, 3.0],
        "max_joint_accelerations": [5.0, 5.0, 5.0, 10.0, 10.0, 10.0],
        "joint_limits_lower": [-3.14, -2.0, -2.0, -3.14, -2.0, -3.14],
        "joint_limits_upper": [3.14, 2.0, 2.0, 3.14, 2.0, 3.14],
        "control_frequency": 100.0,
        "connection_params": {
            "simulation_delay": 0.001,
            "error_probability": 0.0
        }
    }
}

REALMAN_ARM_CONFIG = {
    "type": "realman",
    "config": {
        "name": "realman_rm65",
        "dof": 6,
        "max_joint_velocities": [3.14, 3.14, 3.14, 3.14, 3.14, 3.14],
        "max_joint_accelerations": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
        "joint_limits_lower": [-3.05, -2.35, -2.52, -3.05, -1.57, -3.05],
        "joint_limits_upper": [3.05, 2.35, 0.77, 3.05, 1.57, 3.05],
        "control_frequency": 100.0,
        "connection_params": {
            "ip": "192.168.1.18",
            "port": 8080
        }
    }
}


def save_config_template(template_name: str, output_path: str, format: str = "yaml"):
    """
    Save a configuration template to file.
    
    Args:
        template_name: Name of template ("simulation" or "realman")
        output_path: Path to save configuration
        format: Output format ("yaml" or "json")
    """
    templates = {
        "simulation": SIMULATION_ARM_CONFIG,
        "realman": REALMAN_ARM_CONFIG
    }
    
    if template_name not in templates:
        raise ValueError(f"Unknown template: {template_name}")
    
    config = templates[template_name]
    
    with open(output_path, 'w') as f:
        if format == "yaml":
            yaml.dump(config, f, default_flow_style=False)
        elif format == "json":
            json.dump(config, f, indent=2)
        else:
            raise ValueError(f"Unsupported format: {format}")