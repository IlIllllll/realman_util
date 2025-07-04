"""
Unified Robot Arm Interface
A plugin-based robot arm control interface with thread-safe operation,
frequency control, trajectory filtering, and logging.
"""

from .core.base_arm import BaseRobotArm
from .core.base_arm import ArmConfig
from .core.arm_manager import RobotArmManager
from .core.arm_factory import RobotArmFactory


__version__ = "1.0.0"
__all__ = ["BaseRobotArm", "RobotArmManager", "RobotArmFactory", "ArmConfig"]