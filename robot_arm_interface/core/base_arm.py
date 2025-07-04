"""
Base abstract class for robot arms.
Defines the unified interface that all robot arm implementations must follow.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
import numpy as np


@dataclass
class ArmState:
    """Robot arm state information."""
    joint_positions: np.ndarray  # Joint angles in radians
    joint_velocities: np.ndarray  # Joint velocities in rad/s
    joint_torques: np.ndarray  # Joint torques in Nm
    end_effector_pose: np.ndarray  # End effector pose [x, y, z, rx, ry, rz]
    timestamp: float  # Timestamp in seconds
    is_moving: bool  # Whether the arm is currently in motion
    has_error: bool  # Whether there's an error
    error_message: str = ""  # Error message if any
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert state to dictionary."""
        return {
            "joint_positions": self.joint_positions.tolist(),
            "joint_velocities": self.joint_velocities.tolist(),
            "joint_torques": self.joint_torques.tolist(),
            "end_effector_pose": self.end_effector_pose.tolist(),
            "timestamp": self.timestamp,
            "is_moving": self.is_moving,
            "has_error": self.has_error,
            "error_message": self.error_message
        }


@dataclass
class ArmConfig:
    """Robot arm configuration."""
    name: str  # Arm name/identifier
    dof: int  # Degrees of freedom
    ip: List[str]  # IP address ， 字符串数组， 用于多机械臂
    max_joint_velocities: np.ndarray  # Maximum joint velocities (rad/s)
    max_joint_accelerations: np.ndarray  # Maximum joint accelerations (rad/s^2)
    joint_limits_lower: np.ndarray  # Lower joint limits (rad)
    joint_limits_upper: np.ndarray  # Upper joint limits (rad)
    control_frequency: float = 100.0  # Control frequency in Hz
    connection_params: Dict[str, Any] = None  # Connection parameters (IP, port, etc.)
    
    def __post_init__(self):
        if self.connection_params is None:
            self.connection_params = {}


class BaseRobotArm(ABC):
    """Abstract base class for robot arms."""
    
    def __init__(self, config: ArmConfig):
        """
        Initialize the robot arm.
        
        Args:
            config: Robot arm configuration
        """
        self._config = config
        self._is_connected = False
        self._is_enabled = False
        
    @property
    def config(self) -> ArmConfig:
        """Get robot arm configuration."""
        return self._config
    
    @property
    def is_connected(self) -> bool:
        """Check if connected to the robot arm."""
        return self._is_connected
    
    @property
    def is_enabled(self) -> bool:
        """Check if the robot arm is enabled."""
        return self._is_enabled
    
    @abstractmethod
    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to the robot arm.
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful, False otherwise
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> bool:
        """
        Disconnect from the robot arm.
        
        Returns:
            True if disconnection successful, False otherwise
        """
        pass
    
    @abstractmethod
    def enable(self) -> bool:
        """
        Enable the robot arm (power on motors).
        
        Returns:
            True if enabling successful, False otherwise
        """
        pass
    
    @abstractmethod
    def disable(self) -> bool:
        """
        Disable the robot arm (power off motors).
        
        Returns:
            True if disabling successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_state(self) -> ArmState:
        """
        Get current robot arm state.
        
        Returns:
            Current arm state
        """
        pass
    
    @abstractmethod
    def move_joints(self, joint_positions: np.ndarray, 
                   velocity_limit: Optional[float] = None,
                   acceleration_limit: Optional[float] = None) -> bool:
        """
        Move to target joint positions.
        
        Args:
            joint_positions: Target joint positions in radians
            velocity_limit: Optional velocity limit (0-1, fraction of max velocity)
            acceleration_limit: Optional acceleration limit (0-1, fraction of max acceleration)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        pass
    
    @abstractmethod
    def move_joints_velocity(self, joint_velocities: np.ndarray) -> bool:
        """
        Move joints with specified velocities.
        
        Args:
            joint_velocities: Target joint velocities in rad/s
            
        Returns:
            True if command sent successfully, False otherwise
        """
        pass
    
    @abstractmethod
    def move_cartesian(self, pose: np.ndarray,
                      velocity_limit: Optional[float] = None,
                      acceleration_limit: Optional[float] = None) -> bool:
        """
        Move to target Cartesian pose.
        
        Args:
            pose: Target pose [x, y, z, rx, ry, rz]
            velocity_limit: Optional velocity limit (0-1, fraction of max velocity)
            acceleration_limit: Optional acceleration limit (0-1, fraction of max acceleration)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """
        Stop the robot arm immediately.
        
        Returns:
            True if stop successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Calculate forward kinematics.
        
        Args:
            joint_positions: Joint positions in radians
            
        Returns:
            End effector pose [x, y, z, rx, ry, rz]
        """
        pass
    
    @abstractmethod
    def get_inverse_kinematics(self, pose: np.ndarray,
                              current_joints: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Calculate inverse kinematics.
        
        Args:
            pose: Target pose [x, y, z, rx, ry, rz]
            current_joints: Optional current joint positions for selecting solution
            
        Returns:
            Joint positions in radians, or None if no solution
        """
        pass
    
    def validate_joint_positions(self, joint_positions: np.ndarray) -> Tuple[bool, str]:
        """
        Validate joint positions against limits.
        
        Args:
            joint_positions: Joint positions to validate
            
        Returns:
            (is_valid, error_message)
        """
        if len(joint_positions) != self._config.dof:
            return False, f"Expected {self._config.dof} joints, got {len(joint_positions)}"
        
        for i, pos in enumerate(joint_positions):
            if pos < self._config.joint_limits_lower[i] or pos > self._config.joint_limits_upper[i]:
                return False, f"Joint {i} position {pos} out of limits [{self._config.joint_limits_lower[i]}, {self._config.joint_limits_upper[i]}]"
        
        return True, ""
    
    def validate_joint_velocities(self, joint_velocities: np.ndarray) -> Tuple[bool, str]:
        """
        Validate joint velocities against limits.
        
        Args:
            joint_velocities: Joint velocities to validate
            
        Returns:
            (is_valid, error_message)
        """
        if len(joint_velocities) != self._config.dof:
            return False, f"Expected {self._config.dof} joints, got {len(joint_velocities)}"
        
        for i, vel in enumerate(joint_velocities):
            if abs(vel) > self._config.max_joint_velocities[i]:
                return False, f"Joint {i} velocity {vel} exceeds limit {self._config.max_joint_velocities[i]}"
        
        return True, ""