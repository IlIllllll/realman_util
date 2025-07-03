"""
Realman robot arm plugin implementation.
"""

import numpy as np
import time
from typing import Optional
import logging
import sys
import os

# Add parent directory to path to import realman modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from robot_arm_interface.core.base_arm import BaseRobotArm, ArmState, ArmConfig
from robot_arm_interface.core.arm_factory import register_robot_arm

try:
    from realman_arm.Robotic_Arm.robotic_arm import Arm
    from realman_arm.realman_utils import get_arm_joint_angles, get_arm_joint_angles_velocity
except ImportError:
    logging.warning("Realman arm modules not found. Realman plugin will not be available.")
    Arm = None


@register_robot_arm("realman")
class RealmanRobotArm(BaseRobotArm):
    """
    Realman robot arm implementation.
    
    Wraps the existing Realman arm API to conform to the unified interface.
    """
    
    def __init__(self, config: ArmConfig):
        """Initialize Realman robot arm."""
        super().__init__(config)
        
        if Arm is None:
            raise ImportError("Realman arm modules not available")
        
        self._arm = None
        self._logger = logging.getLogger(f"RealmanArm[{config.name}]")
        
        # Extract connection parameters
        self._ip = config.connection_params.get("ip", "192.168.1.18")
        self._port = config.connection_params.get("port", 8080)
        
    def connect(self, timeout: float = 5.0) -> bool:
        """Connect to Realman arm."""
        try:
            self._logger.info(f"Connecting to Realman arm at {self._ip}:{self._port}")
            
            self._arm = Arm(self._ip, self._port)
            
            # Test connection by getting version
            version = self._arm.get_system_version()
            if version:
                self._logger.info(f"Connected to Realman arm. Version: {version}")
                self._is_connected = True
                return True
            else:
                self._logger.error("Failed to get system version")
                return False
                
        except Exception as e:
            self._logger.error(f"Connection failed: {e}")
            self._is_connected = False
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from Realman arm."""
        try:
            if self._arm:
                # Realman API doesn't have explicit disconnect
                self._arm = None
            self._is_connected = False
            self._logger.info("Disconnected from Realman arm")
            return True
        except Exception as e:
            self._logger.error(f"Disconnect failed: {e}")
            return False
    
    def enable(self) -> bool:
        """Enable the robot arm."""
        try:
            if not self._is_connected:
                return False
            
            # Clear any errors first
            self._arm.clear_system_err()
            
            # Set to position mode
            self._arm.set_arm_servo(1)  # Enable servo
            
            self._is_enabled = True
            self._logger.info("Realman arm enabled")
            return True
            
        except Exception as e:
            self._logger.error(f"Enable failed: {e}")
            return False
    
    def disable(self) -> bool:
        """Disable the robot arm."""
        try:
            if not self._is_connected:
                return False
            
            # Stop any ongoing movement
            self._arm.set_arm_stop()
            
            # Disable servo
            self._arm.set_arm_servo(0)
            
            self._is_enabled = False
            self._logger.info("Realman arm disabled")
            return True
            
        except Exception as e:
            self._logger.error(f"Disable failed: {e}")
            return False
    
    def get_state(self) -> ArmState:
        """Get current robot arm state."""
        try:
            current_time = time.time()
            
            # Get joint angles (convert from degrees to radians)
            joint_angles_deg = get_arm_joint_angles(self._arm)
            joint_positions = np.deg2rad(joint_angles_deg)
            
            # Get joint velocities
            joint_velocities = get_arm_joint_angles_velocity(self._arm)
            joint_velocities = np.deg2rad(joint_velocities)  # Convert to rad/s
            
            # Get joint torques (Realman doesn't provide this directly)
            joint_torques = np.zeros(self._config.dof)
            
            # Get end effector pose
            pose_data = self._arm.get_current_arm_state()
            if pose_data and len(pose_data) >= 6:
                # Convert mm to m for position, keep angles in radians
                end_effector_pose = np.array([
                    pose_data[0] / 1000.0,  # x in m
                    pose_data[1] / 1000.0,  # y in m
                    pose_data[2] / 1000.0,  # z in m
                    np.deg2rad(pose_data[3]),  # rx in rad
                    np.deg2rad(pose_data[4]),  # ry in rad
                    np.deg2rad(pose_data[5])   # rz in rad
                ])
            else:
                end_effector_pose = np.zeros(6)
            
            # Check if moving
            is_moving = self._arm.get_arm_state() == 1  # 1 = moving
            
            # Check for errors
            error_code = self._arm.get_system_err()
            has_error = error_code != 0
            error_message = f"Error code: {error_code}" if has_error else ""
            
            return ArmState(
                joint_positions=joint_positions,
                joint_velocities=joint_velocities,
                joint_torques=joint_torques,
                end_effector_pose=end_effector_pose,
                timestamp=current_time,
                is_moving=is_moving,
                has_error=has_error,
                error_message=error_message
            )
            
        except Exception as e:
            self._logger.error(f"Get state failed: {e}")
            # Return error state
            return ArmState(
                joint_positions=np.zeros(self._config.dof),
                joint_velocities=np.zeros(self._config.dof),
                joint_torques=np.zeros(self._config.dof),
                end_effector_pose=np.zeros(6),
                timestamp=time.time(),
                is_moving=False,
                has_error=True,
                error_message=str(e)
            )
    
    def move_joints(self, joint_positions: np.ndarray,
                   velocity_limit: Optional[float] = None,
                   acceleration_limit: Optional[float] = None) -> bool:
        """Move to target joint positions."""
        try:
            if not self._is_enabled:
                self._logger.error("Arm not enabled")
                return False
            
            # Convert radians to degrees
            joint_angles_deg = np.rad2deg(joint_positions)
            
            # Apply velocity limit if specified
            speed = 50  # Default speed percentage
            if velocity_limit is not None:
                speed = int(velocity_limit * 100)  # Convert 0-1 to 0-100
                speed = max(1, min(100, speed))
            
            # Move joints
            result = self._arm.movej(
                joint_angles_deg.tolist(),
                speed=speed,
                connect=0  # Don't blend with next movement
            )
            
            return result == 1  # 1 = success
            
        except Exception as e:
            self._logger.error(f"Move joints failed: {e}")
            return False
    
    def move_joints_velocity(self, joint_velocities: np.ndarray) -> bool:
        """Move joints with specified velocities."""
        try:
            if not self._is_enabled:
                self._logger.error("Arm not enabled")
                return False
            
            # Convert rad/s to deg/s
            joint_velocities_deg = np.rad2deg(joint_velocities)
            
            # Realman uses position control, so we approximate velocity control
            # by calculating small position increments
            dt = 0.01  # 10ms time step
            current_state = self.get_state()
            
            # Calculate target positions
            target_positions = current_state.joint_positions + joint_velocities * dt
            
            # Validate against limits
            is_valid, error = self.validate_joint_positions(target_positions)
            if not is_valid:
                self._logger.warning(f"Velocity command would exceed limits: {error}")
                return False
            
            # Send position command
            return self.move_joints(target_positions, velocity_limit=1.0)
            
        except Exception as e:
            self._logger.error(f"Move joints velocity failed: {e}")
            return False
    
    def move_cartesian(self, pose: np.ndarray,
                      velocity_limit: Optional[float] = None,
                      acceleration_limit: Optional[float] = None) -> bool:
        """Move to target Cartesian pose."""
        try:
            if not self._is_enabled:
                self._logger.error("Arm not enabled")
                return False
            
            # Convert pose format
            # Input: [x, y, z, rx, ry, rz] in m and rad
            # Realman expects: [x, y, z, rx, ry, rz] in mm and deg
            pose_mm_deg = np.array([
                pose[0] * 1000,  # x in mm
                pose[1] * 1000,  # y in mm
                pose[2] * 1000,  # z in mm
                np.rad2deg(pose[3]),  # rx in deg
                np.rad2deg(pose[4]),  # ry in deg
                np.rad2deg(pose[5])   # rz in deg
            ])
            
            # Apply velocity limit if specified
            speed = 50  # Default speed percentage
            if velocity_limit is not None:
                speed = int(velocity_limit * 100)
                speed = max(1, min(100, speed))
            
            # Move to pose
            result = self._arm.movel(
                pose_mm_deg.tolist(),
                speed=speed,
                connect=0
            )
            
            return result == 1
            
        except Exception as e:
            self._logger.error(f"Move cartesian failed: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop the robot arm immediately."""
        try:
            if not self._is_connected:
                return False
            
            self._arm.set_arm_stop()
            self._logger.info("Arm stopped")
            return True
            
        except Exception as e:
            self._logger.error(f"Stop failed: {e}")
            return False
    
    def get_forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        """Calculate forward kinematics."""
        try:
            # Convert to degrees
            joint_angles_deg = np.rad2deg(joint_positions).tolist()
            
            # Get forward kinematics from Realman
            result = self._arm.get_forward_kinematics(joint_angles_deg)
            
            if result and len(result) >= 6:
                # Convert mm to m, deg to rad
                return np.array([
                    result[0] / 1000.0,
                    result[1] / 1000.0,
                    result[2] / 1000.0,
                    np.deg2rad(result[3]),
                    np.deg2rad(result[4]),
                    np.deg2rad(result[5])
                ])
            else:
                self._logger.error("Invalid forward kinematics result")
                return np.zeros(6)
                
        except Exception as e:
            self._logger.error(f"Forward kinematics failed: {e}")
            return np.zeros(6)
    
    def get_inverse_kinematics(self, pose: np.ndarray,
                              current_joints: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Calculate inverse kinematics."""
        try:
            # Convert pose to Realman format
            pose_mm_deg = [
                pose[0] * 1000,
                pose[1] * 1000,
                pose[2] * 1000,
                np.rad2deg(pose[3]),
                np.rad2deg(pose[4]),
                np.rad2deg(pose[5])
            ]
            
            # Get current joints if not provided
            if current_joints is None:
                current_state = self.get_state()
                current_joints = current_state.joint_positions
            
            current_joints_deg = np.rad2deg(current_joints).tolist()
            
            # Get inverse kinematics
            result = self._arm.get_inverse_kinematics(
                pose_mm_deg,
                current_joints_deg
            )
            
            if result and len(result) == self._config.dof:
                # Convert to radians
                return np.deg2rad(np.array(result))
            else:
                self._logger.warning("No inverse kinematics solution found")
                return None
                
        except Exception as e:
            self._logger.error(f"Inverse kinematics failed: {e}")
            return None