"""
Simulation robot arm plugin for testing and development.
"""

import numpy as np
import time
from typing import Optional
import logging
import threading

from robot_arm_interface.core.base_arm import BaseRobotArm, ArmState, ArmConfig
from robot_arm_interface.core.arm_factory import register_robot_arm


@register_robot_arm("simulation")
class SimulationRobotArm(BaseRobotArm):
    """
    Simulated robot arm for testing.
    
    Provides a fully functional robot arm simulation with:
    - Smooth trajectory interpolation
    - Velocity and acceleration simulation
    - Simple forward/inverse kinematics
    - Configurable delays and errors
    """
    
    def __init__(self, config: ArmConfig):
        """Initialize simulation robot arm."""
        super().__init__(config)
        
        self._logger = logging.getLogger(f"SimulationArm[{config.name}]")
        
        # Simulation state
        self._current_positions = np.zeros(config.dof)
        self._current_velocities = np.zeros(config.dof)
        self._target_positions = np.zeros(config.dof)
        self._start_positions = np.zeros(config.dof)
        self._movement_start_time = 0.0
        self._movement_duration = 0.0
        self._is_moving = False
        
        # Simulation parameters
        self._simulation_delay = config.connection_params.get("simulation_delay", 0.001)
        self._error_probability = config.connection_params.get("error_probability", 0.0)
        self._connection_delay = config.connection_params.get("connection_delay", 0.1)
        
        # Update thread
        self._update_thread = None
        self._thread_running = False
        self._update_lock = threading.Lock()
        
    def connect(self, timeout: float = 5.0) -> bool:
        """Simulate connection to robot arm."""
        self._logger.info("Connecting to simulation robot arm...")
        
        # Simulate connection delay
        time.sleep(self._connection_delay)
        
        # Start update thread
        self._thread_running = True
        self._update_thread = threading.Thread(target=self._update_loop)
        self._update_thread.daemon = True
        self._update_thread.start()
        
        self._is_connected = True
        self._logger.info("Connected to simulation robot arm")
        return True
    
    def disconnect(self) -> bool:
        """Simulate disconnection from robot arm."""
        # Stop update thread
        self._thread_running = False
        if self._update_thread:
            self._update_thread.join(timeout=1.0)
        
        self._is_connected = False
        self._logger.info("Disconnected from simulation robot arm")
        return True
    
    def enable(self) -> bool:
        """Enable the robot arm."""
        if not self._is_connected:
            return False
        
        # Simulate enable delay
        time.sleep(0.1)
        
        self._is_enabled = True
        self._logger.info("Simulation robot arm enabled")
        return True
    
    def disable(self) -> bool:
        """Disable the robot arm."""
        if not self._is_connected:
            return False
        
        # Stop any movement
        with self._update_lock:
            self._is_moving = False
        
        self._is_enabled = False
        self._logger.info("Simulation robot arm disabled")
        return True
    
    def get_state(self) -> ArmState:
        """Get current robot arm state."""
        with self._update_lock:
            # Calculate end effector pose using simple FK
            end_effector_pose = self._simple_forward_kinematics(self._current_positions)
            
            # Simulate random errors
            has_error = False
            error_message = ""
            if np.random.random() < self._error_probability:
                has_error = True
                error_message = "Simulated error occurred"
            
            return ArmState(
                joint_positions=self._current_positions.copy(),
                joint_velocities=self._current_velocities.copy(),
                joint_torques=np.zeros(self._config.dof),  # No torque simulation
                end_effector_pose=end_effector_pose,
                timestamp=time.time(),
                is_moving=self._is_moving,
                has_error=has_error,
                error_message=error_message
            )
    
    def move_joints(self, joint_positions: np.ndarray,
                   velocity_limit: Optional[float] = None,
                   acceleration_limit: Optional[float] = None) -> bool:
        """Move to target joint positions."""
        if not self._is_enabled:
            self._logger.error("Arm not enabled")
            return False
        
        # Calculate movement duration based on max velocity
        with self._update_lock:
            position_diff = np.abs(joint_positions - self._current_positions)
            max_joint_time = np.max(position_diff / self._config.max_joint_velocities)
            
            # Apply velocity limit
            if velocity_limit is not None:
                max_joint_time = max_joint_time / velocity_limit
            
            # Start movement
            self._start_positions = self._current_positions.copy()
            self._target_positions = joint_positions.copy()
            self._movement_start_time = time.time()
            self._movement_duration = max(max_joint_time, 0.1)  # Minimum 0.1s
            self._is_moving = True
        
        self._logger.info(f"Starting movement to {joint_positions}, duration: {self._movement_duration:.2f}s")
        return True
    
    def move_cartesian(self, pose: np.ndarray,
                      velocity_limit: Optional[float] = None,
                      acceleration_limit: Optional[float] = None) -> bool:
        """Move to target Cartesian pose."""
        if not self._is_enabled:
            self._logger.error("Arm not enabled")
            return False
        
        # Use simple inverse kinematics
        target_joints = self._simple_inverse_kinematics(pose)
        
        if target_joints is None:
            self._logger.error("No IK solution found")
            return False
        
        return self.move_joints(target_joints, velocity_limit, acceleration_limit)
    
    def stop(self) -> bool:
        """Stop the robot arm immediately."""
        with self._update_lock:
            self._is_moving = False
            self._current_velocities = np.zeros(self._config.dof)
        
        self._logger.info("Simulation arm stopped")
        return True
    
    def get_forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        """Calculate forward kinematics."""
        return self._simple_forward_kinematics(joint_positions)
    
    def get_inverse_kinematics(self, pose: np.ndarray,
                              current_joints: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Calculate inverse kinematics."""
        return self._simple_inverse_kinematics(pose)
    
    def _update_loop(self):
        """Update simulation state in separate thread."""
        last_update = time.time()
        
        while self._thread_running:
            current_time = time.time()
            dt = current_time - last_update
            
            with self._update_lock:
                if self._is_moving and self._movement_duration > 0:
                    # Update position during movement
                    elapsed = current_time - self._movement_start_time
                    
                    if elapsed >= self._movement_duration:
                        # Movement complete
                        self._current_positions = self._target_positions.copy()
                        self._current_velocities = np.zeros(self._config.dof)
                        self._is_moving = False
                    else:
                        # Interpolate position (simple linear interpolation)
                        t = elapsed / self._movement_duration
                        self._current_positions = (
                            self._start_positions + 
                            (self._target_positions - self._start_positions) * t
                        )
                        
                        # Calculate velocities
                        self._current_velocities = (
                            (self._target_positions - self._start_positions) / 
                            self._movement_duration
                        )
                else:
                    # Velocity control mode
                    if np.any(self._current_velocities != 0):
                        # Update positions based on velocities
                        new_positions = self._current_positions + self._current_velocities * dt
                        
                        # Clamp to joint limits
                        new_positions = np.clip(
                            new_positions,
                            self._config.joint_limits_lower,
                            self._config.joint_limits_upper
                        )
                        
                        self._current_positions = new_positions
            
            last_update = current_time
            time.sleep(self._simulation_delay)
    
    def _simple_forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Simple forward kinematics for simulation.
        
        This is a very simplified FK that assumes a planar arm.
        Real implementations would use DH parameters or other methods.
        """
        # Assume equal link lengths
        link_length = 0.3  # 30cm per link
        
        # Calculate end effector position (2D for simplicity)
        x = 0.0
        y = 0.0
        angle = 0.0
        
        for i, joint_angle in enumerate(joint_positions[:min(3, self._config.dof)]):
            angle += joint_angle
            x += link_length * np.cos(angle)
            y += link_length * np.sin(angle)
        
        # Simple orientation (sum of joint angles for first 3 joints)
        rx = 0.0
        ry = 0.0
        rz = angle
        
        # Add some z component
        z = 0.1 + 0.05 * np.sin(joint_positions[0] if self._config.dof > 0 else 0)
        
        return np.array([x, y, z, rx, ry, rz])
    
    def _simple_inverse_kinematics(self, pose: np.ndarray) -> Optional[np.ndarray]:
        """
        Simple inverse kinematics for simulation.
        
        This is a very simplified IK that only considers position.
        Real implementations would use analytical or numerical methods.
        """
        # Extract target position
        target_x = pose[0]
        target_y = pose[1]
        target_z = pose[2]
        
        # Simple 2-link planar IK
        link_length = 0.3
        distance = np.sqrt(target_x**2 + target_y**2)
        
        # Check reachability
        if distance > 2 * link_length * self._config.dof / 3:
            return None
        
        # Calculate angles (simplified)
        base_angle = np.arctan2(target_y, target_x)
        
        # Generate joint angles
        joint_angles = np.zeros(self._config.dof)
        
        # Distribute the base angle across first few joints
        for i in range(min(3, self._config.dof)):
            joint_angles[i] = base_angle / 3
        
        # Add some variation for remaining joints
        for i in range(3, self._config.dof):
            joint_angles[i] = 0.1 * np.sin(i)
        
        return joint_angles