"""
Robot arm manager with thread-safe control, fixed frequency operation, and monitoring.
"""

import threading
import time
import queue
import logging
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np

from .base_arm import BaseRobotArm, ArmState, ArmConfig


class CommandType(Enum):
    """Command types for the control thread."""
    MOVE_JOINTS = auto()
    MOVE_JOINTS_VELOCITY = auto()
    MOVE_CARTESIAN = auto()
    STOP = auto()
    ENABLE = auto()
    DISABLE = auto()
    SHUTDOWN = auto()


@dataclass
class Command:
    """Command structure for the control thread."""
    type: CommandType
    data: Any = None
    callback: Optional[Callable] = None
    timeout: float = 5.0
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()


class RobotArmManager:
    """
    Robot arm manager with thread-safe operation.
    
    Provides:
    - Exclusive control thread for robot arm
    - Fixed frequency control loop
    - Command queue with timeout protection
    - State monitoring and logging
    - Optional trajectory filtering
    """
    
    def __init__(self, 
                 robot_arm: BaseRobotArm,
                 control_frequency: float = 100.0,
                 enable_filtering: bool = True,
                 log_level: int = logging.INFO):
        """
        Initialize the robot arm manager.
        
        Args:
            robot_arm: Robot arm instance
            control_frequency: Control loop frequency in Hz
            enable_filtering: Enable trajectory filtering
            log_level: Logging level
        """
        self._arm = robot_arm
        self._control_frequency = control_frequency
        self._control_period = 1.0 / control_frequency
        self._enable_filtering = enable_filtering
        
        # Setup logging
        self._logger = logging.getLogger(f"RobotArmManager[{robot_arm.config.name}]")
        self._logger.setLevel(log_level)
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            ))
            self._logger.addHandler(handler)
        
        # Thread control
        self._control_thread: Optional[threading.Thread] = None
        self._thread_running = False
        self._thread_lock = threading.Lock()
        
        # Command queue
        self._command_queue: queue.Queue[Command] = queue.Queue(maxsize=100)
        
        # State management
        self._current_state: Optional[ArmState] = None
        self._state_lock = threading.RLock()
        self._state_callbacks: List[Callable[[ArmState], None]] = []
        
        # Trajectory filter
        self._trajectory_filter = None
        if enable_filtering:
            from ..filters.trajectory_filter import TrajectoryFilter
            self._trajectory_filter = TrajectoryFilter(
                dof=robot_arm.config.dof,
                max_velocities=robot_arm.config.max_joint_velocities,
                max_accelerations=robot_arm.config.max_joint_accelerations
            )
        
        # Statistics
        self._control_loop_count = 0
        self._command_count = 0
        self._error_count = 0
        self._last_command_time = 0.0
        
    def start(self) -> bool:
        """
        Start the control thread.
        
        Returns:
            True if started successfully
        """
        with self._thread_lock:
            if self._thread_running:
                self._logger.warning("Control thread already running")
                return False
            
            # Connect to robot arm
            if not self._arm.is_connected:
                self._logger.info("Connecting to robot arm...")
                if not self._arm.connect():
                    self._logger.error("Failed to connect to robot arm")
                    return False
            
            # Start control thread
            self._thread_running = True
            self._control_thread = threading.Thread(
                target=self._control_loop,
                name=f"RobotArmControl[{self._arm.config.name}]"
            )
            self._control_thread.daemon = True
            self._control_thread.start()
            
            self._logger.info("Control thread started")
            return True
    
    def stop(self) -> bool:
        """
        Stop the control thread.
        
        Returns:
            True if stopped successfully
        """
        with self._thread_lock:
            if not self._thread_running:
                self._logger.warning("Control thread not running")
                return False
            
            # Send shutdown command
            self._command_queue.put(Command(CommandType.SHUTDOWN))
            
            # Wait for thread to stop
            if self._control_thread:
                self._control_thread.join(timeout=5.0)
                if self._control_thread.is_alive():
                    self._logger.error("Control thread failed to stop")
                    return False
            
            self._thread_running = False
            self._logger.info("Control thread stopped")
            return True
    
    def enable(self, timeout: float = 5.0) -> bool:
        """Enable the robot arm."""
        return self._send_command(Command(CommandType.ENABLE, timeout=timeout))
    
    def disable(self, timeout: float = 5.0) -> bool:
        """Disable the robot arm."""
        return self._send_command(Command(CommandType.DISABLE, timeout=timeout))
    
    def move_joints(self, joint_positions: np.ndarray,
                   velocity_limit: Optional[float] = None,
                   acceleration_limit: Optional[float] = None,
                   timeout: float = 5.0) -> bool:
        """
        Move to target joint positions.
        
        Args:
            joint_positions: Target joint positions
            velocity_limit: Optional velocity limit (0-1)
            acceleration_limit: Optional acceleration limit (0-1)
            timeout: Command timeout
            
        Returns:
            True if command accepted
        """
        # Validate positions
        is_valid, error = self._arm.validate_joint_positions(joint_positions)
        if not is_valid:
            self._logger.error(f"Invalid joint positions: {error}")
            return False
        
        # Apply filtering if enabled
        if self._enable_filtering and self._trajectory_filter:
            joint_positions = self._trajectory_filter.filter(joint_positions)
        
        data = {
            "positions": joint_positions.copy(),
            "velocity_limit": velocity_limit,
            "acceleration_limit": acceleration_limit
        }
        
        return self._send_command(Command(CommandType.MOVE_JOINTS, data, timeout=timeout))
    
    def move_joints_velocity(self, joint_velocities: np.ndarray,
                            timeout: float = 0.1) -> bool:
        """
        Move joints with specified velocities.
        
        Args:
            joint_velocities: Target joint velocities
            timeout: Command timeout (short for velocity control)
            
        Returns:
            True if command accepted
        """
        # Validate velocities
        is_valid, error = self._arm.validate_joint_velocities(joint_velocities)
        if not is_valid:
            self._logger.error(f"Invalid joint velocities: {error}")
            return False
        
        data = {"velocities": joint_velocities.copy()}
        
        return self._send_command(Command(CommandType.MOVE_JOINTS_VELOCITY, data, timeout=timeout))
    
    def move_cartesian(self, pose: np.ndarray,
                      velocity_limit: Optional[float] = None,
                      acceleration_limit: Optional[float] = None,
                      timeout: float = 5.0) -> bool:
        """
        Move to target Cartesian pose.
        
        Args:
            pose: Target pose [x, y, z, rx, ry, rz]
            velocity_limit: Optional velocity limit (0-1)
            acceleration_limit: Optional acceleration limit (0-1)
            timeout: Command timeout
            
        Returns:
            True if command accepted
        """
        data = {
            "pose": pose.copy(),
            "velocity_limit": velocity_limit,
            "acceleration_limit": acceleration_limit
        }
        
        return self._send_command(Command(CommandType.MOVE_CARTESIAN, data, timeout=timeout))
    
    def stop(self, timeout: float = 1.0) -> bool:
        """
        Stop the robot arm.
        
        Args:
            timeout: Command timeout
            
        Returns:
            True if command accepted
        """
        return self._send_command(Command(CommandType.STOP, timeout=timeout))
    
    def get_state(self) -> Optional[ArmState]:
        """
        Get current robot arm state.
        
        Returns:
            Current state or None if not available
        """
        with self._state_lock:
            return self._current_state
    
    def add_state_callback(self, callback: Callable[[ArmState], None]):
        """
        Add a callback to be called when state updates.
        
        Args:
            callback: Function to call with new state
        """
        with self._state_lock:
            self._state_callbacks.append(callback)
    
    def remove_state_callback(self, callback: Callable[[ArmState], None]):
        """
        Remove a state callback.
        
        Args:
            callback: Callback to remove
        """
        with self._state_lock:
            if callback in self._state_callbacks:
                self._state_callbacks.remove(callback)
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get manager statistics."""
        return {
            "control_loop_count": self._control_loop_count,
            "command_count": self._command_count,
            "error_count": self._error_count,
            "control_frequency": self._control_frequency,
            "thread_running": self._thread_running,
            "queue_size": self._command_queue.qsize(),
            "last_command_time": self._last_command_time
        }
    
    def _send_command(self, command: Command) -> bool:
        """Send command to control thread."""
        try:
            self._command_queue.put(command, timeout=0.1)
            self._command_count += 1
            self._last_command_time = time.time()
            return True
        except queue.Full:
            self._logger.error("Command queue full")
            return False
    
    def _control_loop(self):
        """Main control loop running in separate thread."""
        self._logger.info("Control loop started")
        last_loop_time = time.time()
        
        try:
            while self._thread_running:
                loop_start = time.time()
                
                # Process commands
                self._process_commands()
                
                # Update state
                self._update_state()
                
                # Control timing
                elapsed = time.time() - loop_start
                if elapsed < self._control_period:
                    time.sleep(self._control_period - elapsed)
                elif elapsed > self._control_period * 1.5:
                    self._logger.warning(f"Control loop took {elapsed:.3f}s, target is {self._control_period:.3f}s")
                
                self._control_loop_count += 1
                
        except Exception as e:
            self._logger.error(f"Control loop error: {e}", exc_info=True)
            self._error_count += 1
        finally:
            # Cleanup
            if self._arm.is_enabled:
                self._arm.disable()
            if self._arm.is_connected:
                self._arm.disconnect()
            self._logger.info("Control loop stopped")
    
    def _process_commands(self):
        """Process commands from queue."""
        current_time = time.time()
        
        try:
            # Get command with short timeout
            command = self._command_queue.get(timeout=0.001)
            
            # Check command timeout
            if current_time - command.timestamp > command.timeout:
                self._logger.warning(f"Command {command.type} timed out")
                if command.callback:
                    command.callback(False)
                return
            
            # Process command
            success = False
            
            if command.type == CommandType.SHUTDOWN:
                self._thread_running = False
                return
            
            elif command.type == CommandType.ENABLE:
                success = self._arm.enable()
                
            elif command.type == CommandType.DISABLE:
                success = self._arm.disable()
                
            elif command.type == CommandType.STOP:
                success = self._arm.stop()
                
            elif command.type == CommandType.MOVE_JOINTS:
                data = command.data
                success = self._arm.move_joints(
                    data["positions"],
                    data.get("velocity_limit"),
                    data.get("acceleration_limit")
                )
                
            elif command.type == CommandType.MOVE_JOINTS_VELOCITY:
                success = self._arm.move_joints_velocity(command.data["velocities"])
                
            elif command.type == CommandType.MOVE_CARTESIAN:
                data = command.data
                success = self._arm.move_cartesian(
                    data["pose"],
                    data.get("velocity_limit"),
                    data.get("acceleration_limit")
                )
            
            if not success:
                self._error_count += 1
                self._logger.error(f"Command {command.type} failed")
            
            # Call callback if provided
            if command.callback:
                command.callback(success)
                
        except queue.Empty:
            # No commands to process
            pass
        except Exception as e:
            self._logger.error(f"Error processing command: {e}")
            self._error_count += 1
    
    def _update_state(self):
        """Update robot arm state."""
        try:
            if self._arm.is_connected:
                new_state = self._arm.get_state()
                
                with self._state_lock:
                    self._current_state = new_state
                    
                    # Call callbacks
                    for callback in self._state_callbacks:
                        try:
                            callback(new_state)
                        except Exception as e:
                            self._logger.error(f"State callback error: {e}")
                            
        except Exception as e:
            self._logger.error(f"Error updating state: {e}")
            self._error_count += 1