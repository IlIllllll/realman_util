"""
Trajectory filter for smoothing robot arm movements.
"""

import numpy as np
from typing import Optional, List
from collections import deque
import time


class TrajectoryFilter:
    """
    Trajectory filter with multiple filtering methods.
    
    Provides:
    - Low-pass filtering
    - Velocity limiting
    - Acceleration limiting
    - Moving average
    """
    
    def __init__(self, 
                 dof: int,
                 max_velocities: np.ndarray,
                 max_accelerations: np.ndarray,
                 filter_window: int = 5,
                 alpha: float = 0.3):
        """
        Initialize trajectory filter.
        
        Args:
            dof: Degrees of freedom
            max_velocities: Maximum joint velocities (rad/s)
            max_accelerations: Maximum joint accelerations (rad/s^2)
            filter_window: Window size for moving average filter
            alpha: Low-pass filter coefficient (0-1, higher = less filtering)
        """
        self._dof = dof
        self._max_velocities = max_velocities
        self._max_accelerations = max_accelerations
        self._filter_window = filter_window
        self._alpha = alpha
        
        # State for filtering
        self._last_positions: Optional[np.ndarray] = None
        self._last_velocities: Optional[np.ndarray] = None
        self._last_time: Optional[float] = None
        self._position_history: deque = deque(maxlen=filter_window)
        
        # Low-pass filter state
        self._filtered_positions: Optional[np.ndarray] = None
        
    def filter(self, target_positions: np.ndarray) -> np.ndarray:
        """
        Apply all filters to target positions.
        
        Args:
            target_positions: Target joint positions
            
        Returns:
            Filtered joint positions
        """
        current_time = time.time()
        
        # Initialize if first call
        if self._last_positions is None:
            self._last_positions = target_positions.copy()
            self._last_velocities = np.zeros(self._dof)
            self._last_time = current_time
            self._filtered_positions = target_positions.copy()
            self._position_history.append(target_positions.copy())
            return target_positions
        
        # Calculate time delta
        dt = current_time - self._last_time
        if dt <= 0:
            dt = 0.001  # Minimum time step
        
        # Apply filters in sequence
        filtered = target_positions.copy()
        
        # 1. Moving average filter
        filtered = self._apply_moving_average(filtered)
        
        # 2. Low-pass filter
        filtered = self._apply_lowpass_filter(filtered)
        
        # 3. Velocity limiting
        filtered = self._apply_velocity_limit(filtered, dt)
        
        # 4. Acceleration limiting
        filtered = self._apply_acceleration_limit(filtered, dt)
        
        # Update state
        self._last_velocities = (filtered - self._last_positions) / dt
        self._last_positions = filtered.copy()
        self._last_time = current_time
        self._position_history.append(filtered.copy())
        
        return filtered
    
    def _apply_moving_average(self, positions: np.ndarray) -> np.ndarray:
        """Apply moving average filter."""
        self._position_history.append(positions.copy())
        
        if len(self._position_history) < 2:
            return positions
        
        # Calculate average
        avg_positions = np.mean(list(self._position_history), axis=0)
        
        return avg_positions
    
    def _apply_lowpass_filter(self, positions: np.ndarray) -> np.ndarray:
        """Apply exponential moving average (low-pass) filter."""
        if self._filtered_positions is None:
            self._filtered_positions = positions.copy()
            return positions
        
        # Exponential moving average
        self._filtered_positions = (self._alpha * positions + 
                                   (1 - self._alpha) * self._filtered_positions)
        
        return self._filtered_positions.copy()
    
    def _apply_velocity_limit(self, positions: np.ndarray, dt: float) -> np.ndarray:
        """Apply velocity limiting."""
        if self._last_positions is None:
            return positions
        
        # Calculate required velocities
        required_velocities = (positions - self._last_positions) / dt
        
        # Limit velocities
        limited_positions = positions.copy()
        for i in range(self._dof):
            if abs(required_velocities[i]) > self._max_velocities[i]:
                # Limit movement to maximum velocity
                max_delta = self._max_velocities[i] * dt
                delta = positions[i] - self._last_positions[i]
                
                if abs(delta) > max_delta:
                    limited_positions[i] = self._last_positions[i] + np.sign(delta) * max_delta
        
        return limited_positions
    
    def _apply_acceleration_limit(self, positions: np.ndarray, dt: float) -> np.ndarray:
        """Apply acceleration limiting."""
        if self._last_positions is None or self._last_velocities is None:
            return positions
        
        # Calculate required velocities and accelerations
        required_velocities = (positions - self._last_positions) / dt
        required_accelerations = (required_velocities - self._last_velocities) / dt
        
        # Limit accelerations
        limited_positions = positions.copy()
        for i in range(self._dof):
            if abs(required_accelerations[i]) > self._max_accelerations[i]:
                # Limit acceleration
                max_accel = self._max_accelerations[i]
                limited_accel = np.sign(required_accelerations[i]) * max_accel
                
                # Calculate limited velocity and position
                limited_velocity = self._last_velocities[i] + limited_accel * dt
                limited_positions[i] = self._last_positions[i] + limited_velocity * dt
        
        return limited_positions
    
    def reset(self):
        """Reset filter state."""
        self._last_positions = None
        self._last_velocities = None
        self._last_time = None
        self._position_history.clear()
        self._filtered_positions = None
    
    def set_parameters(self, 
                      filter_window: Optional[int] = None,
                      alpha: Optional[float] = None):
        """
        Update filter parameters.
        
        Args:
            filter_window: New window size for moving average
            alpha: New low-pass filter coefficient
        """
        if filter_window is not None and filter_window > 0:
            self._filter_window = filter_window
            self._position_history = deque(
                list(self._position_history)[-filter_window:], 
                maxlen=filter_window
            )
        
        if alpha is not None and 0 < alpha <= 1:
            self._alpha = alpha


class AdaptiveTrajectoryFilter(TrajectoryFilter):
    """
    Adaptive trajectory filter that adjusts filtering based on movement speed.
    
    Uses less filtering for slow movements and more filtering for fast movements.
    """
    
    def __init__(self,
                 dof: int,
                 max_velocities: np.ndarray,
                 max_accelerations: np.ndarray,
                 filter_window: int = 5,
                 alpha_min: float = 0.1,
                 alpha_max: float = 0.8):
        """
        Initialize adaptive trajectory filter.
        
        Args:
            dof: Degrees of freedom
            max_velocities: Maximum joint velocities
            max_accelerations: Maximum joint accelerations
            filter_window: Window size for moving average
            alpha_min: Minimum alpha (used for fast movements)
            alpha_max: Maximum alpha (used for slow movements)
        """
        super().__init__(dof, max_velocities, max_accelerations, 
                        filter_window, alpha_max)
        self._alpha_min = alpha_min
        self._alpha_max = alpha_max
        
    def _apply_lowpass_filter(self, positions: np.ndarray) -> np.ndarray:
        """Apply adaptive low-pass filter."""
        if self._filtered_positions is None or self._last_positions is None:
            self._filtered_positions = positions.copy()
            return positions
        
        # Calculate movement speed
        movement = np.linalg.norm(positions - self._last_positions)
        max_movement = np.linalg.norm(self._max_velocities * 0.01)  # Assume 10ms period
        
        # Adaptive alpha based on movement speed
        if max_movement > 0:
            speed_ratio = min(movement / max_movement, 1.0)
            # Fast movement -> low alpha (more filtering)
            # Slow movement -> high alpha (less filtering)
            adaptive_alpha = self._alpha_max - (self._alpha_max - self._alpha_min) * speed_ratio
        else:
            adaptive_alpha = self._alpha_max
        
        # Apply filter with adaptive alpha
        self._filtered_positions = (adaptive_alpha * positions + 
                                   (1 - adaptive_alpha) * self._filtered_positions)
        
        return self._filtered_positions.copy()


def create_filter(filter_type: str, **kwargs):
    """
    根据类型创建滤波器实例。
    filter_type: 'trajectory', 'adaptive', 'kalman'
    其余参数传递给具体滤波器构造函数。
    """
    if filter_type == 'trajectory':
        return TrajectoryFilter(**kwargs)
    elif filter_type == 'adaptive':
        return AdaptiveTrajectoryFilter(**kwargs)
    elif filter_type == 'kalman':
        from .kalman_filter import KalmanPoseFilter
        return KalmanPoseFilter(**kwargs)
    else:
        raise ValueError(f"Unknown filter type: {filter_type}")