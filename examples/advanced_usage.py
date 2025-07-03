"""
Advanced usage example with configuration files and trajectory filtering.
"""

import numpy as np
import time
import logging
import os

from robot_arm_interface import RobotArmManager, RobotArmFactory
from robot_arm_interface.utils.config_loader import (
    save_config_template,
    create_arm_from_config_file
)
from robot_arm_interface.filters.trajectory_filter import AdaptiveTrajectoryFilter

# Import plugins
import robot_arm_interface.plugins.simulation_arm


def demonstrate_configuration():
    """Demonstrate configuration file usage."""
    print("=== Configuration File Demo ===")
    
    # Save configuration templates
    config_dir = "configs"
    os.makedirs(config_dir, exist_ok=True)
    
    # Save simulation arm config
    sim_config_path = os.path.join(config_dir, "simulation_arm.yaml")
    save_config_template("simulation", sim_config_path, format="yaml")
    print(f"Saved simulation config to: {sim_config_path}")
    
    # Save realman arm config
    realman_config_path = os.path.join(config_dir, "realman_arm.yaml")
    save_config_template("realman", realman_config_path, format="yaml")
    print(f"Saved realman config to: {realman_config_path}")
    
    # Load and create arm from config
    robot_arm = create_arm_from_config_file(sim_config_path)
    print(f"Created robot arm: {robot_arm.config.name}")
    
    return robot_arm


def demonstrate_filtering(manager: RobotArmManager):
    """Demonstrate trajectory filtering effects."""
    print("\n=== Trajectory Filtering Demo ===")
    
    # Generate a sine wave trajectory
    duration = 5.0  # seconds
    frequency = 0.5  # Hz
    amplitude = 0.5  # radians
    
    print(f"Generating sine wave trajectory: {frequency}Hz, amplitude={amplitude}")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time
        
        # Generate sine wave for first two joints
        target = np.zeros(6)
        target[0] = amplitude * np.sin(2 * np.pi * frequency * t)
        target[1] = amplitude * np.cos(2 * np.pi * frequency * t)
        
        # Send command
        manager.move_joints(target, velocity_limit=1.0)
        time.sleep(0.01)  # 100Hz control
    
    manager.stop()
    print("Filtering demonstration complete")


def demonstrate_state_monitoring():
    """Demonstrate state monitoring and callbacks."""
    print("\n=== State Monitoring Demo ===")
    
    # Create data storage for monitoring
    state_history = {
        'timestamps': [],
        'positions': [],
        'velocities': [],
        'is_moving': []
    }
    
    def state_recorder(state):
        """Record state data for analysis."""
        state_history['timestamps'].append(state.timestamp)
        state_history['positions'].append(state.joint_positions.copy())
        state_history['velocities'].append(state.joint_velocities.copy())
        state_history['is_moving'].append(state.is_moving)
    
    return state_recorder, state_history


def demonstrate_error_handling(manager: RobotArmManager):
    """Demonstrate error handling and recovery."""
    print("\n=== Error Handling Demo ===")
    
    # Try to move beyond joint limits
    print("Testing joint limit validation...")
    invalid_position = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Way beyond limits
    success = manager.move_joints(invalid_position)
    print(f"Move to invalid position: {'rejected' if not success else 'accepted'}")
    
    # Try invalid velocity
    print("\nTesting velocity limit validation...")
    invalid_velocity = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Way too fast
    success = manager.move_joints_velocity(invalid_velocity)
    print(f"Invalid velocity command: {'rejected' if not success else 'accepted'}")


def main():
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create robot arm from configuration
    robot_arm = demonstrate_configuration()
    
    # Create manager with adaptive filtering
    manager = RobotArmManager(
        robot_arm=robot_arm,
        control_frequency=100.0,
        enable_filtering=True
    )
    
    # Set up state monitoring
    state_recorder, state_history = demonstrate_state_monitoring()
    manager.add_state_callback(state_recorder)
    
    # Start manager
    if not manager.start():
        print("Failed to start manager")
        return
    
    if not manager.enable():
        print("Failed to enable arm")
        manager.stop()
        return
    
    try:
        # Run demonstrations
        demonstrate_error_handling(manager)
        demonstrate_filtering(manager)
        
        # Analyze collected data
        print("\n=== State Analysis ===")
        print(f"Collected {len(state_history['timestamps'])} state samples")
        
        if state_history['positions']:
            positions = np.array(state_history['positions'])
            velocities = np.array(state_history['velocities'])
            
            print(f"Position range: [{np.min(positions):.3f}, {np.max(positions):.3f}]")
            print(f"Velocity range: [{np.min(velocities):.3f}, {np.max(velocities):.3f}]")
            
            # Calculate smoothness metric (lower is smoother)
            if len(velocities) > 1:
                accelerations = np.diff(velocities, axis=0)
                smoothness = np.mean(np.abs(accelerations))
                print(f"Trajectory smoothness metric: {smoothness:.4f}")
        
        # Show final statistics
        stats = manager.get_statistics()
        print(f"\nFinal statistics:")
        print(f"  Total control loops: {stats['control_loop_count']}")
        print(f"  Commands processed: {stats['command_count']}")
        print(f"  Errors encountered: {stats['error_count']}")
        print(f"  Control frequency: {stats['control_frequency']}Hz")
        
    finally:
        # Clean up
        manager.disable()
        manager.stop()
        manager.remove_state_callback(state_recorder)
    
    print("\nAdvanced example completed!")


if __name__ == "__main__":
    main()