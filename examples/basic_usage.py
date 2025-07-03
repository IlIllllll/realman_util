"""
Basic usage example of the robot arm interface.
"""

import numpy as np
import time
import logging

# Import robot arm interface components
from robot_arm_interface import RobotArmManager, RobotArmFactory, ArmConfig

# Import plugins to register them
import robot_arm_interface.plugins.simulation_arm
import robot_arm_interface.plugins.realman_arm


def main():
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Create a simulation robot arm configuration
    config = ArmConfig(
        name="test_arm",
        dof=6,
        max_joint_velocities=np.array([2.0] * 6),
        max_joint_accelerations=np.array([5.0] * 6),
        joint_limits_lower=np.array([-np.pi] * 6),
        joint_limits_upper=np.array([np.pi] * 6),
        control_frequency=100.0,
        connection_params={
            "simulation_delay": 0.001,
            "error_probability": 0.0
        }
    )
    
    # Create robot arm using factory
    print("Available robot arm types:", RobotArmFactory.get_registered_types())
    robot_arm = RobotArmFactory.create("simulation", config)
    
    # Create robot arm manager
    manager = RobotArmManager(
        robot_arm=robot_arm,
        control_frequency=100.0,
        enable_filtering=True
    )
    
    # Add state callback to monitor state
    def state_callback(state):
        print(f"State update: joints={np.round(state.joint_positions, 2)}, "
              f"moving={state.is_moving}")
    
    manager.add_state_callback(state_callback)
    
    # Start the manager
    print("\nStarting robot arm manager...")
    if not manager.start():
        print("Failed to start manager")
        return
    
    # Enable the arm
    print("\nEnabling robot arm...")
    if not manager.enable():
        print("Failed to enable arm")
        manager.stop()
        return
    
    try:
        # Move to home position
        print("\nMoving to home position...")
        home_position = np.zeros(6)
        manager.move_joints(home_position, velocity_limit=0.5)
        time.sleep(2)
        
        # Move to a target position
        print("\nMoving to target position...")
        target_position = np.array([0.5, -0.3, 0.2, 0.0, 0.1, 0.0])
        manager.move_joints(target_position, velocity_limit=0.3)
        time.sleep(3)
        
        # Test velocity control
        print("\nTesting velocity control...")
        velocities = np.array([0.1, -0.1, 0.0, 0.0, 0.0, 0.0])
        for _ in range(50):  # 0.5 seconds at 100Hz
            manager.move_joints_velocity(velocities)
            time.sleep(0.01)
        
        # Stop movement
        print("\nStopping...")
        manager.stop()
        time.sleep(1)
        
        # Move using Cartesian control
        print("\nMoving to Cartesian pose...")
        target_pose = np.array([0.4, 0.2, 0.3, 0.0, 0.0, 0.0])
        manager.move_cartesian(target_pose, velocity_limit=0.5)
        time.sleep(3)
        
        # Get current state
        state = manager.get_state()
        if state:
            print(f"\nCurrent state:")
            print(f"  Joint positions: {np.round(state.joint_positions, 3)}")
            print(f"  End effector pose: {np.round(state.end_effector_pose, 3)}")
            print(f"  Is moving: {state.is_moving}")
        
        # Get statistics
        stats = manager.get_statistics()
        print(f"\nManager statistics:")
        print(f"  Control loops: {stats['control_loop_count']}")
        print(f"  Commands sent: {stats['command_count']}")
        print(f"  Errors: {stats['error_count']}")
        
    finally:
        # Clean up
        print("\nDisabling robot arm...")
        manager.disable()
        time.sleep(0.5)
        
        print("Stopping manager...")
        manager.stop()
        
    print("\nExample completed!")


if __name__ == "__main__":
    main()