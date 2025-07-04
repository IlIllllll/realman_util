"""
Test script for robot arm interface.
"""

import unittest
import numpy as np
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_interface import RobotArmManager, RobotArmFactory, ArmConfig
from robot_arm_interface.filters.trajectory_filter import TrajectoryFilter
import robot_arm_interface.plugins.simulation_arm


class TestRobotArmInterface(unittest.TestCase):
    """Test cases for robot arm interface."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = ArmConfig(
            name="test_arm",
            dof=6,
            ip=["192.168.1.18"],
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
    
    def test_factory_registration(self):
        """Test factory registration and creation."""
        # Check if simulation arm is registered
        self.assertIn("simulation", RobotArmFactory.get_registered_types())
        
        # Create robot arm
        robot_arm = RobotArmFactory.create("simulation", self.config)
        self.assertIsNotNone(robot_arm)
        self.assertEqual(robot_arm.config.name, "test_arm")
    
    def test_arm_connection(self):
        """Test robot arm connection and disconnection."""
        robot_arm = RobotArmFactory.create("simulation", self.config)
        
        # Test connection
        self.assertTrue(robot_arm.connect())
        self.assertTrue(robot_arm.is_connected)
        
        # Test disconnection
        self.assertTrue(robot_arm.disconnect())
        self.assertFalse(robot_arm.is_connected)
    
    def test_manager_lifecycle(self):
        """Test manager start and stop."""
        robot_arm = RobotArmFactory.create("simulation", self.config)
        manager = RobotArmManager(robot_arm, control_frequency=100.0)
        
        # Test start
        self.assertTrue(manager.start())
        time.sleep(0.1)  # Let control thread start
        
        stats = manager.get_statistics()
        self.assertTrue(stats['thread_running'])
        
        # Test stop
        self.assertTrue(manager.stop())
        self.assertFalse(manager.get_statistics()['thread_running'])
    
    def test_joint_movement(self):
        """Test joint movement commands."""
        robot_arm = RobotArmFactory.create("simulation", self.config)
        manager = RobotArmManager(robot_arm, control_frequency=100.0)
        
        manager.start()
        manager.enable()
        
        try:
            # Test joint movement
            target = np.array([0.5, -0.3, 0.2, 0.0, 0.1, 0.0])
            self.assertTrue(manager.move_joints(target))
            
            # Wait for movement
            time.sleep(0.5)
            
            # Check state
            state = manager.get_state()
            self.assertIsNotNone(state)
            
        finally:
            manager.disable()
            manager.stop()
    
    def test_joint_limits(self):
        """Test joint limit validation."""
        robot_arm = RobotArmFactory.create("simulation", self.config)
        
        # Test valid positions
        valid_positions = np.zeros(6)
        is_valid, error = robot_arm.validate_joint_positions(valid_positions)
        self.assertTrue(is_valid)
        self.assertEqual(error, "")
        
        # Test invalid positions (beyond limits)
        invalid_positions = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        is_valid, error = robot_arm.validate_joint_positions(invalid_positions)
        self.assertFalse(is_valid)
        self.assertIn("out of limits", error)
    
    def test_trajectory_filter(self):
        """Test trajectory filtering."""
        filter = TrajectoryFilter(
            dof=6,
            max_velocities=self.config.max_joint_velocities,
            max_accelerations=self.config.max_joint_accelerations,
            filter_window=3,
            alpha=0.5
        )
        
        # Test filtering
        positions1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        positions2 = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        
        filtered1 = filter.filter(positions1)
        np.testing.assert_array_equal(filtered1, positions1)  # First position unchanged
        
        time.sleep(0.01)  # Small delay for time calculation
        filtered2 = filter.filter(positions2)
        
        # Filtered position should be between original positions
        self.assertTrue(np.all(filtered2 >= positions1))
        self.assertTrue(np.all(filtered2 <= positions2))
    
    def test_state_callback(self):
        """Test state callback mechanism."""
        robot_arm = RobotArmFactory.create("simulation", self.config)
        manager = RobotArmManager(robot_arm, control_frequency=100.0)
        
        # State storage
        states_received = []
        
        def state_callback(state):
            states_received.append(state)
        
        manager.add_state_callback(state_callback)
        manager.start()
        
        try:
            # Wait for some states
            time.sleep(0.1)
            
            # Check if callbacks were called
            self.assertGreater(len(states_received), 0)
            
            # Check state content
            for state in states_received:
                self.assertEqual(len(state.joint_positions), 6)
                self.assertEqual(len(state.joint_velocities), 6)
                
        finally:
            manager.remove_state_callback(state_callback)
            manager.stop()


class TestConfigLoader(unittest.TestCase):
    """Test configuration loading utilities."""
    
    def test_config_creation(self):
        """Test creating ArmConfig from dictionary."""
        from robot_arm_interface.utils.config_loader import create_arm_config
        
        config_dict = {
            "name": "test",
            "dof": 6,
            "max_joint_velocities": [1.0] * 6,
            "max_joint_accelerations": [2.0] * 6,
            "joint_limits_lower": [-3.14] * 6,
            "joint_limits_upper": [3.14] * 6,
            "control_frequency": 50.0
        }
        
        config = create_arm_config(config_dict)
        self.assertEqual(config.name, "test")
        self.assertEqual(config.dof, 6)
        self.assertEqual(config.control_frequency, 50.0)
        self.assertTrue(isinstance(config.max_joint_velocities, np.ndarray))


def run_integration_test():
    """Run a simple integration test."""
    print("\n=== Running Integration Test ===")
    
    # Create components
    config = ArmConfig(
        name="integration_test",
        dof=6,
        ip=["192.168.1.18"],
        max_joint_velocities=np.array([2.0] * 6),
        max_joint_accelerations=np.array([5.0] * 6),
        joint_limits_lower=np.array([-np.pi] * 6),
        joint_limits_upper=np.array([np.pi] * 6),
        control_frequency=100.0
    )
    
    robot_arm = RobotArmFactory.create("simulation", config)
    manager = RobotArmManager(robot_arm, control_frequency=100.0)
    
    # Run test sequence
    print("Starting manager...")
    manager.start()
    
    print("Enabling arm...")
    manager.enable()
    
    print("Moving to position...")
    target = np.array([0.5, -0.3, 0.2, 0.0, 0.1, 0.0])
    manager.move_joints(target, velocity_limit=0.5)
    
    time.sleep(2)
    
    state = manager.get_state()
    print(f"Final position: {np.round(state.joint_positions, 3)}")
    
    print("Cleaning up...")
    manager.disable()
    manager.stop()
    
    print("Integration test completed successfully!")


if __name__ == "__main__":
    # Run unit tests
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    # Run integration test
    run_integration_test()