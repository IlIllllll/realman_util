"""
Robot arm factory for plugin-based arm creation.
"""

import importlib
import inspect
from typing import Dict, Type, Optional, Any, List
import logging

from .base_arm import BaseRobotArm, ArmConfig


class RobotArmFactory:
    """
    Factory class for creating robot arm instances.
    
    Supports:
    - Plugin registration
    - Dynamic loading
    - Configuration-based creation
    """
    
    # Class-level registry of robot arm types
    _registry: Dict[str, Type[BaseRobotArm]] = {}
    _logger = logging.getLogger("RobotArmFactory")
    
    @classmethod
    def register(cls, name: str, arm_class: Type[BaseRobotArm]):
        """
        Register a robot arm class.
        
        Args:
            name: Unique name for the arm type
            arm_class: Robot arm class (must inherit from BaseRobotArm)
        """
        if not inspect.isclass(arm_class) or not issubclass(arm_class, BaseRobotArm):
            raise ValueError(f"{arm_class} must be a subclass of BaseRobotArm")
        
        if name in cls._registry:
            cls._logger.warning(f"Overriding existing registration for {name}")
        
        cls._registry[name] = arm_class
        cls._logger.info(f"Registered robot arm type: {name}")
    
    @classmethod
    def unregister(cls, name: str):
        """
        Unregister a robot arm type.
        
        Args:
            name: Name of the arm type to unregister
        """
        if name in cls._registry:
            del cls._registry[name]
            cls._logger.info(f"Unregistered robot arm type: {name}")
    
    @classmethod
    def create(cls, name: str, config: ArmConfig) -> BaseRobotArm:
        """
        Create a robot arm instance.
        
        Args:
            name: Registered name of the arm type
            config: Arm configuration
            
        Returns:
            Robot arm instance
            
        Raises:
            ValueError: If arm type not registered
        """
        if name not in cls._registry:
            available = list(cls._registry.keys())
            raise ValueError(f"Unknown robot arm type: {name}. Available: {available}")
        
        arm_class = cls._registry[name]
        arm_instance = arm_class(config)
        
        cls._logger.info(f"Created {name} robot arm instance")
        return arm_instance
    
    @classmethod
    def create_from_config(cls, config_dict: Dict[str, Any]) -> BaseRobotArm:
        """
        Create a robot arm from configuration dictionary.
        
        Args:
            config_dict: Configuration dictionary with keys:
                - type: Robot arm type name
                - config: ArmConfig parameters
                
        Returns:
            Robot arm instance
        """
        arm_type = config_dict.get("type")
        if not arm_type:
            raise ValueError("Configuration must include 'type' field")
        
        config_params = config_dict.get("config", {})
        
        # Convert config parameters to ArmConfig
        import numpy as np
        arm_config = ArmConfig(
            name=config_params.get("name", arm_type),
            dof=config_params.get("dof", 6),
            max_joint_velocities=np.array(config_params.get("max_joint_velocities", [1.0] * 6)),
            max_joint_accelerations=np.array(config_params.get("max_joint_accelerations", [1.0] * 6)),
            joint_limits_lower=np.array(config_params.get("joint_limits_lower", [-np.pi] * 6)),
            joint_limits_upper=np.array(config_params.get("joint_limits_upper", [np.pi] * 6)),
            control_frequency=config_params.get("control_frequency", 100.0),
            connection_params=config_params.get("connection_params", {})
        )
        
        return cls.create(arm_type, arm_config)
    
    @classmethod
    def load_plugin(cls, module_path: str, class_name: str, register_name: Optional[str] = None):
        """
        Dynamically load and register a robot arm plugin.
        
        Args:
            module_path: Python module path (e.g., "my_package.my_module")
            class_name: Name of the robot arm class in the module
            register_name: Name to register the class as (defaults to class_name)
        """
        try:
            # Import the module
            module = importlib.import_module(module_path)
            
            # Get the class
            arm_class = getattr(module, class_name)
            
            # Register it
            register_name = register_name or class_name
            cls.register(register_name, arm_class)
            
            cls._logger.info(f"Loaded plugin {class_name} from {module_path}")
            
        except ImportError as e:
            cls._logger.error(f"Failed to import module {module_path}: {e}")
            raise
        except AttributeError as e:
            cls._logger.error(f"Class {class_name} not found in {module_path}: {e}")
            raise
    
    @classmethod
    def load_plugins_from_config(cls, plugins_config: List[Dict[str, str]]):
        """
        Load multiple plugins from configuration.
        
        Args:
            plugins_config: List of plugin configurations, each with:
                - module: Module path
                - class: Class name
                - name: Registration name (optional)
        """
        for plugin in plugins_config:
            module_path = plugin.get("module")
            class_name = plugin.get("class")
            register_name = plugin.get("name")
            
            if not module_path or not class_name:
                cls._logger.warning(f"Skipping invalid plugin config: {plugin}")
                continue
            
            try:
                cls.load_plugin(module_path, class_name, register_name)
            except Exception as e:
                cls._logger.error(f"Failed to load plugin {plugin}: {e}")
    
    @classmethod
    def get_registered_types(cls) -> List[str]:
        """Get list of registered robot arm types."""
        return list(cls._registry.keys())
    
    @classmethod
    def is_registered(cls, name: str) -> bool:
        """Check if a robot arm type is registered."""
        return name in cls._registry


# Decorator for auto-registration
def register_robot_arm(name: str):
    """
    Decorator to automatically register a robot arm class.
    
    Usage:
        @register_robot_arm("my_robot")
        class MyRobotArm(BaseRobotArm):
            ...
    """
    def decorator(cls):
        RobotArmFactory.register(name, cls)
        return cls
    return decorator