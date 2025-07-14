"""
Realman robot arm plugin implementation.
"""

import numpy as np
import time
from typing import Optional
import logging
import sys
import os
import threading
from scipy.spatial.transform import Rotation as R

# Add parent directory to path to import realman modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from robot_arm_interface.core.arm_factory import register_robot_arm

try:
    from realman_arm.Robotic_Arm.rm_robot_interface import *
except ImportError:
    logging.warning("Realman arm modules not found. Realman plugin will not be available.")
    Arm = None

# rm_robot_interface 中有定义ArmState 需要覆盖，不能放到rm_robot_interface的引入前
from robot_arm_interface.core.base_arm import BaseRobotArm, ArmState, ArmConfig

single_rm_config = {
    "ip" : "192.168.1.18",
    "gripper": 0,
    "cur_pose": None,
    "cur_pos": [0, 90, 0, 0, 0, 0],
    "cur_speed": [0, 0, 0, 0, 0, 0]
}

def rpy_to_rotation_vector(rpy):
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    rotation = R.from_euler('ZYX', [roll, pitch, yaw], degrees=False)
    rotation_vector = rotation.as_rotvec()
    return rotation_vector

def rotation_vector_to_rpy(rotation_vector):
    rotation = R.from_rotvec(rotation_vector)
    rpy = rotation.as_euler('ZYX', degrees=False)
    return rpy

class GripperControlThread(threading.Thread):
    def __init__(self, robot, gripper_state, speed, force=None):
        super().__init__()
        self.robot = robot
        self.gripper_state = gripper_state
        self.speed = speed
        self.force = force
        self.daemon = True  # 设置为守护线程，主线程退出时自动结束

    def run(self):
        if self.gripper_state == 1:
            self.robot.rm_set_gripper_pick_on(self.speed, self.force, False, 0)
        else:
            self.robot.rm_set_gripper_release(self.speed, False, 0)


@register_robot_arm("realman")
class RealmanRobotArm(BaseRobotArm):
    """
    Realman robot arm implementation.
    
    Wraps the existing Realman arm API to conform to the unified interface.
    """
    
    def __init__(self, config: ArmConfig):
        """Initialize Realman robot arm."""
        super().__init__(config)
        
        # 初始化logger
        self._logger = logging.getLogger(f"RealmanArm[{config.name}]")

        self.rm_arm_num = config.dof // 7    # 单臂7个自由度
        self.robot_list = []
        self.robot_config_list = []
        self.gripper_control_thread_list = []
        self.dh_list = []
        
        self.arm_state_callback = rm_realtime_arm_state_callback_ptr(self.arm_state_func)

        self.arm_model = rm_robot_arm_model_e.RM_MODEL_RM_65_E  # RM_65机械臂
        self.force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 标准版

        # 初始化算法的机械臂及末端型号
        self.algo_handle = Algo(self.arm_model, self.force_type) 

        # 设置逆解求解模式
        self.algo_handle.rm_algo_set_redundant_parameter_traversal_mode(False)

        self.error_code = 0

    def arm_state_func(self, data):
        for i in range(self.rm_arm_num):
            if data.arm_ip.decode() ==  self.robot_config_list[i]["ip"]:
                waypoint = data.waypoint.to_dict()
                position = waypoint["position"]
                euler = waypoint["euler"]
                self.robot_config_list[i]["cur_pose"] = [position['x'], position['y'], position['z'], euler["rx"], euler["ry"], euler["rz"]]
                self.robot_config_list[i]["cur_pos"] = list(data.joint_status.joint_position[:6])
                self.robot_config_list[i]["cur_speed"] = list(data.joint_status.joint_speed[:6])

    def udp_watch(self):
        custom = rm_udp_custom_config_t()
        custom.joint_speed = 0
        custom.lift_state = 0
        custom.expand_state = 0
        custom.arm_current_status = 1
        config = rm_realtime_push_config_t(1, True, 8089, 0, "192.168.1.100", custom)
        self.robot_list[0].rm_set_realtime_push(config)
        self.robot_list[0].rm_get_realtime_push()
        self.robot_list[0].rm_realtime_arm_state_call_back(self.arm_state_callback)
        
    def connect(self, timeout: float = 5.0) -> bool:
        """Connect to Realman arm."""
        try:
            self._logger.info(f"Connecting to Realman arm")

            # 只有第一个机械臂需要mode_e ，其他机械臂使用默认模式，将机械臂放置在一个队列中
            for i in range(self.rm_arm_num):
                self.robot_list.append(RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E) if i == 0 else RoboticArm())
                current_config = single_rm_config.copy()
                current_config["ip"] = self._config.ip[i]
                ret = self.robot_list[i].rm_create_robot_arm(current_config["ip"], port=8080, level=3)
                if ret.id == -1:
                    self._logger.error(f"Failed to create robot arm {i}, ip: {current_config['ip']}, ret: {ret}")
                    raise Exception(f"Failed to create robot arm {i}")
                self.robot_config_list.append(current_config)
                self.gripper_control_thread_list.append(None)
                [ret, dh] = self.robot_list[i].rm_get_DH_data()
                if ret != 0:
                    self._logger.error(f"Get DH data failed for arm {i}: {ret}")
                    raise Exception(f"Get DH data failed for arm {i}")
                self.dh_list.append(dh)
                
            # 设置UDP监听
            self.udp_watch()
            
            self._is_connected = True
            self._logger.info("Successfully connected to Realman arm")

            for i in range(self.rm_arm_num):
                self.robot_list[i].rm_set_gripper_release(1000, False, 0)
                self.robot_config_list[i]["gripper"] = 0
            print("夹爪张开")
            return True
                
        except Exception as e:
            self._logger.error(f"Connection failed: {e}")
            self._is_connected = False
            self.error_code = 1
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from Realman arm."""
        try:
            for i in range(self.rm_arm_num):
                handle = self.robot_list[i].rm_delete_robot_arm()
                if handle == 0:
                    self._logger.info(f"Successfully disconnected from the robot arm {i},ip: {self.robot_config_list[i]['ip']}")
                else:
                    self._logger.error(f"Failed to disconnect from the robot arm {i},ip: {self.robot_config_list[i]['ip']}")
            self._is_connected = False
            self._logger.info("Disconnected from Realman arm")
            return True
        except Exception as e:
            self._logger.error(f"Disconnect failed: {e}")
            self.error_code = 2
            return False
    
    def enable(self) -> bool:
        """Enable the robot arm."""
        try:
            if not self._is_connected:
                self._logger.error("Arm not connected")
                return False
            
            self._is_enabled = True
            self._logger.info("Realman arm enabled")
            return True
            
        except Exception as e:
            self._logger.error(f"Enable failed: {e}")
            self.error_code = 3
            return False
    
    def disable(self) -> bool:
        """Disable the robot arm."""
        try:
            if not self._is_connected:
                self._logger.error("Arm not connected")
                return False
            
            self._is_enabled = False
            self._logger.info("Realman arm disabled")
            return True
            
        except Exception as e:
            self._logger.error(f"Disable failed: {e}")
            self.error_code = 4
            return False
    
    def get_state(self) -> ArmState:
        """Get current robot arm state."""
        try:
            current_time = time.time()
            
            # Get joint angles (convert from degrees to radians) , 将所有机械臂的关节角度拼接成一个数组
            joint_positions = np.zeros(self._config.dof)
            joint_velocities = np.zeros(self._config.dof)
            for i in range(self.rm_arm_num):
                joint_positions[i*7:(i+1)*7] = self.robot_config_list[i]["cur_pos"]+[self.robot_config_list[i]["gripper"]]
                joint_velocities[i*7:(i+1)*7] = self.robot_config_list[i]["cur_speed"]+[0]
            joint_positions = joint_positions
            joint_velocities = joint_velocities
            
            # Get joint torques (Realman doesn't provide this directly)
            joint_torques = np.zeros(self._config.dof)
            
            # Get end effector pose
            end_effector_pose = np.zeros(self.rm_arm_num * 7)
            for i in range(self.rm_arm_num):
                end_effector_pose[i*7:(i+1)*7] = self.robot_config_list[i]["cur_pose"]+[self.robot_config_list[i]["gripper"]]
            end_effector_pose = end_effector_pose
            
            # Check if moving
            is_moving = False
            for i in range(self.rm_arm_num):
                if self.robot_config_list[i]["cur_speed"] != [0, 0, 0, 0, 0, 0, 0]:
                    is_moving = True
                    break
            
            # Check for errors
            has_error = self.error_code != 0
            error_message = f"Error code: {self.error_code}" if has_error else ""
            
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
                end_effector_pose=np.zeros(self._config.dof),
                timestamp=time.time(),
                is_moving=False,
                has_error=True,
                error_message=str(e)
            )
    
    def move_joints(self, joint_positions: np.ndarray,
                   velocity_limit: Optional[float] = None,
                   acceleration_limit: Optional[float] = None,
                   follow: bool = True) -> bool:
        """Move to target joint positions."""
        try:
            if not self._is_enabled:
                self._logger.error("Arm not enabled")
                return False
            
            # Move joints
            for i in range(self.rm_arm_num):
                joint = joint_positions[i*7:(i+1)*7-1]
                gripper = joint_positions[(i+1)*7-1]
                self.robot_list[i].rm_movej_canfd(joint, follow=follow, expand=0, trajectory_mode=0, radio=10)
                if gripper == 1 and self.robot_config_list[i]["gripper"] != 1:
                    self.robot_config_list[i]["gripper"] = 1
                    self.gripper_control_thread_list[i] = GripperControlThread(self.robot_list[i], 1, 1000, 1000)
                    self.gripper_control_thread_list[i].start()
                elif gripper == 0 and self.robot_config_list[i]["gripper"] != 0:
                    self.robot_config_list[i]["gripper"] = 0
                    self.gripper_control_thread_list[i] = GripperControlThread(self.robot_list[i], 0, 1000, 1000)
                    self.gripper_control_thread_list[i].start()
            return True
            
        except Exception as e:
            self._logger.error(f"Move joints failed: {e}")
            self.error_code = 5
            return False
    
    def move_cartesian(self, pose: np.ndarray,
                      velocity_limit: Optional[float] = None,
                      acceleration_limit: Optional[float] = None,
                      follow: bool = True) -> bool:
        """Move to target Cartesian pose."""
        try:
            if not self._is_enabled:
                self._logger.error("Arm not enabled")
                return False

            for i in range(self.rm_arm_num):
                pose_gripper = pose[i*7:(i+1)*7]
                xyz = pose_gripper[:3]
                rpy = pose_gripper[3:6]
                gripper = pose_gripper[6] 

                arm_pose = np.concatenate((xyz, rpy))

                #逆运动学参数结构体
                params = rm_inverse_kinematics_params_t(self.robot_config_list[i]["cur_pos"], arm_pose, 1)

                # 计算逆运动学全解(当前仅支持六自由度机器人)
                result = self.algo_handle.rm_algo_inverse_kinematics_all(params)

                # 从多解中选取最优解(当前仅支持六自由度机器人)
                ret = self.algo_handle.rm_algo_ikine_select_ik_solve([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], result)
                if ret != -1:
                    self.robot_list[i].rm_movej_canfd(joint= list(result.q_solve[ret])[:6], follow=True, expand=0, trajectory_mode=2, radio=20)
                else: 
                    print("no best solution!")

                if gripper == 1 and self.robot_config_list[i]['gripper'] != 1:
                    self.robot_config_list[i]['gripper'] = gripper
                    self.gripper_control_thread_list[i] = GripperControlThread(self.robot_list[i], 1, 1000, 1000)
                    self.gripper_control_thread_list[i].start()
                if gripper == 0 and self.robot_config_list[i]['gripper'] != 0:
                    self.robot_config_list[i]['gripper'] = gripper
                    self.gripper_control_thread_list[i] = GripperControlThread(self.robot_list[i], 0, 1000, 1000)
                    self.gripper_control_thread_list[i].start()
            
            return True
            
        except Exception as e:
            self._logger.error(f"Move cartesian failed: {e}")
            self.error_code = 7
            return False
    
    def stop(self) -> bool:
        """Stop the robot arm immediately."""
        try:
            if not self._is_connected:
                return False
            
            for i in range(self.rm_arm_num):
                self.robot_list[i].rm_set_arm_stop()
            self._logger.info("Arm stopped")
            return True
            
        except Exception as e:
            self._logger.error(f"Stop failed: {e}")
            self.error_code = 8
            return False
    
    def get_forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        """Calculate forward kinematics.返回拼接后的位姿（位置+欧拉角+夹爪）"""
        try:
            pose_library = np.zeros(self.rm_arm_num * 7)
            for i in range(self.rm_arm_num):
                dh_struct = rm_dh_t(d=self.dh_list[i]['d'], a=self.dh_list[i]['a'], alpha=self.dh_list[i]['alpha'], offset=self.dh_list[i]['offset'])
                self.algo_handle.rm_algo_set_dh(dh_struct)
                
                # 使用库函数计算正向运动学
                target_joint_positions = joint_positions[i*7:(i+1)*7-1]
                pose_library[i*7:(i+1)*7] = self.algo_handle.rm_algo_forward_kinematics(target_joint_positions, flag=1)
            return pose_library
        except Exception as e:
            self._logger.error(f"Forward kinematics failed: {e}")
            self.error_code = 9
            return np.zeros(self.rm_arm_num * 7)
    
    def get_inverse_kinematics(self, pose: np.ndarray,
                              current_joints: Optional[np.ndarray] = None) -> tuple[Optional[np.ndarray], bool]:
        """Calculate inverse kinematics.
        
        Returns:
            tuple: (joint_positions, has_optimal_solution)
                - joint_positions: 关节角度数组，如果计算失败返回None
                - has_optimal_solution: 是否有最优解的标志
        """
        try:
            joint_positions = np.zeros(self.rm_arm_num * 7)
            has_optimal_solution = True
            
            for i in range(self.rm_arm_num):
                pose_gripper = pose[i*7:(i+1)*7]
                xyz = pose_gripper[:3]
                rpy = pose_gripper[3:6]
                arm_pose = np.concatenate((xyz, rpy))
                gripper = pose_gripper[6]

                # 逆运动学参数结构体
                params = rm_inverse_kinematics_params_t(self.robot_config_list[i]["cur_pos"], arm_pose, 1)

                # 计算逆运动学全解(当前仅支持六自由度机器人)
                result = self.algo_handle.rm_algo_inverse_kinematics_all(params)
                ret = self.algo_handle.rm_algo_ikine_select_ik_solve([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], result)
                if ret != -1:
                    joint_positions[i*7:(i+1)*7-1] = result.q_solve[ret][:6]
                else:
                    print(f"no best solution for arm {i}!")
                    joint_positions[i*7:(i+1)*7-1] = result.q_solve[0]
                    has_optimal_solution = False
                joint_positions[(i+1)*7-1] = gripper
            return joint_positions, has_optimal_solution
                
        except Exception as e:
            self._logger.error(f"Inverse kinematics failed: {e}")
            self.error_code = 10
            return None, False