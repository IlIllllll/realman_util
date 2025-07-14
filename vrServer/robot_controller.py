import numpy as np
import threading
import time
from collections import deque
import sys
import os

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import RobotArmFactory, ArmConfig
from robot_arm_interface.plugins.realman_arm import RealmanRobotArm, rpy_to_rotation_vector, rotation_vector_to_rpy
from filters import DualHandFilter
from transform_utils import apply_rotation_delta

# ========== 机械臂运动控制类 ==========
class RobotArmController:
    def __init__(self, arm_config, filter_config=None):
        self.robot_arm = RobotArmFactory.create("realman", arm_config)
        self.robot_arm.connect()
        time.sleep(1)
        self.robot_arm.enable()
        print(f"[RobotArm] Connected to robot arm")
        
        # 运动控制参数
        self.control_frequency = 100.0  # 100Hz
        self.control_period = 1.0 / self.control_frequency
        self.interpolation_step = 1.0 / self.control_frequency  # 插值步长（秒）
        
        # 状态变量
        self.running = False
        self.current_pose = None
        self.target_pose = None
        self.motion_queue = deque()
        self.pose_lock = threading.Lock()
        self.last_move_joints = None
        
        # 初始化位置
        self.init_pose = {
            "left": None,
            "right": None
        }
        self.last_pose = {
            "left": None,
            "right": None
        }
        
        # 关节角状态跟踪
        self.current_joints = None
        self.last_joints = {
            "left": None,
            "right": None
        }
        
        # 初始化滑动窗口滤波器
        if filter_config:
            left_config = filter_config.get("left_filter_config")
            right_config = filter_config.get("right_filter_config")
            self.dual_hand_filter = DualHandFilter(left_filter_config=left_config, right_filter_config=right_config)
        else:
            self.dual_hand_filter = DualHandFilter()
        print(f"[RobotArm] Initialized sliding window filter")
        
    def set_initial_pose(self, left_pose, right_pose):
        """设置初始位置"""
        with self.pose_lock:
            self.init_pose["left"] = left_pose.copy()
            self.init_pose["right"] = right_pose.copy()
            self.last_pose["left"] = left_pose.copy()
            self.last_pose["right"] = right_pose.copy()
            self.current_pose = np.concatenate([left_pose, right_pose])
            
            # 计算初始关节角
            full_pose = np.concatenate([left_pose, right_pose])
            initial_joints, has_optimal = self.robot_arm.get_inverse_kinematics(full_pose)
            if has_optimal:
                self.current_joints = initial_joints
                self.last_joints["left"] = initial_joints[:7].copy()
                self.last_joints["right"] = initial_joints[7:].copy()
                print(f"[RobotArm] Initial joints calculated successfully")
            else:
                print(f"[RobotArm] Warning: No optimal solution for initial pose")
        print(f"[RobotArm] Initial pose set: left={left_pose[:3]}, right={right_pose[:3]}")
    
    def calculate_joint_interpolation_points(self, start_joints, end_joints, hand_type):
        """计算关节角插值点"""
        # 计算关节角度差
        joint_diff = np.abs(end_joints - start_joints)
        max_joint_diff = np.max(joint_diff)
        
        # 基于最大关节角度差计算时间
        max_joint_velocity = 60
        total_time = max(max_joint_diff / max_joint_velocity, 0.01)  # 最小0.1秒
        
        # 计算插值点数
        num_steps = int(total_time / self.interpolation_step)
        if num_steps < 1:
            num_steps = 1
            
        interpolation_points = []
        for i in range(num_steps + 1):
            t = i / num_steps
            # 线性插值关节角
            interp_joints = start_joints + t * (end_joints - start_joints)
            interp_joints[6] = end_joints[6]
            interp_joints[13] = end_joints[13]
            interpolation_points.append(interp_joints)
            
        return interpolation_points
    
    def update_both_arms_pose(self, left_delta_pose=None, right_delta_pose=None, left_grip=0.0, right_grip=0.0):
        """同时更新左右臂的目标位置"""
        with self.pose_lock:
            # 检查初始位置是否已设置
            if self.init_pose["left"] is None or self.init_pose["right"] is None:
                print(f"[RobotArm] Warning: Initial poses not set")
                return
            
            # 对输入数据进行滑动窗口滤波
            filtered_data = self.dual_hand_filter.filter_both_hands(
                left_delta_pose=left_delta_pose,
                right_delta_pose=right_delta_pose,
                left_grip=left_grip,
                right_grip=right_grip
            )
            
            # 使用滤波后的数据
            if left_delta_pose is not None:
                left_delta_pose = filtered_data["left_delta_pose"]
                left_grip = filtered_data["left_grip"]
            
            if right_delta_pose is not None:
                right_delta_pose = filtered_data["right_delta_pose"]
                right_grip = filtered_data["right_grip"]
            
            # 计算左臂新位置
            left_new_pose = None
            if left_delta_pose is not None:
                xyzDelta_left = np.array([-left_delta_pose[1], -left_delta_pose[2], -left_delta_pose[0]]) * positionScale
                rpy_left = apply_rotation_delta("left", self.init_pose["left"][3:6], left_delta_pose[3:7], rotationScale)
                xyz_left = self.init_pose["left"][:3] + xyzDelta_left
                left_new_pose = np.concatenate([xyz_left, rpy_left, [left_grip]])
            
            # 计算右臂新位置
            right_new_pose = None
            if right_delta_pose is not None:
                xyzDelta_right = np.array([right_delta_pose[1], -right_delta_pose[2], right_delta_pose[0]]) * positionScale
                rpy_right = apply_rotation_delta("right", self.init_pose["right"][3:6], right_delta_pose[3:7], rotationScale)
                xyz_right = self.init_pose["right"][:3] + xyzDelta_right
                right_new_pose = np.concatenate([xyz_right, rpy_right, [right_grip]])
            
            # 构建完整的目标位置
            full_target_pose = np.zeros(14)
            if left_new_pose is not None:
                full_target_pose[:7] = left_new_pose
            else:
                if self.last_pose["left"] is not None:
                    full_target_pose[:7] = self.last_pose["left"]
                else:
                    full_target_pose[:7] = self.init_pose["left"]  # 或 np.zeros(7)

            if right_new_pose is not None:
                full_target_pose[7:] = right_new_pose
            else:
                if self.last_pose["right"] is not None:
                    full_target_pose[7:] = self.last_pose["right"]
                else:
                    full_target_pose[7:] = self.init_pose["right"]  # 或 np.zeros(7)
            
            # 清空动作队列
            self.motion_queue.clear()

            # 计算逆运动学
            joint_positions, has_optimal_solution = self.robot_arm.get_inverse_kinematics(full_target_pose)
            if not has_optimal_solution:
                print(f"[RobotArm] Warning: No optimal solution for combined pose")
                return
        
            
            # 计算关节角插值点
            interpolation_points = self.calculate_joint_interpolation_points(
                self.current_joints, joint_positions, "both"
            )
            
            # 将插值点加入队列
            for joints in interpolation_points:
                self.motion_queue.append(joints)
            
            # 更新最后位置和关节角
            if left_new_pose is not None:
                self.last_pose["left"] = left_new_pose.copy()
                self.last_joints["left"] = joint_positions[:7].copy()
            if right_new_pose is not None:
                self.last_pose["right"] = right_new_pose.copy()
                self.last_joints["right"] = joint_positions[7:].copy()

    def move_to_joints(self, joints):
        """移动到指定关节角"""
        self.robot_arm.move_joints(joints, follow=False)
        self.current_joints = joints
    
    def control_loop(self):
        """控制循环，以100Hz运行"""
        print(f"[RobotArm] Starting control loop at {self.control_frequency}Hz")
        self.running = True
        
        while self.running:
            start_time = time.time()
            
            # 执行运动控制
            with self.pose_lock:
                if self.motion_queue:
                    target_joints = self.motion_queue.popleft()
                    try:
                        self.robot_arm.move_joints(target_joints, follow=False)
                        self.current_joints = target_joints
                    except Exception as e:
                        print(f"[RobotArm] Move error: {e}")
            
            # 控制频率
            elapsed = time.time() - start_time
            sleep_time = max(0, self.control_period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def start(self):
        """启动控制线程"""
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
    
    def stop(self):
        """停止控制"""
        self.running = False
        if hasattr(self, 'control_thread'):
            self.control_thread.join()
        self.robot_arm.disconnect()
        print(f"[RobotArm] Stopped control loop")
    
    def get_filter_stats(self):
        """获取滤波器统计信息"""
        return self.dual_hand_filter.get_stats()
    
    def reset_filters(self):
        """重置滤波器状态"""
        self.dual_hand_filter.reset()
        print(f"[RobotArm] Reset sliding window filters")
    
    def set_filter_parameters(self, hand_type, **kwargs):
        """
        设置滤波器参数
        
        Args:
            hand_type: "left" 或 "right"
            **kwargs: 滤波器参数 (window_size, filter_type, alpha)
        """
        if hand_type == "left":
            self.dual_hand_filter.set_left_filter_parameters(**kwargs)
            print(f"[RobotArm] Updated left hand filter parameters: {kwargs}")
        elif hand_type == "right":
            self.dual_hand_filter.set_right_filter_parameters(**kwargs)
            print(f"[RobotArm] Updated right hand filter parameters: {kwargs}")
        else:
            print(f"[RobotArm] Warning: Invalid hand type '{hand_type}', use 'left' or 'right'")
    

# 全局变量，用于存储缩放参数
positionScale = 1.2
rotationScale = 1

def set_scale_parameters(pos_scale=1, rot_scale=1):
    """设置位置和旋转缩放参数"""
    global positionScale, rotationScale
    positionScale = pos_scale
    rotationScale = rot_scale 