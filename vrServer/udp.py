import cv2
import opuslib
import pyrealsense2 as rs
import sounddevice as sd
import numpy as np
import socket
import threading
import json
import time
import sys
import os
import argparse
from scipy.spatial.transform import Rotation as R
from collections import deque
import queue

# ========== 添加 realman 模块路径 ==========
# 假设 realman 文件夹与本脚本同级，如有不同请自行调整
# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import  RobotArmFactory, ArmConfig
from robot_arm_interface.plugins.realman_arm import RealmanRobotArm, rpy_to_rotation_vector, rotation_vector_to_rpy

# from realman.realman_utils import Realman, apply_rotation_delta
# from realman.contoller_move import ControlMove

# ========== 滑动窗口滤波类 ==========
class SlidingWindowFilter:
    """
    滑动窗口滤波器，用于平滑机械臂控制信号
    
    支持多种滤波方法：
    - 移动平均滤波
    - 加权移动平均滤波
    - 中值滤波
    - 指数移动平均滤波
    """
    
    def __init__(self, window_size=5, filter_type="moving_average", alpha=0.3):
        """
        初始化滑动窗口滤波器
        
        Args:
            window_size: 滑动窗口大小
            filter_type: 滤波类型 ("moving_average", "weighted_average", "median", "exponential")
            alpha: 指数移动平均的平滑系数 (0-1)
        """
        self.window_size = window_size
        self.filter_type = filter_type
        self.alpha = alpha
        
        # 数据存储
        self.data_buffer = deque(maxlen=window_size)
        self.filtered_value = None
        self.is_initialized = False
        
    def filter(self, new_value):
        """
        对新的输入值进行滤波
        
        Args:
            new_value: 新的输入值 (numpy array 或标量)
            
        Returns:
            滤波后的值
        """
        # 确保输入是numpy数组
        if not isinstance(new_value, np.ndarray):
            new_value = np.array(new_value)
        
        # 添加到缓冲区
        self.data_buffer.append(new_value.copy())
        
        # 如果数据不足，直接返回原始值
        if len(self.data_buffer) < 2:
            self.filtered_value = new_value.copy()
            self.is_initialized = True
            return self.filtered_value
        
        # 根据滤波类型进行处理
        if self.filter_type == "moving_average":
            self.filtered_value = self._moving_average()
        elif self.filter_type == "weighted_average":
            self.filtered_value = self._weighted_average()
        elif self.filter_type == "median":
            self.filtered_value = self._median_filter()
        elif self.filter_type == "exponential":
            self.filtered_value = self._exponential_average(new_value)
        else:
            # 默认使用移动平均
            self.filtered_value = self._moving_average()
        
        return self.filtered_value.copy()
    
    def _moving_average(self):
        """移动平均滤波"""
        return np.mean(list(self.data_buffer), axis=0)
    
    def _weighted_average(self):
        """加权移动平均滤波，越新的数据权重越大"""
        weights = np.linspace(0.1, 1.0, len(self.data_buffer))
        weights = weights / np.sum(weights)  # 归一化权重
        
        weighted_sum = np.zeros_like(self.data_buffer[0])
        for i, data in enumerate(self.data_buffer):
            weighted_sum += weights[i] * data
        
        return weighted_sum
    
    def _median_filter(self):
        """中值滤波"""
        data_array = np.array(list(self.data_buffer))
        return np.median(data_array, axis=0)
    
    def _exponential_average(self, new_value):
        """指数移动平均滤波"""
        if not self.is_initialized:
            self.filtered_value = new_value.copy()
            return self.filtered_value
        
        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        """重置滤波器状态"""
        self.data_buffer.clear()
        self.filtered_value = None
        self.is_initialized = False
    
    def set_parameters(self, window_size=None, filter_type=None, alpha=None):
        """更新滤波器参数"""
        if window_size is not None and window_size > 0:
            old_buffer = list(self.data_buffer)
            self.window_size = window_size
            self.data_buffer = deque(old_buffer[-window_size:], maxlen=window_size)
        
        if filter_type is not None:
            self.filter_type = filter_type
        
        if alpha is not None and 0 < alpha <= 1:
            self.alpha = alpha


class DualHandFilter:
    """
    双手滑动窗口滤波器，分别对左右手进行滤波处理
    """
    
    def __init__(self, left_filter_config=None, right_filter_config=None):
        """
        初始化双手滤波器
        
        Args:
            left_filter_config: 左手滤波器配置字典
            right_filter_config: 右手滤波器配置字典
        """
        # 默认配置
        default_config = {
            "window_size": 5,
            "filter_type": "moving_average",
            "alpha": 0.3
        }
        
        # 左手滤波器
        left_config = default_config.copy()
        if left_filter_config:
            left_config.update(left_filter_config)
        self.left_filter = SlidingWindowFilter(**left_config)
        
        # 右手滤波器
        right_config = default_config.copy()
        if right_filter_config:
            right_config.update(right_filter_config)
        self.right_filter = SlidingWindowFilter(**right_config)
        
        # 滤波统计
        self.filter_stats = {
            "left_calls": 0,
            "right_calls": 0,
            "both_calls": 0
        }
    
    def filter_left_hand(self, left_delta_pose, left_grip=0.0):
        """
        对左手数据进行滤波
        
        Args:
            left_delta_pose: 左手位置增量 [x, y, z, qx, qy, qz, qw]
            left_grip: 左手夹爪值
            
        Returns:
            滤波后的左手数据 (delta_pose, grip)
        """
        # 将位置和夹爪组合成一个数组进行滤波
        left_data = np.concatenate([left_delta_pose, [left_grip]])
        filtered_left_data = self.left_filter.filter(left_data)
        
        # 分离位置和夹爪
        filtered_delta_pose = filtered_left_data[:7]
        filtered_grip = filtered_left_data[7]
        
        self.filter_stats["left_calls"] += 1
        
        return filtered_delta_pose, filtered_grip
    
    def filter_right_hand(self, right_delta_pose, right_grip=0.0):
        """
        对右手数据进行滤波
        
        Args:
            right_delta_pose: 右手位置增量 [x, y, z, qx, qy, qz, qw]
            right_grip: 右手夹爪值
            
        Returns:
            滤波后的右手数据 (delta_pose, grip)
        """
        # 将位置和夹爪组合成一个数组进行滤波
        right_data = np.concatenate([right_delta_pose, [right_grip]])
        filtered_right_data = self.right_filter.filter(right_data)
        
        # 分离位置和夹爪
        filtered_delta_pose = filtered_right_data[:7]
        filtered_grip = filtered_right_data[7]
        
        self.filter_stats["right_calls"] += 1
        
        return filtered_delta_pose, filtered_grip
    
    def filter_both_hands(self, left_delta_pose=None, right_delta_pose=None, 
                         left_grip=0.0, right_grip=0.0):
        """
        对双手数据进行滤波
        
        Args:
            left_delta_pose: 左手位置增量
            right_delta_pose: 右手位置增量
            left_grip: 左手夹爪值
            right_grip: 右手夹爪值
            
        Returns:
            滤波后的双手数据字典
        """
        result = {}
        
        if left_delta_pose is not None:
            filtered_left_pose, filtered_left_grip = self.filter_left_hand(left_delta_pose, left_grip)
            result["left_delta_pose"] = filtered_left_pose
            result["left_grip"] = filtered_left_grip
        
        if right_delta_pose is not None:
            filtered_right_pose, filtered_right_grip = self.filter_right_hand(right_delta_pose, right_grip)
            result["right_delta_pose"] = filtered_right_pose
            result["right_grip"] = filtered_right_grip
        
        if left_delta_pose is not None and right_delta_pose is not None:
            self.filter_stats["both_calls"] += 1
        
        return result
    
    def reset(self):
        """重置两个滤波器"""
        self.left_filter.reset()
        self.right_filter.reset()
        self.filter_stats = {"left_calls": 0, "right_calls": 0, "both_calls": 0}
    
    def get_stats(self):
        """获取滤波统计信息"""
        return self.filter_stats.copy()
    
    def set_left_filter_parameters(self, **kwargs):
        """设置左手滤波器参数"""
        self.left_filter.set_parameters(**kwargs)
    
    def set_right_filter_parameters(self, **kwargs):
        """设置右手滤波器参数"""
        self.right_filter.set_parameters(**kwargs)

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
        print(f"[RobotArm] start_joints: {start_joints}")
        print(f"[RobotArm] end_joints: {end_joints}")
        joint_diff = np.abs(end_joints - start_joints)
        max_joint_diff = np.max(joint_diff)
        
        # 基于最大关节角度差计算时间
        max_joint_velocity = 10
        total_time = max(max_joint_diff / max_joint_velocity, 0.01)  # 最小0.1秒

        print(f"[RobotArm] max_joint_diff: {max_joint_diff}")
        
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
                xyz_left = self.init_pose["left"][:3] + xyzDelta_left * positionScale
                left_new_pose = np.concatenate([xyz_left, rpy_left, [left_grip]])
            
            # 计算右臂新位置
            right_new_pose = None
            if right_delta_pose is not None:
                xyzDelta_right = np.array([right_delta_pose[1], -right_delta_pose[2], right_delta_pose[0]]) * positionScale
                rpy_right = apply_rotation_delta("right", self.init_pose["right"][3:6], right_delta_pose[3:7], rotationScale)
                xyz_right = self.init_pose["right"][:3] + xyzDelta_right * positionScale
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
            
        # 打印更新信息
        if left_new_pose is not None and right_new_pose is not None:
            print(f"[RobotArm] Updated both arms: left={left_new_pose[:3]}, right={right_new_pose[:3]}")
        elif left_new_pose is not None:
            print(f"[RobotArm] Updated left arm: {left_new_pose[:3]}")
        elif right_new_pose is not None:
            print(f"[RobotArm] Updated right arm: {right_new_pose[:3]}")
    
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
                        self.robot_arm.move_joints(target_joints)
                        self.current_joints = target_joints
                        print(f"[RobotArm] Move joints: {target_joints}")
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

# ========== 配置 ==========
# 创建摄像头对应的pipeline和配置
pipelines = {}
AUDIO_PORT = 5007
VIDEO_PORT = 5006
CONTROL_PORT = 5005  # 控制端口，监听 Unity 控制命令

positionScale = 1
rotationScale = 1

def map_unity_quat_to_robot(unity_quat):
    # 输入 Unity 四元数：[x, y, z, w]
    r_unity = R.from_quat(unity_quat)
    
    # 构造坐标系旋转矩阵 M
    M = np.array([
        [-1, 0, 0], 
        [0, -1, 0],
        [0, 0, 1]
    ])
    
    # 旋转矩阵表示姿态变换
    R_unity_matrix = r_unity.as_matrix()
    R_robot_matrix = M @ R_unity_matrix @ M.T

    r_robot = R.from_matrix(R_robot_matrix)
    quat_xyzw = r_robot.as_quat()
    return quat_xyzw

def apply_rotation_delta(handType, init_euler_deg, delta_quat_unity, rotation_scale=1.0):
    # 将 Unity 四元数转换到机器人坐标系
    delta_quat_robot = map_unity_quat_to_robot(delta_quat_unity)

    # 把四元数转为 angle-axis
    delta_rot = R.from_quat(delta_quat_robot)
    angle_axis = delta_rot.as_rotvec()  # 这是 angle * axis 的形式

    if handType == "left":
        angle_axis[2] *= -1  # 手动反转绕 Z 的旋转
    if handType == "right":
        # ✅ 反转 X 轴旋转方向
        angle_axis[0] *= -1  # 手动反转绕 X 的旋转
        
    # 缩放角度
    scaled_rotvec = angle_axis * rotation_scale
    scaled_delta_rot = R.from_rotvec(scaled_rotvec)

    # 初始姿态
    init_rot = R.from_euler('ZYX', init_euler_deg, degrees=False)

    # 应用旋转增量
    final_rot = scaled_delta_rot * init_rot
    return final_rot.as_euler('ZYX', degrees=False)

# ========== 音频 ==========
class AudioStreamer:
    def __init__(self, ip="0.0.0.0", port=AUDIO_PORT):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 添加SO_REUSEADDR选项，允许端口重用
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.sock.bind((ip, port))
            print(f"[Audio] Successfully bound to {ip}:{port}")
        except OSError as e:
            print(f"[Audio] Failed to bind to port {port}: {e}")
            raise
        
        self.running = False
        self.target_addr = None

        # Opus 编码器初始化（采样率: 16000, 通道数: 1, 应用: VOIP）
        self.encoder = opuslib.Encoder(16000, 1, opuslib.APPLICATION_AUDIO)

    def audio_callback(self, indata, frames, time_info, status):
        if not self.running or self.target_addr is None:
            return
        pcm = (indata * 32767).astype(np.int16).tobytes()
        try:
            encoded = self.encoder.encode(pcm, frame_size=320)  # 20ms = 16000Hz * 0.02s = 320 samples
            self.sock.sendto(encoded, self.target_addr)
        except Exception as e:
            print("[Audio Encode Error]", e)

    def run(self):
        print(f"[Audio] Listening on {self.ip}:{self.port}")
        while True:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode()
            if msg == "start":
                self.target_addr = addr
                self.running = True
                print("[Audio] Received start. Begin streaming.")
                stream = sd.InputStream(channels=1, samplerate=16000, dtype='float32', callback=self.audio_callback, blocksize=320)
                with stream:
                    while self.running:
                        time.sleep(0.1)
            elif msg == "stop":
                print("[Audio] Received stop.")
                self.running = False

# ========== 视频流线程 ==========
def video_stream_thread(ip="0.0.0.0", port=VIDEO_PORT, device_serials=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 添加SO_REUSEADDR选项，允许端口重用
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((ip, port))
        print(f"[Video] Listening on {ip}:{port}")
    except OSError as e:
        print(f"[Video] Failed to bind to port {port}: {e}")
        return
    print(f"🔁 等待来自 Unity 的消息（命令或图像请求）...")
    client_addr = None
    while True:
        data, addr = sock.recvfrom(65535)
        # 如果收到的是“start”命令，记录客户端地址
        if data.decode(errors='ignore') == "start":
            client_addr = addr
            print(f"✅ 收到 start 命令，来自 {addr}")
            break  # 跳出等待，开始推流

    for name, serial in device_serials.items():
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        pipelines[name] = pipeline
    print(f"[Video] 开始推流")
    while True:
        # 采集每个命名摄像头的图像
        frames = {}
        for name, pipeline in pipelines.items():
            frame = pipeline.wait_for_frames().get_color_frame()
            if not frame:
                break
            frames[name] = np.asanyarray(frame.get_data())

        if len(frames) != 3:
            continue  # 如果没获取到全部三个图像就跳过

        # 保证固定顺序拼接（比如：左、上、右）
        img_left = cv2.rotate(frames['left'], cv2.ROTATE_90_CLOCKWISE)
        img_top = frames['top']
        img_right = cv2.rotate(frames['right'], cv2.ROTATE_90_CLOCKWISE)

        # 假设 img_left, img_top, img_right 是已经获取到的 3 幅图像
        # 1. 统一尺寸（可选）：resize 所有图像到一致大小
        # 2. 计算缩放尺寸
        top_h, top_w = img_top.shape[:2]
        thumb_w, thumb_h = top_w * 2 // 5, top_h * 2 // 5 # 缩小为 top 图像的 1/4

        # 3. 缩小左、右图像
        img_left_small = cv2.resize(img_left, (thumb_w, thumb_h))
        img_right_small = cv2.resize(img_right, (thumb_w, thumb_h))

        # 4. 拷贝 top 图像以便绘制
        combined_img = img_top.copy()

        # 5. 将缩小后的左图像粘贴到左上角
        combined_img[0:thumb_h, 0:thumb_w] = img_left_small

        # 6. 将缩小后的右图像粘贴到右上角
        combined_img[0:thumb_h, top_w - thumb_w:top_w] = img_right_small

        # 7. combined_img 就是合并后结果

        # JPEG 编码
        _, jpeg = cv2.imencode('.jpg', combined_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        # UDP 发送
        sock.sendto(jpeg.tobytes(), client_addr)

# ========== 控制线程 ==========
def command_server_thread(ip="0.0.0.0", port=CONTROL_PORT):
    global robot_controller
    
    # 检查端口是否被占用
    def is_port_in_use(port):
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            try:
                s.bind((ip, port))
                return False
            except OSError:
                return True
    
    # 如果端口被占用，尝试杀死占用进程
    if is_port_in_use(port):
        print(f"[Control] Port {port} is in use, attempting to kill existing process...")
        try:
            import subprocess
            result = subprocess.run(['lsof', '-ti', f':{port}'], capture_output=True, text=True)
            if result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        subprocess.run(['kill', '-9', pid])
                        print(f"[Control] Killed process {pid}")
                        time.sleep(1)  # 等待进程完全退出
        except Exception as e:
            print(f"[Control] Failed to kill existing process: {e}")
            return
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 添加SO_REUSEADDR选项，允许端口重用
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((ip, port))
        print(f"[Control] Listening on {ip}:{port}")
    except OSError as e:
        print(f"[Control] Failed to bind to port {port}: {e}")
        return

    # 创建机械臂配置
    arm_config = ArmConfig(
        name="double_arm",
        dof=14,
        ip=["192.168.1.18", "192.168.1.19"],
        max_joint_velocities=np.array([2.0] * 14),
        max_joint_accelerations=np.array([5.0] * 14),
        joint_limits_lower=np.array([-np.pi] * 14),
        joint_limits_upper=np.array([np.pi] * 14),
        control_frequency=100.0
    )

    # 创建滤波器配置
    filter_config = {
        "left_filter_config": {
            "window_size": 5,
            "filter_type": "moving_average",
            "alpha": 0.3
        },
        "right_filter_config": {
            "window_size": 5,
            "filter_type": "moving_average", 
            "alpha": 0.3
        }
    }
    
    # 创建机械臂控制器
    robot_controller = RobotArmController(arm_config, filter_config)
    robot_controller.start()
    
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            try:
                msg = data.decode("utf-8")
                data = json.loads(msg)
                cmd = data.get("cmd")
                handType = data.get("handType", "")
                if cmd == "connect":
                    sock.sendto("connect success".encode("utf-8"), addr)
                    # 获取当前机械臂状态
                    current_state = robot_controller.robot_arm.get_state()
                    left_pose = current_state.end_effector_pose[:7]
                    right_pose = current_state.end_effector_pose[7:]
                    
                    # 设置初始位置
                    robot_controller.set_initial_pose(left_pose, right_pose)
                    print(f"[Control] Initial Pose: left={left_pose[:3]}, right={right_pose[:3]}")
                    
                elif cmd == "disconnect":
                    sock.sendto("disconnect success".encode("utf-8"), addr)
                    print(f"[{addr}] DISCONNECT")
                    
                elif cmd in ["start_stick_axis", "stop_stick_axis"]:
                    print(f"[{addr}] {msg}")

                elif cmd == "start_record":
                    print(f"[{addr}] start_record")
                elif cmd == "stop_record":
                    print(f"[{addr}] stop_record")
                elif cmd == "filter_stats":
                    # 获取滤波器统计信息
                    stats = robot_controller.get_filter_stats()
                    response = {"filter_stats": stats}
                    sock.sendto(json.dumps(response).encode("utf-8"), addr)
                    print(f"[{addr}] Filter stats: {stats}")
                elif cmd == "reset_filters":
                    # 重置滤波器
                    robot_controller.reset_filters()
                    sock.sendto("filters reset".encode("utf-8"), addr)
                    print(f"[{addr}] Reset filters")
                elif cmd == "set_filter_params":
                    # 设置滤波器参数
                    hand_type = data.get("hand_type", "")
                    params = data.get("params", {})
                    if hand_type and params:
                        robot_controller.set_filter_parameters(hand_type, **params)
                        sock.sendto("filter params updated".encode("utf-8"), addr)
                        print(f"[{addr}] Set filter params for {hand_type}: {params}")
                    else:
                        sock.sendto("invalid filter params".encode("utf-8"), addr)
                elif cmd == "pose":
                    data = json.loads(msg)
                    if data.get("x"):
                        print(f"x[{addr}] {data}")
                    else:
                        # 处理机械臂控制消息
                        # 检查是否是左臂单独控制
                        if "left" in data and "right" not in data:
                            left_data = data["left"]
                            deltaPose = left_data["deltaPose"]
                            grip = left_data.get("grip", 0.0)
                            robot_controller.update_both_arms_pose(left_delta_pose=deltaPose, left_grip=grip)
                        
                        # 检查是否是右臂单独控制
                        elif "right" in data and "left" not in data:
                            right_data = data["right"]
                            deltaPose = right_data["deltaPose"]
                            grip = right_data.get("grip", 0.0)
                            robot_controller.update_both_arms_pose(right_delta_pose=deltaPose, right_grip=grip)
                        
                        # 检查是否是左右臂同时控制
                        elif "left" in data and "right" in data:
                            print(f"both arms control[{addr}] {data}")
                            left_data = data["left"]
                            right_data = data["right"]
                            left_deltaPose = left_data["deltaPose"]
                            right_deltaPose = right_data["deltaPose"]
                            left_grip = left_data.get("grip", 0.0)
                            right_grip = right_data.get("grip", 0.0)
                            robot_controller.update_both_arms_pose(
                                left_delta_pose=left_deltaPose, 
                                right_delta_pose=right_deltaPose,
                                left_grip=left_grip,
                                right_grip=right_grip
                            )
                        
            except Exception as e:
                print(f"[{addr}] Failed to parse message: {e}")
    except Exception as e:
        print(f"[Control] Thread error: {e}")
    finally:
        # 清理资源
        if robot_controller:
            robot_controller.stop()
        sock.close()

# ========== 启动入口 ==========
import signal

# 全局变量用于存储线程和控制器
threads = []
robot_controller = None
audio_streamer = None

def signal_handler(signum, frame):
    """信号处理函数，用于优雅地关闭程序"""
    print(f"\n[Main] 收到信号 {signum}，正在关闭程序...")
    
    # 停止机械臂控制器
    if robot_controller:
        print("[Main] 停止机械臂控制器...")
        robot_controller.stop()
    
    # 停止音频流
    if audio_streamer:
        print("[Main] 停止音频流...")
        audio_streamer.running = False
    
    # 等待所有线程结束
    for thread in threads:
        if thread.is_alive():
            thread.join(timeout=2)
    
    print("[Main] 程序已关闭")
    sys.exit(0)

if __name__ == "__main__":
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    parser = argparse.ArgumentParser(description="启动视频流或指令服务线程")
    parser.add_argument("--mode", nargs='+', choices=["video", "command", "audio"], default=["video", "command"],
                    help="运行模式: 可选 'video' 'command' 'audio'，支持多个，例如 --mode video command")
    
    parser.add_argument("--ip", type=str, default="17.16.2.88",
                    help="用于选择摄像头序列号配置，例如 '17.16.2.88' 或 '17.16.2.206'")
    args = parser.parse_args()
    print(f"运行模式: {args.mode}, 使用摄像头配置IP: {args.ip}")

    if args.ip == "17.16.2.88":
        DEVICE_SERIALS = {
            'left': '130322272067',
            'right': '130322271353',
            'top': '130322273389'
        }
    elif args.ip == "17.16.2.206":
        DEVICE_SERIALS = {
            'left': '130322270966',
            'right': '130322271356',
            'top': '130322273093'
        }
    else:
        raise ValueError(f"不支持的 IP 配置尾号: {args.ip}，请使用 '88' 或 '206'")
    
    try:
        if "video" in args.mode:
            t1 = threading.Thread(target=video_stream_thread, kwargs={"device_serials": DEVICE_SERIALS})
            t1.daemon = True
            t1.start()
            threads.append(t1)

        if "command" in args.mode:
            t2 = threading.Thread(target=command_server_thread)
            t2.daemon = True
            t2.start()
            threads.append(t2)

        if "audio" in args.mode:
            audio_streamer = AudioStreamer()
            t3 = threading.Thread(target=audio_streamer.run)
            t3.daemon = True
            t3.start()
            threads.append(t3)

        print("[Main] 所有服务已启动，按 Ctrl+C 退出")
        
        # 等待所有线程结束
        for thread in threads:
            thread.join()
            
    except KeyboardInterrupt:
        print("\n[Main] 收到键盘中断，正在关闭...")
        signal_handler(signal.SIGINT, None)
    except Exception as e:
        print(f"[Main] 发生错误: {e}")
        signal_handler(signal.SIGTERM, None)