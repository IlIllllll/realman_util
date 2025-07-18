import socket
import json
import time
import subprocess
import numpy as np
import sys
import os
import threading

import pygame

# 初始化音频模块（只做一次）
pygame.mixer.init()

# 加载提示音
start_sound = pygame.mixer.Sound("sounds/action.wav")
stop_sound = pygame.mixer.Sound("sounds/ka.wav")

def play_audio(status):
    if status == "start":
        start_sound.play()
    elif status == "stop":
        stop_sound.play()

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import ArmConfig
from robot_controller import RobotArmController, set_scale_parameters
from data_recorder import DataRecorder
from camera.camera_thread import RealsenseCamera
from transform_utils import calculate_delta_pose

# 全局变量
robot_controller = None

# 按钮状态跟踪
button_states = {
    "left": {
        "primaryDown": False,
        "secondaryDown": False,
        "axisClickDown": False,
        "gripDown": False
    },
    "right": {
        "primaryDown": False,
        "secondaryDown": False,
        "axisClickDown": False,
        "gripDown": False
    }
}

# 位姿更新控制
pose_update_enabled = {
    "left": False,
    "right": False
}

# 记录上一次的位姿用于计算增量
last_poses = {
    "left": None,
    "right": None
}

def command_server_thread(ip="0.0.0.0", port=5005, repo_id="dual_arm/test_dp", streaming=False):
    """
    控制命令服务器线程
    
    Args:
        ip: 监听IP地址
        port: 监听端口
    """
    global robot_controller, button_states, pose_update_enabled, last_poses
    
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
    
    try:
        sock.bind((ip, port))
        print(f"[Control] Listening on {ip}:{port}")
    except OSError as e:
        print(f"[Control] Failed to bind to port {port}: {e}")
        return

    # 创建机械臂配置
    from config import get_arm_config, get_filter_config
    arm_config = get_arm_config()
    filter_config = get_filter_config()
    
    # 创建机械臂控制器
    robot_controller = RobotArmController(arm_config, filter_config)
    robot_controller.start()
    
    # 初始化相机
    camera = RealsenseCamera()
    camera.start()

    # 创建数据采集器
    data_recorder = DataRecorder(robot_controller, camera, repo_id, streaming=streaming)
    
    def handle_button_press(hand_type, button_name, is_pressed):
        """处理按钮按下事件"""
        global button_states, pose_update_enabled, data_recorder
        
        # 更新按钮状态
        button_states[hand_type][button_name] = is_pressed
        
        # 处理按钮按下事件（只在按下时处理，弹起时不处理）
        if is_pressed:
            if button_name == "primaryDown":
                # 切换位姿更新状态
                pose_update_enabled[hand_type] = not pose_update_enabled[hand_type]
                status = "enabled" if pose_update_enabled[hand_type] else "disabled"
                current_state = robot_controller.robot_arm.get_state()
                left_pose = current_state.end_effector_pose[:7]
                right_pose = current_state.end_effector_pose[7:]
                
                # 设置初始位置
                robot_controller.set_initial_pose(left_pose, right_pose)
                print(f"[Control] {hand_type} hand pose update {status}")
                
            elif button_name == "secondaryDown":
                if hand_type == "left":
                    # 左手secondaryDown：删除数据
                    try:
                        data_recorder.delete_dataset()
                        play_audio("stop")
                        print(f"[Control] {hand_type} hand: dataset deleted")
                    except Exception as e:
                        print(f"[DataRecorder] 删除数据集失败: {e}")
                        
                elif hand_type == "right":
                    # 右手secondaryDown：开始或结束录制
                    if data_recorder.is_recording():
                        # 停止录制
                        try:
                            robot_controller.move_to_joints(np.array([-83,-11,-66,4,-99,6,0,90,-3,82,-5,88,-1.8,0]))
                            time.sleep(1)
                            # 获取当前机械臂状态
                            current_state = robot_controller.robot_arm.get_state()
                            left_pose = current_state.end_effector_pose[:7]
                            right_pose = current_state.end_effector_pose[7:]
                            
                            # 设置初始位置
                            robot_controller.set_initial_pose(left_pose, right_pose)
                            if data_recorder.stop_record():
                                play_audio("stop")
                                print(f"[Control] {hand_type} hand: recording stopped")
                        except Exception as e:
                            print(f"[DataRecorder] 停止录制失败: {e}")
                    else:
                        # 开始录制
                        try:
                            robot_controller.move_to_joints(np.array([-83,-11,-66,4,-99,6,0,90,-3,82,-5,88,-1.8,0]))
                            time.sleep(1)
                            # 获取当前机械臂状态
                            current_state = robot_controller.robot_arm.get_state()
                            left_pose = current_state.end_effector_pose[:7]
                            right_pose = current_state.end_effector_pose[7:]
                            
                            # 设置初始位置
                            robot_controller.set_initial_pose(left_pose, right_pose)
                            if data_recorder.start_record():
                                play_audio("start")
                                print(f"[Control] {hand_type} hand: recording started")
                        except Exception as e:
                            print(f"[DataRecorder] 启动录制失败: {e}")
    
    def process_pose_data(data):
        """处理位姿数据"""
        global last_poses, pose_update_enabled
        
        left_delta_pose = None
        right_delta_pose = None
        left_grip = 0.0
        right_grip = 0.0
        
        # 处理左手数据
        if "left" in data:
            left_data = data["left"]
            current_left_pose = left_data["pose"]
            
            # 检查位姿更新是否启用
            if pose_update_enabled["left"]:
                if last_poses["left"] is not None:
                    # 计算增量位姿
                    left_delta_pose = calculate_delta_pose(last_poses["left"], current_left_pose)
                else:
                    # 第一次接收到数据，设置为零增量
                    left_delta_pose = np.zeros(7)
            else:
                # 更新最后位姿
                last_poses["left"] = current_left_pose.copy()
            
            # 处理夹爪
            left_grip = 1.0 if left_data.get("gripDown", False) else 0.0
            
            # 处理按钮状态变化
            for button_name in ["primaryDown", "secondaryDown", "axisClickDown"]:
                current_state = left_data.get(button_name, False)
                if current_state != button_states["left"][button_name]:
                    handle_button_press("left", button_name, current_state)
        
        # 处理右手数据
        if "right" in data:
            right_data = data["right"]
            current_right_pose = right_data["pose"]
            
            # 检查位姿更新是否启用
            if pose_update_enabled["right"]:
                if last_poses["right"] is not None:
                    # 计算增量位姿
                    right_delta_pose = calculate_delta_pose(last_poses["right"], current_right_pose)
                else:
                    # 第一次接收到数据，设置为零增量
                    right_delta_pose = np.zeros(7)
            else:
                # 更新最后位姿
                last_poses["right"] = current_right_pose.copy()
            
            # 处理夹爪
            right_grip = 1.0 if right_data.get("gripDown", False) else 0.0
            
            # 处理按钮状态变化
            for button_name in ["primaryDown", "secondaryDown", "axisClickDown"]:
                current_state = right_data.get(button_name, False)
                if current_state != button_states["right"][button_name]:
                    handle_button_press("right", button_name, current_state)
        
        # 更新机械臂位姿
        if left_delta_pose is not None or right_delta_pose is not None:
            print(f"[Control] left_delta_pose: {left_delta_pose}, right_delta_pose: {right_delta_pose}")
            robot_controller.update_both_arms_pose(
                left_delta_pose=left_delta_pose,
                right_delta_pose=right_delta_pose,
                left_grip=left_grip,
                right_grip=right_grip
            )
    
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            try:
                msg = data.decode("utf-8")
                data = json.loads(msg)
                
                # 处理新的通信协议格式
                if "left" in data or "right" in data:
                    process_pose_data(data)
                else:
                    print(f"[Control] Received unknown message format: {data}")
                        
            except Exception as e:
                print(f"[{addr}] Failed to parse message: {e}")
    except Exception as e:
        print(f"[Control] Thread error: {e}")
    finally:
        # 清理资源
        if robot_controller:
            robot_controller.stop()
        if camera:
            camera.stop()
        # 停止数据录制线程（如果还在录制）
        try:
            if data_recorder and data_recorder.is_recording():
                data_recorder.stop_record()
        except Exception as e:
            print(f"[DataRecorder] 关闭录制线程异常: {e}")
        sock.close() 