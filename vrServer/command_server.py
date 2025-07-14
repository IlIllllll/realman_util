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

# 全局变量
robot_controller = None

def command_server_thread(ip="0.0.0.0", port=5005, repo_id="dual_arm/test_dp"):
    """
    控制命令服务器线程
    
    Args:
        ip: 监听IP地址
        port: 监听端口
    """
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
    data_recorder = DataRecorder(robot_controller, camera, repo_id)
    
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
                    if handType == "right":
                        # 启动数据录制线程
                        try:
                            robot_controller.move_to_joints(np.array([-83,-11,-66,4,-99,6,0,90,-3,82,-5,88,-1.8,0]))
                            time.sleep(1)
                            # 获取当前机械臂状态
                            current_state = robot_controller.robot_arm.get_state()
                            left_pose = current_state.end_effector_pose[:7]
                            right_pose = current_state.end_effector_pose[7:]
                            
                            # 设置初始位置
                            robot_controller.set_initial_pose(left_pose, right_pose)      
                            sock.sendto("start_record success".encode("utf-8"), addr)                  
                            if data_recorder.start_record():
                                play_audio("start")
                                print(f"[{addr}] start_record")
                        except Exception as e:
                            print(f"[DataRecorder] 启动录制失败: {e}")
                            sock.sendto(f"start_record failed: {e}".encode("utf-8"), addr)
                    elif handType == "left":
                        # 删除数据集
                        sock.sendto("start_record success".encode("utf-8"), addr)
                        data_recorder.delete_dataset()
                        play_audio("stop")
                        print(f"[{addr}] delete_record")
                elif cmd == "stop_record":
                    if handType == "right":
                    # 停止数据录制并保存
                        try:
                            robot_controller.move_to_joints(np.array([-83,-11,-66,4,-99,6,0,90,-3,82,-5,88,-1.8,0]))
                            time.sleep(1)
                            # 获取当前机械臂状态
                            current_state = robot_controller.robot_arm.get_state()
                            left_pose = current_state.end_effector_pose[:7]
                            right_pose = current_state.end_effector_pose[7:]
                            
                            # 设置初始位置
                            robot_controller.set_initial_pose(left_pose, right_pose)
                            sock.sendto("stop_record success".encode("utf-8"), addr)                        
                            if data_recorder.stop_record():
                                play_audio("stop")
                                print(f"[{addr}] stop_record")
                        except Exception as e:
                            print(f"[DataRecorder] 停止录制失败: {e}")
                            sock.sendto(f"stop_record failed: {e}".encode("utf-8"), addr)
                    elif handType == "left":
                        # 删除数据集
                        sock.sendto("stop_record success".encode("utf-8"), addr)
                        data_recorder.delete_dataset()
                        play_audio("stop")
                        print(f"[{addr}] delete_record")
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
        if camera:
            camera.stop()
        # 停止数据录制线程（如果还在录制）
        try:
            if data_recorder and data_recorder.is_recording():
                data_recorder.stop_record()
        except Exception as e:
            print(f"[DataRecorder] 关闭录制线程异常: {e}")
        sock.close() 