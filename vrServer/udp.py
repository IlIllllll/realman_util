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

# ========== 添加 realman 模块路径 ==========
# 假设 realman 文件夹与本脚本同级，如有不同请自行调整
# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import  RobotArmFactory, ArmConfig
from robot_arm_interface.plugins.realman_arm import RealmanRobotArm, rpy_to_rotation_vector, rotation_vector_to_rpy

# from realman.realman_utils import Realman, apply_rotation_delta
# from realman.contoller_move import ControlMove

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
        self.sock.bind((ip, port))
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
    sock.bind((ip, port))
    print(f"[Video] Listening on {ip}:{port}")
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
    # control = ControlMove()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[Control] Listening on {ip}:{port}")

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

    robot_arm = RobotArmFactory.create("realman", arm_config)
    robot_arm.connect()
    time.sleep(1)
    robot_arm.enable()
    print(f"[Control] Connected to robot arm")
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            msg = data.decode("utf-8")
            if msg == "connect":
                # for name in arms:
                #     init_pose[name] = arms[name].read("Present_Pose").tolist()
                sock.sendto("connect success".encode("utf-8"), addr)
                init_pose = {
                    "left": robot_arm.get_state().end_effector_pose[:7],
                    "right": robot_arm.get_state().end_effector_pose[7:]
                }
                last_pose = {
                    "left": init_pose["left"],
                    "right": init_pose["right"]
                }
                print(f"[Control] Initial Pose: {init_pose}")
                # print(f"[{addr}] CONNECT → {init_pose}")
            elif msg == "disconnect":
                sock.sendto("disconnect success".encode("utf-8"), addr)
                print(f"[{addr}] DISCONNECT")
            elif msg in ["start_stick_axis", "stop_stick_axis"]:
                print(f"[{addr}] {msg}")
            else:
                data = json.loads(msg)
                if data.get("x"):
                    # handType = data["handType"]
                    # if handType == "left":
                    #     control.start(0, data["y"])
                    # else:
                    #     control.start(data["x"], 0)
                    print(f"x[{addr}] {data}")
                elif data.get("deltaPose"):
                    print(f"deltaPose[{addr}] {data}")
                    handType = data["handType"]
                    deltaPose = data["deltaPose"]
                    grip = data.get("grip", 0.0)

                    # 坐标变换
                    if handType == "left":
                        xyzDelta = np.array([-deltaPose[1], -deltaPose[2], -deltaPose[0]]) * positionScale
                    else:
                        xyzDelta = np.array([deltaPose[1], -deltaPose[2], deltaPose[0]]) * positionScale
                    
                    rpy = apply_rotation_delta(handType, init_pose[handType][3:6], deltaPose[3:7], rotationScale)
                    xyz = init_pose[handType][:3] + xyzDelta * positionScale
                    pose = np.concatenate([xyz, rpy, [grip]])
                    
                    send_pose = np.zeros(14)
                    if handType == "left":
                        send_pose[:7] = pose
                        send_pose[7:] = last_pose["right"]
                    else:
                        send_pose[:7] = last_pose["left"]
                        send_pose[7:] = pose
                    # 发送的应该是上一个pose
                    robot_arm.move_cartesian(send_pose)
                    last_pose[handType] = pose
        except Exception as e:
            print(f"[{addr}] Failed to parse message: {e}")

# ========== 启动入口 ==========
if __name__ == "__main__":
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
    
    if "video" in args.mode:
        t1 = threading.Thread(target=video_stream_thread, kwargs={"device_serials": DEVICE_SERIALS})
        t1.start()

    if "command" in args.mode:
        t2 = threading.Thread(target=command_server_thread)
        t2.start()

    if "audio" in args.mode:
        audio_streamer = AudioStreamer()
        t3 = threading.Thread(target=audio_streamer.run)
        t3.start()

    # 等待所有线程结束
    if "video" in args.mode:
        t1.join()
    if "command" in args.mode:
        t2.join()
    if "audio" in args.mode:
        t3.join()