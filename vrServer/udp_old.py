import cv2
import pyrealsense2 as rs
import numpy as np
import socket
import threading
import json
import sys
import os
import argparse

# ========== 添加 realman 模块路径 ==========
# 假设 realman 文件夹与本脚本同级，如有不同请自行调整
sys.path.append(os.path.abspath("."))

from realman.realman_utils import Realman, apply_rotation_delta
from realman.contoller_move import ControlMove

# ========== 配置 ==========
# 创建摄像头对应的pipeline和配置
pipelines = {}
VIDEO_PORT = 5006
CONTROL_PORT = 5005  # 控制端口，监听 Unity 控制命令

positionScale = 1
rotationScale = 1

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
    arms = {
        "left": Realman(hand_type="left"),
        "right": Realman(hand_type="right", mode_e=False),
    }
    try:
        for name in arms:
            arms[name].connect()
    except Exception as e:
        print(f"[Control] Failed to connect: {e}")
        return

    control = ControlMove()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[Control] Listening on {ip}:{port}")

    init_pose = {name: arms[name].read("Present_Pose").tolist() for name in arms}
    print(f"[Control] Initial Pose: {init_pose}")

    while True:
        data, addr = sock.recvfrom(65535)
        try:
            msg = data.decode("utf-8")
            if msg == "connect":
                for name in arms:
                    init_pose[name] = arms[name].read("Present_Pose").tolist()
                sock.sendto("connect success".encode("utf-8"), addr)
                print(f"[{addr}] CONNECT → {init_pose}")
            elif msg == "disconnect":
                sock.sendto("disconnect success".encode("utf-8"), addr)
                print(f"[{addr}] DISCONNECT")
            elif msg in ["start_stick_axis", "stop_stick_axis"]:
                print(f"[{addr}] {msg}")
            else:
                data = json.loads(msg)
                if data.get("x"):
                    handType = data["handType"]
                    if handType == "left":
                        control.start(0, data["y"])
                    else:
                        control.start(data["x"], 0)
                elif data.get("deltaPose"):
                    handType = data["handType"]
                    deltaPose = data["deltaPose"]
                    grip = data.get("grip", 0.0)

                    # 坐标变换
                    if handType == "left":
                        xyzDelta = np.array([-deltaPose[1], -deltaPose[2], -deltaPose[0]]) * positionScale
                    else:
                        xyzDelta = np.array([deltaPose[1], -deltaPose[2], deltaPose[0]]) * positionScale

                    rpy = apply_rotation_delta(handType, init_pose[handType][3:6], deltaPose[3:7], rotationScale)
                    xyz = init_pose[handType][:3] + xyzDelta
                    pose = np.concatenate([xyz, rpy, [grip]])

                    print(f"[{handType.upper()}] Pose → {pose.tolist()}")
                    arms[handType].write("Goal_Position", pose) 
        except Exception as e:
            print(f"[{addr}] Failed to parse message: {e}")

# ========== 启动入口 ==========
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="启动视频流或指令服务线程")
    parser.add_argument("--mode", choices=["video", "command", "both"], default="both",
                        help="运行模式: 'video' 只启动视频线程, 'command' 只启动指令线程, 'both' 启动全部")
    
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
        raise ValueError(f"不支持的 IP 配置: {args.ip}，请使用'17.16.2.88' 或 '17.16.2.206'")
    

    if args.mode == "video":
        t1 = threading.Thread(target=video_stream_thread, kwargs={"device_serials": DEVICE_SERIALS})
        t1.start()
        t1.join()

    elif args.mode == "command":
        t2 = threading.Thread(target=command_server_thread)
        t2.start()
        t2.join()

    elif args.mode == "both":
        t1 = threading.Thread(target=video_stream_thread, kwargs={"device_serials": DEVICE_SERIALS})
        t2 = threading.Thread(target=command_server_thread)
        t1.start()
        t2.start()
        t1.join()
        t2.join()