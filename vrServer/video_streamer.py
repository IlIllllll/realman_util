import cv2
import pyrealsense2 as rs
import numpy as np
import socket
import threading

# 创建摄像头对应的pipeline和配置
pipelines = {}

def video_stream_thread(ip="0.0.0.0", port=5006, device_serials=None):
    """
    视频流处理线程
    
    Args:
        ip: 监听IP地址
        port: 监听端口
        device_serials: 设备序列号字典
    """
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
        # 如果收到的是"start"命令，记录客户端地址
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