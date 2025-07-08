import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

# 创建 RealSense 设备管道
camera = rs.pipeline()
config = rs.config()

# 绑定设备
device_serial = "130322271356"
config.enable_device(device_serial)

# 启用彩色摄像头流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动管道
profile = camera.start(config)
print("RealSense Camera Started:", profile)

# 初始化状态
recording = False
video_writer = None
video_dir = "recordings"
os.makedirs(video_dir, exist_ok=True)
frame_size = (640, 480)
fps = 30
clip_index = 0

def get_new_filename():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(video_dir, f"clip_{timestamp}_{clip_index:03d}.mp4")

while True:
    frames = camera.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())

    # 显示状态文本
    display_text = "Recording..." if recording else "Press 's' to start recording"
    cv2.putText(color_image, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 0, 255) if recording else (255, 255, 255), 2)
    cv2.imshow('RealSense Frame', color_image)

    if recording and video_writer is not None:
        video_writer.write(color_image)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        if not recording:
            clip_index += 1
            filename = get_new_filename()
            print(f"[INFO] Start recording: {filename}")
            video_writer = cv2.VideoWriter(
                filename, cv2.VideoWriter_fourcc(*'mp4v'), fps, frame_size)
            recording = True
        else:
            print("[INFO] Already recording. Press 'q' to stop current recording.")

    elif key == ord('q'):
        if recording:
            print("[INFO] Stop recording.")
            recording = False
            if video_writer:
                video_writer.release()
                video_writer = None
        else:
            print("[INFO] Not currently recording.")

    elif key == 27:  # ESC 键
        print("[INFO] Exiting program.")
        break

# 释放资源
if video_writer:
    video_writer.release()
camera.stop()
cv2.destroyAllWindows()