import pyrealsense2 as rs
import numpy as np
import cv2

# 创建 RealSense 设备管道
camera = rs.pipeline()
config = rs.config()

# 绑定设备（确保你的设备序列号正确）
device_serial = "130322273093"  # 确保这是你的 RealSense 设备的正确序列号
config.enable_device(device_serial)

# 启用彩色摄像头流（RGB）
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动管道
profile = camera.start(config)
print("RealSense Camera Started:", profile)

while True:
    # 获取数据帧
    frames = camera.wait_for_frames()
    color_frame = frames.get_color_frame()  # 获取彩色帧

    if not color_frame:
        continue  # 如果无效，跳过当前循环

    # 转换为 NumPy 格式
    color_image = np.asanyarray(color_frame.get_data())

    # 显示图像
    cv2.imshow('RealSense Frame', color_image)

    # 等待 1 毫秒，看是否有键盘事件
    key = cv2.waitKey(1) & 0xFF

    # 按 'q' 退出
    if key == ord('q'):
        break

# 释放资源
camera.stop()
cv2.destroyAllWindows()
