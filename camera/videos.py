import pyrealsense2 as rs
import numpy as np
import cv2

# 创建 RealSense 上下文
ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    print("未检测到任何 RealSense 设备，程序退出。")
    exit(1)

print(f"检测到 {len(devices)} 台 RealSense 摄像头：")
serial_list = []
for i, dev in enumerate(devices):
    name = dev.get_info(rs.camera_info.name)
    serial = dev.get_info(rs.camera_info.serial_number)
    print(f"设备 {i+1}: 名称={name}, 序列号={serial}")
    serial_list.append(serial)

# 为每台摄像头创建独立的 pipeline
pipelines = []
for serial in serial_list:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    pipelines.append(pipeline)

print("所有摄像头已启动")

try:
    while True:
        frames_list = []
        for pipeline in pipelines:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                frames_list.append(color_image)

        # 拼接所有图像（按横向拼接）
        if frames_list:
            concat_image = np.hstack(frames_list)
            cv2.imshow("imgShow", concat_image)

        # 按 q 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    for pipeline in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()