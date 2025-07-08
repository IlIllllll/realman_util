import pyrealsense2 as rs  # 导入 RealSense SDK

# 创建一个 RealSense 上下文对象，用于管理连接的设备
ctx = rs.context()

# 查询当前连接的所有 RealSense 设备
devices = ctx.query_devices()

# 打印检测到的设备数量
print("检测到的 RealSense 设备：")

# 遍历所有检测到的设备
for dev in devices:
    # 获取设备名称
    device_name = dev.get_info(rs.camera_info.name)
    # 获取设备序列号（用于唯一标识每台设备）
    serial_number = dev.get_info(rs.camera_info.serial_number)

    # 打印设备信息
    print(f"设备名称: {device_name}")
    print(f"序列号: {serial_number}\n")