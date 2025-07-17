from openpi_client import image_tools
from openpi_client import websocket_client_policy
import os
import sys

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import pyrealsense2 as rs
import numpy as np
import robot_arm_interface.plugins.realman_arm
from robot_arm_interface import RobotArmFactory, ArmConfig
import time

# Outside of episode loop, initialize the policy client.
# Point to the host and port of the policy server (localhost and 8000 are the defaults).
client = websocket_client_policy.WebsocketClientPolicy(uri="wss://sv-6360e381-0998-4746-ac37-229543e01243-8000-x-defau-da2d73a9c8.sproxy.hd-01.alayanew.com:22443/")

if __name__ == "__main__":
    # 配置双臂
    config = ArmConfig(
        name="realman_double",
        dof=14,
        ip=["192.168.1.18", "192.168.1.19"],
        max_joint_velocities=np.array([3.14]*14),
        max_joint_accelerations=np.array([10.0]*14),
        joint_limits_lower=np.array([-3.05, -2.35, -2.52, -3.05, -1.57, -3.05, 0]*2),
        joint_limits_upper=np.array([3.05, 2.35, 0.77, 3.05, 1.57, 3.05, 1]*2),
        control_frequency=100.0,
        connection_params={"ip": ["192.168.1.18", "192.168.1.19"], "port": 8080}
    )
    arm = RobotArmFactory.create("realman", config)

    try:
        arm.connect()
        arm.enable()
    except Exception as e:
        print(f"[Control] Failed to connect/enable: {e}")
        exit(1)

    # 初始化摄像头 pipeline（只做一次）
    pipelines = {}
    DEVICE_SERIALS = {
        'left': '218622272712', # Realman左摄像头
        'right': '130322271356', # Realman右摄像头
        'top': '130322273093'      # Realman上摄像头
    }
    for name, serial in DEVICE_SERIALS.items():
        pipeline = rs.pipeline()
        config_rs = rs.config()
        config_rs.enable_device(serial)
        config_rs.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config_rs)
        pipelines[name] = pipeline

    NUM_STEPS = 200
    ACTION_INTERVAL = 0.1
    last_action = None  # 记录上一次执行的 action[:14]

    # 单线程推理-执行循环
    step = 0
    while step < NUM_STEPS:
        try:
            # 获取摄像头图像
            frames = {}
            for name, pipeline in pipelines.items():
                frame = pipeline.wait_for_frames().get_color_frame()
                if not frame:
                    break
                frames[name] = np.asanyarray(frame.get_data())
            required_keys = ['top', 'right', 'left']
            if not all(k in frames for k in required_keys):
                print("[WARN] Incomplete frames, skipping...")
                continue
        except Exception as e:
            print("[ERROR] Send failed:", e)
            arm.disable()
            arm.disconnect()
            continue

        # 推理阶段
        infer_start_time = time.time()
        state = arm.get_state().joint_positions  # 14维关节角度
        observation = {
            "images": {
                "cam_high": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(frames['top'], 224, 224)
                ).transpose(2, 0, 1),
                "cam_left_wrist": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(frames['left'], 224, 224)
                ).transpose(2, 0, 1),
                "cam_right_wrist": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(frames['right'], 224, 224)
                ).transpose(2, 0, 1),
            },
            "state": state,
            "prompt": "dual_arm task",
        }
        action_chunk = client.infer(observation)["actions"]
        infer_end_time = time.time()
        inference_time = infer_end_time - infer_start_time
        print(f"[INFO] 推理时间: {inference_time:.3f}s")

        # 执行阶段
        for i, action in enumerate(action_chunk[:10]):
            action = list(action)  # 确保action是可变的
            # 处理夹爪状态（每臂第7维）
            for j in [6, 13]:
                if action[j] < 0.5:
                    action[j] = 0
                else:
                    action[j] = 1
            arm.move_joints(np.array(action), follow=False)
            time.sleep(ACTION_INTERVAL)
            print(f"arm.get_state().joints: {arm.get_state().joint_positions}")
            print("action:", action)
            print("[EXEC] Executed action at", time.time())
        step += 1
        print(f"[INFO] 完成步骤 {step}/{NUM_STEPS}")

    print("[INFO] 任务完成")
    arm.disable()
    arm.disconnect()
