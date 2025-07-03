from openpi_client import image_tools
from openpi_client import websocket_client_policy

import pyrealsense2 as rs
import numpy as np
from realman_arm.realman_utils import Realman
import time

# Outside of episode loop, initialize the policy client.
# Point to the host and port of the policy server (localhost and 8000 are the defaults).
client = websocket_client_policy.WebsocketClientPolicy(uri="wss://sv-fd9e865a-29de-4be8-8291-3bca742f241f-8000-x-defau-811fd93b30.sproxy.hd-01.alayanew.com:22443/")

class ArmHandler:
    def __init__(self, arms={"right": Realman(robot_ip="192.168.1.19")}):
        self.arms = arms
        self.upper_command = "idle_for_grab"
        self.last_command = "idle_for_grab"
        self.change_command_time = time.time()
        self.grap_time = 120 # seconds
        self.release_time = 120 # seconds
        self.state = "hold"
        try:
            for name in arms.keys():
                arms[name].connect()
        except Exception as e:
            print(f"[Control] Failed to connect: {e}")

    def update_upper_command(self,upper_command,camera_obs,arm_name="right"):
        self.last_command = self.upper_command
        self.upper_command = upper_command
        if self.last_command != self.upper_command:
            self.change_command_time = time.time()
        if self.upper_command == "idle_for_grab":
            self.hold_arms_open_loop_eep(gripper_command=0, arm_name=arm_name)
        elif self.upper_command == "idle_for_release":
            self.hold_arms_open_loop_eep(gripper_command=1, arm_name=arm_name)
        else:
            self.infer_arms(camera_obs, command=self.upper_command, arm_name=arm_name)

    def hold_arms_open_loop_eep(self,gripper_command,arm_name="right"):
        """
        Hold the arms in an open-loop control mode for a specified duration.
        :param gripper: The gripper to control.
        :param arm_name: The name of the arm to control.
        """
        if arm_name not in self.arms:
            raise ValueError(f"Arm '{arm_name}' not found in arms dictionary.")
        current_state = self.arms[arm_name].getActualTCPPose()
        current_state[6] = gripper_command  # Set gripper state
        self.arms[arm_name].servo_p(current_state)
        time.sleep(0.1)  # Adjust the sleep duration as needed

    def infer_arms(self, camera_obs, command='do nothing', arm_name="right"):
        """
        Infer actions for the arms based on the observation.
        :param observation: The observation containing camera images and state.
        :return: The action chunk to be executed by the arms.
        """
        arm_state = self.arms[arm_name].getActualTCPPose()
        if "release" in command:
            arm_state[6] = abs(arm_state[6])  # Ensure the gripper state is non-negative
        observation = {
            "observation/image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(camera_obs['top'], 224, 224)
            ),
            "observation/wrist_image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(camera_obs['right'], 224, 224)
            ),
            "observation/state": arm_state,
            "prompt": command,
        }
        action_chunk = client.infer(observation)["actions"]
        for action in action_chunk[:20]:
            action = list(action)  # Convert to a mutable list
            if action[6] < 0.01:
                action[6] = 0
            else:
                action[6] = 1
            self.arms[arm_name].servo_p(action)
            time.sleep(0.1)
        current_time = time.time()
        if "pick up" in command and current_time - self.change_command_time > self.grap_time:
            self.state = "succeeded"
        elif "put down" in command and current_time - self.change_command_time > self.release_time:
            self.state = "succeeded"
        else:
            self.state = "hold"

if __name__ == "__main__":
    # Connect arms
    arms = {
        "right": Realman(robot_ip="192.168.1.19"),
    }
    try:
        for name in arms:
            arms[name].connect()
    except Exception as e:
        print(f"[Control] Failed to connect: {e}")

    # 初始化摄像头 pipeline（只做一次）
    pipelines = {}
    DEVICE_SERIALS = {
        'right': '130322271356', # Realman右摄像头
        'top': '130322273093'      # Realman上摄像头
    }
    for name, serial in DEVICE_SERIALS.items():
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        pipelines[name] = pipeline

    NUM_STEPS = 200
    ACTION_INTERVAL = 0.1
    last_action = None  # 记录上一次执行的 action[:6]

    # raise_arm_angle = [87.23600006103516, 4.288000106811523, 78.60700225830078, -13.020999908447266, 80.54199981689453, -7.546999931335449]
    # state = arms['right'].getActualQ()
    # state[:6] = raise_arm_angle
    # state[6] = 0 
    # for i in range(50):
    #     arms['right'].servo_j(state)
    #     time.sleep(0.1)

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
            required_keys = ['top', 'right']
            if not all(k in frames for k in required_keys):
                print("[WARN] Incomplete frames, skipping...")
                continue
        except Exception as e:
            print("[ERROR] Send failed:", e)
            for arm in arms.values():
                arm.close()
            continue

        # 推理阶段
        infer_start_time = time.time()
        state = arms['right'].getActualTCPPose()
        observation = {
            "observation/image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(frames['top'], 224, 224)
            ),
            "observation/wrist_image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(frames['right'], 224, 224)
            ),
            "observation/state": state,
            "prompt": "put yellow cube into box",
        }
        action_chunk = client.infer(observation)["actions"]
        infer_end_time = time.time()
        inference_time = infer_end_time - infer_start_time
        print(f"[INFO] 推理时间: {inference_time:.3f}s")

        # 执行阶段
        for i, action in enumerate(action_chunk[:10]):
            # 平滑处理
            action = list(action)  # 确保action是可变的
            # 处理夹爪状态
            if action[6] < 0.01:
                action[6] = 0
            else:
                action[6] = 1
            arms['right'].move_to_pose(action)
            time.sleep(ACTION_INTERVAL)
            print("action:", action)
            print("[EXEC] Executed action at", time.time())
        
        step += 1
        print(f"[INFO] 完成步骤 {step}/{NUM_STEPS}")

    print("[INFO] 任务完成")
