from openpi_client import image_tools
from openpi_client import websocket_client_policy

import pyrealsense2 as rs
import numpy as np
from realman_arm.realman_utils import Realman
import time
import threading

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

    action_dict = {}  # {timestamp: action}
    action_lock = threading.Lock()
    last_infer_time = time.time()  # 记录推理时间
    ACTION_EXPIRE_TIME = -0.5  # 动作过期时间（秒）
    INFER_INTERVAL = 0.1  # 推理间隔（秒）
    NUM_STEPS = 200
    ACTION_INTERVAL = 0.1

    # raise_arm_angle = [87.23600006103516, 4.288000106811523, 78.60700225830078, -13.020999908447266, 80.54199981689453, -7.546999931335449]
    # state = arms['right'].getActualQ()
    # state[:6] = raise_arm_angle
    # state[6] = 0 
    # for i in range(50):
    #     arms['right'].servo_j(state)
    #     time.sleep(0.1)

    def infer_thread():
        global last_infer_time
        step = 0
        while step < NUM_STEPS:
            try:
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
            infer_start_time = time.time()
            state = arms['right'].getActualQ()
            state[:6] = np.deg2rad(state[:6])
            observation = {
                "observation/image": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(frames['top'], 224, 224)
                ),
                "observation/wrist_image": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(frames['right'], 224, 224)
                ),
                "observation/state": state,
                "prompt": "clean_desk",
            }
            action_chunk = client.infer(observation)["actions"]
            now = time.time()

            with action_lock:
                # 清空所有旧动作
                action_dict.clear()
                
                # 计算推理延迟时间（从推理开始到填充队列的时间）
                inference_delay = now - infer_start_time
                
                # 添加新动作，跳过推理延迟期间的动作
                skipped_count = 0  # 统计跳过的步骤数
                inserted_count = 0  # 统计实际插入的步骤数
                
                for i, action in enumerate(action_chunk[:25]):
                    # 计算动作的相对时间
                    relative_time = (i+1) * ACTION_INTERVAL
                    
                    # 如果动作时间小于推理延迟，跳过这个动作
                    if relative_time < inference_delay:
                        skipped_count += 1
                        continue
                    
                    # 调整时间戳，减去推理延迟
                    adjusted_time = now + (relative_time - inference_delay)
                    action_dict[adjusted_time] = list(action)
                    inserted_count += 1
                
                # 输出统计信息
                print(f"[INFO] 推理延迟: {inference_delay:.3f}s, 跳过步骤: {skipped_count}, 插入步骤: {inserted_count}")
            step += 1
            after_time = time.time()
            print("infer time:", after_time - now)
            
    def exec_thread():
        last_action = None  # 记录上一次执行的 action[:6]
        while True:
            with action_lock:
                # 取所有动作，按相对时间戳排序
                if not action_dict:
                    action = None
                    relative_time = None
                else:
                    relative_time, action = sorted(action_dict.items(), key=lambda x: x[0])[0]
                    # 取出后删除
                    del action_dict[relative_time]
            if action is None:
                time.sleep(0.01)
                continue
            # 将相对时间戳转换为绝对执行时间
            exec_time = last_infer_time + relative_time
            now = time.time()
            if now > exec_time + ACTION_EXPIRE_TIME:
                print("[INFO] Discard expired action.")
                continue
            wait_time = exec_time - now
            if wait_time > 0:
                time.sleep(wait_time)
            # 平滑处理
            action[:6] = np.rad2deg(action[:6])
            if last_action is not None:
                diff = np.abs(np.array(action[:6]) - np.array(last_action))
                threshold = 0.1  # 角度阈值，可根据实际调整
                if np.any(diff > threshold):
                    # 差值过大，插入平滑点
                    num_interp = int(np.max(diff) // threshold)  # 插值点数
                    for i in range(1, num_interp + 1):
                        interp = np.array(last_action) + (np.array(action[:6]) - np.array(last_action)) * (i / (num_interp + 1))
                        smooth_action = action.copy()
                        smooth_action[:6] = interp.tolist()
                        arms['right'].servo_j(smooth_action)
                        print("smooth action:", smooth_action)
                        time.sleep(0.01)  # 插值点间隔，可调整
            if action[6] < 0.01:
                action[6] = 0
            else:
                action[6] = 1
            arms['right'].servo_j(action)
            time.sleep(0.01)
            print("action:", action)
            print("[EXEC] Executed action at", time.time())
            last_action = action[:6].copy()

    t_infer = threading.Thread(target=infer_thread, daemon=True)
    t_exec = threading.Thread(target=exec_thread, daemon=True)
    t_infer.start()
    t_exec.start()
    t_infer.join()
    # t_exec是daemon线程，主线程结束时自动退出
