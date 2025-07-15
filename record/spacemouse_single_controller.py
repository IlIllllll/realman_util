import sys
import os
import numpy as np
import time
import threading
from typing import List
import scipy.spatial.transform as st
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path
import shutil
import cv2

import select
import tty
import termios

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
import pyspacemouse
from camera.camera_thread import RealsenseCamera
from realman_arm.realman_utils import Realman

LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", "/home/ls/lerobot_dataset")).expanduser()

send_pose = None
dpos = None
drot_xyz = None
pose_lock = threading.Lock()

recording = False
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons)

def spacemouse_thread_func(realman, tx_zup_spnav, motion_event, button_state):
    global send_pose, dpos, drot_xyz
    prev_button_pressed = False
    while True:
        state = pyspacemouse.read()
        motion_event = np.array([state.x,state.y,state.z,state.roll,state.pitch,state.yaw])
        button_state = np.array(state.buttons)
        state = motion_event[:6] / 500

        tf_state = np.zeros_like(state)
        tf_state[:3] = tx_zup_spnav @ state[:3]
        tf_state[3:] = tx_zup_spnav @ state[3:]

        # xyz -> z -y x
        tf_state[:3] = np.array([-tf_state[0], -tf_state[1], -tf_state[2]])
        tf_state[3:6] = np.array([tf_state[5], tf_state[3], tf_state[4]])
        dpos_local = tf_state[:3] * (1)
        drot_xyz_local = tf_state[3:] * (1.2)

        if not button_state[1]:
            drot_xyz_local[:] = 0
        else:
            dpos_local[:] = 0

        drot = st.Rotation.from_euler('ZYX', drot_xyz_local)

        button_pressed = button_state[0]
        with pose_lock:
            if button_pressed and not prev_button_pressed:
                send_pose[6] = 1.0 - send_pose[6]

            send_pose[:3] += dpos_local
            send_pose[3:6] = (drot * st.Rotation.from_rotvec(send_pose[3:6])).as_rotvec()
            dpos = dpos_local.copy()
            drot_xyz = drot_xyz_local.copy()
            realman.servo_p(send_pose)

        prev_button_pressed = button_pressed
        time.sleep(0.01)

if __name__ == "__main__":
    # 初始化LeRobot数据集
    REPO_NAME = "single_arm/test_dp"
    output_path = LEROBOT_HOME / REPO_NAME
    if output_path.exists():
        dataset = LeRobotDataset( repo_id=REPO_NAME,local_files_only=True)
    else:
        dataset = LeRobotDataset.create(
            repo_id=REPO_NAME,
            robot_type="panda",
            fps=10,
            features={
                "top_image": {
                    "dtype": "video",
                    "shape": (480, 640, 3),
                    "names": ["height", "width", "channel"],
                },
                "wrist_image": {
                    "dtype": "video",
                    "shape": (480, 640, 3),
                    "names": ["height", "width", "channel"],
                },
                "state": {
                    "dtype": "float32",
                    "shape": (7,),
                    "names": ["state"],
                },
                "actions": {
                    "dtype": "float32",
                    "shape": (7,),
                    "names": ["actions"],
                },
                "joint": {
                    "dtype": "float32",
                    "shape": (7,),
                    "names": ["joint"],
                },
            },
            image_writer_threads=4,
            image_writer_processes=4,
        )
    # 初始化相机
    camera = RealsenseCamera()
    camera.start()
    import time
    time.sleep(2)  # 等待相机启动
    realman = Realman("192.168.1.19")
    realman.connect()
    tx_zup_spnav = np.array([
                [0,0,-1],
                [1,0,0],
                [0,1,0]
            ], dtype=np.float32)
    motion_event = np.zeros((6,), dtype=np.int64)
    button_state = np.zeros((2,), dtype=bool)
    prev_button_pressed = False
    send_pose = realman.getActualTCPPose()
    send_pose[6] = 1
    realman.servo_p(send_pose)
    time.sleep(0.1)

    # 启动空间鼠标线程
    t = threading.Thread(target=spacemouse_thread_func, args=(realman, tx_zup_spnav, motion_event, button_state), daemon=True)
    t.start()

    # 主线程10Hz采集
    try:
        while True:
            # 检查键盘输入
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'c' and not recording:
                    print("开始录制...")
                    recording = True
                if key.lower() == 'q':
                    print("退出...")
                    break
                elif key.lower() == 's' and recording:
                    print("停止录制，等待下一次开始...")
                    dataset.save_episode(task="pick up the blue cube", encode_videos=True)
                    recording = False
            if recording:
                with pose_lock:
                    pose = realman.getActualTCPPose()
                    joint = realman.getActualQ()
                    dpos_sample = dpos.copy()
                    drot_xyz_sample = drot_xyz.copy()
                frames = camera.get_frame()
                top_image = frames.get("top", None)
                wrist_image = frames.get("right", None)
                
                # # 打印图像数据类型信息
                # if top_image is not None:
                #     print(f"top_image dtype: {top_image.dtype}, shape: {top_image.shape}, min: {top_image.min()}, max: {top_image.max()}")
                # if wrist_image is not None:
                #     print(f"wrist_image dtype: {wrist_image.dtype}, shape: {wrist_image.shape}, min: {wrist_image.min()}, max: {wrist_image.max()}")
                
                if top_image is not None and wrist_image is not None:
                    dataset.add_frame({
                        "top_image": top_image,
                        "wrist_image": wrist_image,
                        "state": np.array(pose, dtype=np.float32),
                        "actions": np.array(joint, dtype=np.float32),
                        "joint": np.array(joint, dtype=np.float32),
                    })
                else:
                    print("相机未准备好")
            time.sleep(0.1)  # 10Hz
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        camera.stop()
        pyspacemouse.close()
