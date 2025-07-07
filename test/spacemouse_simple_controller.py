import sys
import os
import numpy as np
import time
import threading
from typing import List
import scipy.spatial.transform as st

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from realman_arm.realman_utils import Realman



if __name__ == "__main__":
    spnav_open()
    realman = Realman("192.168.1.19")
    realman.connect()
    tx_zup_spnav = np.array([
                [0,0,-1],
                [1,0,0],
                [0,1,0]
            ], dtype=np.float32)
    motion_event = np.zeros((7,), dtype=np.int64)
    button_state = np.zeros((10,), dtype=bool)
    prev_button_pressed = False
    send_pose = realman.getActualTCPPose()
    send_pose[6] = 1
    realman.servo_p(send_pose)
    time.sleep(0.1)
    while True:
        event = spnav_poll_event()
        if isinstance(event, SpnavMotionEvent):
            motion_event[:3] = event.translation
            motion_event[3:6] = event.rotation
            motion_event[6] = event.period
        elif isinstance(event, SpnavButtonEvent):
            button_state[event.bnum] = event.press
        state = motion_event[:6] / 500

        tf_state = np.zeros_like(state)
        tf_state[:3] = tx_zup_spnav @ state[:3]
        tf_state[3:] = tx_zup_spnav @ state[3:]

        # xyz -> z -y x
        tf_state[:3] = np.array([tf_state[2], -tf_state[1], tf_state[0]])
        tf_state[3:6] = np.array([-tf_state[4],tf_state[5],-tf_state[3]])
        dpos = tf_state[:3] * (0.005)
        drot_xyz = tf_state[3:] * (0.005)

        print(f"drot: {drot_xyz}")

        if not button_state[1]:
            drot_xyz[:] = 0
        else:
            dpos[:] = 0

        drot = st.Rotation.from_euler('ZYX', drot_xyz)

        button_pressed = button_state[0]
        if button_pressed and not prev_button_pressed:
            send_pose[6] = 1.0 - send_pose[6]

        send_pose[:3] += dpos
        send_pose[3:6] = (drot * st.Rotation.from_rotvec(send_pose[3:6])).as_rotvec()

        realman.servo_p(send_pose)

        prev_button_pressed = button_pressed
        # 鼠标会有通讯阻塞情况
        time.sleep(0.01)
    spnav_close()
