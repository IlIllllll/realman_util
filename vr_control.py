from realman_arm.realman_utils import Realman
import socket
import threading
import json
import numpy as np
from realman_arm.contoller_move import ControlMove
from realman_arm.realman_utils import apply_rotation_delta

# 控制参数
positionScale = 1
rotationScale = 1

# ========== 控制线程 ==========
def command_server_thread(ip="0.0.0.0", port=5005):
    arms = {
        "left": Realman(hand_type="left"),
        "right": Realman(hand_type="right", mode_e=False),
    }
    try:
        for name in arms:
            arms[name].connect()
    except Exception as e:
        print(f"[Control] Failed to connect: {e}")
        return

    control = ControlMove()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[Control] Listening on {ip}:{port}")

    init_pose = {name: arms[name].read("Present_Pose").tolist() for name in arms}
    print(f"[Control] Initial Pose: {init_pose}")

    while True:
        data, addr = sock.recvfrom(65535)
        try:
            msg = data.decode("utf-8")
            if msg == "connect":
                for name in arms:
                    init_pose[name] = arms[name].read("Present_Pose").tolist()
                sock.sendto("connect success".encode("utf-8"), addr)
                print(f"[{addr}] CONNECT → {init_pose}")
            elif msg == "disconnect":
                sock.sendto("disconnect success".encode("utf-8"), addr)
                print(f"[{addr}] DISCONNECT")
            elif msg in ["start_stick_axis", "stop_stick_axis"]:
                print(f"[{addr}] {msg}")
            else:
                data = json.loads(msg)
                if data.get("x"):
                    handType = data["handType"]
                    if handType == "left":
                        control.start(0, data["y"])
                    else:
                        control.start(data["x"], 0)
                elif data.get("deltaPose"):
                    handType = data["handType"]
                    deltaPose = data["deltaPose"]
                    grip = data.get("grip", 0.0)

                    # 坐标变换
                    if handType == "left":
                        xyzDelta = np.array([-deltaPose[1], -deltaPose[2], -deltaPose[0]]) * positionScale
                    else:
                        xyzDelta = np.array([deltaPose[1], -deltaPose[2], deltaPose[0]]) * positionScale

                    rpy = apply_rotation_delta(handType, init_pose[handType][3:6], deltaPose[3:7], rotationScale)
                    xyz = init_pose[handType][:3] + xyzDelta
                    pose = np.concatenate([xyz, rpy, [grip]])

                    print(f"[{handType.upper()}] Pose → {pose.tolist()}")
                    arms[handType].write("Goal_Position", pose) 
        except Exception as e:
            print(f"[{addr}] Failed to parse message: {e}")

if __name__ == "__main__":
    command_server_thread()

