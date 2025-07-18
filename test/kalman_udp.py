import socket
import time
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from kalman_filter_test import RobotArmKalmanFilter
import pandas as pd

ip = "0.0.0.0"
port = 5005

def quat_to_euler(q):
    # q: [x, y, z, w]
    r = R.from_quat(q)
    return r.as_euler('xyz', degrees=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))
print(f"[Kalman] Listening on {ip}:{port}")

raw_x = []
raw_yaw = []
filtered_x = []
filtered_yaw = []
timestamps = []

data_list = []
start_time = time.time()
last_time = start_time

print("开始接收数据（20秒）...")
while True:
    now = time.time()
    if now - start_time > 20:
        break
    sock.settimeout(start_time + 20 - now)
    try:
        data, addr = sock.recvfrom(65535)
        msg = data.decode("utf-8")
        d = json.loads(msg)
        pose = d["right"]["pose"]
        x = pose[0]
        y = pose[1]
        z = pose[2]
        quat = pose[3:7]
        euler = quat_to_euler(quat)
        yaw = euler[0]  # xyz顺序，yaw为第一个
        pitch = euler[1]
        roll = euler[2]
        dt = now - last_time if last_time != start_time else 0.01
        data_list.append({
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
            "dt": dt,
            "raw": [x, y, z, yaw, pitch, roll],
        })
        last_time = now
    except Exception as e:
        print(f"[Kalman] Failed to decode message: {e}")

# 构造测量序列和dt序列
dt_seq = [d["dt"] for d in data_list]
meas_seq = [[d["x"], d["y"], d["z"], d["yaw"], d["pitch"], d["roll"], 0] for d in data_list]  # 只用x和yaw，其他补0
time_seq = np.cumsum(dt_seq)
raw_x = [d["x"] for d in data_list]
raw_y = [d["y"] for d in data_list]
raw_z = [d["z"] for d in data_list]
raw_yaw = [d["yaw"] for d in data_list]
raw_pitch = [d["pitch"] for d in data_list]
raw_roll = [d["roll"] for d in data_list]

# 卡尔曼滤波
kf = None
filt_x = []
filt_y = []
filt_z = []
filt_yaw = []
filt_pitch = []
filt_roll = []
for i, m in enumerate(meas_seq):
    dt = dt_seq[i] if i > 0 else 0.01
    if kf is None:
        kf = RobotArmKalmanFilter(dt=dt)
    else:
        kf.dt = dt
        # 更新F矩阵
        F_block = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        kf.F = np.kron(np.eye(6), F_block)
    pose, _ = kf.step(m)
    filt_x.append(pose[0])
    filt_y.append(pose[1])
    filt_z.append(pose[2])
    filt_yaw.append(pose[3])
    filt_pitch.append(pose[4])
    filt_roll.append(pose[5])

# 生成数据表格
if len(time_seq) == len(raw_x) == len(filt_x):
    df = pd.DataFrame({
        '时间(s)': time_seq,
        '观测x': raw_x,
        '滤波x': filt_x,
        '观测y': raw_y,
        '滤波y': filt_y,
        '观测z': raw_z,
        '滤波z': filt_z,
        '观测yaw': raw_yaw,
        '滤波yaw': filt_yaw,
        '观测pitch': raw_pitch,
        '滤波pitch': filt_pitch,
        '观测roll': raw_roll,
        '滤波roll': filt_roll
    })
    print("\n数据表格：")
    print(df.to_string(index=False, float_format=lambda x: f"{x:.3f}"))
    df.to_csv('kalman_data.csv', index=False)
else:
    print("数据长度不一致，无法生成表格")

# 绘图
plt.figure(figsize=(12, 16))
labels = [
    (raw_x, filt_x, 'x (m)'),
    (raw_y, filt_y, 'y (m)'),
    (raw_z, filt_z, 'z (m)'),
    (raw_yaw, filt_yaw, 'yaw (deg)'),
    (raw_pitch, filt_pitch, 'pitch (deg)'),
    (raw_roll, filt_roll, 'roll (deg)'),
]
for i, (raw, filt, label) in enumerate(labels):
    plt.subplot(6, 1, i+1)
    plt.scatter(time_seq, raw, label=f'obs {label}', s=7)  # 只画点，s为点的大小
    plt.scatter(time_seq, filt, label=f'kalman {label}', s=7)
    plt.ylabel(label)
    plt.legend()
plt.xlabel('时间 (s)')
plt.tight_layout()
plt.show() 