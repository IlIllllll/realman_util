import socket
import subprocess
import time
import numpy as np
import json
from scipy.spatial.transform import Rotation as R

ip = "0.0.0.0"
# 检查端口是否被占用
port = 5005

def is_port_in_use(port):
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind((ip, port))
            return False
        except OSError:
            return True

# 如果端口被占用，尝试杀死占用进程
if is_port_in_use(port):
    print(f"[Control] Port {port} is in use, attempting to kill existing process...")
    try:
        result = subprocess.run(['lsof', '-ti', f':{port}'], capture_output=True, text=True)
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    subprocess.run(['kill', '-9', pid])
                    print(f"[Control] Killed process {pid}")
                    time.sleep(1)  # 等待进程完全退出
    except Exception as e:
        print(f"[Control] Failed to kill existing process: {e}")
        exit(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    sock.bind((ip, port))
    print(f"[Control] Listening on {ip}:{port}")
except OSError as e:
    print(f"[Control] Failed to bind to port {port}: {e}")
    exit(1)

try:
    xyz_list = []
    rpy_list = []
    print("开始接收数据")
    start_time = time.time()
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            msg = data.decode("utf-8")
            print(msg)
            # 尝试解析pose数据
            d = json.loads(msg)
            # 计算right的pose
            xyz_list.append(d["right"]["pose"][:3])
            rpy_list.append(d["right"]["pose"][3:7])
        except Exception as e:
            print(f"[Control] Failed to decode message: {e}")
        # 检查是否到达10秒
        if time.time() - start_time > 10:
            if xyz_list and rpy_list:
                arr_xyz = np.array(xyz_list)
                var_xyz = np.var(arr_xyz, axis=0)
                print(f"10秒内xyz各分量方差: {var_xyz}")
                arr_quat = np.array(rpy_list)
                # 四元数转欧拉角（假设顺序为x, y, z, w）
                try:
                    r = R.from_quat(arr_quat[:, [0,1,2,3]])
                    euler = r.as_euler('xyz', degrees=True)  # 转为角度
                    var_euler = np.var(euler, axis=0)
                    print(f"10秒内欧拉角（角度）各分量方差: {var_euler}")
                except Exception as e:
                    print(f"[Control] 四元数转欧拉角失败: {e}")
            else:
                print("10秒内未收到有效pose数据")
            break
except KeyboardInterrupt:
    print("[Control] Keyboard interrupt detected, exiting...")
    sock.close()
    exit(0)