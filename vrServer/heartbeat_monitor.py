import subprocess
import socket
import time
import threading
import os
import signal
import argparse

# 保存子进程信息
processes = {}

def start_service(name, mode, ip):
    # 先停止同名服务（如果存在）
    if name in processes:
        stop_service(name)
        time.sleep(1)  # 等待进程完全退出
    
    print(f"[Monitor] Starting {name} service with mode '{mode}' and ip {ip}...")
    p = subprocess.Popen(["python3", "vrServer/udp.py", "--mode", mode, "--ip", ip])
    processes[name] = p

def stop_service(name):
    if name in processes and processes[name].poll() is None:
        print(f"[Monitor] Stopping {name} service...")
        os.kill(processes[name].pid, signal.SIGTERM)
        processes[name].wait()

def heartbeat_listener(ip, modes, port=5008, timeout=5):
    last_heartbeat = time.time()
    unity_connected = False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[Monitor] Listening for heartbeat on port {port}...")

    while True:
        sock.settimeout(1)
        try:
            data, addr = sock.recvfrom(1024)
            if data.decode().strip() == "heartbeat":
                if not unity_connected:
                    print(f"[Monitor] ✅ Unity connected from {addr}")
                    unity_connected = True
                    for mode in modes:
                        start_service(mode, mode, ip)
                last_heartbeat = time.time()
                # print(f"[Monitor] ❤️ Heartbeat received from {addr}")
        except socket.timeout:
            pass

        if unity_connected and time.time() - last_heartbeat > timeout:
            for name in list(processes.keys()):
                stop_service(name)
            unity_connected = False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unity VR Monitor")
    parser.add_argument("--ip", type=str, default="17.16.2.88", help="IP to use (default: 17.16.2.88)")
    parser.add_argument(
        "--modes",
        nargs="+",
        default=["video", "audio", "command"],
        help="List of modes to start (default: video audio command)"
    )
    args = parser.parse_args()

    print(f"[Monitor] Running with IP: {args.ip}")
    print(f"[Monitor] Modes to start: {args.modes}")

    threading.Thread(
        target=heartbeat_listener,
        args=(args.ip, args.modes),
        daemon=True
    ).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[Monitor] Ctrl+C received. Shutting down...")
        for name in list(processes.keys()):
            stop_service(name)