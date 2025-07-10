# audio_udp_sender.py
import socket
import sounddevice as sd
import numpy as np
import threading
import time
import json

target_addr = None
LISTEN_PORT = 5007  # 用于接收启动请求

listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listen_socket.bind(('0.0.0.0', LISTEN_PORT))

running = False

def audio_callback(indata, frames, time_info, status):
    if not running:
        return
    pcm = (indata * 32767).astype(np.int16).tobytes()
    print("pcm", pcm)
    listen_socket.sendto(pcm, target_addr)

def listen_for_unity():
    global running, target_addr
    print("Waiting for Unity UDP start signal...")
    while True:
        data, addr = listen_socket.recvfrom(1024)
        msg = data.decode()
        if msg == "start":
            target_addr = addr  # 记录发送者 IP 和端口
            print("[Python] Received start. Streaming audio.")
            running = True
            stream = sd.InputStream(channels=1, samplerate=16000, dtype='float32', callback=audio_callback)
            with stream:
                while running:
                    time.sleep(0.1)
        elif msg == "stop":
            print("[Python] Received stop.")
            running = False

if __name__ == "__main__":
    threading.Thread(target=listen_for_unity, daemon=True).start()
    while True:
        time.sleep(1)