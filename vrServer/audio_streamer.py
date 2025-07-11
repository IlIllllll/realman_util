import opuslib
import sounddevice as sd
import numpy as np
import socket
import time

class AudioStreamer:
    """
    音频流处理类，负责音频的采集、编码和传输
    """
    
    def __init__(self, ip="0.0.0.0", port=5007):
        """
        初始化音频流处理器
        
        Args:
            ip: 监听IP地址
            port: 监听端口
        """
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 添加SO_REUSEADDR选项，允许端口重用
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.sock.bind((ip, port))
            print(f"[Audio] Successfully bound to {ip}:{port}")
        except OSError as e:
            print(f"[Audio] Failed to bind to port {port}: {e}")
            raise
        
        self.running = False
        self.target_addr = None

        # Opus 编码器初始化（采样率: 16000, 通道数: 1, 应用: VOIP）
        self.encoder = opuslib.Encoder(16000, 1, opuslib.APPLICATION_AUDIO)

    def audio_callback(self, indata, frames, time_info, status):
        """
        音频回调函数，处理音频数据
        
        Args:
            indata: 输入的音频数据
            frames: 帧数
            time_info: 时间信息
            status: 状态信息
        """
        if not self.running or self.target_addr is None:
            return
        pcm = (indata * 32767).astype(np.int16).tobytes()
        try:
            encoded = self.encoder.encode(pcm, frame_size=320)  # 20ms = 16000Hz * 0.02s = 320 samples
            self.sock.sendto(encoded, self.target_addr)
        except Exception as e:
            print("[Audio Encode Error]", e)

    def run(self):
        """
        运行音频流服务
        """
        print(f"[Audio] Listening on {self.ip}:{self.port}")
        while True:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode()
            if msg == "start":
                self.target_addr = addr
                self.running = True
                print("[Audio] Received start. Begin streaming.")
                stream = sd.InputStream(channels=1, samplerate=16000, dtype='float32', callback=self.audio_callback, blocksize=320)
                with stream:
                    while self.running:
                        time.sleep(0.1)
            elif msg == "stop":
                print("[Audio] Received stop.")
                self.running = False 