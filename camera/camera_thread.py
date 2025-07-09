import pyrealsense2 as rs
import yaml
import os
import threading
import time
import numpy as np

class RealsenseCamera:
    def __init__(self, config_path='config/cameras.yaml'):
        self.config_path = config_path
        self.name_serial_map = self._load_config()
        self.pipelines = {}
        self.threads = {}
        self.frames = {}
        self.running = False
        self.lock = threading.Lock()

    def _load_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        with open(self.config_path, 'r') as f:
            data = yaml.safe_load(f)
        return data  # 期望格式: {name: serial}

    def _camera_thread(self, name, serial):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        self.pipelines[name] = pipeline
        while self.running:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    # 红蓝通道交换
                    color_image = color_image[:, :, ::-1]
                    with self.lock:
                        self.frames[name] = color_image
            except Exception as e:
                print(f"[{name}] 采集异常: {e}")
                time.sleep(0.1)
        pipeline.stop()

    def start(self):
        self.running = True
        for name, serial in self.name_serial_map.items():
            t = threading.Thread(target=self._camera_thread, args=(name, serial), daemon=True)
            self.threads[name] = t
            t.start()

    def get_frame(self):
        with self.lock:
            # 返回当前所有相机的最新图像快照
            return dict(self.frames)

    def get_image_by_name(self, name):
        with self.lock:
            return self.frames.get(name, None)

    def stop(self):
        self.running = False
        # 等待所有线程结束
        for t in self.threads.values():
            t.join(timeout=2)
        for pipeline in self.pipelines.values():
            try:
                pipeline.stop()
            except Exception:
                pass
        self.pipelines = {}
        self.threads = {}
        self.frames = {}

# 示例用法
if __name__ == "__main__":
    cam = RealsenseCamera()
    cam.start()
    time.sleep(2)  # 等待相机采集
    frames = cam.get_frame()
    for name, img in frames.items():
        print(f"相机: {name}, 图像shape: {img.shape if img is not None else None}")
    cam.stop()
