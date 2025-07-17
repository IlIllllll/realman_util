import threading
import time
import numpy as np
from pathlib import Path
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import os
import cv2
import socket

class DataRecorder:
    def __init__(self, robot_controller, camera, repo_id, fps=10, streaming=True, stream_ip="0.0.0.0", stream_port=5006):
        self.robot_controller = robot_controller
        self.camera = camera
        self.repo_id = repo_id
        self.fps = fps
        self.dataset = None
        self.recording = False
        self.running = False  # 控制数据采集线程
        self.streaming = streaming
        self.thread = None
        self.lock = threading.Lock()
        self.LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", "/home/ls/lerobot_dataset")).expanduser()
        self.output_path = self.LEROBOT_HOME / repo_id
        self.stream_ip = stream_ip
        self.stream_port = stream_port
        self.sock = None
        self.client_addr = None
        if self.output_path.exists():
            self.dataset = LeRobotDataset(repo_id=self.repo_id, local_files_only=True)
        else:
            self.dataset = LeRobotDataset.create(
                repo_id=self.repo_id,
                robot_type="panda",
                fps=self.fps,
                features={
                    "top_image": {"dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channel"]},
                    "right_image": {"dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channel"]},
                    "left_image": {"dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channel"]},
                    "state": {"dtype": "float32", "shape": (14,), "names": ["state"]},
                    "actions": {"dtype": "float32", "shape": (14,), "names": ["actions"]},
                    "joint": {"dtype": "float32", "shape": (14,), "names": ["joint"]},
                },
                image_writer_threads=4,
                image_writer_processes=4,
            )
        print(f"[DataRecorder] 数据集{self.repo_id}已创建")
        self.start_data_collection()

    def start_data_collection(self):
        """启动数据收集线程"""
        with self.lock:
            if self.running:
                print(f"[DataRecorder] 数据收集线程已在运行")
                return
            print(f"[DataRecorder] 启动数据收集线程")
            self.running = True
            self.thread = threading.Thread(target=self._data_collection_loop, daemon=True)
            self.thread.start()

    def stop_data_collection(self):
        """停止数据收集线程"""
        with self.lock:
            if not self.running:
                print(f"[DataRecorder] 数据收集线程未在运行")
                return
            print(f"[DataRecorder] 停止数据收集线程")
            self.running = False
        if self.thread:
            self.thread.join()
        if self.sock:
            self.sock.close()
            self.sock = None
            self.client_addr = None

    def start_record(self):
        with self.lock:
            if self.recording:
                print(f"[DataRecorder] 已在录制中")
                return False
            print(f"[DataRecorder] 开始录制数据集")
            self.recording = True
            return True

    def stop_record(self):
        with self.lock:
            if not self.recording:
                print(f"[DataRecorder] 未在录制状态")
                return False
            print(f"[DataRecorder] 停止录制，保存数据集")
            self.recording = False
        self.dataset.save_episode(task="dual_arm task", encode_videos=True)
        print(f"[DataRecorder] 数据集{self.dataset.episode_data_index}已保存")
        return True

    def is_recording(self):
        with self.lock:
            return self.recording

    def delete_dataset(self):
        with self.lock:
            self.recording = False
        self.dataset.clear_episode_buffer()
        return True

    def __del__(self):
        self.stop_data_collection()

    def _data_collection_loop(self):
        """持续采集数据，只在recording时add_frame，并在此处同步推流"""
        interval = 1.0 / self.fps
        if self.streaming:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.sock.bind((self.stream_ip, self.stream_port))
                print(f"[DataRecorder] 推流监听 {self.stream_ip}:{self.stream_port}")
            except OSError as e:
                print(f"[DataRecorder] 绑定端口失败: {e}")
                self.sock = None
            if self.sock:
                print(f"[DataRecorder] 等待客户端 start 命令...")
                while self.running:
                    data, addr = self.sock.recvfrom(65535)
                    if data.decode(errors='ignore') == "start":
                        self.client_addr = addr
                        print(f"[DataRecorder] 收到 start 命令，来自 {addr}")
                        break
        while True:
            with self.lock:
                if not self.running:
                    break
                is_recording = self.recording
            # 获取机械臂状态
            state = self.robot_controller.robot_arm.get_state()
            pose = np.array(state.end_effector_pose, dtype=np.float32)  # 14维
            joint = np.array(state.joint_positions, dtype=np.float32)   # 14维
            # 获取相机图像
            frames = self.camera.get_frame()
            top_image = frames.get("top", None)
            right_image = frames.get("right", None)
            left_image = frames.get("left", None)
            # 只在录制时add_frame
            if is_recording:
                if top_image is not None and right_image is not None and left_image is not None:
                    self.dataset.add_frame({
                        "top_image": top_image,
                        "right_image": right_image,
                        "left_image": left_image,
                        "state": pose,
                        "actions": joint,
                        "joint": joint,
                    })
                else:
                    print(f"[DataRecorder] 相机未准备好，跳过当前帧")
            # 推流（和采集同步）
            if self.streaming and self.sock and self.client_addr and top_image is not None and right_image is not None and left_image is not None:
                img_left = cv2.rotate(left_image, cv2.ROTATE_90_CLOCKWISE)
                img_top = top_image
                img_right = cv2.rotate(right_image, cv2.ROTATE_90_CLOCKWISE)
                top_h, top_w = img_top.shape[:2]
                thumb_w, thumb_h = top_w * 2 // 5, top_h * 2 // 5
                img_left_small = cv2.resize(img_left, (thumb_w, thumb_h))
                img_right_small = cv2.resize(img_right, (thumb_w, thumb_h))
                combined_img = img_top.copy()
                combined_img[0:thumb_h, 0:thumb_w] = img_left_small
                combined_img[0:thumb_h, top_w - thumb_w:top_w] = img_right_small
                # 推流时转换BGR到RGB，解决红蓝通道反转问题
                combined_img_rgb = cv2.cvtColor(combined_img, cv2.COLOR_BGR2RGB)
                _, jpeg = cv2.imencode('.jpg', combined_img_rgb, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.sock.sendto(jpeg.tobytes(), self.client_addr)
            time.sleep(interval) 