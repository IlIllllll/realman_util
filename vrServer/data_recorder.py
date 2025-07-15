import threading
import time
import numpy as np
from pathlib import Path
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import os

class DataRecorder:
    def __init__(self, robot_controller, camera, repo_id, fps=10):
        self.robot_controller = robot_controller
        self.camera = camera
        self.repo_id = repo_id
        self.fps = fps
        self.dataset = None
        self.recording = False
        self.thread = None
        self.lock = threading.Lock()
        self.LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", "/home/ls/lerobot_dataset")).expanduser()
        self.output_path = self.LEROBOT_HOME / repo_id
        if self.output_path.exists():
            self.dataset = LeRobotDataset(repo_id=self.repo_id, local_files_only=True)
        else:
            self.dataset = LeRobotDataset.create(
                repo_id=self.repo_id,
                robot_type="panda",
                fps=self.fps,
                features={
                    "top_image": {
                        "dtype": "video",
                        "shape": (480, 640, 3),
                        "names": ["height", "width", "channel"],
                    },
                    "right_image": {
                        "dtype": "video",
                        "shape": (480, 640, 3),
                        "names": ["height", "width", "channel"],
                    },
                    "left_image": {
                        "dtype": "video",
                        "shape": (480, 640, 3),
                        "names": ["height", "width", "channel"],
                    },
                    "state": {
                        "dtype": "float32",
                        "shape": (14,),
                        "names": ["state"],
                    },
                    "actions": {
                        "dtype": "float32",
                        "shape": (14,),
                        "names": ["actions"],
                    },
                    "joint": {
                        "dtype": "float32",
                        "shape": (14,),
                        "names": ["joint"],
                    },
                },
                image_writer_threads=4,
                image_writer_processes=4,
            )
        print(f"[DataRecorder] 数据集{self.repo_id}已创建")

    def start_record(self):
        with self.lock:
            if self.recording:
                print("[DataRecorder] 已在录制中")
                return False
            print("[DataRecorder] 开始录制数据集")

            self.recording = True
            self.thread = threading.Thread(target=self._record_loop, daemon=True)
            self.thread.start()
            return True
        
    def stop_record(self):
        with self.lock:
            if not self.recording:
                print("[DataRecorder] 未在录制状态")
                return False
            print("[DataRecorder] 停止录制，保存数据集")
            self.recording = False
        if self.thread:
            self.thread.join()
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

    def _record_loop(self):
        interval = 1.0 / self.fps
        while True:
            with self.lock:
                if not self.recording:
                    break
            # 获取机械臂状态
            state = self.robot_controller.robot_arm.get_state()
            pose = np.array(state.end_effector_pose, dtype=np.float32)  # 14维
            joint = np.array(state.joint_positions, dtype=np.float32)   # 14维
            # 获取相机图像
            frames = self.camera.get_frame()
            top_image = frames.get("top", None)
            right_image = frames.get("right", None)
            left_image = frames.get("left", None)
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
                print("[DataRecorder] 相机未准备好")
            time.sleep(interval) 