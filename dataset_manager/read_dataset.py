import os
from pathlib import Path
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
repo_id = "dual_arm/test_dp"
LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", "/home/ls/lerobot_dataset")).expanduser()
output_path = LEROBOT_HOME / repo_id
if output_path.exists():
    dataset = LeRobotDataset(repo_id=repo_id, local_files_only=True)
else:
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        robot_type="panda",
        fps=10,
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

print(dataset.num_frames)