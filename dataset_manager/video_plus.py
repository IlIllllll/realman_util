import shutil
import os

src = '/home/ls/huggingface/test/test'
n = 2  # 你想复制的份数

for i in range(n):
    dst = src + f'_{i}'
    shutil.copytree(src, dst)

import cv2
import numpy as np
import random
import glob

def mask_video_with_random_circle(video_path):
    cap = cv2.VideoCapture(video_path)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        h, w, _ = frame.shape
        # 随机圆心
        center_x = random.randint(0, w-1)
        center_y = random.randint(0, h-1)
        # 随机半径（比如最大为宽高较小值的1/4）
        max_radius = min(w, h) // 4
        radius = random.randint(max_radius//2, max_radius)
        # 随机颜色
        color = [random.randint(0, 255) for _ in range(3)]
        # 画圆
        cv2.circle(frame, (center_x, center_y), radius, color, -1)
        frames.append(frame)
    cap.release()
    if frames:
        h, w, _ = frames[0].shape
        out = cv2.VideoWriter(video_path, fourcc, 30, (w, h))
        for frame in frames:
            out.write(frame)
        out.release()

for i in range(n):
    base = f'{src}_{i}/videos/chunk-000'
    cam = random.choice(['left_image', 'right_image', 'top_image'])
    video_dir = os.path.join(base, cam)
    for video_file in glob.glob(os.path.join(video_dir, '*.mp4')):
        mask_video_with_random_circle(video_file)
