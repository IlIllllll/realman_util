import shutil
import os
import subprocess

src = '/home/ls/huggingface/dual_arm/0714_grub_box'
n = 2 # 你想复制的份数

for i in range(n):
    dst = src + f'_{i}'
    shutil.copytree(src, dst)

import cv2
import numpy as np
import random
import glob

def mask_video_with_random_circle_opencv(video_path):
    """
    使用OpenCV处理视频，确保编码格式与目标格式完全匹配
    """
    import tempfile
    import random
    
    # 创建临时文件
    with tempfile.NamedTemporaryFile(suffix='.mp4', delete=False) as temp_output:
        temp_output_path = temp_output.name
    
    try:
        # 使用OpenCV读取视频
        cap = cv2.VideoCapture(video_path)
        
        # 获取原始视频参数
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # 处理每一帧
        frames = []
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 生成随机圆形参数
            center_x = random.randint(0, width-1)
            center_y = random.randint(0, height-1)
            max_radius = min(width, height) // 3
            radius = random.randint(max_radius//2, max_radius)
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            
            # 在帧上画圆
            cv2.circle(frame, (center_x, center_y), radius, color, -1)
            frames.append(frame)
        
        cap.release()
        
        if frames:
            # 使用OpenCV写入临时文件
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(temp_output_path, fourcc, fps, (width, height))
            for frame in frames:
                out.write(frame)
            out.release()
            
            # 使用ffmpeg重新编码以确保格式匹配
            cmd = [
                'ffmpeg', '-y',
                '-i', temp_output_path,
                '-c:v', 'libx264',
                '-preset', 'slow',  # 改为slow以获得更好的压缩效果
                '-crf', '18',       # 降低CRF值以提高质量
                '-pix_fmt', 'yuv420p',
                '-r', str(fps),
                '-profile:v', 'high',  # 使用high profile获得更好质量
                '-level', '4.1',       # 设置编码级别
                '-metadata', 'encoder=Lavf58.76.100',
                video_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"成功处理视频: {video_path}")
                return True
            else:
                print(f"FFmpeg重新编码失败: {result.stderr}")
                return False
        else:
            print("没有读取到任何帧")
            return False
            
    except Exception as e:
        print(f"OpenCV处理视频时出错: {e}")
        return False
    finally:
        # 清理临时文件
        if os.path.exists(temp_output_path):
            try:
                os.remove(temp_output_path)
            except:
                pass

for i in range(n):
    base = f'{src}_{i}/videos/chunk-000'
    # 随机选择1-2个摄像头文件夹
    all_cams = ['left_image', 'right_image', 'top_image']
    num_cams_to_process = random.randint(1, 2)
    selected_cams = random.sample(all_cams, num_cams_to_process)
    
    for cam in selected_cams:
        video_dir = os.path.join(base, cam)
        video_files = glob.glob(os.path.join(video_dir, '*.mp4'))
        if not video_files:
            print(f"文件夹 {video_dir} 中没有找到视频文件")
            continue
        
        print(f"处理文件夹 {video_dir} 中的 {len(video_files)} 个视频")
        # 处理该文件夹中的所有视频
        for video_file in video_files:
            # 使用ffmpeg版本确保编码格式匹配
            mask_video_with_random_circle_opencv(video_file)
