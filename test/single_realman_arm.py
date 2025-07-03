import sys
import os
# 获取当前脚本所在目录的父目录（项目根目录）
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
print("Current working directory:", os.getcwd())
print("Project root:", project_root)
print("Python path:", sys.path)
from realman_arm.realman_utils import Realman
import numpy as np
import time
from scipy.spatial.transform import Rotation as st

realman = Realman("192.168.1.19")

realman.connect()

print(realman.getActualTCPPose())
print(realman.read("Present_Joint"))

realman.disconnect()