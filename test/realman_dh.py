import sys
import os
# 获取当前脚本所在目录的父目录（项目根目录）
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from robot_arm_interface.plugins.realman_arm import *

# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)

[ret, dh] = arm.rm_get_DH_data()
print(dh)
# print(dh.to_dict())

arm.rm_delete_robot_arm()