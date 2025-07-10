import sys
import os
import numpy as np
import time
import threading
from typing import List
import select
import tty
import termios

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import  RobotArmFactory, ArmConfig
from robot_arm_interface.plugins.realman_arm import RealmanRobotArm

arm_config = ArmConfig(
    name="double_arm",
    dof=14,
    ip=["192.168.1.18", "192.168.1.19"],
    max_joint_velocities=np.array([2.0] * 14),
    max_joint_accelerations=np.array([5.0] * 14),
    joint_limits_lower=np.array([-np.pi] * 14),
    joint_limits_upper=np.array([np.pi] * 14),
    control_frequency=100.0
)

robot_arm = RobotArmFactory.create("realman", arm_config)

if __name__ == "__main__":
    robot_arm.connect()
    time.sleep(1)
    robot_arm.enable()

    while True:
        state = robot_arm.get_state()
        left_eef_pose = state.end_effector_pose[:7]
        right_eef_pose = state.end_effector_pose[7:]
        print(left_eef_pose, right_eef_pose)
        left_eef_pose[6] = 1
        right_eef_pose[6] = 1
        right_eef_pose[0] = right_eef_pose[0] + 0.02
        move_pose = np.concatenate([left_eef_pose, right_eef_pose])
        robot_arm.move_cartesian(move_pose)
        time.sleep(1)



