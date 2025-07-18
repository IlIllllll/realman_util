import numpy as np
from scipy.spatial.transform import Rotation as R

def map_unity_quat_to_robot(unity_quat):
    """
    将 Unity 四元数转换为机器人坐标系四元数
    
    Args:
        unity_quat: Unity 四元数 [x, y, z, w]
        
    Returns:
        机器人坐标系四元数 [x, y, z, w]
    """
    # 输入 Unity 四元数：[x, y, z, w]
    r_unity = R.from_quat(unity_quat)
    
    # 构造坐标系旋转矩阵 M
    M = np.array([
        [-1, 0, 0], 
        [0, -1, 0],
        [0, 0, 1]
    ])
    
    # 旋转矩阵表示姿态变换
    R_unity_matrix = r_unity.as_matrix()
    R_robot_matrix = M @ R_unity_matrix @ M.T

    r_robot = R.from_matrix(R_robot_matrix)
    quat_xyzw = r_robot.as_quat()
    return quat_xyzw

def calculate_delta_pose(current_pose, target_pose):
    """
    计算从当前位姿到目标位姿的增量
    
    Args:
        current_pose: 当前位姿 [x, y, z, qx, qy, qz, qw]
        target_pose: 目标位姿 [x, y, z, qx, qy, qz, qw]
        
    Returns:
        delta_pose: 位姿增量 [dx, dy, dz, dqx, dqy, dqz, dqw]
    """
    # 位置增量
    position_delta = np.array(target_pose[:3]) - np.array(current_pose[:3])
    
    # 四元数增量
    current_quat = np.array(current_pose[3:7])
    target_quat = np.array(target_pose[3:7])
    
    # 计算四元数差值
    current_rot = R.from_quat(current_quat)
    target_rot = R.from_quat(target_quat)
    
    # 计算相对旋转
    delta_rot = target_rot * current_rot.inv()
    delta_quat = delta_rot.as_quat()
    
    # 组合增量
    delta_pose = np.concatenate([position_delta, delta_quat])
    
    return delta_pose

def apply_rotation_delta(handType, init_euler_deg, delta_quat_unity, rotation_scale=1.0):
    """
    应用旋转增量到初始欧拉角
    
    Args:
        handType: 手类型 ("left" 或 "right")
        init_euler_deg: 初始欧拉角 [roll, pitch, yaw] (弧度)
        delta_quat_unity: Unity 四元数增量 [x, y, z, w]
        rotation_scale: 旋转缩放系数
        
    Returns:
        最终的欧拉角 [roll, pitch, yaw] (弧度)
    """
    # 将 Unity 四元数转换到机器人坐标系
    delta_quat_robot = map_unity_quat_to_robot(delta_quat_unity)

    # 把四元数转为 angle-axis
    delta_rot = R.from_quat(delta_quat_robot)
    angle_axis = delta_rot.as_rotvec()  # 这是 angle * axis 的形式

    if handType == "left":
        angle_axis[2] *= -1  # 手动反转绕 Z 的旋转
    if handType == "right":
        # ✅ 反转 X 轴旋转方向
        angle_axis[0] *= -1  # 手动反转绕 X 的旋转
        
    # 缩放角度
    scaled_rotvec = angle_axis * rotation_scale
    scaled_delta_rot = R.from_rotvec(scaled_rotvec)

    # 初始姿态
    init_rot = R.from_euler('ZYX', init_euler_deg, degrees=False)

    # 应用旋转增量
    final_rot = scaled_delta_rot * init_rot
    return final_rot.as_euler('ZYX', degrees=False) 