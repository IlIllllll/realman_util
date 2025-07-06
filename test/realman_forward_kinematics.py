import sys
import os
import numpy as np
import math

# 获取当前脚本所在目录的父目录（项目根目录）
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from robot_arm_interface.plugins.realman_arm import *
from realman_arm.Robotic_Arm.rm_ctypes_wrap import rm_dh_t

def dh_to_transformation_matrix(d, a, alpha, theta):
    """
    根据DH参数计算变换矩阵
    
    Args:
        d (float): 连杆偏移量，单位：m
        a (float): 连杆长度，单位：m  
        alpha (float): 连杆扭转角，单位：度
        theta (float): 关节角，单位：度
    
    Returns:
        numpy.ndarray: 4x4变换矩阵
    """
    # 转换为弧度
    alpha_rad = math.radians(alpha)
    theta_rad = math.radians(theta)
    
    # 计算变换矩阵
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    cos_alpha = math.cos(alpha_rad)
    sin_alpha = math.sin(alpha_rad)
    
    T = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    
    return T

def forward_kinematics_dh(dh_params, joint_angles):
    """
    使用DH参数计算正向运动学
    
    Args:
        dh_params (dict): DH参数字典，包含'd', 'a', 'alpha', 'offset'四个列表
        joint_angles (list): 关节角度列表，单位：度
    
    Returns:
        numpy.ndarray: 末端执行器的4x4变换矩阵
    """
    # 初始化单位矩阵
    T_total = np.eye(4)
    
    # 遍历每个关节
    for i in range(len(joint_angles)):
        # 获取当前关节的DH参数
        d = dh_params['d'][i]
        a = dh_params['a'][i]
        alpha = dh_params['alpha'][i]
        offset = dh_params['offset'][i]
        
        # 计算当前关节的实际角度（关节角度 + 偏移量）
        theta = joint_angles[i] + offset
        
        # 计算当前关节的变换矩阵
        T_i = dh_to_transformation_matrix(d, a, alpha, theta)
        
        # 累积变换矩阵
        T_total = T_total @ T_i
    
    return T_total

def matrix_to_pose(T):
    """
    从变换矩阵提取位置和姿态
    
    Args:
        T (numpy.ndarray): 4x4变换矩阵
    
    Returns:
        tuple: (position, euler_angles)
            - position: [x, y, z] 位置，单位：m
            - euler_angles: [rx, ry, rz] 欧拉角，单位：弧度
    """
    # 提取位置
    position = T[:3, 3]
    
    # 提取旋转矩阵
    R = T[:3, :3]
    
    # 从旋转矩阵计算欧拉角 (ZYX顺序)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    if sy > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0
    
    euler_angles = [rx, ry, rz]
    
    return position, euler_angles

def main():
    # 实例化RoboticArm类
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    
    # 创建机械臂连接
    handle = arm.rm_create_robot_arm("192.168.1.19", 8080)
    print(f"机械臂连接ID: {handle.id}")
    
    # 获取DH参数
    [ret, dh] = arm.rm_get_DH_data()
    if ret == 0:
        print("DH参数获取成功:")
        print(f"d: {dh['d']}")
        print(f"a: {dh['a']}")
        print(f"alpha: {dh['alpha']}")
        print(f"offset: {dh['offset']}")
    else:
        print(f"DH参数获取失败，错误码: {ret}")
        return
    
    # 获取当前关节角度
    [ret, joint_angles] = arm.rm_get_joint_degree()
    if ret == 0:
        print(f"\n当前关节角度: {joint_angles}")
    else:
        print(f"获取关节角度失败，错误码: {ret}")
        return
    
    # 获取当前位姿
    [ret, arm_all_state] = arm.rm_get_arm_all_state()
    if ret == 0:
        print(f"当前位姿: {arm_all_state}")
    else:
        print(f"获取位姿失败，错误码: {ret}")
        return
    
    # 使用库函数进行正向解算
    print("\n=== 使用库函数进行正向解算 ===")
    arm_model = rm_robot_arm_model_e.RM_MODEL_RM_65_E  # RM_65机械臂
    force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 标准版
    algo = Algo(arm_model, force_type)
    
    # 将字典转换为rm_dh_t实例
    dh_struct = rm_dh_t(d=dh['d'], a=dh['a'], alpha=dh['alpha'], offset=dh['offset'])
    algo.rm_algo_set_dh(dh_struct)
    
    # 使用库函数计算正向运动学
    pose_library = algo.rm_algo_forward_kinematics(joint_angles, flag=1)  # 返回欧拉角格式
    print(f"pose_library type: {type(pose_library)}")
    print(f"库函数计算结果 (位置+欧拉角): {pose_library}")
    
    # 使用自定义DH参数计算正向解算
    print("\n=== 使用自定义DH参数计算正向解算 ===")
    
    # 只使用前6个关节（6自由度机械臂）
    joint_angles_6dof = joint_angles[:6]
    dh_6dof = {
        'd': dh['d'][:6],
        'a': dh['a'][:6], 
        'alpha': dh['alpha'][:6],
        'offset': dh['offset'][:6]
    }
    
    print(f"使用前6个关节角度: {joint_angles_6dof}")
    print(f"使用前6个关节的DH参数:")
    for i in range(6):
        print(f"关节{i+1}: d={dh_6dof['d'][i]:.6f}, a={dh_6dof['a'][i]:.6f}, "
              f"alpha={dh_6dof['alpha'][i]:.6f}, offset={dh_6dof['offset'][i]:.6f}")
    
    # 计算正向运动学
    T_end = forward_kinematics_dh(dh_6dof, joint_angles_6dof)
    
    # 提取位置和姿态
    position, euler_angles = matrix_to_pose(T_end)
    
    print(f"\n自定义计算结果:")
    print(f"位置 (m): [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]")
    print(f"欧拉角 (rad): [{euler_angles[0]:.6f}, {euler_angles[1]:.6f}, {euler_angles[2]:.6f}]")
    print(f"欧拉角 (度): [{math.degrees(euler_angles[0]):.6f}, {math.degrees(euler_angles[1]):.6f}, {math.degrees(euler_angles[2]):.6f}]")
    
    # 比较结果
    print(f"\n=== 结果比较 ===")
    print(f"库函数位置: [{pose_library[0]:.6f}, {pose_library[1]:.6f}, {pose_library[2]:.6f}]")
    print(f"自定义位置: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]")
    print(f"位置误差: {np.linalg.norm(np.array(pose_library[:3]) - position):.6f} m")
    
    print(f"库函数欧拉角 (度): [{math.degrees(pose_library[3]):.6f}, {math.degrees(pose_library[4]):.6f}, {math.degrees(pose_library[5]):.6f}]")
    print(f"自定义欧拉角 (度): [{math.degrees(euler_angles[0]):.6f}, {math.degrees(euler_angles[1]):.6f}, {math.degrees(euler_angles[2]):.6f}]")
    
    # 测试不同的关节角度
    print(f"\n=== 测试不同关节角度 ===")
    test_angles = [0, 0, 0, 0, 0, 0]  # 零位角度
    print(f"测试关节角度: {test_angles}")
    
    pose_test = algo.rm_algo_forward_kinematics(test_angles, flag=1)
    print(f"零位角度下的位姿: {pose_test}")
    
    T_test = forward_kinematics_dh(dh_6dof, test_angles)
    pos_test, eul_test = matrix_to_pose(T_test)
    print(f"零位角度下自定义计算结果:")
    print(f"位置: [{pos_test[0]:.6f}, {pos_test[1]:.6f}, {pos_test[2]:.6f}]")
    print(f"欧拉角 (度): [{math.degrees(eul_test[0]):.6f}, {math.degrees(eul_test[1]):.6f}, {math.degrees(eul_test[2]):.6f}]")
    
    # 删除机械臂连接
    arm.rm_delete_robot_arm()

if __name__ == "__main__":
    main() 