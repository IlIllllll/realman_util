# ========== 端口配置 ==========
AUDIO_PORT = 5007
VIDEO_PORT = 5006
CONTROL_PORT = 5005  # 控制端口，监听 Unity 控制命令

# ========== 设备序列号配置 ==========
DEVICE_SERIALS_88 = {
    'left': '130322272067',
    'right': '130322271353',
    'top': '130322273389'
}

DEVICE_SERIALS_206 = {
    'left': '130322270966',
    'right': '130322271356',
    'top': '130322273093'
}

def get_device_serials(ip_suffix):
    """
    根据IP后缀获取设备序列号配置
    
    Args:
        ip_suffix: IP地址后缀 ('88' 或 '206')
        
    Returns:
        设备序列号字典
    """
    if ip_suffix == "88":
        return DEVICE_SERIALS_88
    elif ip_suffix == "206":
        return DEVICE_SERIALS_206
    else:
        raise ValueError(f"不支持的 IP 配置尾号: {ip_suffix}，请使用 '88' 或 '206'")

# ========== 机械臂配置 ==========
def get_arm_config():
    """
    获取机械臂配置
    
    Returns:
        ArmConfig 对象
    """
    import numpy as np
    from robot_arm_interface import ArmConfig
    
    return ArmConfig(
        name="double_arm",
        dof=14,
        ip=["192.168.1.18", "192.168.1.19"],
        max_joint_velocities=np.array([2.0] * 14),
        max_joint_accelerations=np.array([5.0] * 14),
        joint_limits_lower=np.array([-np.pi] * 14),
        joint_limits_upper=np.array([np.pi] * 14),
        control_frequency=100.0
    )

# ========== 滤波器配置 ==========
def get_filter_config():
    """
    获取滤波器配置
    
    Returns:
        滤波器配置字典
    """
    return {
        "left_filter_config": {
            "window_size": 1,
            "filter_type": "moving_average",
            "alpha": 0.3
        },
        "right_filter_config": {
            "window_size": 1,
            "filter_type": "moving_average", 
            "alpha": 0.3
        }
    } 