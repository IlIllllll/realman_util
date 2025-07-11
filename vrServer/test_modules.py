#!/usr/bin/env python3
"""
模块导入测试脚本
验证重构后的所有模块是否能正常导入
"""

def test_imports():
    """测试所有模块的导入"""
    print("开始测试模块导入...")
    
    try:
        # 测试配置模块
        print("✓ 测试 config 模块...")
        from config import get_device_serials, get_arm_config, get_filter_config
        print("  - get_device_serials: OK")
        print("  - get_arm_config: OK") 
        print("  - get_filter_config: OK")
        
        # 测试滤波器模块
        print("✓ 测试 filters 模块...")
        from filters import SlidingWindowFilter, DualHandFilter
        print("  - SlidingWindowFilter: OK")
        print("  - DualHandFilter: OK")
        
        # 测试坐标变换模块
        print("✓ 测试 transform_utils 模块...")
        from transform_utils import map_unity_quat_to_robot, apply_rotation_delta
        print("  - map_unity_quat_to_robot: OK")
        print("  - apply_rotation_delta: OK")
        
        # 测试机械臂控制器模块
        print("✓ 测试 robot_controller 模块...")
        from robot_controller import RobotArmController, set_scale_parameters
        print("  - RobotArmController: OK")
        print("  - set_scale_parameters: OK")
        
        # 测试音频流模块
        print("✓ 测试 audio_streamer 模块...")
        from audio_streamer import AudioStreamer
        print("  - AudioStreamer: OK")
        
        # 测试视频流模块
        print("✓ 测试 video_streamer 模块...")
        from video_streamer import video_stream_thread
        print("  - video_stream_thread: OK")
        
        # 测试命令服务器模块
        print("✓ 测试 command_server 模块...")
        from command_server import command_server_thread
        print("  - command_server_thread: OK")
        
        print("\n🎉 所有模块导入测试通过！")
        return True
        
    except ImportError as e:
        print(f"\n❌ 模块导入失败: {e}")
        return False
    except Exception as e:
        print(f"\n❌ 其他错误: {e}")
        return False

def test_config_functions():
    """测试配置函数"""
    print("\n测试配置函数...")
    
    try:
        from config import get_device_serials, get_arm_config, get_filter_config
        
        # 测试设备序列号配置
        device_serials_88 = get_device_serials("88")
        device_serials_206 = get_device_serials("206")
        print(f"✓ 设备序列号配置: 88={len(device_serials_88)}个设备, 206={len(device_serials_206)}个设备")
        
        # 测试机械臂配置
        arm_config = get_arm_config()
        print(f"✓ 机械臂配置: {arm_config.name}, DOF={arm_config.dof}")
        
        # 测试滤波器配置
        filter_config = get_filter_config()
        print(f"✓ 滤波器配置: 左手={filter_config['left_filter_config']['filter_type']}, 右手={filter_config['right_filter_config']['filter_type']}")
        
        return True
        
    except Exception as e:
        print(f"❌ 配置函数测试失败: {e}")
        return False

def test_filter_classes():
    """测试滤波器类"""
    print("\n测试滤波器类...")
    
    try:
        from filters import SlidingWindowFilter, DualHandFilter
        import numpy as np
        
        # 测试滑动窗口滤波器
        filter1 = SlidingWindowFilter(window_size=3, filter_type="moving_average")
        test_data = np.array([1.0, 2.0, 3.0])
        filtered = filter1.filter(test_data)
        print(f"✓ 滑动窗口滤波器: 输入={test_data}, 输出={filtered}")
        
        # 测试双手滤波器
        dual_filter = DualHandFilter()
        left_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        right_pose = np.array([0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0])
        result = dual_filter.filter_both_hands(left_pose, right_pose, 0.5, 0.7)
        print(f"✓ 双手滤波器: 左手夹爪={result['left_grip']:.1f}, 右手夹爪={result['right_grip']:.1f}")
        
        return True
        
    except Exception as e:
        print(f"❌ 滤波器类测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=" * 50)
    print("VR服务器模块重构测试")
    print("=" * 50)
    
    # 测试模块导入
    import_success = test_imports()
    
    if import_success:
        # 测试配置函数
        config_success = test_config_functions()
        
        # 测试滤波器类
        filter_success = test_filter_classes()
        
        if config_success and filter_success:
            print("\n" + "=" * 50)
            print("🎉 所有测试通过！模块重构成功！")
            print("=" * 50)
        else:
            print("\n" + "=" * 50)
            print("⚠️  部分测试失败，请检查相关模块")
            print("=" * 50)
    else:
        print("\n" + "=" * 50)
        print("❌ 模块导入失败，请检查依赖和导入路径")
        print("=" * 50)

if __name__ == "__main__":
    main() 