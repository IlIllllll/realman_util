"""
Realman机械臂管理器集成测试脚本

专门测试RobotArmManager与Realman机械臂的集成功能：
1. 管理器启动和停止
2. 状态回调机制
3. 通过管理器进行运动控制
4. 统计信息获取
5. 错误处理
"""

import sys
import os
import numpy as np
import time
import threading
from typing import List

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from robot_arm_interface import RobotArmManager, RobotArmFactory, ArmConfig
from robot_arm_interface.plugins.realman_arm import RealmanRobotArm


class RealmanManagerTester:
    """Realman机械臂管理器测试器"""
    
    def __init__(self, arm_ips: List[str] = None):
        if arm_ips is None:
            arm_ips = ["192.168.1.18"]
        
        self.arm_ips = arm_ips
        self.config = self._create_config()
        self.robot_arm = None
        self.manager = None
        self.test_results = {}
        
    def _create_config(self) -> ArmConfig:
        """创建机械臂配置"""
        return ArmConfig(
            name="realman_manager_test",
            dof=7,
            ip=self.arm_ips,
            max_joint_velocities=np.array([20.0] * 7),
            max_joint_accelerations=np.array([10.0] * 7),
            joint_limits_lower=np.array([-180] * 7),
            joint_limits_upper=np.array([180] * 7),
            control_frequency=100.0,
            connection_params={
                "port": 8080,
                "level": 3
            }
        )
    
    def test_manager_creation(self) -> bool:
        """测试管理器创建"""
        print("\n=== 测试管理器创建 ===")
        try:
            # 创建机械臂实例
            self.robot_arm = RobotArmFactory.create("realman", self.config)
            if self.robot_arm is None:
                print("❌ 机械臂实例创建失败")
                return False
            print("✅ 机械臂实例创建成功")
            
            # 连接机械臂
            if not self.robot_arm.connect():
                print("❌ 机械臂连接失败")
                return False
            print("✅ 机械臂连接成功")
            
            # 创建管理器
            self.manager = RobotArmManager(self.robot_arm, control_frequency=100.0)
            if self.manager is None:
                print("❌ 管理器创建失败")
                return False
            print("✅ 管理器创建成功")
            
            return True
            
        except Exception as e:
            print(f"❌ 管理器创建测试异常: {e}")
            return False
    
    def test_manager_start_stop(self) -> bool:
        """测试管理器启动和停止"""
        print("\n=== 测试管理器启动和停止 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            print("✅ 管理器启动成功")
            
            # 等待控制线程启动
            time.sleep(0.1)
            
            # 检查统计信息
            stats = self.manager.get_statistics()
            if not stats['thread_running']:
                print("❌ 控制线程未运行")
                return False
            print("✅ 控制线程运行正常")
            
            # 停止管理器
            if not self.manager.stop():
                print("❌ 管理器停止失败")
                return False
            print("✅ 管理器停止成功")
            
            # 检查线程是否已停止
            stats = self.manager.get_statistics()
            if stats['thread_running']:
                print("❌ 控制线程未停止")
                return False
            print("✅ 控制线程已停止")
            
            return True
            
        except Exception as e:
            print(f"❌ 管理器启动停止测试异常: {e}")
            return False
    
    def test_manager_enable_disable(self) -> bool:
        """测试管理器启用和禁用"""
        print("\n=== 测试管理器启用和禁用 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            
            # 启用机械臂
            if not self.manager.enable():
                print("❌ 管理器启用失败")
                return False
            print("✅ 管理器启用成功")
            
            # 等待enable命令被处理
            max_wait_time = 2.0
            wait_start = time.time()
            while not self.robot_arm.is_enabled and (time.time() - wait_start) < max_wait_time:
                time.sleep(0.01)
            
            # 检查机械臂状态
            if not self.robot_arm.is_enabled:
                print("❌ 机械臂未启用")
                return False
            print("✅ 机械臂已启用")
            
            # 禁用机械臂
            if not self.manager.disable():
                print("❌ 管理器禁用失败")
                return False
            print("✅ 管理器禁用成功")
            
            # 等待disable命令被处理
            max_wait_time = 2.0
            wait_start = time.time()
            while self.robot_arm.is_enabled and (time.time() - wait_start) < max_wait_time:
                time.sleep(0.01)
            
            # 检查机械臂状态
            if self.robot_arm.is_enabled:
                print("❌ 机械臂未禁用")
                return False
            print("✅ 机械臂已禁用")
            
            # 停止管理器
            self.manager.stop()
            
            return True
            
        except Exception as e:
            print(f"❌ 管理器启用禁用测试异常: {e}")
            return False
    
    def test_manager_joint_movement(self) -> bool:
        """测试通过管理器的关节运动"""
        print("\n=== 测试通过管理器的关节运动 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            
            # 启用机械臂
            if not self.manager.enable():
                print("❌ 管理器启用失败")
                return False
            
            # 获取当前状态
            current_state = self.manager.get_state()
            if current_state is None:
                print("❌ 获取当前状态失败")
                return False
            
            current_joints = current_state.joint_positions
            print(f"当前位置: {np.array(current_joints)}°")
            
            # 计算目标位置
            target_joints = current_joints.copy()
            target_joints[:6] += np.array([3.0, -3.0, 2.0, -2.0, 1.0, -1.0])
            target_joints[6] = 1.0  # 夹爪关闭
            
            print(f"目标位置: {np.array(target_joints)}°")
            
            # 通过管理器发送运动指令
            if not self.manager.move_joints(target_joints):
                print("❌ 管理器关节运动失败")
                return False
            print("✅ 管理器关节运动指令发送成功")
            
            # 等待运动完成
            time.sleep(2.0)
            
            # 获取最终状态
            final_state = self.manager.get_state()
            if final_state is None:
                print("❌ 获取最终状态失败")
                return False
            
            final_joints = final_state.joint_positions
            print(f"最终位置: {np.array(final_joints)}°")
            
            # 计算位置误差
            position_error = np.linalg.norm(target_joints - final_joints)
            print(f"位置误差: {position_error:.2f}°")
            
            if position_error < 5.0:  # 允许5度误差
                print("✅ 管理器关节运动测试成功")
                return True
            else:
                print("❌ 管理器关节运动精度不满足要求")
                return False
                
        except Exception as e:
            print(f"❌ 管理器关节运动测试异常: {e}")
            return False
        finally:
            # 清理
            if self.manager:
                self.manager.disable()
                self.manager.stop()
    
    def test_manager_cartesian_movement(self) -> bool:
        """测试通过管理器的笛卡尔运动"""
        print("\n=== 测试通过管理器的笛卡尔运动 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            
            # 启用机械臂
            if not self.manager.enable():
                print("❌ 管理器启用失败")
                return False
            
            # 获取当前状态
            current_state = self.manager.get_state()
            if current_state is None:
                print("❌ 获取当前状态失败")
                return False
            
            current_pose = current_state.end_effector_pose
            print(f"当前位置: {np.round(current_pose, 3)}")
            
            # 计算目标位置
            target_pose = current_pose.copy()
            target_pose[:3] += [0.03, 0.03, 0.03]  # 位置偏移3cm
            target_pose[3:6] += [0.05, 0.05, 0.05]  # 姿态偏移
            
            print(f"目标位置: {np.round(target_pose, 3)}")
            
            # 通过管理器发送运动指令
            if not self.manager.move_cartesian(target_pose, velocity_limit=0.05):
                print("❌ 管理器笛卡尔运动失败")
                return False
            print("✅ 管理器笛卡尔运动指令发送成功")
            
            # 等待运动完成
            time.sleep(2.0)
            
            # 获取最终状态
            final_state = self.manager.get_state()
            if final_state is None:
                print("❌ 获取最终状态失败")
                return False
            
            final_pose = final_state.end_effector_pose
            print(f"最终位置: {np.round(final_pose, 3)}")
            
            # 计算位置误差
            position_error = np.linalg.norm(target_pose[:3] - final_pose[:3])
            print(f"位置误差: {position_error:.3f} m")
            
            if position_error < 0.01:  # 允许1cm误差
                print("✅ 管理器笛卡尔运动测试成功")
                return True
            else:
                print("❌ 管理器笛卡尔运动精度不满足要求")
                return False
                
        except Exception as e:
            print(f"❌ 管理器笛卡尔运动测试异常: {e}")
            return False
        finally:
            # 清理
            if self.manager:
                self.manager.disable()
                self.manager.stop()
    
    def test_manager_statistics(self) -> bool:
        """测试管理器统计信息"""
        print("\n=== 测试管理器统计信息 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            
            # 获取统计信息
            stats = self.manager.get_statistics()
            print("统计信息:")
            for key, value in stats.items():
                print(f"  {key}: {value}")
            
            # 检查关键统计信息
            if not stats['thread_running']:
                print("❌ 控制线程未运行")
                return False
            
            if stats['control_frequency'] != 100.0:
                print("❌ 控制频率不正确")
                return False
            
            print("✅ 统计信息获取成功")
            
            # 停止管理器
            self.manager.stop()
            
            # 再次获取统计信息
            stats = self.manager.get_statistics()
            if stats['thread_running']:
                print("❌ 停止后线程仍在运行")
                return False
            
            print("✅ 停止后统计信息正确")
            
            return True
            
        except Exception as e:
            print(f"❌ 统计信息测试异常: {e}")
            return False
    
    def test_manager_performance(self) -> bool:
        """测试管理器性能"""
        print("\n=== 测试管理器性能 ===")
        try:
            # 启动管理器
            if not self.manager.start():
                print("❌ 管理器启动失败")
                return False
            
            # 启用机械臂
            if not self.manager.enable():
                print("❌ 管理器启用失败")
                return False
            
            # 测试状态获取性能
            start_time = time.time()
            for i in range(100):
                state = self.manager.get_state()
                if state is None:
                    print(f"❌ 第{i+1}次状态获取失败")
                    return False
            
            end_time = time.time()
            avg_time = (end_time - start_time) / 100
            print(f"状态获取平均时间: {avg_time*1000:.2f} ms")
            
            if avg_time < 0.01:  # 平均时间小于10ms
                print("✅ 状态获取性能满足要求")
            else:
                print("❌ 状态获取性能不满足要求")
                return False
            
            # 测试统计信息获取性能
            start_time = time.time()
            for i in range(50):
                stats = self.manager.get_statistics()
                if stats is None:
                    print(f"❌ 第{i+1}次统计信息获取失败")
                    return False
            
            end_time = time.time()
            avg_time = (end_time - start_time) / 50
            print(f"统计信息获取平均时间: {avg_time*1000:.2f} ms")
            
            if avg_time < 0.01:  # 平均时间小于10ms
                print("✅ 统计信息获取性能满足要求")
            else:
                print("❌ 统计信息获取性能不满足要求")
                return False
            
            # 清理
            self.manager.disable()
            self.manager.stop()
            
            return True
            
        except Exception as e:
            print(f"❌ 性能测试异常: {e}")
            return False
    
    def run_all_tests(self) -> dict:
        """运行所有管理器集成测试"""
        print("🚀 开始Realman机械臂管理器集成测试")
        print("=" * 60)
        
        test_functions = [
            ("管理器创建测试", self.test_manager_creation),
            ("管理器启动停止测试", self.test_manager_start_stop),
            ("管理器启用禁用测试", self.test_manager_enable_disable),
            ("管理器关节运动测试", self.test_manager_joint_movement),
            ("管理器笛卡尔运动测试", self.test_manager_cartesian_movement),
            ("管理器统计信息测试", self.test_manager_statistics),
            ("管理器性能测试", self.test_manager_performance),
        ]
        
        results = {}
        
        for test_name, test_func in test_functions:
            try:
                print(f"\n{'='*25} {test_name} {'='*25}")
                result = test_func()
                results[test_name] = result
                
                if result:
                    print(f"✅ {test_name} 通过")
                else:
                    print(f"❌ {test_name} 失败")
                    
            except Exception as e:
                print(f"❌ {test_name} 异常: {e}")
                results[test_name] = False
        
        # 打印测试总结
        print("\n" + "="*60)
        print("📊 管理器集成测试结果总结")
        print("="*60)
        
        passed = sum(1 for result in results.values() if result)
        total = len(results)
        
        for test_name, result in results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"{test_name}: {status}")
        
        print(f"\n总计: {passed}/{total} 个测试通过")
        
        if passed == total:
            print("🎉 所有管理器集成测试都通过了！")
        else:
            print("⚠️  部分管理器集成测试失败，请检查机械臂状态和连接")
        
        return results
    
    def cleanup(self):
        """清理资源"""
        if self.manager:
            try:
                self.manager.disable()
                self.manager.stop()
            except:
                pass
        
        if self.robot_arm:
            try:
                self.robot_arm.disconnect()
            except:
                pass


def main():
    """主函数"""
    print("Realman机械臂管理器集成测试程序")
    print("请确保机械臂已正确连接并处于安全状态")
    
    arm_ips = ["192.168.1.19"]  # 根据实际情况修改
    
    tester = RealmanManagerTester(arm_ips)
    
    try:
        results = tester.run_all_tests()
        return results
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main() 