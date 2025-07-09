import sys
import os
import numpy as np
import time
import threading
import cv2
from typing import List, Optional
from pathlib import Path
import argparse
import select
import tty
import termios

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from realman_arm.realman_utils import Realman

LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", "/home/ls/lerobot_dataset")).expanduser()

class SingleArmReplay:
    """单臂机器人数据集回放类"""
    
    def __init__(self, repo_id: str, robot_ip: str = "192.168.1.19"):
        """
        初始化回放器
        
        Args:
            repo_id: 数据集路径
            robot_ip: 机器人IP地址
        """
        self.repo_id = repo_id
        self.robot_ip = robot_ip
        self.robot = None
        self.dataset = None
        self.replaying = False
        self.paused = False
        self.current_episode = 0
        self.current_frame = 0
        self.total_episodes = 0
        self.total_frames = 0
        
        # 控制参数
        self.replay_speed = 1.0  # 回放速度倍数
        self.frame_delay = 0.1   # 帧间隔时间(秒)
        
        # 终端设置 - 只在需要时设置
        self.fd = sys.stdin.fileno()
        self.old_settings = None
        self.terminal_mode = False
        
    def load_dataset(self):
        """加载LeRobot数据集"""
        try:
            print(f"正在加载数据集: {self.repo_id}")
            self.dataset = LeRobotDataset(self.repo_id, local_files_only=True)
            self.total_episodes = self.dataset.num_episodes
            print(f"数据集加载成功，共 {self.total_episodes} 个episode")
                
        except Exception as e:
            print(f"数据集加载失败: {e}")
            return False
        return True
    
    def connect_robot(self):
        """连接机器人"""
        try:
            print(f"正在连接机器人: {self.robot_ip}")
            self.robot = Realman(self.robot_ip)
            self.robot.connect()
            print("机器人连接成功")
            return True
        except Exception as e:
            print(f"机器人连接失败: {e}")
            return False
    
    def show_frame_info(self, episode_idx: int, frame_idx: int, frame_data: dict):
        """显示帧信息"""
        print(f"\n=== Episode {episode_idx + 1}/{self.total_episodes}, Frame {frame_idx + 1}/{self.total_frames} ===")
        
        if 'state' in frame_data:
            pose = frame_data['state']
            print(f"目标位姿: [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}, "
                  f"{pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.1f}]")
        
        if 'joint' in frame_data:
            joint = frame_data['joint']
            print(f"目标关节: [{joint[0]:.3f}, {joint[1]:.3f}, {joint[2]:.3f}, "
                  f"{joint[3]:.3f}, {joint[4]:.3f}, {joint[5]:.3f}, {joint[6]:.1f}]")
    
    def display_images(self, frame_data: dict):
        """显示图像"""
        try:
            # 显示顶部相机图像
            if 'top_image' in frame_data and frame_data['top_image'] is not None:
                top_img = frame_data['top_image']
                if len(top_img.shape) == 3:
                    # 调整图像大小以便显示
                    top_img_resized = cv2.resize(top_img, (640, 480))
                    cv2.imshow('Top Camera', top_img_resized)
            
            # 显示手腕相机图像
            if 'wrist_image' in frame_data and frame_data['wrist_image'] is not None:
                wrist_img = frame_data['wrist_image']
                if len(wrist_img.shape) == 3:
                    # 调整图像大小以便显示
                    wrist_img_resized = cv2.resize(wrist_img, (640, 480))
                    cv2.imshow('Wrist Camera', wrist_img_resized)
            
            # 等待按键，但设置较短的超时时间
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return False
            elif key == ord(' '):  # 空格键暂停/继续
                self.paused = not self.paused
                print("暂停" if self.paused else "继续")
            
        except Exception as e:
            print(f"显示图像时出错: {e}")
        
        return True
    
    def replay_episode(self, episode_idx: int):
        """回放指定的episode"""
        if self.dataset is None or episode_idx >= self.total_episodes:
            print(f"无效的episode索引: {episode_idx}")
            return False
        
        # 使用正确的方式获取episode数据
        from_idx = self.dataset.episode_data_index["from"][episode_idx].item()
        to_idx = self.dataset.episode_data_index["to"][episode_idx].item()
        
        # 获取episode的所有帧数据，使用numpy格式
        episode_data = self.dataset.hf_dataset.select(range(from_idx, to_idx)).with_format("numpy")
        
        print(f"episode数据键: {episode_data.column_names}")
        total_frames = len(episode_data)
        print(f"总帧数: {total_frames}")
        
        print(f"\n开始回放 Episode {episode_idx + 1}/{self.total_episodes}")
        
        # 获取机器人当前位置
        current_pose = self.robot.getActualTCPPose()
        print(f"当前机器人位姿: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}, "
              f"{current_pose[3]:.3f}, {current_pose[4]:.3f}, {current_pose[5]:.3f}, {current_pose[6]:.1f}]")
        current_pose[6] = 1
        self.robot.move_to_pose(current_pose)
        print("机器人已回到初始位置")
        
        self.replaying = True
        self.paused = False
        
        try:
            for frame_idx in range(total_frames):
                # 获取帧数据（numpy格式）
                frame_data = {key: episode_data[key][frame_idx] for key in episode_data.column_names}
                
                # 显示图像
                if not self.display_images(frame_data):
                    break
                
                # 执行机器人动作
                if 'state' in frame_data and self.robot is not None:
                    target_pose = frame_data['state']
                    print(f"target_pose: {target_pose}")
                    print(f"执行动作: 移动到目标位姿")
                    
                    try:
                        # 使用move_to_pose进行位置控制
                        self.robot.move_to_pose(target_pose)
                        
                    
                        # 等待执行完成
                        time.sleep(self.frame_delay / self.replay_speed)
                        
                    except Exception as e:
                        print(f"机器人动作执行失败: {e}")
                        break
                
            
            print(f"\nEpisode {episode_idx + 1} 回放完成")
            
        except Exception as e:
            print(f"回放过程中出错: {e}")
            return False
        
        return True
    
    def _setup_terminal_mode(self):
        """设置终端为非阻塞模式"""
        if not self.terminal_mode:
            try:
                self.old_settings = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                self.terminal_mode = True
            except Exception as e:
                print(f"设置终端模式失败: {e}")
    
    def _restore_terminal_mode(self):
        """恢复终端设置"""
        if self.terminal_mode and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
                self.terminal_mode = False
            except Exception as e:
                print(f"恢复终端设置失败: {e}")
    
    def list_episodes(self):
        """列出所有episode的信息"""
        if self.dataset is None:
            print("数据集未加载")
            return
        
        print(f"\n=== 数据集信息 ===")
        print(f"总episode数: {self.total_episodes}")
        print(f"数据集路径: {self.repo_id}")
        print(f"数据集特征: {list(self.dataset.features.keys())}")
        
        for i in range(min(5, self.total_episodes)):  # 只显示前5个episode
            # 获取episode的帧数
            from_idx = self.dataset.episode_data_index["from"][i].item()
            to_idx = self.dataset.episode_data_index["to"][i].item()
            frames = to_idx - from_idx
            
            # 获取第一帧来显示任务信息
            first_frame = self.dataset[from_idx]
            task = first_frame.get('task', '未知任务') if 'task' in first_frame else '未知任务'
            
            print(f"Episode {i + 1}: {task} ({frames} 帧)")
        
        if self.total_episodes > 5:
            print(f"... 还有 {self.total_episodes - 5} 个episode")
    
    def interactive_replay(self):
        """交互式回放模式"""
        print("\n=== 交互式回放模式 ===")
        print("命令说明:")
        print("  l - 列出所有episode")
        print("  r <episode_id> - 回放指定episode (从0开始)")
        print("  n - 回放下一个episode")
        print("  p - 回放上一个episode")
        print("  q - 退出")
        print("  空格 - 暂停/继续")
        print("  s - 加快回放速度")
        print("  a - 减慢回放速度")
        
        while True:
            try:
                print("\n请输入命令: ", end='', flush=True)
                command = input().strip()
                
                if command.lower() == 'q':
                    break
                elif command.lower() == 'l':
                    self.list_episodes()
                elif command.lower().startswith('r '):
                    try:
                        episode_id = int(command.split()[1])
                        if 0 <= episode_id < self.total_episodes:
                            self.current_episode = episode_id
                            self.replay_episode(episode_id)
                        else:
                            print(f"无效的episode ID: {episode_id}")
                    except (ValueError, IndexError):
                        print("格式错误，请使用: r <episode_id>")
                elif command.lower() == 'n':
                    if self.current_episode < self.total_episodes - 1:
                        self.current_episode += 1
                        self.replay_episode(self.current_episode)
                        print(f"\n当前episode: {self.current_episode + 1}/{self.total_episodes}")
                    else:
                        print("已经是最后一个episode")
                elif command.lower() == 'p':
                    if self.current_episode > 0:
                        self.current_episode -= 1
                        self.replay_episode(self.current_episode)
                        print(f"\n当前episode: {self.current_episode + 1}/{self.total_episodes}")
                    else:
                        print("已经是第一个episode")
                elif command == '':
                    continue
                else:
                    print("未知命令")
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\n用户中断")
                break
    
    def cleanup(self):
        """清理资源"""
        self.replaying = False
        
        # 恢复终端设置
        self._restore_terminal_mode()
        
        # 关闭图像窗口
        cv2.destroyAllWindows()
        
        # 断开机器人连接
        if self.robot is not None:
            try:
                self.robot.disconnect()
                print("机器人连接已断开")
            except:
                pass

def main():
    parser = argparse.ArgumentParser(description='LeRobot数据集单臂机器人回放')
    parser.add_argument('repo_id', help='数据集路径')
    parser.add_argument('--robot_ip', default='192.168.1.19', help='机器人IP地址')
    parser.add_argument('--episode', type=int, help='指定回放的episode ID (从0开始)')
    parser.add_argument('--interactive', action='store_true', help='交互式模式')
    
    args = parser.parse_args()
    
    # 创建回放器
    replay = SingleArmReplay(args.repo_id, args.robot_ip)
    
    try:
        # 加载数据集
        if not replay.load_dataset():
            return
        
        # 连接机器人
        if not replay.connect_robot():
            print("警告: 机器人连接失败，将只显示图像回放")
        
        # 列出episode信息
        replay.list_episodes()
        
        if args.interactive:
            # 交互式模式
            replay.interactive_replay()
        elif args.episode is not None:
            # 回放指定episode
            replay.replay_episode(args.episode)
        else:
            # 默认回放第一个episode
            print("未指定episode，将回放第一个episode")
            replay.replay_episode(0)
    
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"程序出错: {e}")
    finally:
        replay.cleanup()

if __name__ == "__main__":
    main()
