#!/usr/bin/env python3
"""
VR服务器主程序
整合了视频流、音频流、机械臂控制等功能
"""

import threading
import signal
import sys
import argparse
import time

# 导入各个模块
from config import get_device_serials, AUDIO_PORT, VIDEO_PORT, CONTROL_PORT
from audio_streamer import AudioStreamer
from command_server import command_server_thread

# 全局变量用于存储线程和控制器
threads = []
robot_controller = None
audio_streamer = None

def signal_handler(signum, frame):
    """信号处理函数，用于优雅地关闭程序"""
    print(f"\n[Main] 收到信号 {signum}，正在关闭程序...")
    
    # 停止机械臂控制器
    if robot_controller:
        print("[Main] 停止机械臂控制器...")
        robot_controller.stop()
    
    # 停止音频流
    if audio_streamer:
        print("[Main] 停止音频流...")
        audio_streamer.running = False
    
    # 等待所有线程结束
    for thread in threads:
        if thread.is_alive():
            thread.join(timeout=2)
    
    print("[Main] 程序已关闭")
    sys.exit(0)

def main():
    """主函数"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    parser = argparse.ArgumentParser(description="启动视频流或指令服务线程")
    parser.add_argument("--mode", nargs='+', choices=["video", "command", "audio"], default=["video", "command"],
                    help="运行模式: 可选 'video' 'command' 'audio'，支持多个，例如 --mode video command")
    
    parser.add_argument("--ip", type=str, default="17.16.2.88",
                    help="用于选择摄像头序列号配置，例如 '17.16.2.88' 或 '17.16.2.206'")
    parser.add_argument("--repo_id", type=str, default="dual_arm/test_dp",
                        help="LeRobot数据集 repo_id")
    args = parser.parse_args()
    print(f"运行模式: {args.mode}, 使用摄像头配置IP: {args.ip}, 数据集repo_id: {args.repo_id}")

    # 根据IP获取设备序列号配置
    ip_suffix = args.ip.split('.')[-1]
    device_serials = get_device_serials(ip_suffix)
    
    global robot_controller, audio_streamer
    try:
        # if "video" in args.mode:
        #     from video_streamer import video_stream_thread
        #     t1 = threading.Thread(target=video_stream_thread, kwargs={"device_serials": device_serials})
        #     t1.daemon = True
        #     t1.start()
        #     threads.append(t1)

        if "command" in args.mode:
            t2 = threading.Thread(target=command_server_thread, kwargs={
                "repo_id": args.repo_id
            })
            t2.daemon = True
            t2.start()
            threads.append(t2)

        if "audio" in args.mode:
            audio_streamer = AudioStreamer()
            t3 = threading.Thread(target=audio_streamer.run)
            t3.daemon = True
            t3.start()
            threads.append(t3)

        print("[Main] 所有服务已启动，按 Ctrl+C 退出")
        
        # 等待所有线程结束
        for thread in threads:
            thread.join()
            
    except KeyboardInterrupt:
        print("\n[Main] 收到键盘中断，正在关闭...")
        signal_handler(signal.SIGINT, None)
    except Exception as e:
        print(f"[Main] 发生错误: {e}")
        signal_handler(signal.SIGTERM, None)

if __name__ == "__main__":
    main() 