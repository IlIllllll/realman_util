#!/usr/bin/env python3
"""
通讯延时测试脚本
测试与策略服务器的WebSocket通讯延时
"""

import asyncio
import websockets
import msgpack
import time
import numpy as np
from typing import Dict, Any
import logging

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def pack_array(obj):
    """msgpack的numpy数组打包函数"""
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }
    return obj

def unpack_array(obj):
    """msgpack的numpy数组解包函数"""
    if b"__ndarray__" in obj:
        return np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"])
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj

class LatencyTester:
    def __init__(self, uri: str):
        self.uri = uri
        self.websocket = None
        self.test_count = 0
        self.latencies = []
        
    async def connect(self):
        """连接到WebSocket服务器"""
        try:
            logger.info(f"正在连接到服务器: {self.uri}")
            self.websocket = await websockets.connect(self.uri)
            
            # 接收服务器元数据
            metadata_bytes = await self.websocket.recv()
            metadata = msgpack.unpackb(metadata_bytes, object_hook=unpack_array)
            logger.info(f"连接成功! 服务器元数据: {metadata}")
            return True
        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False
    
    async def disconnect(self):
        """断开WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            logger.info("连接已断开")
    
    def create_test_observation(self) -> Dict[str, Any]:
        """创建测试用的观察数据"""
        # 创建模拟的图像数据 (224x224 RGB图像)
        mock_image = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
        mock_wrist_image = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
        
        # 创建模拟的机械臂状态 (7维向量: x, y, z, rx, ry, rz, gripper)
        mock_state = np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0,0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        observation = {
            "images": {
                "cam_high": mock_image,
                "cam_left_wrist": mock_wrist_image,
                "cam_right_wrist": mock_wrist_image,
            },
            "state": mock_state,
            "prompt": "pick up the blue cube",
        }
        return observation
    
    async def test_single_request(self) -> float:
        """测试单次请求的延时"""
        if not self.websocket:
            raise RuntimeError("WebSocket未连接")
        
        # 创建测试数据
        observation = self.create_test_observation()
        
        # 记录发送时间
        send_time = time.time()
        
        try:
            # 使用msgpack打包数据
            data = msgpack.packb(observation, default=pack_array)
            
            # 发送数据
            await self.websocket.send(data)
            
            # 接收响应
            response_bytes = await self.websocket.recv()
            
            # 记录接收时间
            receive_time = time.time()
            
            # 计算延时
            latency = (receive_time - send_time) * 1000  # 转换为毫秒
            
            # 使用msgpack解包响应
            response_data = msgpack.unpackb(response_bytes, object_hook=unpack_array)
            
            logger.info(f"请求 {self.test_count + 1}: 延时 {latency:.2f}ms")
            logger.info(f"响应包含 {len(response_data.get('actions', []))} 个动作")
            
            return latency
            
        except Exception as e:
            logger.error(f"请求失败: {e}")
            return -1
    
    async def run_latency_test(self, num_tests: int = 10, interval: float = 1.0):
        """运行延时测试"""
        logger.info(f"开始延时测试，共 {num_tests} 次请求，间隔 {interval} 秒")
        
        if not await self.connect():
            return
        
        try:
            for i in range(num_tests):
                self.test_count = i
                latency = await self.test_single_request()
                
                if latency >= 0:
                    self.latencies.append(latency)
                
                # 等待间隔时间
                if i < num_tests - 1:  # 最后一次不需要等待
                    await asyncio.sleep(interval)
                    
        finally:
            await self.disconnect()
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.latencies:
            logger.warning("没有成功的测试数据")
            return
        
        latencies = np.array(self.latencies)
        
        logger.info("=" * 50)
        logger.info("延时测试统计结果:")
        logger.info(f"总测试次数: {self.test_count + 1}")
        logger.info(f"成功次数: {len(self.latencies)}")
        logger.info(f"平均延时: {np.mean(latencies):.2f}ms")
        logger.info(f"最小延时: {np.min(latencies):.2f}ms")
        logger.info(f"最大延时: {np.max(latencies):.2f}ms")
        logger.info(f"标准差: {np.std(latencies):.2f}ms")
        logger.info(f"中位数: {np.median(latencies):.2f}ms")
        logger.info("=" * 50)

async def main():
    """主函数"""
    # 使用与原始脚本相同的服务器地址
    uri = "wss://sv-6360e381-0998-4746-ac37-229543e01243-8000-x-defau-da2d73a9c8.sproxy.hd-01.alayanew.com:22443/"
    
    # 创建测试器
    tester = LatencyTester(uri)
    
    try:
        # 运行延时测试
        await tester.run_latency_test(num_tests=20, interval=0.5)
        
        # 打印统计结果
        tester.print_statistics()
        
    except KeyboardInterrupt:
        logger.info("测试被用户中断")
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
    finally:
        await tester.disconnect()

if __name__ == "__main__":
    # 运行异步主函数
    asyncio.run(main()) 
