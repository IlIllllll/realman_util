import numpy as np
from collections import deque

# ========== 滑动窗口滤波类 ==========
class SlidingWindowFilter:
    """
    滑动窗口滤波器，用于平滑机械臂控制信号
    
    支持多种滤波方法：
    - 移动平均滤波
    - 加权移动平均滤波
    - 中值滤波
    - 指数移动平均滤波
    """
    
    def __init__(self, window_size=5, filter_type="moving_average", alpha=0.3):
        """
        初始化滑动窗口滤波器
        
        Args:
            window_size: 滑动窗口大小
            filter_type: 滤波类型 ("moving_average", "weighted_average", "median", "exponential")
            alpha: 指数移动平均的平滑系数 (0-1)
        """
        self.window_size = window_size
        self.filter_type = filter_type
        self.alpha = alpha
        
        # 数据存储
        self.data_buffer = deque(maxlen=window_size)
        self.filtered_value = None
        self.is_initialized = False
        
    def filter(self, new_value):
        """
        对新的输入值进行滤波
        
        Args:
            new_value: 新的输入值 (numpy array 或标量)
            
        Returns:
            滤波后的值
        """
        # 确保输入是numpy数组
        if not isinstance(new_value, np.ndarray):
            new_value = np.array(new_value)
        
        # 添加到缓冲区
        self.data_buffer.append(new_value.copy())
        
        # 如果数据不足，直接返回原始值
        if len(self.data_buffer) < 2:
            self.filtered_value = new_value.copy()
            self.is_initialized = True
            return self.filtered_value
        
        # 根据滤波类型进行处理
        if self.filter_type == "moving_average":
            self.filtered_value = self._moving_average()
        elif self.filter_type == "weighted_average":
            self.filtered_value = self._weighted_average()
        elif self.filter_type == "median":
            self.filtered_value = self._median_filter()
        elif self.filter_type == "exponential":
            self.filtered_value = self._exponential_average(new_value)
        else:
            # 默认使用移动平均
            self.filtered_value = self._moving_average()
        
        return self.filtered_value.copy()
    
    def _moving_average(self):
        """移动平均滤波"""
        return np.mean(list(self.data_buffer), axis=0)
    
    def _weighted_average(self):
        """加权移动平均滤波，越新的数据权重越大"""
        weights = np.linspace(0.1, 1.0, len(self.data_buffer))
        weights = weights / np.sum(weights)  # 归一化权重
        
        weighted_sum = np.zeros_like(self.data_buffer[0])
        for i, data in enumerate(self.data_buffer):
            weighted_sum += weights[i] * data
        
        return weighted_sum
    
    def _median_filter(self):
        """中值滤波"""
        data_array = np.array(list(self.data_buffer))
        return np.median(data_array, axis=0)
    
    def _exponential_average(self, new_value):
        """指数移动平均滤波"""
        if not self.is_initialized:
            self.filtered_value = new_value.copy()
            return self.filtered_value
        
        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        """重置滤波器状态"""
        self.data_buffer.clear()
        self.filtered_value = None
        self.is_initialized = False
    
    def set_parameters(self, window_size=None, filter_type=None, alpha=None):
        """更新滤波器参数"""
        if window_size is not None and window_size > 0:
            old_buffer = list(self.data_buffer)
            self.window_size = window_size
            self.data_buffer = deque(old_buffer[-window_size:], maxlen=window_size)
        
        if filter_type is not None:
            self.filter_type = filter_type
        
        if alpha is not None and 0 < alpha <= 1:
            self.alpha = alpha


class DualHandFilter:
    """
    双手滑动窗口滤波器，分别对左右手进行滤波处理
    """
    
    def __init__(self, left_filter_config=None, right_filter_config=None):
        """
        初始化双手滤波器
        
        Args:
            left_filter_config: 左手滤波器配置字典
            right_filter_config: 右手滤波器配置字典
        """
        # 默认配置
        default_config = {
            "window_size": 5,
            "filter_type": "moving_average",
            "alpha": 0.3
        }
        
        # 左手滤波器
        left_config = default_config.copy()
        if left_filter_config:
            left_config.update(left_filter_config)
        self.left_filter = SlidingWindowFilter(**left_config)
        
        # 右手滤波器
        right_config = default_config.copy()
        if right_filter_config:
            right_config.update(right_filter_config)
        self.right_filter = SlidingWindowFilter(**right_config)
        
        # 滤波统计
        self.filter_stats = {
            "left_calls": 0,
            "right_calls": 0,
            "both_calls": 0
        }
    
    def filter_left_hand(self, left_delta_pose, left_grip=0.0):
        """
        对左手数据进行滤波
        
        Args:
            left_delta_pose: 左手位置增量 [x, y, z, qx, qy, qz, qw]
            left_grip: 左手夹爪值
            
        Returns:
            滤波后的左手数据 (delta_pose, grip)
        """
        # 将位置和夹爪组合成一个数组进行滤波
        left_data = np.concatenate([left_delta_pose, [left_grip]])
        filtered_left_data = self.left_filter.filter(left_data)
        
        # 分离位置和夹爪
        filtered_delta_pose = filtered_left_data[:7]
        filtered_grip = filtered_left_data[7]
        
        self.filter_stats["left_calls"] += 1
        
        return filtered_delta_pose, filtered_grip
    
    def filter_right_hand(self, right_delta_pose, right_grip=0.0):
        """
        对右手数据进行滤波
        
        Args:
            right_delta_pose: 右手位置增量 [x, y, z, qx, qy, qz, qw]
            right_grip: 右手夹爪值
            
        Returns:
            滤波后的右手数据 (delta_pose, grip)
        """
        # 将位置和夹爪组合成一个数组进行滤波
        right_data = np.concatenate([right_delta_pose, [right_grip]])
        filtered_right_data = self.right_filter.filter(right_data)
        
        # 分离位置和夹爪
        filtered_delta_pose = filtered_right_data[:7]
        filtered_grip = filtered_right_data[7]
        
        self.filter_stats["right_calls"] += 1
        
        return filtered_delta_pose, filtered_grip
    
    def filter_both_hands(self, left_delta_pose=None, right_delta_pose=None, 
                         left_grip=0.0, right_grip=0.0):
        """
        对双手数据进行滤波
        
        Args:
            left_delta_pose: 左手位置增量
            right_delta_pose: 右手位置增量
            left_grip: 左手夹爪值
            right_grip: 右手夹爪值
            
        Returns:
            滤波后的双手数据字典
        """
        result = {}
        
        if left_delta_pose is not None:
            filtered_left_pose, filtered_left_grip = self.filter_left_hand(left_delta_pose, left_grip)
            result["left_delta_pose"] = filtered_left_pose
            result["left_grip"] = filtered_left_grip
        
        if right_delta_pose is not None:
            filtered_right_pose, filtered_right_grip = self.filter_right_hand(right_delta_pose, right_grip)
            result["right_delta_pose"] = filtered_right_pose
            result["right_grip"] = filtered_right_grip
        
        if left_delta_pose is not None and right_delta_pose is not None:
            self.filter_stats["both_calls"] += 1
        
        return result
    
    def reset(self):
        """重置两个滤波器"""
        self.left_filter.reset()
        self.right_filter.reset()
        self.filter_stats = {"left_calls": 0, "right_calls": 0, "both_calls": 0}
    
    def get_stats(self):
        """获取滤波统计信息"""
        return self.filter_stats.copy()
    
    def set_left_filter_parameters(self, **kwargs):
        """设置左手滤波器参数"""
        self.left_filter.set_parameters(**kwargs)
    
    def set_right_filter_parameters(self, **kwargs):
        """设置右手滤波器参数"""
        self.right_filter.set_parameters(**kwargs) 