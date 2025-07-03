# 统一机械臂接口层 (Unified Robot Arm Interface)

一个插件式、线程安全的机械臂控制接口，支持多种机械臂型号的统一控制。

## 特性

### ✅ 核心功能
- **统一接口**: 所有机械臂使用相同的API
- **线程安全**: 独占控制线程，保证操作安全
- **固定频率控制**: 可配置的控制频率（默认100Hz）
- **轨迹滤波**: 可选的输入滤波，平滑运动轨迹
- **状态监控**: 实时状态获取和回调机制
- **日志记录**: 完整的操作日志
- **超时保护**: 命令超时自动处理
- **插件架构**: 易于添加新的机械臂型号

### 🔧 设计原则
1. **统一接口** - 所有机械臂实现相同的抽象接口
2. **线程安全** - 使用独占控制线程和锁机制
3. **可扩展** - 插件式架构，易于添加新型号
4. **可观测** - 提供状态查询和回调接口
5. **可控** - 支持位置、速度、笛卡尔控制
6. **可选滤波** - 支持多种滤波算法
7. **有日志** - 详细的操作日志记录

## 架构设计

```
robot_arm_interface/
├── core/                    # 核心组件
│   ├── base_arm.py         # 抽象基类定义
│   ├── arm_manager.py      # 机械臂管理器
│   └── arm_factory.py      # 工厂类
├── plugins/                # 机械臂插件
│   ├── simulation_arm.py   # 模拟机械臂
│   └── realman_arm.py      # Realman机械臂
├── filters/                # 轨迹滤波器
│   └── trajectory_filter.py
└── utils/                  # 工具函数
    └── config_loader.py
```

## 安装

```bash
# 安装依赖
pip install numpy pyyaml

# 如果使用Realman机械臂，需要确保realman_arm模块可用
```

## 快速开始

### 基本使用

```python
from robot_arm_interface import RobotArmManager, RobotArmFactory, ArmConfig
import robot_arm_interface.plugins.simulation_arm
import numpy as np

# 创建配置
config = ArmConfig(
    name="test_arm",
    dof=6,
    max_joint_velocities=np.array([2.0] * 6),
    max_joint_accelerations=np.array([5.0] * 6),
    joint_limits_lower=np.array([-np.pi] * 6),
    joint_limits_upper=np.array([np.pi] * 6),
    control_frequency=100.0
)

# 创建机械臂
robot_arm = RobotArmFactory.create("simulation", config)

# 创建管理器
manager = RobotArmManager(robot_arm, control_frequency=100.0)

# 启动并使用
manager.start()
manager.enable()

# 移动到目标位置
target = np.array([0.5, -0.3, 0.2, 0.0, 0.1, 0.0])
manager.move_joints(target, velocity_limit=0.5)

# 清理
manager.disable()
manager.stop()
```

### 使用配置文件

```python
from robot_arm_interface.utils.config_loader import create_arm_from_config_file

# 从配置文件创建机械臂
robot_arm = create_arm_from_config_file("configs/simulation_arm.yaml")
```

配置文件示例 (YAML):
```yaml
type: simulation
config:
  name: simulation_arm
  dof: 6
  max_joint_velocities: [2.0, 2.0, 2.0, 3.0, 3.0, 3.0]
  max_joint_accelerations: [5.0, 5.0, 5.0, 10.0, 10.0, 10.0]
  joint_limits_lower: [-3.14, -2.0, -2.0, -3.14, -2.0, -3.14]
  joint_limits_upper: [3.14, 2.0, 2.0, 3.14, 2.0, 3.14]
  control_frequency: 100.0
  connection_params:
    simulation_delay: 0.001
    error_probability: 0.0
```

## API 参考

### BaseRobotArm (抽象基类)

所有机械臂必须实现的接口：

```python
# 连接控制
connect(timeout: float = 5.0) -> bool
disconnect() -> bool

# 使能控制
enable() -> bool
disable() -> bool

# 状态获取
get_state() -> ArmState

# 运动控制
move_joints(joint_positions: np.ndarray, velocity_limit: float = None) -> bool
move_joints_velocity(joint_velocities: np.ndarray) -> bool
move_cartesian(pose: np.ndarray, velocity_limit: float = None) -> bool
stop() -> bool

# 运动学
get_forward_kinematics(joint_positions: np.ndarray) -> np.ndarray
get_inverse_kinematics(pose: np.ndarray) -> Optional[np.ndarray]
```

### RobotArmManager

管理器提供线程安全的控制：

```python
# 生命周期管理
start() -> bool
stop() -> bool

# 控制命令（线程安全）
enable(timeout: float = 5.0) -> bool
disable(timeout: float = 5.0) -> bool
move_joints(positions: np.ndarray, ...) -> bool
move_joints_velocity(velocities: np.ndarray) -> bool
move_cartesian(pose: np.ndarray, ...) -> bool
stop() -> bool

# 状态监控
get_state() -> Optional[ArmState]
add_state_callback(callback: Callable[[ArmState], None])
remove_state_callback(callback: Callable[[ArmState], None])
get_statistics() -> Dict[str, Any]
```

### 轨迹滤波

支持多种滤波方式：

```python
from robot_arm_interface.filters.trajectory_filter import TrajectoryFilter

# 创建滤波器
filter = TrajectoryFilter(
    dof=6,
    max_velocities=np.array([2.0] * 6),
    max_accelerations=np.array([5.0] * 6),
    filter_window=5,  # 移动平均窗口
    alpha=0.3  # 低通滤波系数
)

# 应用滤波
filtered_position = filter.filter(target_position)
```

## 创建新的机械臂插件

1. 继承 `BaseRobotArm` 类：

```python
from robot_arm_interface.core.base_arm import BaseRobotArm
from robot_arm_interface.core.arm_factory import register_robot_arm

@register_robot_arm("my_robot")
class MyRobotArm(BaseRobotArm):
    def connect(self, timeout: float = 5.0) -> bool:
        # 实现连接逻辑
        pass
    
    def move_joints(self, joint_positions: np.ndarray, ...) -> bool:
        # 实现运动控制
        pass
    
    # 实现其他必需方法...
```

2. 导入插件以注册：

```python
import robot_arm_interface.plugins.my_robot_arm
```

## 高级功能

### 状态回调

```python
def state_callback(state: ArmState):
    print(f"Position: {state.joint_positions}")
    print(f"Moving: {state.is_moving}")
    if state.has_error:
        print(f"Error: {state.error_message}")

manager.add_state_callback(state_callback)
```

### 自适应滤波

```python
from robot_arm_interface.filters.trajectory_filter import AdaptiveTrajectoryFilter

# 创建自适应滤波器（根据速度调整滤波强度）
adaptive_filter = AdaptiveTrajectoryFilter(
    dof=6,
    max_velocities=velocities,
    max_accelerations=accelerations,
    alpha_min=0.1,  # 快速运动时的滤波系数
    alpha_max=0.8   # 缓慢运动时的滤波系数
)
```

### 动态加载插件

```python
# 从模块动态加载
RobotArmFactory.load_plugin(
    module_path="my_package.my_robot",
    class_name="MyRobotArm",
    register_name="my_robot"
)

# 从配置文件加载多个插件
plugins_config = [
    {"module": "my_package.robot1", "class": "Robot1Arm", "name": "robot1"},
    {"module": "my_package.robot2", "class": "Robot2Arm", "name": "robot2"}
]
RobotArmFactory.load_plugins_from_config(plugins_config)
```

## 示例程序

- `examples/basic_usage.py` - 基本使用示例
- `examples/advanced_usage.py` - 高级功能示例

## 注意事项

1. **线程安全**: Manager的所有控制方法都是线程安全的
2. **频率控制**: 控制循环运行在独立线程，保证固定频率
3. **资源清理**: 使用完毕后务必调用 `stop()` 释放资源
4. **错误处理**: 检查返回值和状态中的错误信息
5. **单位约定**: 
   - 关节角度：弧度
   - 笛卡尔位置：米
   - 速度限制：0-1的比例值

## 许可证

MIT License