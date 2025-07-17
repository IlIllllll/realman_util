# LeRobot数据集单臂机器人回放

这个模块提供了LeRobot数据集的回放功能，可以将录制的机器人动作数据重新执行，同时显示对应的相机图像。

## 功能特性

- **数据集加载**: 支持加载LeRobot格式的数据集
- **机器人控制**: 通过Realman机器人API执行回放动作
- **图像显示**: 实时显示顶部相机和手腕相机的图像
- **交互控制**: 支持暂停、继续、速度调节等交互功能
- **多种模式**: 支持单次回放和交互式回放模式

## 文件结构

```
replay/
├── single_arm_replay.py    # 主要的回放脚本
├── example_usage.py        # 使用示例脚本
└── README.md              # 说明文档
```

## 安装依赖

确保已安装以下依赖包：

```bash
pip install numpy opencv-python lerobot
```

## 使用方法

### 1. 命令行使用

#### 基本回放
```bash
python replay/single_arm_replay.py /path/to/dataset
```

#### 回放指定episode
```bash
python replay/single_arm_replay.py /path/to/dataset --episode 1
```

#### 交互式模式
```bash
python replay/single_arm_replay.py /path/to/dataset --interactive
```

#### 指定机器人IP
```bash
python replay/single_arm_replay.py /path/to/dataset --robot_ip 192.168.1.20
```

### 2. 命令行参数

- `dataset_path`: 数据集路径（必需）
- `--robot_ip`: 机器人IP地址（默认: 192.168.1.19）
- `--episode`: 指定回放的episode ID（从0开始）
- `--interactive`: 启用交互式模式

### 3. 交互控制

在回放过程中，可以使用以下按键进行控制：

- `q`: 退出回放
- `空格`: 暂停/继续
- `s`: 加快回放速度
- `a`: 减慢回放速度

### 4. 交互式模式命令

在交互式模式下，可以使用以下命令：

- `l`: 列出所有episode
- `r <episode_id>`: 回放指定episode
- `n`: 回放下一个episode
- `p`: 回放上一个episode
- `q`: 退出

## 数据集格式

回放脚本支持的数据集格式与录制脚本一致，包含以下特征：

- `top_image`: 顶部相机图像 (480, 640, 3)
- `wrist_image`: 手腕相机图像 (480, 640, 3)
- `state`: 机器人位姿状态 (7,) - [x, y, z, rx, ry, rz, gripper]
- `actions`: 动作数据 (7,) - [x, y, z, rx, ry, rz, gripper]
- `joint`: 关节角度 (7,) - [j1, j2, j3, j4, j5, j6, gripper]

## 使用示例

### 基本使用示例

```python
from replay.single_arm_replay import SingleArmReplay

# 创建回放器
replay = SingleArmReplay("/path/to/dataset", robot_ip="192.168.1.19")

# 加载数据集
if replay.load_dataset():
    # 连接机器人
    if replay.connect_robot():
        # 回放第一个episode
        replay.replay_episode(0)
    
    # 清理资源
    replay.cleanup()
```

### 交互式使用示例

```python
from replay.single_arm_replay import SingleArmReplay

# 创建回放器
replay = SingleArmReplay("/path/to/dataset", robot_ip="192.168.1.19")

# 加载数据集和连接机器人
if replay.load_dataset() and replay.connect_robot():
    # 启动交互式模式
    replay.interactive_replay()
    
    # 清理资源
    replay.cleanup()
```

## 运行示例脚本

运行示例脚本来了解各种使用方法：

```bash
python replay/example_usage.py
```

## 注意事项

1. **机器人连接**: 确保机器人IP地址正确且网络连接正常
2. **数据集路径**: 确保数据集路径存在且格式正确
3. **权限设置**: 确保有足够的权限访问数据集和连接机器人
4. **安全考虑**: 在回放前确保机器人周围环境安全
5. **图像显示**: 需要图形界面支持来显示相机图像

## 故障排除

### 常见问题

1. **数据集加载失败**
   - 检查数据集路径是否正确
   - 确认数据集格式是否符合LeRobot标准

2. **机器人连接失败**
   - 检查机器人IP地址是否正确
   - 确认网络连接正常
   - 检查机器人是否处于可连接状态

3. **图像显示问题**
   - 确认系统支持图形界面
   - 检查OpenCV是否正确安装

4. **回放速度问题**
   - 调整`frame_delay`参数
   - 使用`s`和`a`键调节回放速度

### 调试模式

可以通过修改代码中的调试输出来获取更多信息：

```python
# 在SingleArmReplay类中添加调试输出
print(f"调试信息: {debug_data}")
```

## 扩展功能

可以根据需要扩展以下功能：

1. **轨迹平滑**: 添加轨迹插值和平滑处理
2. **错误恢复**: 添加机器人错误检测和恢复机制
3. **数据验证**: 添加数据集完整性检查
4. **性能优化**: 优化图像处理和机器人控制性能
5. **多机器人支持**: 扩展支持多机器人协同回放

## 贡献

欢迎提交问题报告和功能请求。如需贡献代码，请遵循项目的编码规范。

## 许可证

本项目遵循项目主许可证。 