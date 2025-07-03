import time
import socket

# 云迹底盘 - WIFI Addr
yunji_server_ip = "17.16.3.13"
yunji_server_port = 31001


class ControlMove:
    def __init__(self, deadzone=0.1):
        self.deadzone = deadzone
        # 底盘控制参数
        self.linear_speed = 0.45  # 线速度默认值 (m/s)
        self.angular_speed = 0.5  # 角速度默认值 (rad/s)

        self.left_stick_x = 0.0
        self.left_stick_y = 0.0
        # 命令控制参数
        self.last_command_time = time.time()
        self.command_interval = 0.1  # 发送命令的间隔时间(秒)，减小以提高响应性
        # 速度渐变控制参数
        self.current_linear_velocity = 0.0  # 当前线速度
        self.current_angular_velocity = 0.0  # 当前角速度
        self.target_linear_velocity = 0.0  # 目标线速度
        self.target_angular_velocity = 0.0  # 目标角速度
        self.max_linear_accel = 0.05  # 每个间隔最大线加速度 (m/s^2)
        self.max_angular_accel = 0.15  # 每个间隔最大角加速度 (rad/s^2)
        self.max_linear_velocity = 0.5  # 最大线速度限制 (m/s)
        self.max_angular_velocity = 0.5  # 最大角速度限制 (rad/s)

    def start(self, x, y):
        if abs(x) < self.deadzone and abs(y) < self.deadzone:
            self.left_stick_x = 0
            self.left_stick_y = 0
        else:
            self.left_stick_x = x
            self.left_stick_y = y
    
        self._update_target_velocity_by_stick()
        # 周期性应用速度渐变和发送命令
        current_time = time.time()
        if current_time - self.last_command_time >= self.command_interval:
            self.last_command_time = current_time
            # 应用速度渐变并发送命令
            self._apply_velocity_limits()

            # 发送相应的移动命令
            if abs(self.current_linear_velocity) < 0.01 and abs(self.current_angular_velocity) < 0.01:
                # 如果速度几乎为零，发送停止命令
                # 确保停止机器人移动
                try:
                    # 渐进停止而不是突然停止
                    self.target_linear_velocity = 0
                    self.target_angular_velocity = 0
                    self._apply_velocity_limits()
                    self.stop()
                    print("已停止机器人移动")
                except Exception as e:
                    print(f"停止机器人移动时出错: {e}")
            else:
                # 否则，发送带有当前渐变速度的移动命令
                self.move(self.current_linear_velocity, self.current_angular_velocity)

    def move(self, linear_velocity, angular_velocity):
        """发送移动指令到服务器
        参数:
            linear_velocity: 线速度，正值表示前进，负值表示后退
            angular_velocity: 角速度，正值表示左旋，负值表示右旋
        返回:
            服务器的响应或错误信息
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # 连接到云迹服务器
            sock.connect((yunji_server_ip, yunji_server_port))

            # 构建查询字符串
            print([angular_velocity, linear_velocity])
            query = f'/api/joy_control?angular_velocity={angular_velocity}&linear_velocity={linear_velocity}'
            # 发送移动命令
            sock.sendall(query.encode('utf-8'))
            # 接收响应
            response = sock.recv(4096).decode('utf-8')
            return response
        except Exception as e:
            return f"Error: {e}"
        finally:
            # 确保套接字关闭
            sock.close()

    def stop(self):
        """停止底盘移动

        返回:
            服务器的响应或错误信息
        """
        return self.move(linear_velocity=0, angular_velocity=0)

    def _update_target_velocity_by_stick(self):
        """根据左摇杆的输入更新目标速度"""
        try:
            # 如果摇杆在中央位置（考虑死区），目标速度设为0
            if abs(self.left_stick_x) < self.deadzone and abs(self.left_stick_y) < self.deadzone:
                self.target_linear_velocity = 0
                self.target_angular_velocity = 0
                return

            # 计算目标线速度和角速度（使用二次方映射实现更精细的控制）
            # 使用平方映射可以在低速时提供更平滑的控制
            y_sign = 1 if self.left_stick_y > 0 else -1
            x_sign = -1 if self.left_stick_x > 0 else 1  # 注意X轴方向是反的

            # 二次方映射，保留符号
            self.target_linear_velocity = (self.left_stick_y ** 2) * self.max_linear_velocity * y_sign
            self.target_angular_velocity = (self.left_stick_x ** 2) * self.max_angular_velocity * x_sign

            # 当高速行驶时限制转向速度，当快速转向时限制行驶速度
            # 这样可以防止同时快速前进和快速转向导致机器人失控
            linear_scale = 1.0 - abs(self.current_angular_velocity) / self.max_angular_velocity * 0.5
            angular_scale = 1.0 - abs(self.current_linear_velocity) / self.max_linear_velocity * 0.5
            self.target_linear_velocity *= max(0.5, linear_scale)  # 最多降低50%
            self.target_angular_velocity *= max(0.5, angular_scale)  # 最多降低50%

        except Exception as e:
            print(f"更新目标速度时出错: {e}")

    def _apply_velocity_limits(self):
        """应用速度渐变限制，使速度平滑变化"""
        try:
            # 计算线速度偏差
            linear_diff = self.target_linear_velocity - self.current_linear_velocity

            # 应用线加速度限制
            if abs(linear_diff) <= self.max_linear_accel:
                # 如果差值小于加速度限制，一次到位
                self.current_linear_velocity = self.target_linear_velocity
            else:
                # 否则，按最大加速度逐渐靠近目标速度
                self.current_linear_velocity += self.max_linear_accel if linear_diff > 0 else -self.max_linear_accel

            # 计算角速度偏差
            angular_diff = self.target_angular_velocity - self.current_angular_velocity

            # 应用角加速度限制
            if abs(angular_diff) <= self.max_angular_accel:
                # 如果差值小于加速度限制，一次到位
                self.current_angular_velocity = self.target_angular_velocity
            else:
                # 否则，按最大加速度逐渐靠近目标速度
                self.current_angular_velocity += self.max_angular_accel if angular_diff > 0 else -self.max_angular_accel

            # 特殊处理停止时的情况（应用更强的减速以快速停止）
            if abs(self.target_linear_velocity) < 0.01 and abs(self.current_linear_velocity) > 0:
                # 停止时使用更强的减速
                self.current_linear_velocity *= 0.8  # 减速20%
                if abs(self.current_linear_velocity) < 0.01:
                    self.current_linear_velocity = 0

            if abs(self.target_angular_velocity) < 0.01 and abs(self.current_angular_velocity) > 0:
                # 停止时使用更强的减速
                self.current_angular_velocity *= 0.8  # 减速20%
                if abs(self.current_angular_velocity) < 0.01:
                    self.current_angular_velocity = 0

        except Exception as e:
            print(f"应用速度渐变限制时出错: {e}")
