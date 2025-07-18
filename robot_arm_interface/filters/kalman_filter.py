import numpy as np
from typing import Iterable, List, Tuple, Optional

class KalmanPoseFilter:
    """
    适用于6自由度机械臂末端位姿的Kalman滤波器。
    状态向量: [x, vx, y, vy, z, vz, yaw, vyaw, pitch, vpitch, roll, vroll]
    观测向量: [x, y, z, yaw, pitch, roll]
    """
    def __init__(
        self,
        dt: float = 0.01,
        process_var_pos: float = 1.0e-3,
        process_var_vel: float = 1.0e-2,
        meas_var_pos: float = 0.05,
        meas_var_ang: float = 2.0,
    ):
        self.dt = dt
        F_block = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        self.F = np.kron(np.eye(6), F_block)  # 12x12
        self.H = np.zeros((6, 12), dtype=float)
        for i in range(6):
            self.H[i, 2 * i] = 1.0
        q_diag = []
        for _ in range(6):
            q_diag.extend([process_var_pos, process_var_vel])
        self.Q = np.diag(q_diag)
        r_diag = [meas_var_pos] * 3 + [meas_var_ang] * 3
        self.R = np.diag(r_diag)
        self.x = np.zeros((12, 1), dtype=float)
        self.P = np.eye(12, dtype=float)

    def reset(self):
        self.x = np.zeros((12, 1), dtype=float)
        self.P = np.eye(12, dtype=float)

    def filter(self, measurement: Iterable[float]) -> np.ndarray:
        """
        输入: [x, y, z, yaw, pitch, roll]
        输出: 滤波后的[x, y, z, yaw, pitch, roll]
        """
        z_pose = np.asarray(measurement, dtype=float).reshape((6, 1))
        # 预测
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # 更新
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z_pose - self.H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(12) - K @ self.H) @ self.P
        filtered_pose = (self.H @ self.x).flatten()
        return filtered_pose 