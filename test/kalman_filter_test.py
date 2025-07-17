# robot_arm_kalman_filter.py
"""
Kalman filtering for a 6‑DoF robotic arm end‑effector pose
(x, y, z in metres; yaw, pitch, roll in degrees) plus a binary gripper state.
The *grip* signal (open = 0 / closed = 1) is **forwarded without filtering** but is
robustly converted to an integer so that boolean / NumPy boolean / float values
all work without errors.

State vector (size 12)
----------------------
    [x, vx, y, vy, z, vz, yaw, vyaw, pitch, vpitch, roll, vroll]^T
    even indices → positions / angles
    odd  indices → linear / angular velocities

Measurement vector (size 6)
---------------------------
    [x, y, z, yaw, pitch, roll]^T

A constant‑velocity motion model is used between samples separated by a fixed
``dt``.  If you need a different kinematics model (e.g. constant‑acceleration),
modify ``self.F`` and ``self.Q`` accordingly.

Quick start
~~~~~~~~~~~
```python
from robot_arm_kalman_filter import RobotArmKalmanFilter
kf = RobotArmKalmanFilter(dt=0.01)  # 100 Hz
pose, grip = kf.step([x, y, z, yaw, pitch, roll, grip_raw])
```

Run the file directly to see a noisy circular trajectory filtered and plotted.
Run ``python robot_arm_kalman_filter.py --test`` to execute the built‑in unit
tests.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

import numpy as np

__all__ = ["RobotArmKalmanFilter"]


class RobotArmKalmanFilter:
    """Lightweight Kalman filter tailored to a robot end‑effector pose."""

    # ------------------------------------------------------------------
    # Construction helpers
    def __init__(
        self,
        dt: float = 0.01,
        process_var_pos: float = 1.0e-3,
        process_var_vel: float = 1.0e-2,
        meas_var_pos: float = 0.05,
        meas_var_ang: float = 2,
    ) -> None:
        """Create a Kalman filter for a 6‑DoF pose.

        Parameters
        ----------
        dt
            Sampling interval (seconds).
        process_var_pos
            Process‑noise variance for the *position/angle* states.
        process_var_vel
            Process‑noise variance for the *velocity* states.
        meas_var_pos
            Measurement‑noise variance for x, y, z (metres²).
        meas_var_ang
            Measurement‑noise variance for yaw, pitch, roll (degrees²).
        """
        self.dt = dt

        # Constant‑velocity state‑transition (block‑diag of [[1 dt]; [0 1]])
        F_block = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        self.F = np.kron(np.eye(6), F_block)  # 12×12

        # Observation matrix – selects the 6 position/angle states
        self.H = np.zeros((6, 12), dtype=float)
        for i in range(6):
            self.H[i, 2 * i] = 1.0

        # Process‑noise covariance (diagonal)
        q_diag: list[float] = []
        for _ in range(6):
            q_diag.extend([process_var_pos, process_var_vel])
        self.Q = np.diag(q_diag)

        # Measurement‑noise covariance (diagonal)
        r_diag = [meas_var_pos] * 3 + [meas_var_ang] * 3
        self.R = np.diag(r_diag)

        # Initial state estimate x and covariance P
        self.x = np.zeros((12, 1), dtype=float)
        self.P = np.eye(12, dtype=float)

    # ------------------------------------------------------------------
    # Internal Kalman math
    def _predict(self) -> None:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def _update(self, z: np.ndarray) -> None:
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(12) - K @ self.H) @ self.P

    # ------------------------------------------------------------------
    # Public API
    def step(self, measurement: Iterable[float | bool]) -> Tuple[np.ndarray, int]:
        """Advance the filter by one measurement sample.

        Parameters
        ----------
        measurement
            ``[x, y, z, yaw, pitch, roll, grip]`` where *grip* may be 0/1,
            ``True``/``False``, or a noisy float in [0, 1].

        Returns
        -------
        filtered_pose, grip
            ``filtered_pose`` – 6‑element NumPy array (x, y, z, yaw, pitch, roll)
            ``grip`` – integer 0 or 1 forwarded from the input.
        """
        m = list(measurement)
        if len(m) != 7:
            raise ValueError(
                "Measurement must have 7 elements (x, y, z, yaw, pitch, roll, grip)"
            )

        # Pose part
        z_pose = np.asarray(m[:6], dtype=float).reshape((6, 1))

        # Robust grip conversion – allows numpy.bool_, bool, float, int
        grip_raw = m[6]
        grip = int(round(float(grip_raw)))  # bool→float works, numpy.bool_→float too

        self._predict()
        self._update(z_pose)

        filtered_pose = (self.H @ self.x).flatten()
        return filtered_pose, grip

    # Convenience
    def filter_sequence(
        self, measurements: Iterable[Iterable[float | bool]]
    ) -> List[Tuple[np.ndarray, int]]:
        """Filter an entire iterable of measurements (e.g. a generator or list)."""
        return [self.step(m) for m in measurements]


# -------------------------------------------------------------------------
# Built‑in unit tests (run with: python robot_arm_kalman_filter.py --test)
# -------------------------------------------------------------------------
import unittest


class _GripConversionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.kf = RobotArmKalmanFilter()

    def _run_once(self, grip_val):
        pose_in = [0, 0, 0, 0, 0, 0, grip_val]
        _, grip_out = self.kf.step(pose_in)
        return grip_out

    def test_bool_true(self):
        self.assertEqual(self._run_once(True), 1)

    def test_bool_false(self):
        self.assertEqual(self._run_once(False), 0)

    def test_numpy_bool(self):
        self.assertEqual(self._run_once(np.bool_(True)), 1)

    def test_float_rounding(self):
        self.assertEqual(self._run_once(0.6), 1)
        self.assertEqual(self._run_once(0.4), 0)


# -------------------------------------------------------------------------
# Demo / visual sanity‑check
# -------------------------------------------------------------------------

def _demo() -> None:
    import matplotlib.pyplot as plt

    np.random.seed(1)
    dt = 0.02  # 50 Hz
    kf = RobotArmKalmanFilter(dt=dt)

    # Synthetic circular XY trajectory with a ramp in Z
    t = np.arange(0, 4.0, dt)
    radius = 0.3  # m
    ang_speed = 45.0  # deg/s
    true_traj = [
        (
            radius * np.cos(0.5 * np.pi * ti),  # x
            radius * np.sin(0.5 * np.pi * ti),  # y
            0.5 * ti,  # z ramp
            ang_speed * ti,  # yaw
            10.0 * np.sin(0.25 * np.pi * ti),  # pitch
            5.0,  # roll constant
            ti > 2.0,  # grip closes halfway through
        )
        for ti in t
    ]

    # Add noise
    meas = [
        (
            x + np.random.normal(0, 0.02),
            y + np.random.normal(0, 0.02),
            z + np.random.normal(0, 0.02),
            yaw + np.random.normal(0, 2),
            pitch + np.random.normal(0, 2),
            roll + np.random.normal(0, 2),
            grip,
        )
        for x, y, z, yaw, pitch, roll, grip in true_traj
    ]

    filt = kf.filter_sequence(meas)
    filt_pose = np.stack([fp for fp, _ in filt])
    true_pose = np.stack([tp[:6] for tp in true_traj])

    plt.figure()
    plt.plot(t, true_pose[:, 0], label="True x")
    plt.plot(t, [m[0] for m in meas], label="Measured x", linestyle=":")
    plt.plot(t, filt_pose[:, 0], label="Filtered x")
    plt.xlabel("Time [s]")
    plt.ylabel("x [m]")
    plt.legend()
    plt.title("Kalman filtering example – X coordinate")
    plt.show()


# -------------------------------------------------------------------------
# Main entry point – switch between tests and demo via CLI flag
# -------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    if "--test" in sys.argv:
        sys.argv = [arg for arg in sys.argv if arg != "--test"]
        unittest.main(verbosity=2, exit=False)
    else:
        _demo()
