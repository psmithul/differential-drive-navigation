import math

import numpy as np

from .robot import wrap


class EKF:
    """State: x, y, heading, forward speed, yaw rate, gyro bias."""

    def __init__(self, pose):
        self.state = np.array([*pose, 0.0, 0.0, 0.0], dtype=float)
        self.cov = np.diag([0.01**2, 0.01**2, 0.01**2, 0.1**2, 0.1**2, 0.08**2])
        self.H = np.zeros((3, 6))
        self.H[0, 3] = self.H[1, 4] = self.H[2, 4] = self.H[2, 5] = 1
        self.R = np.diag([0.003**2, 0.012**2, 0.015**2])
        self.slip_score = 0.0

    def predict(self, dt):
        yaw, speed, rate = self.state[2:5]
        mid = yaw + rate * dt / 2
        c, s = math.cos(mid), math.sin(mid)
        F = np.eye(6)
        F[0, 2], F[0, 3], F[0, 4] = -speed * s * dt, c * dt, -speed * s * dt**2 / 2
        F[1, 2], F[1, 3], F[1, 4] = speed * c * dt, s * dt, speed * c * dt**2 / 2
        F[2, 4] = dt
        self.state[0] += speed * c * dt
        self.state[1] += speed * s * dt
        self.state[2] = wrap(yaw + rate * dt)
        G = np.zeros((6, 2))
        G[:, 0] = [c * dt, s * dt, 0, 1, 0, 0]
        G[:, 1] = [-speed * s * dt**2 / 2, speed * c * dt**2 / 2, dt, 0, 1, 0]
        Q = G @ np.diag([0.6**2, 1.8**2]) @ G.T * dt
        Q += np.diag([0.003**2, 0.003**2, 0.005**2, 0, 0, 0.008**2]) * dt
        if self.slip_score > 3:
            Q[:2, :2] += np.eye(2) * (0.3 * abs(speed))**2 * dt
        self.cov = F @ self.cov @ F.T + Q

    def update(self, measurement):
        residual = measurement - self.H @ self.state
        mismatch = measurement[2] - measurement[1] - self.state[5]
        variance = self.R[1, 1] + self.R[2, 2] + self.cov[5, 5]
        self.slip_score = abs(mismatch) / math.sqrt(variance)
        R = self.R.copy()
        if self.slip_score > 3:
            R[0, 0] *= 25
            R[1, 1] *= 100
        S = self.H @ self.cov @ self.H.T + R
        K = np.linalg.solve(S, self.H @ self.cov).T
        self.state += K @ residual
        self.state[2] = wrap(self.state[2])
        correction = np.eye(6) - K @ self.H
        self.cov = correction @ self.cov @ correction.T + K @ R @ K.T
        self.cov = (self.cov + self.cov.T) / 2

    @property
    def position_sigma(self):
        return math.sqrt(float(np.linalg.eigvalsh(self.cov[:2, :2])[-1]))
