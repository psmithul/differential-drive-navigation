import math

import numpy as np

from .robot import wrap


class Controller:
    def __init__(self, world, adaptive=True):
        self.world = world
        self.adaptive = adaptive
        self.path = world.plan(world.start, world.goal)
        self.target = 1
        self.recoveries = 0
        self.recovery_until = 0.0
        self.cooldown_until = 2.0
        self.slip_duration = 0.0
        self.finished = False
        self.failure = None

    def command(self, estimate, position_sigma, slip_score, time, dt):
        pose = estimate[:3]
        distance = np.linalg.norm(np.asarray(self.world.goal) - pose[:2])
        if distance < 0.18:
            self.finished = True
            return 0.0, 0.0
        if time < self.recovery_until:
            return 0.0, 0.0
        if self.recovery_until:
            try:
                self.path = self.world.plan(pose, self.world.goal)
            except ValueError:
                self.failure = "replan_failed"
                return 0.0, 0.0
            self.target = 1
            self.recovery_until = 0.0
        self.slip_duration = self.slip_duration + dt if slip_score > 3 else 0.0
        if self.adaptive and time >= self.cooldown_until and self.slip_duration >= 0.3:
            if self.recoveries >= 8:
                self.failure = "recovery_limit"
                return 0.0, 0.0
            self.recoveries += 1
            self.recovery_until = time + 1.0
            self.cooldown_until = time + 4.0
            self.slip_duration = 0.0
            return 0.0, 0.0
        while self.target < len(self.path) - 1 and np.linalg.norm(
                np.asarray(self.path[self.target]) - pose[:2]) < 0.20:
            self.target += 1
        offset = np.asarray(self.path[self.target]) - pose[:2]
        error = wrap(math.atan2(offset[1], offset[0]) - pose[2])
        speed = min(0.65, 1.2 * float(np.linalg.norm(offset)))
        speed *= max(0.0, math.cos(error))**3
        if self.adaptive:
            speed *= max(0.30, 1 / (1 + 3.0 * position_sigma))
            if slip_score > 3:
                speed *= 0.4
        return speed, float(np.clip(2.5 * error, -1.6, 1.6))
