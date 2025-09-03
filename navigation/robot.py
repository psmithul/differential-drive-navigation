from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def integrate(pose, speed, yaw_rate, dt):
    x, y, yaw = pose
    turn = yaw_rate * dt
    distance = speed * dt * np.sinc(turn / (2 * math.pi))
    return np.array([x + distance * math.cos(yaw + turn / 2),
                     y + distance * math.sin(yaw + turn / 2), wrap(yaw + turn)])


@dataclass(frozen=True)
class Scenario:
    encoder_scale: float = 0.0005
    encoder_noise: float = 0.0002
    gyro_noise: float = 0.015
    initial_bias: float = 0.015
    bias_walk: float = 0.001
    slip_rate: float = 0.0
    slip_loss: float = 0.0


SCENARIOS = {
    "nominal": Scenario(),
    "slip": Scenario(slip_rate=0.18, slip_loss=0.65),
    "drift": Scenario(initial_bias=0.06, bias_walk=0.008),
    "combined": Scenario(initial_bias=0.06, bias_walk=0.008,
                         slip_rate=0.18, slip_loss=0.65),
}


class Robot:
    track = 0.36
    radius = 0.20
    wheel_radius = 0.06
    ticks_per_turn = 2048

    def __init__(self, pose, scenario, seed):
        self.pose = np.array(pose, dtype=float)
        self.scenario = scenario
        self.rng = np.random.default_rng(seed)
        self.scale = self.rng.normal(1.0, scenario.encoder_scale, 2)
        self.bias = self.rng.normal(0.0, scenario.initial_bias)
        self.slip_remaining = 0.0
        self.traction = np.ones(2)
        self.encoder_total = np.zeros(2)
        self.previous_ticks = np.zeros(2)

    def step(self, speed, yaw_rate, dt):
        s = self.scenario
        event, side, loss, duration = self.rng.random(4)
        if self.slip_remaining <= 0:
            self.traction[:] = 1.0
            if event < 1 - math.exp(-s.slip_rate * dt):
                self.traction[int(side * 2)] = 1 - s.slip_loss * (0.5 + 0.5 * loss)
                self.slip_remaining = 0.4 + duration * 1.2
        self.slip_remaining -= dt
        wheels = np.array([speed - yaw_rate * self.track / 2,
                           speed + yaw_rate * self.track / 2])
        ground = wheels * self.traction
        true_speed = float(ground.mean())
        true_yaw_rate = float((ground[1] - ground[0]) / self.track)
        self.pose = integrate(self.pose, true_speed, true_yaw_rate, dt)
        self.bias += self.rng.normal(0, s.bias_walk * math.sqrt(dt))
        self.encoder_total += wheels * dt * self.scale + self.rng.normal(0, s.encoder_noise, 2)
        tick_size = 2 * math.pi * self.wheel_radius / self.ticks_per_turn
        ticks = np.rint(self.encoder_total / tick_size)
        wheel_speed = (ticks - self.previous_ticks) * tick_size / dt
        self.previous_ticks = ticks
        measurements = np.array([wheel_speed.mean(),
                                 (wheel_speed[1] - wheel_speed[0]) / self.track,
                                 true_yaw_rate + self.bias + self.rng.normal(0, s.gyro_noise)])
        return measurements
