from dataclasses import dataclass
import math

import numpy as np

from .controller import Controller
from .ekf import EKF
from .robot import Robot, SCENARIOS, integrate, wrap
from .world import World


@dataclass
class Result:
    seed: int
    scenario: str
    policy: str
    outcome: str
    success: bool
    collisions: int
    position_rmse_m: float
    odometry_rmse_m: float
    heading_rmse_rad: float
    final_error_m: float
    goal_error_m: float
    recoveries: int
    elapsed_s: float
    time_to_goal_s: float | None


def run(seed=0, scenario="nominal", policy="adaptive", max_time=120.0, dt=0.1,
        world=None, trace=False):
    if scenario not in SCENARIOS or policy not in ("adaptive", "fixed"):
        raise ValueError("Unknown scenario or policy")
    if not math.isfinite(dt) or not math.isfinite(max_time) or dt <= 0 or max_time < dt:
        raise ValueError("Require finite max_time >= dt > 0")
    world = world or World()
    robot = Robot(world.start, SCENARIOS[scenario], seed)
    ekf = EKF(world.start)
    controller = Controller(world, policy == "adaptive")
    odometry = np.array(world.start, dtype=float)
    errors, odom_errors, heading_errors, history = [], [], [], []
    outcome, elapsed = "timeout", 0.0
    initial_path = list(controller.path)
    for step in range(math.floor(math.nextafter(max_time / dt, math.inf))):
        speed, turn = controller.command(ekf.state, ekf.position_sigma,
                                         ekf.slip_score, step * dt, dt)
        if controller.finished:
            outcome = "success" if np.linalg.norm(robot.pose[:2] - world.goal) < 0.35 else "missed_goal"
            break
        if controller.failure:
            outcome = controller.failure
            break
        previous = robot.pose.copy()
        measurement = robot.step(speed, turn, dt)
        ekf.predict(dt)
        ekf.update(measurement)
        odometry = integrate(odometry, measurement[0], measurement[1], dt)
        elapsed = (step + 1) * dt
        errors.append(float(np.linalg.norm(robot.pose[:2] - ekf.state[:2])))
        odom_errors.append(float(np.linalg.norm(robot.pose[:2] - odometry[:2])))
        heading_errors.append(wrap(robot.pose[2] - ekf.state[2]))
        if trace:
            history.append([elapsed, *robot.pose, *ekf.state[:3], *odometry,
                            errors[-1], ekf.position_sigma, speed, turn,
                            ekf.state[5], robot.bias, ekf.slip_score, controller.recoveries])
        if not world.segment_free(previous, robot.pose, robot.radius):
            outcome = "collision"
            break
    if outcome == "timeout" and np.linalg.norm(ekf.state[:2] - world.goal) < controller.goal_tolerance:
        outcome = "success" if np.linalg.norm(robot.pose[:2] - world.goal) < 0.35 else "missed_goal"

    def rmse(values):
        return float(np.sqrt(np.mean(np.square(values)))) if values else 0.0

    result = Result(seed, scenario, policy, outcome, outcome == "success", int(outcome == "collision"),
                    rmse(errors), rmse(odom_errors), rmse(heading_errors),
                    float(np.linalg.norm(robot.pose[:2] - ekf.state[:2])),
                    float(np.linalg.norm(robot.pose[:2] - world.goal)), controller.recoveries,
                    elapsed, elapsed if outcome == "success" else None)
    return result, np.array(history), initial_path


TRACE_COLUMNS = ["time_s", "true_x", "true_y", "true_yaw", "ekf_x", "ekf_y", "ekf_yaw",
                 "odom_x", "odom_y", "odom_yaw", "position_error_m", "position_sigma_m",
                 "command_speed_mps", "command_yaw_rate_radps", "estimated_bias_radps",
                 "true_bias_radps", "slip_score", "recoveries"]


def summarize(results):
    groups = {}
    for result in results:
        groups.setdefault((result.scenario, result.policy), []).append(result)
    summary = []
    for (scenario, policy), trials in groups.items():
        n = len(trials)
        success_count = sum(r.success for r in trials)
        p, z = success_count / n, 1.96
        center = (p + z*z / (2*n)) / (1 + z*z/n)
        half = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / (1 + z*z/n)
        times = [r.time_to_goal_s for r in trials if r.success]
        summary.append({
            "scenario": scenario, "policy": policy, "trials": n,
            "successes": success_count, "success_rate": p,
            "success_ci95": [center - half, center + half],
            "collisions": sum(r.collisions for r in trials),
            "mean_position_rmse_m": float(np.mean([r.position_rmse_m for r in trials])),
            "mean_odometry_rmse_m": float(np.mean([r.odometry_rmse_m for r in trials])),
            "mean_recoveries": float(np.mean([r.recoveries for r in trials])),
            "mean_elapsed_s": float(np.mean([r.elapsed_s for r in trials])),
            "mean_goal_error_m": float(np.mean([r.goal_error_m for r in trials])),
            "mean_time_to_goal_s": float(np.mean(times)) if times else None,
            "outcomes": {name: sum(r.outcome == name for r in trials)
                         for name in sorted({r.outcome for r in trials})},
        })
    return summary
