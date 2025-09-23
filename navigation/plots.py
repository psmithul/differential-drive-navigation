from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def plot_run(world, history, path, result, output):
    if history.size == 0:
        raise ValueError("Cannot plot an empty simulation")
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    ax = axes[0, 0]
    for x0, y0, x1, y1 in world.obstacles:
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, color="0.25"))
    path = np.array(path)
    ax.plot(path[:, 0], path[:, 1], "--", color="0.65", label="Initial A* route")
    ax.plot(history[:, 1], history[:, 2], color="#147d64", label="True path")
    ax.plot(history[:, 4], history[:, 5], color="#cc651b", label="EKF")
    ax.plot(history[:, 7], history[:, 8], color="#688db1", alpha=0.7, label="Encoders only")
    ax.scatter(*world.start[:2], marker="o", color="black", s=35)
    ax.scatter(*world.goal, marker="*", color="black", s=100)
    ax.set(xlim=(0, world.width), ylim=(0, world.height), aspect="equal", xlabel="x (m)", ylabel="y (m)")
    ax.legend(fontsize=8)
    time = history[:, 0]
    axes[0, 1].plot(time, history[:, 10], label="Position error")
    axes[0, 1].plot(time, 2 * history[:, 11], "--", label="2 × largest position sigma")
    axes[0, 1].set(xlabel="Time (s)", ylabel="Distance (m)")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].plot(time, history[:, 12], color="#147d64")
    axes[1, 0].set(xlabel="Time (s)", ylabel="Commanded speed (m/s)")
    changes = np.flatnonzero(np.diff(history[:, 17], prepend=0) > 0)
    for i in changes:
        axes[1, 0].axvline(time[i], color="#cc651b", alpha=0.5)
    axes[1, 1].plot(time, history[:, 15], label="True gyro bias")
    axes[1, 1].plot(time, history[:, 14], label="Estimated gyro bias")
    axes[1, 1].set(xlabel="Time (s)", ylabel="Bias (rad/s)")
    axes[1, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=0.15)
    fig.suptitle(f"{result.scenario} · {result.policy} · seed {result.seed} · {result.outcome}")
    fig.savefig(Path(output), dpi=160)
    plt.close(fig)


def plot_summary(summary, output):
    scenarios = list(dict.fromkeys(r["scenario"] for r in summary))
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for policy, offset, color in (("fixed", -0.18, "#688db1"), ("adaptive", 0.18, "#147d64")):
        rows = [next(r for r in summary if r["scenario"] == s and r["policy"] == policy) for s in scenarios]
        x = np.arange(len(rows)) + offset
        rates = np.array([r["success_rate"] for r in rows])
        intervals = np.array([r["success_ci95"] for r in rows]).T
        axes[0].bar(x, rates, width=0.34, color=color, label=policy)
        axes[0].errorbar(x, rates, yerr=np.maximum(0, np.vstack((rates-intervals[0], intervals[1]-rates))),
                        fmt="none", color="black", capsize=3, linewidth=1)
        axes[1].bar(x, [r["collisions"]/r["trials"] for r in rows], width=0.34, color=color)
        axes[2].bar(x, [r["mean_position_rmse_m"] for r in rows], width=0.34, color=color)
    for ax, label in zip(axes, ["Success rate (95% Wilson interval)", "Collision rate", "Mean position RMSE (m)"]):
        ax.set_xticks(range(len(scenarios)), scenarios)
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.15)
    axes[0].set_ylim(0, 1.08)
    axes[0].legend()
    fig.savefig(output, dpi=160)
    plt.close(fig)
