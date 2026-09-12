#!/usr/bin/env python3
"""Build report figures from ROS 2 CSV topic recordings."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


ROOT = Path(__file__).resolve().parent
IMAGES = ROOT / "images"


def read_rows(name: str) -> list[list[str]]:
    with (ROOT / name).open(newline="", encoding="utf-8") as source:
        return [row for row in csv.reader(source) if row]


def odometry(name: str) -> tuple[list[float], list[float], list[float]]:
    rows = read_rows(name)
    time = [float(row[0]) + 1e-9 * float(row[1]) for row in rows]
    x = [float(row[4]) for row in rows]
    y = [float(row[5]) for row in rows]
    t0 = time[0]
    return [value - t0 for value in time], x, y


def slam_pose() -> tuple[list[float], list[float], list[float]]:
    rows = read_rows("slam_pose.csv")
    time = [float(row[0]) + 1e-9 * float(row[1]) for row in rows]
    x = [float(row[3]) for row in rows]
    y = [float(row[4]) for row in rows]
    t0 = time[0]
    return [value - t0 for value in time], x, y


def wheel_speeds() -> tuple[list[float], list[float], list[float]]:
    rows = read_rows("wheel_speed.csv")
    # Float64MultiArray has no timestamp; the controller publishes at 20 Hz.
    time = [0.05 * i for i in range(len(rows))]
    left = [float(row[-2]) for row in rows]
    right = [float(row[-1]) for row in rows]
    return time, left, right


def style() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def trajectory_plot() -> None:
    _, x_true, y_true = odometry("odom.csv")
    _, x_wheel, y_wheel = odometry("wheel_odom.csv")
    _, x_ekf, y_ekf = slam_pose()

    fig, ax = plt.subplots(figsize=(10.0, 4.6), constrained_layout=True)
    ax.plot(x_true, y_true, color="#222222", linewidth=2.2,
            label="Модель Gazebo (/odom)")
    ax.plot(x_wheel, y_wheel, color="#2878b5", linewidth=1.6,
            label="Колёсная одометрия (/wheel/odom)")
    ax.plot(x_ekf, y_ekf, color="#d62728", linewidth=1.8,
            label="Оценка EKF-SLAM (/slam/pose)")
    ax.axhline(0.0, color="#d4aa00", linewidth=1.2, linestyle="--",
               label="Исходная прямая y = 0")

    obstacle = Rectangle((3.65, -0.475), 0.70, 0.95,
                         facecolor="#c43c39", edgecolor="#7f1d1d",
                         alpha=0.55, label="Препятствие")
    ax.add_patch(obstacle)
    landmarks = [(1.45, 1.35), (2.45, -1.45), (4.15, 2.80),
                 (5.60, -1.35), (7.20, 1.40), (9.10, -1.50)]
    for index, (x, y) in enumerate(landmarks, start=1):
        ax.add_patch(Circle((x, y), 0.12, facecolor="#2b6cb0",
                            edgecolor="#174a7e", alpha=0.85))
        ax.annotate(f"L{index}", (x, y), xytext=(4, 5),
                    textcoords="offset points", fontsize=8)

    ax.set_xlabel("x, м")
    ax.set_ylabel("y, м")
    ax.set_xlim(0.0, 11.0)
    ax.set_ylim(-3.1, 3.2)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=3,
              frameon=False, fontsize=8.5)
    fig.savefig(IMAGES / "trajectory.png", dpi=240)
    plt.close(fig)


def wheel_speed_plot() -> None:
    time, left, right = wheel_speeds()
    fig, ax = plt.subplots(figsize=(10.0, 4.0), constrained_layout=True)
    ax.plot(time, left, color="#2878b5", linewidth=1.6,
            label="Переднее левое колесо")
    ax.plot(time, right, color="#d62728", linewidth=1.6,
            label="Переднее правое колесо")
    ax.fill_between(time, left, right, where=[r > l for l, r in zip(left, right)],
                    color="#8dd3c7", alpha=0.28,
                    label="Подтормаживание левого колеса")
    ax.fill_between(time, left, right, where=[l > r for l, r in zip(left, right)],
                    color="#fdb462", alpha=0.25,
                    label="Подтормаживание правого колеса")
    ax.axhline(0.0, color="#555555", linewidth=0.8)
    ax.set_xlabel("Время записи, с")
    ax.set_ylabel("Угловая скорость, рад/с")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=2,
              frameon=False, fontsize=8.5)
    fig.savefig(IMAGES / "wheel_speeds.png", dpi=240)
    plt.close(fig)


def print_metrics() -> None:
    _, x_true, y_true = odometry("odom.csv")
    _, x_wheel, y_wheel = odometry("wheel_odom.csv")
    _, x_ekf, y_ekf = slam_pose()
    error = math.hypot(x_ekf[-1] - x_true[-1], y_ekf[-1] - y_true[-1])
    print(f"true_finish=({x_true[-1]:.3f}, {y_true[-1]:.3f})")
    print(f"wheel_finish=({x_wheel[-1]:.3f}, {y_wheel[-1]:.3f})")
    print(f"ekf_finish=({x_ekf[-1]:.3f}, {y_ekf[-1]:.3f})")
    print(f"max_true_y={max(y_true):.3f}")
    print(f"finish_position_error={error:.3f}")


if __name__ == "__main__":
    IMAGES.mkdir(parents=True, exist_ok=True)
    style()
    trajectory_plot()
    wheel_speed_plot()
    print_metrics()
