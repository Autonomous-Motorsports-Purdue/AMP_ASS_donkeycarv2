#!/usr/bin/env python3
import argparse
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


GPS_RE = re.compile(r"GPS_to_xy:.*?x\s*(-?\d+(?:\.\d+)?)\s*m,\s*y\s*(-?\d+(?:\.\d+)?)\s*m")
MPC_RE = re.compile(r"\[MPC_Part\]\s*dx=(-?\d+(?:\.\d+)?),\s*dy=(-?\d+(?:\.\d+)?)")
IMU_RE = re.compile(
    r"Returning IMU values:\s*yaw_rate=(-?\d+(?:\.\d+)?),\s*yaw=(-?\d+(?:\.\d+)?)"
)


def wrap_deg(values):
    return (values + 180.0) % 360.0 - 180.0


def parse_log(path: Path):
    latest_xy = None
    latest_imu = None
    rows = []

    for line in path.read_text(errors="replace").splitlines():
        m_imu = IMU_RE.search(line)
        if m_imu:
            latest_imu = (float(m_imu.group(1)), float(m_imu.group(2)))

        m_gps = GPS_RE.search(line)
        if m_gps:
            latest_xy = (float(m_gps.group(1)), float(m_gps.group(2)))

        m_mpc = MPC_RE.search(line)
        if m_mpc and latest_xy is not None and latest_imu is not None:
            dx = float(m_mpc.group(1))
            dy = float(m_mpc.group(2))
            x, y = latest_xy
            yaw_rate, yaw_imu = latest_imu
            rows.append((x, y, x + dx, y + dy, yaw_imu, yaw_rate))

    if not rows:
        raise ValueError("No usable samples found. Expected GPS_to_xy, IMU, and [MPC_Part] lines.")

    arr = np.array(rows, dtype=float)
    x, y, dx_abs, dy_abs, yaw_imu_deg, yaw_rate = arr.T

    # GPS heading estimated from trajectory direction (consecutive XY points).
    yaw_gps_deg = np.full_like(yaw_imu_deg, np.nan)
    if len(x) >= 2:
        vx = np.diff(x)
        vy = np.diff(y)
        yaw_step = np.degrees(np.arctan2(vy, vx))
        yaw_gps_deg[1:] = yaw_step
        yaw_gps_deg[0] = yaw_step[0]
        valid = np.isfinite(yaw_gps_deg)
        yaw_gps_deg[valid] = np.degrees(np.unwrap(np.radians(yaw_gps_deg[valid])))

    yaw_desired_deg = np.degrees(np.arctan2(dy_abs - y, dx_abs - x))
    return {
        "x": x,
        "y": y,
        "dx_abs": dx_abs,
        "dy_abs": dy_abs,
        "yaw_imu_deg": yaw_imu_deg,
        "yaw_gps_deg": yaw_gps_deg,
        "yaw_desired_deg": yaw_desired_deg,
        "yaw_rate": yaw_rate,
    }


def build_animation(data, fps=20, step=1):
    x = data["x"]
    y = data["y"]
    dx_abs = data["dx_abs"]
    dy_abs = data["dy_abs"]
    yaw_imu = wrap_deg(data["yaw_imu_deg"])
    yaw_gps = wrap_deg(data["yaw_gps_deg"])
    yaw_des = wrap_deg(data["yaw_desired_deg"])
    yaw_rate = data["yaw_rate"]

    n = len(x)
    frame_ids = np.arange(0, n, max(1, step))

    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.4, 1.0])
    ax_traj = fig.add_subplot(gs[:, 0])
    ax_yaw = fig.add_subplot(gs[0, 1])
    ax_rate = fig.add_subplot(gs[1, 1])

    ax_traj.set_title("Trajectory: actual (x,y) vs desired (x+dx, y+dy)")
    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_aspect("equal", adjustable="box")

    all_x = np.concatenate([x, dx_abs])
    all_y = np.concatenate([y, dy_abs])
    x_pad = max(1.0, 0.1 * (all_x.max() - all_x.min() + 1e-6))
    y_pad = max(1.0, 0.1 * (all_y.max() - all_y.min() + 1e-6))
    ax_traj.set_xlim(all_x.min() - x_pad, all_x.max() + x_pad)
    ax_traj.set_ylim(all_y.min() - y_pad, all_y.max() + y_pad)

    (line_actual,) = ax_traj.plot([], [], color="#00aaee", lw=2, label="actual")
    (line_desired,) = ax_traj.plot([], [], color="#ff8800", lw=2, label="desired")
    (dot_actual,) = ax_traj.plot([], [], "o", color="#006699", ms=8)
    (dot_desired,) = ax_traj.plot([], [], "o", color="#cc6600", ms=8)
    (imu_heading_line,) = ax_traj.plot([], [], color="#44dd44", lw=2, label="IMU yaw")
    (gps_heading_line,) = ax_traj.plot([], [], color="#aa44ff", lw=2, label="GPS yaw")
    ax_traj.legend(loc="best")

    ax_yaw.set_title("Yaw comparison")
    ax_yaw.set_ylabel("yaw (deg)")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.set_xlim(0, n - 1)
    ax_yaw.set_ylim(-190, 190)
    (line_yaw_imu,) = ax_yaw.plot([], [], color="#44dd44", lw=1.8, label="IMU yaw")
    (line_yaw_gps,) = ax_yaw.plot([], [], color="#aa44ff", lw=1.8, label="GPS yaw (from XY)")
    (line_yaw_des,) = ax_yaw.plot([], [], color="#ff8800", lw=1.4, label="desired yaw")
    cursor_yaw = ax_yaw.axvline(0, color="#999999", lw=1)
    ax_yaw.legend(loc="best", fontsize=8)

    ax_rate.set_title("Yaw rate")
    ax_rate.set_xlabel("sample")
    ax_rate.set_ylabel("yaw_rate (rad/s)")
    ax_rate.grid(True, alpha=0.3)
    ax_rate.set_xlim(0, n - 1)
    rate_min = min(-0.2, float(np.nanmin(yaw_rate)) - 0.02)
    rate_max = max(0.2, float(np.nanmax(yaw_rate)) + 0.02)
    ax_rate.set_ylim(rate_min, rate_max)
    (line_rate,) = ax_rate.plot([], [], color="#ff3355", lw=1.8)
    cursor_rate = ax_rate.axvline(0, color="#999999", lw=1)

    heading_len = max(0.6, 0.07 * max(all_x.max() - all_x.min(), all_y.max() - all_y.min()))

    def update(frame_idx):
        i = int(frame_idx)

        line_actual.set_data(x[: i + 1], y[: i + 1])
        line_desired.set_data(dx_abs[: i + 1], dy_abs[: i + 1])
        dot_actual.set_data([x[i]], [y[i]])
        dot_desired.set_data([dx_abs[i]], [dy_abs[i]])

        imu_theta = math.radians(yaw_imu[i])
        imu_heading_line.set_data(
            [x[i], x[i] + heading_len * math.cos(imu_theta)],
            [y[i], y[i] + heading_len * math.sin(imu_theta)],
        )

        if np.isfinite(yaw_gps[i]):
            gps_theta = math.radians(yaw_gps[i])
            gps_heading_line.set_data(
                [x[i], x[i] + heading_len * math.cos(gps_theta)],
                [y[i], y[i] + heading_len * math.sin(gps_theta)],
            )
        else:
            gps_heading_line.set_data([], [])

        idx = np.arange(i + 1)
        line_yaw_imu.set_data(idx, yaw_imu[: i + 1])
        line_yaw_gps.set_data(idx, yaw_gps[: i + 1])
        line_yaw_des.set_data(idx, yaw_des[: i + 1])
        line_rate.set_data(idx, yaw_rate[: i + 1])
        cursor_yaw.set_xdata([i, i])
        cursor_rate.set_xdata([i, i])

        ax_traj.set_title(
            f"Trajectory frame {i + 1}/{n} | x={x[i]:.2f}, y={y[i]:.2f}, yaw_rate={yaw_rate[i]:.3f}"
        )
        return (
            line_actual,
            line_desired,
            dot_actual,
            dot_desired,
            imu_heading_line,
            gps_heading_line,
            line_yaw_imu,
            line_yaw_gps,
            line_yaw_des,
            line_rate,
            cursor_yaw,
            cursor_rate,
        )

    anim = FuncAnimation(fig, update, frames=frame_ids, interval=1000.0 / fps, blit=False)
    fig.tight_layout()
    return fig, anim


def save_animation(anim, output_path: Path, fps=20):
    suffix = output_path.suffix.lower()
    if suffix == ".gif":
        from matplotlib.animation import PillowWriter

        anim.save(str(output_path), writer=PillowWriter(fps=fps))
        return output_path

    try:
        from matplotlib.animation import FFMpegWriter

        anim.save(str(output_path), writer=FFMpegWriter(fps=fps))
        return output_path
    except Exception:
        from matplotlib.animation import PillowWriter

        fallback = output_path.with_suffix(".gif")
        anim.save(str(fallback), writer=PillowWriter(fps=fps))
        return fallback


def main():
    parser = argparse.ArgumentParser(description="Visualize trajectory/yaw/yaw_rate from out*.txt logs.")
    parser.add_argument("input", help="Path to outN.txt log file")
    parser.add_argument("-o", "--output", help="Output file (.mp4 or .gif). Default: <input>_viz.mp4")
    parser.add_argument("--fps", type=int, default=20, help="Output frame rate (default: 20)")
    parser.add_argument("--step", type=int, default=1, help="Frame decimation step (default: 1)")
    parser.add_argument("--show", action="store_true", help="Display interactive window in addition to saving")
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        raise FileNotFoundError(f"Input file not found: {in_path}")

    out_path = Path(args.output) if args.output else in_path.with_name(f"{in_path.stem}_viz.mp4")

    data = parse_log(in_path)
    fig, anim = build_animation(data, fps=args.fps, step=args.step)
    saved = save_animation(anim, out_path, fps=args.fps)
    print(f"Saved visualization: {saved}")
    print(f"Parsed samples: {len(data['x'])}")

    if args.show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
