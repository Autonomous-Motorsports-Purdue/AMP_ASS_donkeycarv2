#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from parts.simple_gps_pid import SimpleGPSPIDController, normalize_angle


@dataclass
class SimplePIDSimConfig:
    dt: float
    gps_rate_hz: float
    laps: int
    gps_noise_m: float
    yaw_noise_deg: float
    lookahead_m: float
    throttle_cmd: float
    throttle_to_speed_mps: float
    speed_response_s: float
    wheelbase_m: float
    max_steer_deg: float
    kp: float
    ki: float
    kd: float
    seed: int


def run_simple_pid_sim(track_csv: Path, config: SimplePIDSimConfig):
    controller = SimpleGPSPIDController(
        path_csv=track_csv,
        lookahead_m=config.lookahead_m,
        throttle=config.throttle_cmd,
        kp=config.kp,
        ki=config.ki,
        kd=config.kd,
        max_steer_deg=config.max_steer_deg,
        fixed_dt=config.dt,
        verbose=False,
    )

    true_x_m = float(controller.path_x_m[0])
    true_y_m = float(controller.path_y_m[0])
    true_yaw_rad = float(controller.path_heading_rad[0])
    true_speed_mps = config.throttle_cmd * config.throttle_to_speed_mps

    latest_gps_x_m = true_x_m
    latest_gps_y_m = true_y_m
    latest_gps_lat, latest_gps_lon = controller.local_xy_to_latlon(latest_gps_x_m, latest_gps_y_m)

    rng = np.random.default_rng(config.seed)
    gps_period_s = 1.0 / max(config.gps_rate_hz, 1e-6)
    next_gps_t_s = 0.0

    logs = {
        "time_s": [],
        "true_x_m": [],
        "true_y_m": [],
        "gps_x_m": [],
        "gps_y_m": [],
        "true_yaw_rad": [],
        "measured_yaw_rad": [],
        "target_x_m": [],
        "target_y_m": [],
        "heading_error_rad": [],
        "steering_cmd": [],
        "throttle_cmd": [],
        "true_speed_mps": [],
        "cross_track_error_m": [],
        "progress_m": [],
    }

    completed_laps = 0
    previous_idx = 0
    num_steps = int(math.ceil((controller.path_length_m / max(true_speed_mps, 0.1)) * config.laps * 1.6 / config.dt))

    for step in range(num_steps):
        t_s = step * config.dt

        if t_s + 1e-9 >= next_gps_t_s:
            latest_gps_x_m = true_x_m + rng.normal(0.0, config.gps_noise_m)
            latest_gps_y_m = true_y_m + rng.normal(0.0, config.gps_noise_m)
            latest_gps_lat, latest_gps_lon = controller.local_xy_to_latlon(latest_gps_x_m, latest_gps_y_m)
            next_gps_t_s += gps_period_s

        measured_yaw_rad = normalize_angle(true_yaw_rad + rng.normal(0.0, math.radians(config.yaw_noise_deg)))
        steering_cmd, throttle_cmd = controller.run(latest_gps_lat, latest_gps_lon, measured_yaw_rad)
        debug = controller.last_debug
        if debug is None:
            raise RuntimeError("Controller returned no debug data during pseudo-sim.")

        target_speed_mps = throttle_cmd * config.throttle_to_speed_mps
        speed_alpha = min(1.0, config.dt / max(config.speed_response_s, config.dt))
        true_speed_mps += (target_speed_mps - true_speed_mps) * speed_alpha

        steering_angle_rad = float(np.clip(steering_cmd, -1.0, 1.0)) * math.radians(config.max_steer_deg)
        yaw_rate_radps = true_speed_mps / config.wheelbase_m * math.tan(steering_angle_rad)

        true_x_m += true_speed_mps * math.cos(true_yaw_rad) * config.dt
        true_y_m += true_speed_mps * math.sin(true_yaw_rad) * config.dt
        true_yaw_rad = normalize_angle(true_yaw_rad + yaw_rate_radps * config.dt)

        closest_idx = controller._find_closest_idx(true_x_m, true_y_m)
        if previous_idx > int(0.9 * len(controller.path_x_m)) and closest_idx < int(0.1 * len(controller.path_x_m)):
            completed_laps += 1
        previous_idx = closest_idx

        cross_track_error_m = float(
            np.hypot(
                controller.path_x_m[closest_idx] - true_x_m,
                controller.path_y_m[closest_idx] - true_y_m,
            )
        )
        progress_m = completed_laps * controller.path_length_m + float(controller.path_s_m[closest_idx])

        logs["time_s"].append(t_s)
        logs["true_x_m"].append(true_x_m)
        logs["true_y_m"].append(true_y_m)
        logs["gps_x_m"].append(latest_gps_x_m)
        logs["gps_y_m"].append(latest_gps_y_m)
        logs["true_yaw_rad"].append(true_yaw_rad)
        logs["measured_yaw_rad"].append(measured_yaw_rad)
        logs["target_x_m"].append(float(debug["target_x_m"]))
        logs["target_y_m"].append(float(debug["target_y_m"]))
        logs["heading_error_rad"].append(float(debug["heading_error_rad"]))
        logs["steering_cmd"].append(float(steering_cmd))
        logs["throttle_cmd"].append(float(throttle_cmd))
        logs["true_speed_mps"].append(float(true_speed_mps))
        logs["cross_track_error_m"].append(cross_track_error_m)
        logs["progress_m"].append(progress_m)

        if completed_laps >= config.laps:
            break

    controller.shutdown()

    log_arrays = {key: np.asarray(values, dtype=float) for key, values in logs.items()}
    summary = {
        "completed_laps": float(completed_laps),
        "requested_laps": float(config.laps),
        "sim_time_s": float(log_arrays["time_s"][-1]) if len(log_arrays["time_s"]) else 0.0,
        "mean_cross_track_error_m": float(np.mean(log_arrays["cross_track_error_m"])),
        "p95_cross_track_error_m": float(np.percentile(log_arrays["cross_track_error_m"], 95.0)),
        "max_cross_track_error_m": float(np.max(log_arrays["cross_track_error_m"])),
        "steering_limit_fraction": float(np.mean(np.abs(log_arrays["steering_cmd"]) >= 0.95)),
        "avg_speed_mps": float(np.mean(log_arrays["true_speed_mps"])),
    }
    return controller, log_arrays, summary


def save_logs(logs, output_csv: Path):
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(logs.keys()))
        for row in zip(*[logs[key] for key in logs]):
            writer.writerow(row)


def save_plot(controller, logs, output_png: Path):
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    axes[0, 0].plot(controller.path_x_m, controller.path_y_m, "k--", linewidth=1.5, label="Reference Path")
    axes[0, 0].plot(logs["true_x_m"], logs["true_y_m"], color="tab:blue", label="True Vehicle")
    axes[0, 0].scatter(logs["gps_x_m"], logs["gps_y_m"], s=10, alpha=0.35, color="tab:green", label="GPS")
    axes[0, 0].set_title("Track View")
    axes[0, 0].set_xlabel("X East (m)")
    axes[0, 0].set_ylabel("Y North (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(logs["time_s"], logs["steering_cmd"], label="Steering")
    axes[0, 1].plot(logs["time_s"], logs["throttle_cmd"], label="Throttle")
    axes[0, 1].set_title("Controller Outputs")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Normalized Command")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(logs["time_s"], logs["cross_track_error_m"], label="Cross-Track Error")
    axes[1, 0].set_title("Tracking Error")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Error (m)")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(logs["time_s"], logs["heading_error_rad"], label="Heading Error")
    axes[1, 1].plot(logs["time_s"], logs["true_speed_mps"], label="Speed")
    axes[1, 1].set_title("Heading Error And Speed")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)


def print_summary(track_csv: Path, summary, output_csv: Path, output_png: Path):
    print(f"Track: {track_csv}")
    print(
        "Lap completion: "
        f"{summary['completed_laps']:.0f}/{summary['requested_laps']:.0f} | "
        f"sim_time={summary['sim_time_s']:.1f} s | "
        f"avg_speed={summary['avg_speed_mps']:.2f} m/s"
    )
    print(
        "Tracking: "
        f"mean_cte={summary['mean_cross_track_error_m']:.2f} m | "
        f"p95_cte={summary['p95_cross_track_error_m']:.2f} m | "
        f"max_cte={summary['max_cross_track_error_m']:.2f} m"
    )
    print(f"Actuator saturation: steering={100.0 * summary['steering_limit_fraction']:.1f}%")

    warnings = []
    if summary["completed_laps"] < summary["requested_laps"]:
        warnings.append("vehicle did not complete the requested laps")
    if summary["mean_cross_track_error_m"] > 2.0:
        warnings.append("mean cross-track error is above 2 m")
    if summary["steering_limit_fraction"] > 0.2:
        warnings.append("steering is saturated more than 20% of the run")

    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"  - {warning}")
    else:
        print("Warnings: none")

    print(f"Saved log CSV: {output_csv}")
    print(f"Saved plot PNG: {output_png}")


def parse_args():
    parser = argparse.ArgumentParser(description="Pseudo-sim for the simplest GPS+IMU+PID path follower.")
    parser.add_argument("track_csv", help="GPS track CSV with latitude/longitude columns")
    parser.add_argument("--lookahead-m", type=float, default=6.0, help="distance ahead of the closest path point")
    parser.add_argument("--throttle", type=float, default=0.22, help="constant throttle command")
    parser.add_argument("--kp", type=float, default=1.0, help="heading error proportional gain")
    parser.add_argument("--ki", type=float, default=0.0, help="heading error integral gain")
    parser.add_argument("--kd", type=float, default=0.15, help="heading error derivative gain")
    parser.add_argument("--gps-rate-hz", type=float, default=4.0, help="GPS update rate")
    parser.add_argument("--gps-noise-m", type=float, default=0.5, help="GPS position noise in meters")
    parser.add_argument("--yaw-noise-deg", type=float, default=2.0, help="yaw noise in degrees")
    parser.add_argument("--dt", type=float, default=0.05, help="simulation timestep")
    parser.add_argument("--laps", type=int, default=1, help="number of laps to simulate")
    parser.add_argument("--throttle-to-speed-mps", type=float, default=20.0, help="maps throttle command to simulated m/s")
    parser.add_argument("--speed-response-s", type=float, default=0.5, help="first-order speed response")
    parser.add_argument("--wheelbase-m", type=float, default=1.000506, help="vehicle wheelbase")
    parser.add_argument("--max-steer-deg", type=float, default=35.0, help="maximum steering angle")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--save-dir", default="sim_outputs", help="directory for CSV and PNG outputs")
    return parser.parse_args()


def main():
    args = parse_args()
    track_csv = Path(args.track_csv).expanduser().resolve()
    config = SimplePIDSimConfig(
        dt=args.dt,
        gps_rate_hz=args.gps_rate_hz,
        laps=args.laps,
        gps_noise_m=args.gps_noise_m,
        yaw_noise_deg=args.yaw_noise_deg,
        lookahead_m=args.lookahead_m,
        throttle_cmd=args.throttle,
        throttle_to_speed_mps=args.throttle_to_speed_mps,
        speed_response_s=args.speed_response_s,
        wheelbase_m=args.wheelbase_m,
        max_steer_deg=args.max_steer_deg,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        seed=args.seed,
    )

    controller, logs, summary = run_simple_pid_sim(track_csv, config)

    save_dir = Path(args.save_dir).expanduser().resolve()
    output_stem = f"{track_csv.stem}_gps_pid_pseudo_sim"
    output_csv = save_dir / f"{output_stem}.csv"
    output_png = save_dir / f"{output_stem}.png"
    save_logs(logs, output_csv)
    save_plot(controller, logs, output_png)
    print_summary(track_csv, summary, output_csv, output_png)


if __name__ == "__main__":
    main()
