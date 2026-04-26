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

from parts.gps_pure_pursuit_waypoint import GPSPurePursuitWaypointGenerator
from parts.pure_pursuit import Pure_Pursuit


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class PurePursuitSimConfig:
    dt: float
    laps: int
    gps_noise_m: float
    lookahead_m: float
    throttle_cmd: float
    throttle_to_speed_mps: float
    speed_response_s: float
    seed: int


def run_pure_pursuit_sim(track_csv: Path, config: PurePursuitSimConfig):
    waypoint_generator = GPSPurePursuitWaypointGenerator(
        path_csv=track_csv,
        lookahead_m=config.lookahead_m,
        verbose=False,
    )
    pure_pursuit = Pure_Pursuit(config.throttle_cmd, verbose=False)

    true_x = float(waypoint_generator.path_x[0])
    true_y = float(waypoint_generator.path_y[0])
    true_yaw = float(waypoint_generator.path_heading[0])
    true_speed = config.throttle_cmd * config.throttle_to_speed_mps
    wheelbase = pure_pursuit.wheelbase

    rng = np.random.default_rng(config.seed)

    logs = {
        "time_s": [],
        "true_x_m": [],
        "true_y_m": [],
        "gps_x_m": [],
        "gps_y_m": [],
        "waypoint_x_m": [],
        "waypoint_y_m": [],
        "true_yaw_rad": [],
        "steering_cmd": [],
        "throttle_cmd": [],
        "true_speed_mps": [],
        "cross_track_error_m": [],
        "progress_m": [],
    }

    completed_laps = 0
    previous_idx = 0
    lap_length_m = float(waypoint_generator.path_s[-1])
    max_time_s = max(waypoint_generator.lap_length_m / max(true_speed, 0.1) * config.laps * 1.5, config.dt * 10.0)
    num_steps = int(math.ceil(max_time_s / config.dt))

    for step in range(num_steps):
        t_s = step * config.dt

        gps_x = true_x + rng.normal(0.0, config.gps_noise_m)
        gps_y = true_y + rng.normal(0.0, config.gps_noise_m)
        gps_lat, gps_lon = waypoint_generator.local_xy_to_latlon(gps_x, gps_y)
        target = waypoint_generator.compute_target_from_latlon(gps_lat, gps_lon)
        if target is None:
            raise RuntimeError("Waypoint generator returned no target in pseudo-sim.")

        steering_cmd, throttle_cmd = pure_pursuit.run(target["waypoint"])

        target_speed = throttle_cmd * config.throttle_to_speed_mps
        speed_alpha = min(1.0, config.dt / max(config.speed_response_s, config.dt))
        true_speed += (target_speed - true_speed) * speed_alpha

        steering_deg = steering_cmd * Pure_Pursuit.STEERING_DEG_PER_UNIT
        steering_rad = math.radians(steering_deg)
        yaw_rate = true_speed / wheelbase * math.tan(steering_rad)

        true_x += true_speed * math.cos(true_yaw) * config.dt
        true_y += true_speed * math.sin(true_yaw) * config.dt
        true_yaw = normalize_angle(true_yaw + yaw_rate * config.dt)

        closest_idx = waypoint_generator._find_closest_idx(true_x, true_y)
        if previous_idx > int(0.9 * len(waypoint_generator.path_x)) and closest_idx < int(0.1 * len(waypoint_generator.path_x)):
            completed_laps += 1
        previous_idx = closest_idx

        cross_track_error = float(
            np.hypot(
                waypoint_generator.path_x[closest_idx] - true_x,
                waypoint_generator.path_y[closest_idx] - true_y,
            )
        )
        progress_m = completed_laps * lap_length_m + float(waypoint_generator.path_s[closest_idx])

        logs["time_s"].append(t_s)
        logs["true_x_m"].append(true_x)
        logs["true_y_m"].append(true_y)
        logs["gps_x_m"].append(gps_x)
        logs["gps_y_m"].append(gps_y)
        logs["waypoint_x_m"].append(float(target["target_x_m"]))
        logs["waypoint_y_m"].append(float(target["target_y_m"]))
        logs["true_yaw_rad"].append(true_yaw)
        logs["steering_cmd"].append(float(steering_cmd))
        logs["throttle_cmd"].append(float(throttle_cmd))
        logs["true_speed_mps"].append(float(true_speed))
        logs["cross_track_error_m"].append(cross_track_error)
        logs["progress_m"].append(progress_m)

        if completed_laps >= config.laps:
            break

    pure_pursuit.shutdown()

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
    return waypoint_generator, log_arrays, summary


def save_logs(logs, output_csv: Path):
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(logs.keys()))
        for row in zip(*[logs[key] for key in logs]):
            writer.writerow(row)


def save_plot(waypoint_generator, logs, output_png: Path):
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    axes[0, 0].plot(waypoint_generator.path_x, waypoint_generator.path_y, "k--", linewidth=1.5, label="Reference Path")
    axes[0, 0].plot(logs["true_x_m"], logs["true_y_m"], color="tab:blue", label="True Vehicle")
    axes[0, 0].scatter(logs["gps_x_m"], logs["gps_y_m"], s=8, alpha=0.4, color="tab:green", label="GPS")
    axes[0, 0].set_title("Track View")
    axes[0, 0].set_xlabel("X East (m)")
    axes[0, 0].set_ylabel("Y North (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(logs["time_s"], logs["steering_cmd"], label="Steering Command")
    axes[0, 1].plot(logs["time_s"], logs["throttle_cmd"], label="Throttle Command")
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

    axes[1, 1].plot(logs["time_s"], logs["true_speed_mps"], label="True Speed")
    axes[1, 1].set_title("Speed")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("m/s")
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
    print(
        "Actuator saturation: "
        f"steering={100.0 * summary['steering_limit_fraction']:.1f}%"
    )
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
    parser = argparse.ArgumentParser(description="Pseudo-sim for the simple GPS pure pursuit stack.")
    parser.add_argument("track_csv", help="GPS track CSV with latitude/longitude columns")
    parser.add_argument("--lookahead-m", type=float, default=6.0, help="lookahead distance along path")
    parser.add_argument("--throttle", type=float, default=0.35, help="pure pursuit throttle command")
    parser.add_argument("--throttle-to-speed-mps", type=float, default=20.0, help="maps throttle command to simulated m/s")
    parser.add_argument("--speed-response-s", type=float, default=0.5, help="first-order speed response")
    parser.add_argument("--gps-noise-m", type=float, default=0.5, help="GPS noise in meters")
    parser.add_argument("--dt", type=float, default=0.1, help="simulation timestep")
    parser.add_argument("--laps", type=int, default=1, help="number of laps to simulate")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--save-dir", default="sim_outputs", help="directory for CSV and PNG outputs")
    return parser.parse_args()


def main():
    args = parse_args()
    track_csv = Path(args.track_csv).expanduser().resolve()
    config = PurePursuitSimConfig(
        dt=args.dt,
        laps=args.laps,
        gps_noise_m=args.gps_noise_m,
        lookahead_m=args.lookahead_m,
        throttle_cmd=args.throttle,
        throttle_to_speed_mps=args.throttle_to_speed_mps,
        speed_response_s=args.speed_response_s,
        seed=args.seed,
    )
    waypoint_generator, logs, summary = run_pure_pursuit_sim(track_csv, config)

    save_dir = Path(args.save_dir).expanduser().resolve()
    output_stem = f"{track_csv.stem}_gps_pure_pursuit_pseudo_sim"
    output_csv = save_dir / f"{output_stem}.csv"
    output_png = save_dir / f"{output_stem}.png"
    save_logs(logs, output_csv)
    save_plot(waypoint_generator, logs, output_png)
    print_summary(track_csv, summary, output_csv, output_png)


if __name__ == "__main__":
    main()
