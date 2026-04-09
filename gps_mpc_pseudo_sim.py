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

from parts.controller import ClosedLoopController, MPC_Part, find_closest_point
from parts.ekf_localizer import EKFLocalizer
from parts.gps_to_xy import GPS_to_xy


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class TrackData:
    csv_path: Path
    lat: np.ndarray
    lon: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    s: np.ndarray
    vx: np.ndarray | None
    ax: np.ndarray | None
    psi_offset_rad: float
    heading_alignment_mean_rad: float
    raw_xy_alignment_mean_m: float | None
    raw_xy_alignment_max_m: float | None

    @property
    def ref_lat(self) -> float:
        return float(self.lat[0])

    @property
    def ref_lon(self) -> float:
        return float(self.lon[0])

    @property
    def lap_length_m(self) -> float:
        return float(self.s[-1])

    @property
    def xy_path(self) -> np.ndarray:
        return np.column_stack((self.x, self.y, self.psi))


@dataclass
class SimConfig:
    dt: float
    gps_rate_hz: float
    laps: int
    localization_mode: str
    wheelbase_m: float
    max_steer_deg: float
    steering_kp: float
    steering_ki: float
    steering_kd: float
    speed_to_throttle_scale: float
    max_throttle: float
    throttle_speed_lag_s: float
    steering_lag_s: float
    gps_noise_m: float
    yaw_noise_deg: float
    yaw_rate_noise_degps: float
    accel_noise_mps2: float
    initial_speed_mps: float | None
    seed: int


def load_raceline_track(csv_path: Path) -> TrackData:
    data = np.genfromtxt(
        csv_path,
        delimiter=",",
        names=True,
        comments="#",
        autostrip=True,
        dtype=float,
        encoding="utf-8",
    )

    if data.dtype.names is None:
        raise ValueError(f"Could not parse header from {csv_path}")

    required_fields = {"latitude", "longitude", "psi_rad"}
    if not required_fields.issubset(data.dtype.names):
        raise ValueError(f"{csv_path} must include {sorted(required_fields)}")

    lat = np.asarray(data["latitude"], dtype=float)
    lon = np.asarray(data["longitude"], dtype=float)
    raw_psi = np.asarray(data["psi_rad"], dtype=float)

    gps_frame = GPS_to_xy(ref_lat_deg=float(lat[0]), ref_lon_deg=float(lon[0]))
    x_rel, y_rel = gps_frame.to_xy(lat, lon)
    x_rel = np.asarray(x_rel, dtype=float)
    y_rel = np.asarray(y_rel, dtype=float)

    path_heading = np.arctan2(np.diff(y_rel), np.diff(x_rel))
    candidate_offsets = (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi)
    best_offset = min(
        candidate_offsets,
        key=lambda offset: float(
            np.mean(np.abs(np.arctan2(np.sin((raw_psi[:-1] + offset) - path_heading), np.cos((raw_psi[:-1] + offset) - path_heading))))
        ),
    )
    psi = np.arctan2(np.sin(raw_psi + best_offset), np.cos(raw_psi + best_offset))
    heading_alignment = np.arctan2(np.sin(psi[:-1] - path_heading), np.cos(psi[:-1] - path_heading))
    heading_alignment_mean_rad = float(np.mean(np.abs(heading_alignment)))

    s = np.asarray(data["s_m"], dtype=float) if "s_m" in data.dtype.names else None
    if s is None:
        segment_lengths = np.hypot(np.diff(x_rel), np.diff(y_rel))
        s = np.concatenate(([0.0], np.cumsum(segment_lengths)))

    vx = np.asarray(data["vx_mps"], dtype=float) if "vx_mps" in data.dtype.names else None
    ax = np.asarray(data["ax_mps2"], dtype=float) if "ax_mps2" in data.dtype.names else None

    raw_xy_alignment_mean_m = None
    raw_xy_alignment_max_m = None
    if {"x_m", "y_m"}.issubset(data.dtype.names):
        x_raw = np.asarray(data["x_m"], dtype=float)
        y_raw = np.asarray(data["y_m"], dtype=float)
        xy_error = np.hypot((x_raw - x_raw[0]) - x_rel, (y_raw - y_raw[0]) - y_rel)
        raw_xy_alignment_mean_m = float(np.mean(xy_error))
        raw_xy_alignment_max_m = float(np.max(xy_error))

    return TrackData(
        csv_path=csv_path,
        lat=lat,
        lon=lon,
        x=x_rel,
        y=y_rel,
        psi=psi,
        s=np.asarray(s, dtype=float),
        vx=vx,
        ax=ax,
        psi_offset_rad=float(best_offset),
        heading_alignment_mean_rad=heading_alignment_mean_rad,
        raw_xy_alignment_mean_m=raw_xy_alignment_mean_m,
        raw_xy_alignment_max_m=raw_xy_alignment_max_m,
    )


def write_xy_sidecar(track: TrackData, overwrite: bool = True) -> Path:
    xy_path = track.csv_path.with_name(f"{track.csv_path.stem}_xy.csv")
    if xy_path.exists() and not overwrite:
        return xy_path

    np.savetxt(
        xy_path,
        track.xy_path,
        delimiter=",",
        fmt="%.6f",
        header="x_m,y_m, psi_rad",
        comments="",
    )
    return xy_path


def run_pseudo_sim(track: TrackData, xy_path: Path, config: SimConfig) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    rng = np.random.default_rng(config.seed)

    mpc = MPC_Part(
        path_csv=xy_path,
        horizon=2,
        dt_mpc=config.dt,
        wheelbase=config.wheelbase_m,
        max_steer=np.radians(config.max_steer_deg),
        verbose=False,
    )
    controller = ClosedLoopController(
        kp=config.steering_kp,
        ki=config.steering_ki,
        kd=config.steering_kd,
        wheelbase=config.wheelbase_m,
        steering_scale=1.0 / config.max_steer_deg,
        max_steering_deg=config.max_steer_deg,
        speed_to_throttle_scale=config.speed_to_throttle_scale,
        max_throttle=config.max_throttle,
        verbose=False,
        fixed_dt=config.dt,
    )
    ekf = None
    if config.localization_mode == "ekf":
        ekf = EKFLocalizer(
            init_lat=track.ref_lat,
            init_lon=track.ref_lon,
            imu_rate=max(1, int(round(1.0 / config.dt))),
            gps_rate=max(1, int(round(config.gps_rate_hz))),
            heading_unit="rad",
            fixed_dt=config.dt,
        )
    gps_frame = GPS_to_xy(ref_lat_deg=track.ref_lat, ref_lon_deg=track.ref_lon)

    true_x = float(track.x[0])
    true_y = float(track.y[0])
    true_yaw = float(track.psi[0])
    true_speed = (
        float(config.initial_speed_mps)
        if config.initial_speed_mps is not None
        else float(track.vx[0] if track.vx is not None else 7.5)
    )
    steering_angle = 0.0
    yaw_rate = 0.0
    longitudinal_accel = 0.0

    yaw_noise_rad = math.radians(config.yaw_noise_deg)
    yaw_rate_noise_radps = math.radians(config.yaw_rate_noise_degps)
    gps_period_s = 1.0 / config.gps_rate_hz
    next_gps_t = 0.0

    estimated_lap_time_s = track.lap_length_m / max(true_speed, 0.1)
    max_time_s = max(estimated_lap_time_s * config.laps * 1.5, config.dt * 10.0)

    logs: dict[str, list[float]] = {
        "time_s": [],
        "true_x_m": [],
        "true_y_m": [],
        "est_x_m": [],
        "est_y_m": [],
        "true_yaw_rad": [],
        "measured_yaw_rad": [],
        "true_yaw_rate_radps": [],
        "measured_yaw_rate_radps": [],
        "desired_yaw_rate_radps": [],
        "steering_cmd": [],
        "steering_angle_deg": [],
        "throttle_cmd": [],
        "true_speed_mps": [],
        "desired_speed_mps": [],
        "cross_track_error_m": [],
        "estimation_error_m": [],
        "progress_m": [],
        "gps_lat": [],
        "gps_lon": [],
    }

    completed_laps = 0
    previous_idx = 0
    previous_speed = true_speed

    num_steps = int(math.ceil(max_time_s / config.dt))
    for step in range(num_steps):
        t_s = step * config.dt

        if t_s + 1e-9 >= next_gps_t:
            gps_noise_e = rng.normal(0.0, config.gps_noise_m)
            gps_noise_n = rng.normal(0.0, config.gps_noise_m)
            gps_lat, gps_lon = gps_frame.to_latlon(true_x + gps_noise_e, true_y + gps_noise_n)
            next_gps_t += gps_period_s
        else:
            gps_lat, gps_lon = None, None

        measured_yaw = normalize_angle(true_yaw + rng.normal(0.0, yaw_noise_rad))
        measured_yaw_rate = yaw_rate + rng.normal(0.0, yaw_rate_noise_radps)
        measured_ax_body = longitudinal_accel + rng.normal(0.0, config.accel_noise_mps2)
        measured_ay_body = (true_speed * yaw_rate) + rng.normal(0.0, config.accel_noise_mps2)

        if config.localization_mode == "ekf":
            assert ekf is not None
            est_lat, est_lon, _, _ = ekf.run(measured_ax_body, measured_ay_body, measured_yaw, gps_lat, gps_lon)
            est_x, est_y = gps_frame.run(est_lat, est_lon)
        else:
            est_x, est_y = true_x, true_y

        desired_yaw_rate, desired_speed = mpc.run(float(est_x), float(est_y), measured_yaw)
        steering_cmd, throttle_cmd = controller.run(desired_yaw_rate, measured_yaw_rate, desired_speed)

        if not np.isfinite([desired_yaw_rate, desired_speed, steering_cmd, throttle_cmd]).all():
            raise RuntimeError("Controller produced non-finite outputs during pseudo-sim.")

        desired_steering_angle = float(np.clip(steering_cmd, -1.0, 1.0) * np.radians(config.max_steer_deg))
        steering_alpha = min(1.0, config.dt / max(config.steering_lag_s, config.dt))
        steering_angle += (desired_steering_angle - steering_angle) * steering_alpha

        target_speed = throttle_cmd / max(config.speed_to_throttle_scale, 1e-6)
        speed_alpha = min(1.0, config.dt / max(config.throttle_speed_lag_s, config.dt))
        true_speed += (target_speed - true_speed) * speed_alpha
        true_speed = max(0.0, true_speed)

        yaw_rate = true_speed / config.wheelbase_m * math.tan(steering_angle)
        true_x += true_speed * math.cos(true_yaw) * config.dt
        true_y += true_speed * math.sin(true_yaw) * config.dt
        true_yaw = normalize_angle(true_yaw + yaw_rate * config.dt)

        longitudinal_accel = (true_speed - previous_speed) / config.dt
        previous_speed = true_speed

        closest_idx = int(find_closest_point(track.xy_path, true_x, true_y))
        if previous_idx > int(0.9 * len(track.x)) and closest_idx < int(0.1 * len(track.x)):
            completed_laps += 1
        previous_idx = closest_idx

        progress_m = completed_laps * track.lap_length_m + float(track.s[closest_idx])
        cross_track_error_m = float(np.hypot(track.x[closest_idx] - true_x, track.y[closest_idx] - true_y))
        estimation_error_m = float(np.hypot(est_x - true_x, est_y - true_y))

        logs["time_s"].append(t_s)
        logs["true_x_m"].append(true_x)
        logs["true_y_m"].append(true_y)
        logs["est_x_m"].append(float(est_x))
        logs["est_y_m"].append(float(est_y))
        logs["true_yaw_rad"].append(true_yaw)
        logs["measured_yaw_rad"].append(measured_yaw)
        logs["true_yaw_rate_radps"].append(yaw_rate)
        logs["measured_yaw_rate_radps"].append(measured_yaw_rate)
        logs["desired_yaw_rate_radps"].append(float(desired_yaw_rate))
        logs["steering_cmd"].append(float(steering_cmd))
        logs["steering_angle_deg"].append(float(np.degrees(steering_angle)))
        logs["throttle_cmd"].append(float(throttle_cmd))
        logs["true_speed_mps"].append(true_speed)
        logs["desired_speed_mps"].append(float(desired_speed))
        logs["cross_track_error_m"].append(cross_track_error_m)
        logs["estimation_error_m"].append(estimation_error_m)
        logs["progress_m"].append(progress_m)
        logs["gps_lat"].append(float(gps_lat) if gps_lat is not None else float("nan"))
        logs["gps_lon"].append(float(gps_lon) if gps_lon is not None else float("nan"))

        if completed_laps >= config.laps:
            break

    log_arrays = {key: np.asarray(values, dtype=float) for key, values in logs.items()}
    summary = {
        "completed_laps": float(completed_laps),
        "requested_laps": float(config.laps),
        "sim_time_s": float(log_arrays["time_s"][-1]) if len(log_arrays["time_s"]) else 0.0,
        "mean_cross_track_error_m": float(np.mean(log_arrays["cross_track_error_m"])),
        "p95_cross_track_error_m": float(np.percentile(log_arrays["cross_track_error_m"], 95.0)),
        "max_cross_track_error_m": float(np.max(log_arrays["cross_track_error_m"])),
        "mean_estimation_error_m": float(np.mean(log_arrays["estimation_error_m"])),
        "max_estimation_error_m": float(np.max(log_arrays["estimation_error_m"])),
        "steering_limit_fraction": float(np.mean(np.abs(log_arrays["steering_cmd"]) >= 0.95)),
        "throttle_limit_fraction": float(np.mean(np.abs(log_arrays["throttle_cmd"]) >= 0.99 * config.max_throttle)),
        "avg_speed_mps": float(np.mean(log_arrays["true_speed_mps"])),
    }

    mpc.shutdown()
    controller.shutdown()
    return log_arrays, summary


def save_logs(logs: dict[str, np.ndarray], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(logs.keys()))
        for row in zip(*[logs[key] for key in logs]):
            writer.writerow(row)


def save_plot(track: TrackData, logs: dict[str, np.ndarray], output_png: Path) -> None:
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    axes[0, 0].plot(track.x, track.y, "k--", linewidth=1.5, label="Reference Path")
    axes[0, 0].plot(logs["true_x_m"], logs["true_y_m"], color="tab:blue", label="True Vehicle")
    axes[0, 0].plot(logs["est_x_m"], logs["est_y_m"], color="tab:orange", alpha=0.8, label="EKF Estimate")
    axes[0, 0].set_title("Track View")
    axes[0, 0].set_xlabel("X East (m)")
    axes[0, 0].set_ylabel("Y North (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(logs["time_s"], logs["steering_cmd"], label="Steering Command")
    axes[0, 1].plot(logs["time_s"], logs["throttle_cmd"], label="Throttle Command")
    axes[0, 1].set_title("Actuator Outputs")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Normalized Command")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(logs["time_s"], logs["cross_track_error_m"], label="Cross-Track Error")
    axes[1, 0].plot(logs["time_s"], logs["estimation_error_m"], label="Estimate Error")
    axes[1, 0].set_title("Tracking Error")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Error (m)")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(logs["time_s"], logs["desired_yaw_rate_radps"], label="Desired Yaw Rate")
    axes[1, 1].plot(logs["time_s"], logs["true_yaw_rate_radps"], label="True Yaw Rate")
    axes[1, 1].set_title("Yaw Rate")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("rad/s")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)


def print_summary(track: TrackData, summary: dict[str, float], xy_path: Path, output_csv: Path, output_png: Path) -> None:
    print(f"Track: {track.csv_path}")
    print(f"Generated XY path: {xy_path}")
    print(
        "Heading alignment: "
        f"psi_offset={track.psi_offset_rad:.3f} rad | "
        f"mean_residual={track.heading_alignment_mean_rad:.3f} rad"
    )
    if track.raw_xy_alignment_mean_m is not None and track.raw_xy_alignment_max_m is not None:
        print(
            "GPS-derived XY vs raw XY shift error: "
            f"mean={track.raw_xy_alignment_mean_m:.3f} m, max={track.raw_xy_alignment_max_m:.3f} m"
        )
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
        "Estimation: "
        f"mean_err={summary['mean_estimation_error_m']:.2f} m | "
        f"max_err={summary['max_estimation_error_m']:.2f} m"
    )
    print(
        "Actuator saturation: "
        f"steering={100.0 * summary['steering_limit_fraction']:.1f}% | "
        f"throttle={100.0 * summary['throttle_limit_fraction']:.1f}%"
    )

    warnings = []
    if summary["completed_laps"] < summary["requested_laps"]:
        warnings.append("vehicle did not complete the requested laps")
    if summary["mean_cross_track_error_m"] > 2.0:
        warnings.append("mean cross-track error is above 2 m")
    if summary["steering_limit_fraction"] > 0.2:
        warnings.append("steering is saturated more than 20% of the run")
    if summary["mean_estimation_error_m"] > 2.0:
        warnings.append("EKF estimate is drifting more than 2 m on average")

    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"  - {warning}")
    else:
        print("Warnings: none")

    print(f"Saved log CSV: {output_csv}")
    print(f"Saved plot PNG: {output_png}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a pseudo-simulation of the GPS MPC pipeline on a raceline CSV.")
    parser.add_argument("track_csv", help="track CSV with latitude, longitude, and psi_rad columns")
    parser.add_argument("--laps", type=int, default=1, help="number of laps to simulate")
    parser.add_argument("--dt", type=float, default=0.1, help="simulation timestep in seconds")
    parser.add_argument("--gps-rate", type=float, default=4.0, help="GPS update rate in Hz")
    parser.add_argument("--localization-mode", choices=["ekf", "perfect"], default="ekf", help="whether MPC sees EKF state or true XY")
    parser.add_argument("--gps-noise-m", type=float, default=0.5, help="GPS position noise in meters")
    parser.add_argument("--yaw-noise-deg", type=float, default=0.5, help="yaw measurement noise in degrees")
    parser.add_argument("--yaw-rate-noise-degps", type=float, default=0.5, help="yaw-rate measurement noise in deg/s")
    parser.add_argument("--accel-noise-mps2", type=float, default=0.05, help="IMU acceleration noise in m/s^2")
    parser.add_argument("--initial-speed-mps", type=float, default=None, help="initial true speed; defaults to raceline speed or 7.5")
    parser.add_argument("--steering-lag-s", type=float, default=0.15, help="first-order steering lag")
    parser.add_argument("--throttle-speed-lag-s", type=float, default=0.75, help="first-order speed response lag")
    parser.add_argument("--speed-to-throttle-scale", type=float, default=0.05, help="same mapping used by ClosedLoopController")
    parser.add_argument("--max-throttle", type=float, default=0.40, help="throttle clamp used by ClosedLoopController")
    parser.add_argument("--wheelbase-m", type=float, default=1.000506, help="vehicle wheelbase in meters")
    parser.add_argument("--max-steer-deg", type=float, default=35.0, help="maximum steering angle in degrees")
    parser.add_argument("--kp", type=float, default=0.5, help="closed-loop steering kp")
    parser.add_argument("--ki", type=float, default=0.0, help="closed-loop steering ki")
    parser.add_argument("--kd", type=float, default=0.0, help="closed-loop steering kd")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--save-dir", default="sim_outputs", help="directory for csv and png outputs")
    parser.add_argument("--keep-existing-xy", action="store_true", help="leave an existing *_xy.csv sidecar untouched")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    track_csv = Path(args.track_csv).expanduser().resolve()
    track = load_raceline_track(track_csv)
    xy_path = write_xy_sidecar(track, overwrite=not args.keep_existing_xy)

    config = SimConfig(
        dt=args.dt,
        gps_rate_hz=args.gps_rate,
        laps=args.laps,
        localization_mode=args.localization_mode,
        wheelbase_m=args.wheelbase_m,
        max_steer_deg=args.max_steer_deg,
        steering_kp=args.kp,
        steering_ki=args.ki,
        steering_kd=args.kd,
        speed_to_throttle_scale=args.speed_to_throttle_scale,
        max_throttle=args.max_throttle,
        throttle_speed_lag_s=args.throttle_speed_lag_s,
        steering_lag_s=args.steering_lag_s,
        gps_noise_m=args.gps_noise_m,
        yaw_noise_deg=args.yaw_noise_deg,
        yaw_rate_noise_degps=args.yaw_rate_noise_degps,
        accel_noise_mps2=args.accel_noise_mps2,
        initial_speed_mps=args.initial_speed_mps,
        seed=args.seed,
    )

    logs, summary = run_pseudo_sim(track, xy_path, config)

    save_dir = Path(args.save_dir).expanduser().resolve()
    output_stem = f"{track_csv.stem}_gps_mpc_pseudo_sim_{config.localization_mode}"
    output_csv = save_dir / f"{output_stem}.csv"
    output_png = save_dir / f"{output_stem}.png"
    save_logs(logs, output_csv)
    save_plot(track, logs, output_png)
    print_summary(track, summary, xy_path, output_csv, output_png)


if __name__ == "__main__":
    main()
