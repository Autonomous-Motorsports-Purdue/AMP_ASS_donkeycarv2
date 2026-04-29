import math
import time
from pathlib import Path

import numpy as np


R_EARTH_M = 6378137.0


def _find_column(dtype_names, candidates):
    lower_map = {name.lower(): name for name in dtype_names}
    for candidate in candidates:
        key = candidate.lower()
        if key in lower_map:
            return lower_map[key]
    raise ValueError(f"Could not find any of columns {candidates} in {dtype_names}")


def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class SimpleGPSPIDController:
    """
    Minimal GPS path follower:
    1. Convert the GPS fix into local XY.
    2. Pick a lookahead point on the path.
    3. PID on heading error to that point.
    4. Return steering plus constant throttle.
    """

    def __init__(
        self,
        path_csv,
        lookahead_m=6.0,
        throttle=0.22,
        kp=1.0,
        ki=0.0,
        kd=0.15,
        max_steer_deg=35.0,
        waypoint_reached_m=1.5,
        search_window=20,
        fixed_dt=None,
        verbose=True,
    ):
        self.path_csv = Path(path_csv)
        self.lookahead_m = float(lookahead_m)
        self.throttle = float(throttle)
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_steer_deg = float(max_steer_deg)
        self.max_steer_rad = math.radians(self.max_steer_deg)
        self.waypoint_reached_m = float(waypoint_reached_m)
        self.search_window = int(search_window)
        self.last_closest_idx = None
        self.fixed_dt = fixed_dt
        self.verbose = verbose

        self.lat_path, self.lon_path = self._load_latlon_path(self.path_csv)
        self.ref_lat_deg = float(self.lat_path[0])
        self.ref_lon_deg = float(self.lon_path[0])
        self.ref_lat_rad = math.radians(self.ref_lat_deg)
        self.ref_lon_rad = math.radians(self.ref_lon_deg)
        self.cos_ref_lat = max(math.cos(self.ref_lat_rad), 1e-6)

        self.path_x_m, self.path_y_m = self._latlon_to_local_xy(self.lat_path, self.lon_path)
        self.path_s_m = self._build_path_s(self.path_x_m, self.path_y_m)
        self.path_heading_rad = self._build_path_heading(self.path_x_m, self.path_y_m)
        self.path_length_m = float(self.path_s_m[-1]) if len(self.path_s_m) else 0.0

        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_debug = None

        if self.verbose:
            print(
                f"[SimpleGPSPIDController] Loaded {len(self.path_x_m)} points from "
                f"{self.path_csv} with lookahead={self.lookahead_m:.2f} m"
            )

    def _load_latlon_path(self, csv_path):
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
            raise ValueError(f"Failed to parse path CSV header from {csv_path}")

        lat_col = _find_column(data.dtype.names, ("latitude", "lat", "lat_deg"))
        lon_col = _find_column(data.dtype.names, ("longitude", "lon", "lon_deg"))

        lat = np.asarray(data[lat_col], dtype=float)
        lon = np.asarray(data[lon_col], dtype=float)
        if lat.size < 2:
            raise ValueError(f"Path must contain at least 2 points: {csv_path}")
        return lat, lon

    def _latlon_to_local_xy(self, lat_deg, lon_deg):
        lat_rad = np.radians(lat_deg)
        lon_rad = np.radians(lon_deg)
        dlat = lat_rad - self.ref_lat_rad
        dlon = lon_rad - self.ref_lon_rad
        x_east = dlon * self.cos_ref_lat * R_EARTH_M
        y_north = dlat * R_EARTH_M
        return np.asarray(x_east, dtype=float), np.asarray(y_north, dtype=float)

    def local_xy_to_latlon(self, x_east_m, y_north_m):
        lat_rad = self.ref_lat_rad + (float(y_north_m) / R_EARTH_M)
        lon_rad = self.ref_lon_rad + (float(x_east_m) / (R_EARTH_M * self.cos_ref_lat))
        return float(np.degrees(lat_rad)), float(np.degrees(lon_rad))

    def _build_path_s(self, path_x, path_y):
        segment_lengths = np.hypot(np.diff(path_x), np.diff(path_y))
        return np.concatenate(([0.0], np.cumsum(segment_lengths)))

    def _build_path_heading(self, path_x, path_y):
        dx = np.diff(path_x)
        dy = np.diff(path_y)
        heading = np.arctan2(dy, dx)
        return np.concatenate((heading, [heading[-1]]))

    def _find_closest_idx(self, x_m, y_m):
        distances = np.hypot(self.path_x_m - x_m, self.path_y_m - y_m)
        return int(np.argmin(distances))

    def _find_closest_idx_forward(self, x_m, y_m):
        n = len(self.path_x_m)
        if self.last_closest_idx is None:
            best = self._find_closest_idx(x_m, y_m)
        else:
            window = np.arange(self.last_closest_idx, self.last_closest_idx + self.search_window + 1) % n
            distances = np.hypot(self.path_x_m[window] - x_m, self.path_y_m[window] - y_m)
            best = int(window[int(np.argmin(distances))])
        self.last_closest_idx = best
        return best

    def reset_path_tracking(self):
        self.last_closest_idx = None

    def _find_target_idx(self, closest_idx):
        target_s_m = self.path_s_m[closest_idx] + self.lookahead_m
        if target_s_m <= self.path_s_m[-1]:
            return int(np.searchsorted(self.path_s_m, target_s_m, side="left"))

        wrapped_s_m = target_s_m - self.path_s_m[-1]
        return int(np.searchsorted(self.path_s_m, wrapped_s_m, side="left"))

    def _compute_dt(self):
        if self.fixed_dt is not None:
            return float(self.fixed_dt)

        current_time = time.time()
        if self.last_time is None:
            dt = 0.05
        else:
            dt = max(current_time - self.last_time, 1e-3)
        self.last_time = current_time
        return dt

    def compute_control_from_xy(self, x_m, y_m, yaw_rad):
        closest_idx = self._find_closest_idx_forward(float(x_m), float(y_m))
        target_idx = self._find_target_idx(closest_idx)

        target_x_m = float(self.path_x_m[target_idx])
        target_y_m = float(self.path_y_m[target_idx])
        dx_m = target_x_m - float(x_m)
        dy_m = target_y_m - float(y_m)

        desired_heading_rad = math.atan2(dy_m, dx_m)
        heading_error_rad = normalize_angle(desired_heading_rad - float(yaw_rad))

        dt = self._compute_dt()
        self.error_integral += heading_error_rad * dt
        derivative = (heading_error_rad - self.last_error) / dt
        self.last_error = heading_error_rad

        steering_angle_rad = (
            self.kp * heading_error_rad
            + self.ki * self.error_integral
            + self.kd * derivative
        )
        steering_cmd = float(np.clip(steering_angle_rad / self.max_steer_rad, -1.0, 1.0))

        self.last_debug = {
            "x_m": float(x_m),
            "y_m": float(y_m),
            "yaw_rad": float(yaw_rad),
            "closest_idx": closest_idx,
            "target_idx": target_idx,
            "target_x_m": target_x_m,
            "target_y_m": target_y_m,
            "desired_heading_rad": desired_heading_rad,
            "heading_error_rad": heading_error_rad,
            "steering_angle_rad": steering_angle_rad,
            "steering_cmd": steering_cmd,
            "throttle_cmd": self.throttle,
        }
        return steering_cmd, self.throttle

    def run(self, gps_lat, gps_lon, yaw_rad):
        if gps_lat is None or gps_lon is None or yaw_rad is None:
            self.last_debug = None
            return 0.0, 0.0

        x_m, y_m = self._latlon_to_local_xy(float(gps_lat), float(gps_lon))
        return self.compute_control_from_xy(float(x_m), float(y_m), float(yaw_rad))

    def shutdown(self):
        pass
