import math
from pathlib import Path

import numpy as np


R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


def _find_column(dtype_names, candidates):
    lower_map = {name.lower(): name for name in dtype_names}
    for candidate in candidates:
        key = candidate.lower()
        if key in lower_map:
            return lower_map[key]
    raise ValueError(f"Could not find any of columns {candidates} in {dtype_names}")


class GPSPurePursuitWaypointGenerator:
    def __init__(
        self,
        path_csv,
        lookahead_m=6.0,
        use_course_heading=True,
        course_heading_min_distance_m=0.75,
        verbose=True,
    ):
        self.path_csv = Path(path_csv)
        self.lookahead_m = float(lookahead_m)
        self.use_course_heading = use_course_heading
        self.course_heading_min_distance_m = float(course_heading_min_distance_m)
        self.verbose = verbose

        self.lat_path, self.lon_path = self._load_latlon_path(self.path_csv)
        self.ref_lat = float(self.lat_path[0])
        self.ref_lon = float(self.lon_path[0])
        self.ref_lat_rad = math.radians(self.ref_lat)
        self.ref_lon_rad = math.radians(self.ref_lon)
        self.cos_ref_lat = max(math.cos(self.ref_lat_rad), 1e-6)

        self.path_x, self.path_y = self._latlon_to_local_m(self.lat_path, self.lon_path)
        self.path_s = self._build_path_s(self.path_x, self.path_y)
        self.path_heading = self._build_path_heading(self.path_x, self.path_y)
        self.lap_length_m = float(self.path_s[-1]) if len(self.path_s) > 0 else 0.0
        self.last_debug = None
        self.prev_gps_x = None
        self.prev_gps_y = None
        self.course_heading_rad = None

        if self.verbose:
            print(
                f"[GPSPurePursuitWaypointGenerator] Loaded {len(self.path_x)} points from "
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
        return np.asarray(data[lat_col], dtype=float), np.asarray(data[lon_col], dtype=float)

    def _latlon_to_local_m(self, lat_deg, lon_deg):
        lat_rad = np.radians(lat_deg)
        lon_rad = np.radians(lon_deg)
        dlat = lat_rad - self.ref_lat_rad
        dlon = lon_rad - self.ref_lon_rad
        x_east = dlon * self.cos_ref_lat * R_EARTH
        y_north = dlat * R_EARTH
        return np.asarray(x_east, dtype=float), np.asarray(y_north, dtype=float)

    def local_xy_to_latlon(self, x_east_m, y_north_m):
        lat_rad = self.ref_lat_rad + (y_north_m / R_EARTH)
        lon_rad = self.ref_lon_rad + (x_east_m / (R_EARTH * self.cos_ref_lat))
        return float(np.degrees(lat_rad)), float(np.degrees(lon_rad))

    def _build_path_s(self, path_x, path_y):
        if len(path_x) <= 1:
            return np.zeros(len(path_x), dtype=float)
        segment_lengths = np.hypot(np.diff(path_x), np.diff(path_y))
        return np.concatenate(([0.0], np.cumsum(segment_lengths)))

    def _build_path_heading(self, path_x, path_y):
        if len(path_x) == 1:
            return np.zeros(1, dtype=float)
        dx = np.diff(path_x)
        dy = np.diff(path_y)
        heading = np.arctan2(dy, dx)
        return np.concatenate((heading, [heading[-1]]))

    def _find_closest_idx(self, x, y):
        distances = np.hypot(self.path_x - x, self.path_y - y)
        return int(np.argmin(distances))

    def _find_lookahead_idx(self, closest_idx):
        target_s = self.path_s[closest_idx] + self.lookahead_m
        if target_s <= self.path_s[-1]:
            return int(np.searchsorted(self.path_s, target_s, side="left"))

        wrapped_s = target_s - self.path_s[-1]
        return int(np.searchsorted(self.path_s, wrapped_s, side="left"))

    def _target_vehicle_frame(self, current_x, current_y, heading_rad, target_x, target_y):
        dx = target_x - current_x
        dy = target_y - current_y

        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        x_forward = cos_h * dx + sin_h * dy
        y_right = sin_h * dx - cos_h * dy
        return float(x_forward), float(y_right)

    def _estimate_heading(self, current_x, current_y, closest_idx):
        heading_rad = float(self.path_heading[closest_idx])
        if self.use_course_heading and self.prev_gps_x is not None and self.prev_gps_y is not None:
            dx = current_x - self.prev_gps_x
            dy = current_y - self.prev_gps_y
            if math.hypot(dx, dy) >= self.course_heading_min_distance_m:
                self.course_heading_rad = math.atan2(dy, dx)
        if self.course_heading_rad is not None:
            heading_rad = self.course_heading_rad
        return heading_rad

    def compute_target_from_latlon(self, gps_lat, gps_lon):
        if gps_lat is None or gps_lon is None:
            self.last_debug = None
            return None

        current_x, current_y = self._latlon_to_local_m(float(gps_lat), float(gps_lon))
        current_x = float(current_x)
        current_y = float(current_y)

        closest_idx = self._find_closest_idx(current_x, current_y)
        lookahead_idx = self._find_lookahead_idx(closest_idx)
        heading_rad = self._estimate_heading(current_x, current_y, closest_idx)

        waypoint = self._target_vehicle_frame(
            current_x,
            current_y,
            heading_rad,
            float(self.path_x[lookahead_idx]),
            float(self.path_y[lookahead_idx]),
        )

        if waypoint[0] < 0.0:
            for _ in range(len(self.path_x)):
                lookahead_idx = (lookahead_idx + 1) % len(self.path_x)
                waypoint = self._target_vehicle_frame(
                    current_x,
                    current_y,
                    heading_rad,
                    float(self.path_x[lookahead_idx]),
                    float(self.path_y[lookahead_idx]),
                )
                if waypoint[0] >= 0.0:
                    break

        self.last_debug = {
            "current_x_m": current_x,
            "current_y_m": current_y,
            "closest_idx": closest_idx,
            "lookahead_idx": lookahead_idx,
            "heading_rad": heading_rad,
            "target_x_m": float(self.path_x[lookahead_idx]),
            "target_y_m": float(self.path_y[lookahead_idx]),
            "waypoint": (waypoint[0], waypoint[1], 0.0),
        }
        self.prev_gps_x = current_x
        self.prev_gps_y = current_y
        return self.last_debug

    def run(self, gps_lat, gps_lon):
        target = self.compute_target_from_latlon(gps_lat, gps_lon)
        if target is None:
            return None
        return target["waypoint"]

    def shutdown(self):
        pass
