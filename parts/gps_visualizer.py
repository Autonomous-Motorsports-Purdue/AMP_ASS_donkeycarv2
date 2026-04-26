import matplotlib.pyplot as plt
import math
import numpy as np
import time
from collections import deque

BUFFER_SIZE = 500
OFFLINE_MODE = True

class GPSVisualizer:
    """
    Donkeycar part that plots GPS coordinates on a real map using contextily.

    Inputs: lat_deg, lon_deg (raw GPS coordinates in degrees), yaw (radians)
    Outputs: none

    Usage:
        from parts.gps_visualizer import GPSVisualizer

        V.add(GPSVisualizer(), inputs=['lat_raw', 'lon_raw', 'yaw'], outputs=[])
    """

    def __init__(self, path_csv=None, buffer_size=BUFFER_SIZE, tile_source=None,
                 max_zoom=22, min_span_m=60.0, offline_map_path="maps/wlaf_z18.tif",
                 view_span_m=80.0):
        self.lats = deque(maxlen=buffer_size)
        self.lons = deque(maxlen=buffer_size)
        self.tile_source = tile_source
        self.max_zoom = max_zoom
        self.min_span_deg = min_span_m / 111_320.0  # ~meters per degree lat
        self.view_span_deg = view_span_m / 111_320.0
        self._cached_bounds = None
        
        if (path_csv is not None):
            data = np.genfromtxt(path_csv, delimiter=',', skip_header=1, dtype=float, encoding='utf-8')
            self.lat_path = data[:,0]
            self.lon_path = data[:,1]
        else:
            self.lat_path = None
            self.lon_path = None

        # Drawing must happen on the main thread (Tk/Qt event loops),
        # so run_threaded does the draw and update() is a keep-alive.
        self._running = True
        self._redraw_hz = 15.0
        self._last_draw = 0.0

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.ax.set_title("GPS Track")
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")

        self._map_extent = None
        # if offline_map_path:
        #     self._load_offline_map(offline_map_path)

        (self.trail_line,) = self.ax.plot([], [], '-', color='#00ff88', linewidth=2, alpha=0.8, zorder=3)
        (self.current_dot,) = self.ax.plot([], [], 'o', color='#ff3333', markersize=10, zorder=4)
        if (self.lat_path is not None and self.lon_path is not None):
            (self.path,) = self.ax.plot(self.lon_path, self.lat_path, '--', color='#3333ff', linewidth=2, alpha=0.5, zorder=2)

        plt.ion()
        plt.show(block=False)

    def _follow_view(self, lat, lon):
        """Recenter the axis around (lat, lon) at self.view_span_deg, clamped
        to the loaded raster extent so we don't pan off the imagery."""
        half_lat = 0.5 * self.view_span_deg
        half_lon = half_lat / max(math.cos(math.radians(lat)), 1e-3)
        lat_min, lat_max = lat - half_lat, lat + half_lat
        lon_min, lon_max = lon - half_lon, lon + half_lon

        if self._map_extent is not None:
            w, e, s, n = self._map_extent
            if lon_max - lon_min < e - w:
                if lon_min < w:
                    lon_min, lon_max = w, w + (lon_max - lon_min)
                elif lon_max > e:
                    lon_min, lon_max = e - (lon_max - lon_min), e
            if lat_max - lat_min < n - s:
                if lat_min < s:
                    lat_min, lat_max = s, s + (lat_max - lat_min)
                elif lat_max > n:
                    lat_min, lat_max = n - (lat_max - lat_min), n

        self.ax.set_xlim(lon_min, lon_max)
        self.ax.set_ylim(lat_min, lat_max)

    def _pad_bounds(self, lat_min, lat_max, lon_min, lon_max, pad_frac=0.3):
        lat_c = 0.5 * (lat_min + lat_max)
        lon_c = 0.5 * (lon_min + lon_max)
        # enforce a minimum visible window so contextily doesn't request
        # zoom levels beyond what the tile provider supports
        min_lat_span = self.min_span_deg
        min_lon_span = self.min_span_deg / max(math.cos(math.radians(lat_c)), 1e-3)
        dlat = max(lat_max - lat_min, min_lat_span)
        dlon = max(lon_max - lon_min, min_lon_span)
        half_lat = 0.5 * dlat * (1.0 + 2.0 * pad_frac)
        half_lon = 0.5 * dlon * (1.0 + 2.0 * pad_frac)
        return (
            lat_c - half_lat,
            lat_c + half_lat,
            lon_c - half_lon,
            lon_c + half_lon,
        )

    def _needs_tile_refresh(self, bounds):
        if self._cached_bounds is None:
            return True
        
        # refresh if the current position is near the edge of the cached view
        lat_min, lat_max, lon_min, lon_max = self._cached_bounds
        lat, lon = list(self.lats)[-1], list(self.lons)[-1]
        lat_margin = (lat_max - lat_min) * 0.1
        lon_margin = (lon_max - lon_min) * 0.1
        return (
            lat < lat_min + lat_margin or lat > lat_max - lat_margin or
            lon < lon_min + lon_margin or lon > lon_max - lon_margin
        )

    def run_threaded(self, lat_deg, lon_deg, yaw=0.0):
        if lat_deg is None or lon_deg is None:
            return
        now = time.time()
        if now - self._last_draw < 1.0 / self._redraw_hz:
            return
        self._last_draw = now
        self._draw_sample(lat_deg, lon_deg, yaw)

    def update(self):
        """No-op worker; drawing happens in run_threaded on the main thread."""
        while self._running:
            time.sleep(0.1)

    def run(self, lat_deg, lon_deg, yaw=90.0):
        if lat_deg is None or lon_deg is None:
            return
        self._draw_sample(lat_deg, lon_deg, yaw)

    def _draw_sample(self, lat_deg, lon_deg, yaw):
        self.lats.append(lat_deg)
        self.lons.append(lon_deg)

        lats = list(self.lats)
        lons = list(self.lons)

        self.trail_line.set_data(lons, lats)
        self.current_dot.set_data([lons[-1]], [lats[-1]])
        
        self._follow_view(lats[-1], lons[-1])

        theta = yaw
        lat_span = max(max(lats) - min(lats), 1e-5)
        lon_span = max(max(lons) - min(lons), 1e-5)
        arrow_len = 0.08 * max(lat_span, lon_span)

        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)

        if not hasattr(self, "yaw_arrow") or self.yaw_arrow is None:
            self.yaw_arrow = self.ax.annotate(
                "",
                xy=(lons[-1] + dx, lats[-1] + dy),
                xytext=(lons[-1], lats[-1]),
                arrowprops=dict(arrowstyle="->", color="#ffd400", lw=2),
                zorder=5,
            )
        else:
            self.yaw_arrow.xy = (lons[-1] + dx, lats[-1] + dy)
            self.yaw_arrow.set_position((lons[-1], lats[-1]))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def shutdown(self):
        self._running = False
        plt.close(self.fig)
