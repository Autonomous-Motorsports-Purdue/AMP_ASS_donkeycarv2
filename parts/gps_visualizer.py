import matplotlib.pyplot as plt
import contextily as cx
import math
from collections import deque

BUFFER_SIZE = 500


class GPSVisualizer:
    """
    Donkeycar part that plots GPS coordinates on a real map using contextily.

    Inputs: lat_deg, lon_deg (raw GPS coordinates in degrees), yaw (radians)
    Outputs: none

    Usage:
        from parts.gps_visualizer import GPSVisualizer

        V.add(GPSVisualizer(), inputs=['lat_raw', 'lon_raw', 'yaw'], outputs=[])
    """

    def __init__(self, buffer_size=BUFFER_SIZE, tile_source=cx.providers.Esri.WorldImagery):
        self.lats = deque(maxlen=buffer_size)
        self.lons = deque(maxlen=buffer_size)
        self.tile_source = tile_source
        self._cached_bounds = None

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.ax.set_title("GPS Track")
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")

        (self.trail_line,) = self.ax.plot([], [], '-', color='#00ff88', linewidth=2, alpha=0.8, zorder=3)
        (self.current_dot,) = self.ax.plot([], [], 'o', color='#ff3333', markersize=10, zorder=4)

        plt.ion()
        plt.show(block=False)

    def _pad_bounds(self, lat_min, lat_max, lon_min, lon_max, pad_frac=0.3):
        dlat = max(lat_max - lat_min, 1e-5)
        dlon = max(lon_max - lon_min, 1e-5)
        return (
            lat_min - dlat * pad_frac,
            lat_max + dlat * pad_frac,
            lon_min - dlon * pad_frac,
            lon_max + dlon * pad_frac,
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

    def _refresh_basemap(self):
        lats = list(self.lats)
        lons = list(self.lons)
        bounds = self._pad_bounds(min(lats), max(lats), min(lons), max(lons))
        lat_min, lat_max, lon_min, lon_max = bounds

        self.ax.set_xlim(lon_min, lon_max)
        self.ax.set_ylim(lat_min, lat_max)

        # remove old basemap images but keep our plot lines
        for img in self.ax.images:
            img.remove()

        try:
            cx.add_basemap(
                self.ax,
                crs="EPSG:4326",
                source=self.tile_source,
                zoom='auto',
            )
        except Exception:
            pass

        self._cached_bounds = bounds

    def run(self, lat_deg, lon_deg, yaw=0.0):
        if lat_deg is None or lon_deg is None:
            return

        self.lats.append(lat_deg)
        self.lons.append(lon_deg)

        lats = list(self.lats)
        lons = list(self.lons)

        self.trail_line.set_data(lons, lats)
        self.current_dot.set_data([lons[-1]], [lats[-1]])

        if self._needs_tile_refresh((min(lats), max(lats), min(lons), max(lons))):
            self._refresh_basemap()

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
        plt.close(self.fig)
