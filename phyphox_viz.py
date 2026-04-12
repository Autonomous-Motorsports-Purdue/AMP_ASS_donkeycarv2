"""
Real-time visualization of phyphox sensor data.

Auto-discovers available buffers from the running phyphox experiment via /config,
then groups them by time buffer and plots each group.

Usage:
    python phyphox_viz.py [--host HOST] [--port PORT] [--interval MS]
"""

import argparse
import sys
import requests
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

BUFFER_SIZE = 200


# Known time buffers and the data buffers that share them.
# Order matters: first match wins when grouping discovered buffers.
KNOWN_GROUPS = [
    ("acc_time",      ["accX", "accY", "accZ"],           "Accelerometer",       "m/s²"),
    ("gyr_time",      ["gyrX", "gyrY", "gyrZ"],           "Gyroscope",           "rad/s"),
    ("mag_time",      ["magX", "magY", "magZ"],            "Magnetometer",        "µT"),
    ("lin_time",      ["linX", "linY", "linZ"],            "Linear Acceleration", "m/s²"),
    ("pressure_time", ["pressure"],                        "Pressure",            "hPa"),
    ("prox_time",     ["prox"],                            "Proximity",           "cm"),
    ("light_time",    ["light"],                           "Light",               "lx"),
    ("loc_time",      ["locLat", "locLon", "locZ", "locV", "locDir",
                       "locAccuracy", "locZAccuracy", "locSatellites", "locStatus"],
                                                           "Location",            "mixed"),
    ("attT",          ["yaw", "pitch", "roll", "direct"],  "Attitude (Euler)",    "degrees"),
]


def discover_buffers(base_url):
    """Query /config to find all available buffer names, then group them."""
    resp = requests.get(f"{base_url}/config", timeout=3)
    resp.raise_for_status()
    config = resp.json()

    available = {b["name"] for b in config.get("buffers", [])}
    print(f"Experiment: {config.get('title', '(unknown)')}")
    print(f"Available buffers: {sorted(available)}")

    groups = []  # list of (time_key, [data_keys], title, ylabel)
    claimed = set()

    for time_key, data_keys, title, ylabel in KNOWN_GROUPS:
        if time_key not in available:
            continue
        present = [k for k in data_keys if k in available]
        if not present:
            continue
        groups.append((time_key, present, title, ylabel))
        claimed.add(time_key)
        claimed.update(present)

    # Group any remaining unclaimed buffers that look like they share a time buffer
    # (buffers ending in _time that we haven't claimed yet)
    remaining_time = [b for b in available - claimed if b.endswith("_time")]
    for tb in sorted(remaining_time):
        prefix = tb.replace("_time", "")
        data = [b for b in available - claimed - {tb}
                if b.startswith(prefix) or b == prefix]
        if data:
            groups.append((tb, sorted(data), prefix.title(), ""))
            claimed.add(tb)
            claimed.update(data)

    if not groups:
        print("No recognized sensor groups found in this experiment.")
        sys.exit(1)

    for time_key, data_keys, title, _ in groups:
        print(f"  {title}: {data_keys} (time: {time_key})")

    return groups


class PhyphoxClient:
    def __init__(self, base_url, groups):
        self.base_url = base_url
        self.groups = groups
        self.last_time = {g[0]: 0.0 for g in groups}
        self.data = {}
        for time_key, data_keys, _, _ in groups:
            self.data[time_key] = deque(maxlen=BUFFER_SIZE)
            for dk in data_keys:
                self.data[dk] = deque(maxlen=BUFFER_SIZE)

    def _build_url(self, time_key, data_keys):
        """Build a single /get URL for one sensor group."""
        t = self.last_time[time_key]
        parts = []
        # Time buffer: threshold against itself
        parts.append(f"{time_key}={t}")
        # Data buffers: threshold against the time buffer
        for dk in data_keys:
            parts.append(f"{dk}={t}|{time_key}")
        return f"{self.base_url}/get?{'&'.join(parts)}"

    def fetch_group(self, time_key, data_keys):
        url = self._build_url(time_key, data_keys)
        try:
            resp = requests.get(url, timeout=1)
            resp.raise_for_status()
            payload = resp.json()
        except requests.RequestException as e:
            print(f"  [{time_key}] request failed: {e}")
            return
        except ValueError:
            print(f"  [{time_key}] invalid JSON response")
            return

        buffers = payload.get("buffer", {})

        time_vals = buffers.get(time_key, {}).get("buffer", [])
        if not time_vals:
            return

        self.data[time_key].extend(time_vals)
        self.last_time[time_key] = time_vals[-1]

        for dk in data_keys:
            vals = buffers.get(dk, {}).get("buffer", [])
            self.data[dk].extend(vals)

    def fetch_all(self):
        for time_key, data_keys, _, _ in self.groups:
            self.fetch_group(time_key, data_keys)


def main():
    parser = argparse.ArgumentParser(description="Phyphox real-time visualizer")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--interval", type=int, default=100, help="Update interval in ms")
    args = parser.parse_args()

    base_url = f"http://{args.host}:{args.port}"

    print(f"Connecting to {base_url} ...")
    try:
        groups = discover_buffers(base_url)
    except requests.RequestException as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    client = PhyphoxClient(base_url, groups)

    n = len(groups)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3 * n), squeeze=False)
    axes = [ax[0] for ax in axes]
    fig.suptitle("Phyphox Live Data", fontsize=14)

    colors = ["#e74c3c", "#2ecc71", "#3498db", "#f39c12", "#9b59b6",
              "#1abc9c", "#e67e22", "#8e44ad", "#2c3e50"]

    line_objs = []  # parallel to groups: list of (data_key, line) pairs
    for i, (time_key, data_keys, title, ylabel) in enumerate(groups):
        ax = axes[i]
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)
        group_lines = []
        for j, dk in enumerate(data_keys):
            (line,) = ax.plot([], [], label=dk, color=colors[j % len(colors)], linewidth=1)
            group_lines.append((dk, line))
        if len(data_keys) > 1:
            ax.legend(loc="upper left", fontsize=8)
        line_objs.append(group_lines)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    def update(_frame):
        client.fetch_all()
        artists = []
        for i, (time_key, data_keys, _, _) in enumerate(groups):
            t = list(client.data[time_key])
            ax = axes[i]
            for dk, line in line_objs[i]:
                vals = list(client.data[dk])
                min_len = min(len(t), len(vals))
                if min_len > 0:
                    line.set_data(t[:min_len], vals[:min_len])
                artists.append(line)
            if t:
                ax.set_xlim(t[0], t[-1] + 0.01)
            ax.relim()
            ax.autoscale_view(scalex=False)
        return artists

    _ani = animation.FuncAnimation(fig, update, interval=args.interval, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
