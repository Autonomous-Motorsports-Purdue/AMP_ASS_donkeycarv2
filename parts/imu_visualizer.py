import matplotlib.pyplot as plt
import math
import time
from collections import deque

BUFFER_SIZE = 500


class IMUVisualizer:
    """
    Donkeycar part that plots IMU outputs in matplotlib.

    Inputs: yaw_rate (rad/s), yaw (rad), ax (m/s^2), ay (m/s^2)
    Outputs: none

    Usage:
        from parts.imu_visualizer import IMUVisualizer
        V.add(IMUVisualizer(), inputs=['yaw_rate', 'yaw', 'ax', 'ay'], outputs=[],
              threaded=True)
    """

    def __init__(self, buffer_size=BUFFER_SIZE, redraw_hz=15.0, time_window_s=20.0):
        self.buffer_size = buffer_size
        self.time_window_s = time_window_s

        self.t = deque(maxlen=buffer_size)
        self.yaw_rates = deque(maxlen=buffer_size)
        self.yaws = deque(maxlen=buffer_size)
        self.axs = deque(maxlen=buffer_size)
        self.ays = deque(maxlen=buffer_size)

        self._t0 = time.time()
        self._last_draw = 0.0
        self._running = True
        self._redraw_hz = redraw_hz

        self.fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        self.fig.suptitle("IMU")
        self.ax_yawrate, self.ax_yaw = axes[0]
        self.ax_accel, self.ax_compass = axes[1]

        self.ax_yawrate.set_title("Yaw rate (rad/s)")
        self.ax_yawrate.set_xlabel("t (s)")
        self.ax_yawrate.grid(True, alpha=0.3)
        (self.line_yawrate,) = self.ax_yawrate.plot([], [], color="#00ff88", lw=1.5)

        self.ax_yaw.set_title("Yaw (deg)")
        self.ax_yaw.set_xlabel("t (s)")
        self.ax_yaw.grid(True, alpha=0.3)
        (self.line_yaw,) = self.ax_yaw.plot([], [], color="#3399ff", lw=1.5)

        self.ax_accel.set_title("Acceleration (m/s^2)")
        self.ax_accel.set_xlabel("t (s)")
        self.ax_accel.grid(True, alpha=0.3)
        (self.line_ax,) = self.ax_accel.plot([], [], color="#ff5555", lw=1.5, label="ax")
        (self.line_ay,) = self.ax_accel.plot([], [], color="#ffaa00", lw=1.5, label="ay")
        self.ax_accel.legend(loc="upper right")

        self.ax_compass.set_title("Heading")
        self.ax_compass.set_aspect("equal")
        self.ax_compass.set_xlim(-1.2, 1.2)
        self.ax_compass.set_ylim(-1.2, 1.2)
        self.ax_compass.axhline(0, color="#888", lw=0.5)
        self.ax_compass.axvline(0, color="#888", lw=0.5)
        circle = plt.Circle((0, 0), 1.0, fill=False, color="#888")
        self.ax_compass.add_patch(circle)
        (self.heading_arrow,) = self.ax_compass.plot([0, 1], [0, 0], color="#ffd400", lw=2)

        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def run_threaded(self, yaw_rate, yaw, ax, ay):
        # Called from the main vehicle loop thread; matplotlib GUI calls
        # must happen here, not in update().
        if yaw_rate is None:
            return
        now = time.time()
        if now - self._last_draw < 1.0 / self._redraw_hz:
            return
        self._last_draw = now
        self._draw_sample(yaw_rate, yaw, ax, ay, now - self._t0)

    def run(self, yaw_rate, yaw, ax, ay):
        if yaw_rate is None:
            return
        self._draw_sample(yaw_rate, yaw, ax, ay, time.time() - self._t0)

    def update(self):
        # No-op worker; drawing happens in run_threaded on the main thread.
        while self._running:
            time.sleep(0.1)

    def _draw_sample(self, yaw_rate, yaw, ax, ay, t):
        self.t.append(t)
        self.yaw_rates.append(float(yaw_rate))
        self.yaws.append(math.degrees(float(yaw)))
        self.axs.append(float(ax))
        self.ays.append(float(ay))

        ts = list(self.t)
        self.line_yawrate.set_data(ts, list(self.yaw_rates))
        self.line_yaw.set_data(ts, list(self.yaws))
        self.line_ax.set_data(ts, list(self.axs))
        self.line_ay.set_data(ts, list(self.ays))

        t_max = ts[-1]
        t_min = max(ts[0], t_max - self.time_window_s)
        for a in (self.ax_yawrate, self.ax_yaw, self.ax_accel):
            a.set_xlim(t_min, t_max if t_max > t_min else t_min + 1e-3)
            a.relim()
            a.autoscale_view(scalex=False, scaley=True)

        theta = float(yaw)
        self.heading_arrow.set_data([0, math.cos(theta)], [0, math.sin(theta)])

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def shutdown(self):
        self._running = False
        plt.close(self.fig)
