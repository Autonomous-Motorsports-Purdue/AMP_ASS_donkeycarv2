import threading
import time
import numpy as np


class FakeImu:
    """
    Donkeycar part that simulates world-frame IMU acceleration.

    Outputs:
        imu/accel_x, imu/accel_y

    Notes:
    - Acceleration is in world East/North frame to match EKFLocalizer.
    - Motion is a constant-speed circle to produce non-zero dynamics.
    """

    def __init__(
        self,
        imu_rate=100,
        speed_mps=2.0,
        turn_radius_m=8.0,
        accel_noise_std=0.03,
        seed=42,
    ):
        self.imu_rate = float(imu_rate)
        self.imu_dt = 1.0 / self.imu_rate

        self.speed_mps = float(speed_mps)
        self.turn_radius_m = float(turn_radius_m)
        self.accel_noise_std = float(accel_noise_std)

        self.rng = np.random.default_rng(seed)
        self.lock = threading.Lock()
        self.running = True

        self.theta = 0.0
        self.yaw_rate = self.speed_mps / self.turn_radius_m

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.x_east = self.turn_radius_m
        self.y_north = 0.0

    def _step(self, dt):
        self.theta += self.yaw_rate * dt

        # Position is kept for optional reuse by other parts/tests.
        self.x_east = self.turn_radius_m * np.cos(self.theta)
        self.y_north = self.turn_radius_m * np.sin(self.theta)

        # World-frame centripetal acceleration.
        ax_true = -self.speed_mps * self.yaw_rate * np.cos(self.theta)
        ay_true = -self.speed_mps * self.yaw_rate * np.sin(self.theta)

        ax_meas = ax_true + self.rng.normal(0.0, self.accel_noise_std)
        ay_meas = ay_true + self.rng.normal(0.0, self.accel_noise_std)

        with self.lock:
            self.accel_x = float(ax_meas)
            self.accel_y = float(ay_meas)

    def update(self):
        next_tick = time.monotonic()
        last_tick = next_tick

        while self.running:
            now = time.monotonic()
            if now >= next_tick:
                dt = max(1e-4, now - last_tick)
                last_tick = now
                self._step(dt)
                next_tick += self.imu_dt

            sleep_s = max(0.0, next_tick - time.monotonic())
            time.sleep(min(sleep_s, 0.001))

    def run_threaded(self):
        with self.lock:
            return self.accel_x, self.accel_y

    def run(self):
        return self.run_threaded()

    def shutdown(self):
        self.running = False
