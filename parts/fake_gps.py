import threading
import time
import numpy as np

R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


class FakeGps:
    """
    Donkeycar part that simulates GPS latitude/longitude at a lower rate.

    Outputs:
        gps/lat, gps/lon

    Notes:
    - Generates a circular path in local East/North meters around origin.
    - Returns None between GPS ticks so consumers can detect new fixes.
    """

    def __init__(
        self,
        origin_lat,
        origin_lon,
        gps_rate=10,
        speed_mps=2.0,
        turn_radius_m=8.0,
        gps_noise_std_m=1.5,
        seed=123,
    ):
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)

        self.gps_rate = float(gps_rate)
        self.gps_dt = 1.0 / self.gps_rate

        self.speed_mps = float(speed_mps)
        self.turn_radius_m = float(turn_radius_m)
        self.gps_noise_std_m = float(gps_noise_std_m)

        self.rng = np.random.default_rng(seed)
        self.lock = threading.Lock()
        self.running = True

        self.theta = 0.0
        self.yaw_rate = self.speed_mps / self.turn_radius_m

        self._pending_lat = None
        self._pending_lon = None

    def _meters_to_latlon(self, x_east_m, y_north_m):
        cos_lat0 = np.cos(np.radians(self.origin_lat))
        cos_lat0 = np.clip(cos_lat0, 1e-6, None)
        lat = self.origin_lat + (y_north_m / R_EARTH) * RAD2DEG
        lon = self.origin_lon + (x_east_m / (R_EARTH * cos_lat0)) * RAD2DEG
        return lat, lon

    def _step(self, dt):
        self.theta += self.yaw_rate * dt

        x_east = self.turn_radius_m * np.cos(self.theta)
        y_north = self.turn_radius_m * np.sin(self.theta)

        noise_e = self.rng.normal(0.0, self.gps_noise_std_m)
        noise_n = self.rng.normal(0.0, self.gps_noise_std_m)

        lat, lon = self._meters_to_latlon(x_east + noise_e, y_north + noise_n)

        with self.lock:
            self._pending_lat = float(lat)
            self._pending_lon = float(lon)

    def update(self):
        next_tick = time.monotonic()
        last_tick = next_tick
        while self.running:
            now = time.monotonic()
            if now >= next_tick:
                dt = max(1e-4, now - last_tick)
                last_tick = now
                self._step(dt)
                next_tick = now + self.gps_dt

            sleep_s = max(0.0, next_tick - time.monotonic())
            time.sleep(min(sleep_s, 0.001))

    def run_threaded(self):
        with self.lock:
            lat = self._pending_lat
            lon = self._pending_lon
            self._pending_lat = None
            self._pending_lon = None
        return lat, lon

    def run(self):
        return self.run_threaded()

    def shutdown(self):
        self.running = False
