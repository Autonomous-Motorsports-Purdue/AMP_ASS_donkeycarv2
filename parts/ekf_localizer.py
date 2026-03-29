import time
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


class EKFLocalizer:
    """
    DonkeyCar part: fuses GPS + IMU into [lat, lon, vx, vy] using an EKF.

    IMPORTANT: ax, ay must be in the world (North/East) frame before passing in.
    If your IMU gives body-frame accelerations, rotate them by heading first:
        ax_world = ax_body * cos(heading) - ay_body * sin(heading)
        ay_world = ax_body * sin(heading) + ay_body * cos(heading)

    Add to manage.py:
        from ekf_localizer import EKFLocalizer
        ekf = EKFLocalizer(init_lat=37.7749, init_lon=-122.4194)
        V.add(ekf,
              inputs=['imu/accel_x', 'imu/accel_y', 'gps/lat', 'gps/lon'],
              outputs=['ekf/lat', 'ekf/lon', 'ekf/vx', 'ekf/vy'],
              threaded=True)
    """

    def __init__(self, init_lat, init_lon, imu_rate=100, gps_rate=10):
        self.nominal_dt = 1.0 / imu_rate
        self.gps_rate = gps_rate
        self.running = True
        self.last_predict_time = None

        # Outputs
        self.lat = init_lat
        self.lon = init_lon
        self.vx = 0.0
        self.vy = 0.0

        # EKF: state = [lat, lon, vx, vy]
        self.ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
        self.ekf.x = np.array([init_lat, init_lon, 0.0, 0.0])

        # P: initial uncertainty
        # lat/lon in degrees (~1e-5 deg ~= 1m), velocity in m/s
        self.ekf.P = np.diag([1e-8, 1e-8, 1.0, 1.0])

        # R: GPS measurement noise (~5m = 4.5e-5 deg std dev)
        self.ekf.R = np.diag([4.5e-5**2, 4.5e-5**2])

    def _build_Q(self, dt, accel_std=0.1):
        """
        Process noise Q scales with dt.
        accel_std: expected IMU acceleration noise in m/s^2
        """
        q_pos = (0.5 * accel_std * dt**2) ** 2
        q_vel = (accel_std * dt) ** 2

        # Convert q_pos from m^2 to deg^2.
        cos_lat = np.cos(np.radians(self.ekf.x[0]))
        cos_lat = np.clip(cos_lat, 1e-6, None)
        q_lat = q_pos / R_EARTH**2 * (RAD2DEG**2)
        q_lon = q_pos / (R_EARTH * cos_lat) ** 2 * (RAD2DEG**2)

        return np.diag([q_lat, q_lon, q_vel, q_vel])

    def _f(self, x, dt, ax, ay):
        """State transition: propagate [lat, lon, vx, vy] forward by dt."""
        lat, lon, vx, vy = x
        cos_lat = np.cos(np.radians(lat))
        cos_lat = np.clip(cos_lat, 1e-6, None)
        return np.array([
            lat + (vy / R_EARTH) * dt * RAD2DEG,            # lat from vy (North)
            lon + (vx / (R_EARTH * cos_lat)) * dt * RAD2DEG,  # lon from vx (East)
            vx + ax * dt,                                   # vx from East accel
            vy + ay * dt,                                   # vy from North accel
        ])

    def _F_jacobian(self, x, dt):
        """Jacobian of _f with respect to state x."""
        lat, _, vx, _ = x
        cos_lat = np.cos(np.radians(lat))
        cos_lat = np.clip(cos_lat, 1e-6, None)
        sin_lat = np.sin(np.radians(lat))

        F = np.eye(4)
        F[0, 3] = dt / R_EARTH * RAD2DEG
        F[1, 0] = vx * dt * sin_lat / (R_EARTH * cos_lat**2)
        F[1, 2] = dt / (R_EARTH * cos_lat) * RAD2DEG
        return F

    def _h(self, x):
        """Measurement function: we observe [lat, lon] directly."""
        return x[0:2]

    def _H_jacobian(self, x):
        """Jacobian of _h constant since measurement model is linear."""
        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        return H

    def _predict(self, ax, ay, dt):
        x = self.ekf.x
        F = self._F_jacobian(x, dt)
        Q = self._build_Q(dt)
        self.ekf.x = self._f(x, dt, ax, ay)
        self.ekf.P = F @ self.ekf.P @ F.T + Q

    def _update_gps(self, lat, lon):
        self.ekf.update(
            z=np.array([lat, lon]),
            HJacobian=self._H_jacobian,
            Hx=self._h,
        )

    def run(self, ax, ay, gps_lat, gps_lon):
        """
        Synchronous part.
        ax, ay: world-frame acceleration in m/s^2 (East, North)
        """
        now = time.monotonic()
        if self.last_predict_time is None:
            dt = self.nominal_dt
        else:
            dt = now - self.last_predict_time
            dt = float(np.clip(dt, 1e-4, 0.2))
        self.last_predict_time = now

        self._predict(ax or 0.0, ay or 0.0, dt)
        if gps_lat is not None and gps_lon is not None:
            self._update_gps(gps_lat, gps_lon)
        self.lat, self.lon, self.vx, self.vy = self.ekf.x
        return self.lat, self.lon, self.vx, self.vy

    def run_threaded(self, ax, ay, gps_lat, gps_lon):
        return self.run(ax, ay, gps_lat, gps_lon)

    def update(self):
        while self.running:
            time.sleep(self.nominal_dt)

    def shutdown(self):
        self.running = False
