import time
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


class EKFLocalizer:
    """
    DonkeyCar part: fuses GPS + IMU into [lat, lon, vx, vy] using an EKF.

    Expects IMU acceleration in body frame plus a yaw heading in either
    degrees or radians, depending on ``heading_unit``.
    The part rotates accel into world East/North before prediction.

    Add to manage.py:
        from ekf_localizer import EKFLocalizer
        ekf = EKFLocalizer(init_lat=37.7749, init_lon=-122.4194)
        V.add(ekf,
              inputs=['imu/accel_x', 'imu/accel_y', 'imu/heading_deg', 'gps/lat', 'gps/lon'],
              outputs=['ekf/lat', 'ekf/lon', 'ekf/vx', 'ekf/vy'],
              threaded=True)
    """

    def __init__(
        self,
        init_lat,
        init_lon,
        imu_rate=100,
        gps_rate=10,
        heading_unit="deg",
        fixed_dt=None,
        gps_pos_std_m=2.5,
        gps_vel_std_mps=1.5,
        use_gps_velocity=True,
        gps_repeat_epsilon_deg=1e-12,
    ):
        self.nominal_dt = 1.0 / imu_rate
        self.gps_rate = gps_rate
        self.running = True
        self.last_predict_time = None
        self.heading_unit = heading_unit.lower()
        self.fixed_dt = fixed_dt
        self.gps_pos_std_m = float(gps_pos_std_m)
        self.gps_vel_std_mps = float(gps_vel_std_mps)
        self.use_gps_velocity = use_gps_velocity
        self.gps_repeat_epsilon_deg = float(gps_repeat_epsilon_deg)
        self.current_time = 0.0 if self.fixed_dt is not None else None
        self.last_gps_fix = None

        if self.heading_unit not in {"deg", "rad"}:
            raise ValueError(f"Unsupported heading_unit={heading_unit!r}. Use 'deg' or 'rad'.")

        # Outputs
        self.lat = init_lat
        self.lon = init_lon
        self.vx = 0.0
        self.vy = 0.0

        # EKF: state = [lat, lon, vx, vy]
        self.ekf = ExtendedKalmanFilter(dim_x=4, dim_z=4)
        self.ekf.x = np.array([init_lat, init_lon, 0.0, 0.0])

        # P: initial uncertainty
        # lat/lon in degrees (~1e-5 deg ~= 1m), velocity in m/s
        self.ekf.P = np.diag([1e-8, 1e-8, 25.0, 25.0])

    def _position_std_deg(self, lat_deg):
        cos_lat = np.cos(np.radians(lat_deg))
        cos_lat = np.clip(cos_lat, 1e-6, None)
        lat_std_deg = (self.gps_pos_std_m / R_EARTH) * RAD2DEG
        lon_std_deg = (self.gps_pos_std_m / (R_EARTH * cos_lat)) * RAD2DEG
        return lat_std_deg, lon_std_deg

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

    def _h_position(self, x):
        """Measurement function: we observe [lat, lon] directly."""
        return x[0:2]

    def _H_position_jacobian(self, x):
        """Jacobian of _h constant since measurement model is linear."""
        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        return H

    def _h_full(self, x):
        return x

    def _H_full_jacobian(self, x):
        return np.eye(4)

    def _predict(self, ax, ay, dt):
        x = self.ekf.x
        F = self._F_jacobian(x, dt)
        Q = self._build_Q(dt)
        self.ekf.x = self._f(x, dt, ax, ay)
        self.ekf.P = F @ self.ekf.P @ F.T + Q

    def _update_gps_position(self, lat, lon):
        lat_std_deg, lon_std_deg = self._position_std_deg(lat)
        self.ekf.R = np.diag([lat_std_deg**2, lon_std_deg**2])
        self.ekf.update(
            z=np.array([lat, lon]),
            HJacobian=self._H_position_jacobian,
            Hx=self._h_position,
        )

    def _update_gps_full(self, lat, lon, vx, vy):
        lat_std_deg, lon_std_deg = self._position_std_deg(lat)
        self.ekf.R = np.diag([
            lat_std_deg**2,
            lon_std_deg**2,
            self.gps_vel_std_mps**2,
            self.gps_vel_std_mps**2,
        ])
        self.ekf.update(
            z=np.array([lat, lon, vx, vy]),
            HJacobian=self._H_full_jacobian,
            Hx=self._h_full,
        )

    def _gps_velocity_from_fix_pair(self, prev_lat, prev_lon, prev_time, lat, lon, now):
        dt = now - prev_time
        if dt <= 1e-3:
            return None

        avg_lat = 0.5 * (prev_lat + lat)
        cos_lat = np.cos(np.radians(avg_lat))
        cos_lat = np.clip(cos_lat, 1e-6, None)
        vx = ((lon - prev_lon) / RAD2DEG) * (R_EARTH * cos_lat) / dt
        vy = ((lat - prev_lat) / RAD2DEG) * R_EARTH / dt
        return float(vx), float(vy)

    def _is_new_gps_fix(self, lat, lon):
        if self.last_gps_fix is None:
            return True
        prev_lat, prev_lon, _ = self.last_gps_fix
        return not (
            abs(float(lat) - prev_lat) <= self.gps_repeat_epsilon_deg
            and abs(float(lon) - prev_lon) <= self.gps_repeat_epsilon_deg
        )

    def _heading_to_radians(self, heading):
        if heading is None:
            return 0.0
        if self.heading_unit == "rad":
            return float(heading)
        return float(np.radians(heading))

    def run(self, ax, ay, heading, gps_lat, gps_lon):
        """
        Synchronous part.
        ax, ay: body-frame acceleration in m/s^2
        heading: yaw heading in ``heading_unit``
        """
        if self.fixed_dt is not None:
            dt = float(self.fixed_dt)
            self.current_time += dt
            now = self.current_time
        else:
            now = time.monotonic()
            if self.last_predict_time is None:
                dt = self.nominal_dt
            else:
                dt = now - self.last_predict_time
                dt = float(np.clip(dt, 1e-4, 0.2))
            self.last_predict_time = now

        heading_rad = self._heading_to_radians(heading)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        ax_body = ax or 0.0
        ay_body = ay or 0.0
        ax_world = ax_body * cos_h - ay_body * sin_h
        ay_world = ax_body * sin_h + ay_body * cos_h

        self._predict(ax_world, ay_world, dt)
        if gps_lat is not None and gps_lon is not None and self._is_new_gps_fix(gps_lat, gps_lon):
            gps_velocity = None
            if self.use_gps_velocity and self.last_gps_fix is not None:
                gps_velocity = self._gps_velocity_from_fix_pair(*self.last_gps_fix, gps_lat, gps_lon, now)

            if gps_velocity is None:
                self._update_gps_position(gps_lat, gps_lon)
            else:
                self._update_gps_full(gps_lat, gps_lon, *gps_velocity)

            self.last_gps_fix = (float(gps_lat), float(gps_lon), float(now))
        self.lat, self.lon, self.vx, self.vy = self.ekf.x
        return self.lat, self.lon, self.vx, self.vy

    def run_threaded(self, ax, ay, heading, gps_lat, gps_lon):
        return self.run(ax, ay, heading, gps_lat, gps_lon)

    def update(self):
        while self.running:
            time.sleep(self.nominal_dt)

    def shutdown(self):
        self.running = False
