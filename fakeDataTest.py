import donkeycar.donkeycar as dk
import time

from parts.fake_imu import FakeImu
from parts.fake_gps import FakeGps
from parts.ekf_localizer import EKFLocalizer


class EKFPrinter:
    def __init__(self, print_rate_hz=10):
        self.print_dt = 1.0 / float(print_rate_hz)
        self.last_print = 0.0

    def run(self, lat, lon, vx, vy):
        now = time.monotonic()
        if now - self.last_print >= self.print_dt:
            self.last_print = now
            print(f"ekf lat={lat:.8f} lon={lon:.8f} vx={vx:.3f} vy={vy:.3f}")


if __name__ == "__main__":
    imu_rate_hz = 100
    gps_rate_hz = 10
    origin_lat = 37.7749
    origin_lon = -122.4194

    V = dk.vehicle.Vehicle()
    print("starting")

    fake_imu = FakeImu(imu_rate=imu_rate_hz)
    fake_gps = FakeGps(origin_lat=origin_lat, origin_lon=origin_lon, gps_rate=gps_rate_hz)
    ekf = EKFLocalizer(
        init_lat=origin_lat,
        init_lon=origin_lon,
        imu_rate=imu_rate_hz,
        gps_rate=gps_rate_hz,
    )

    V.add(fake_imu, outputs=["imu/accel_x", "imu/accel_y"], threaded=True)
    V.add(fake_gps, outputs=["gps/lat", "gps/lon"], threaded=True)
    V.add(
        ekf,
        inputs=["imu/accel_x", "imu/accel_y", "gps/lat", "gps/lon"],
        outputs=["ekf/lat", "ekf/lon", "ekf/vx", "ekf/vy"],
        threaded=False,
    )
    V.add(
        EKFPrinter(print_rate_hz=10),
        inputs=["ekf/lat", "ekf/lon", "ekf/vx", "ekf/vy"],
        outputs=[],
        threaded=False,
    )

    V.start(rate_hz=imu_rate_hz)
    
