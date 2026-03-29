import threading
import time
import numpy as np
import matplotlib.pyplot as plt

from parts.fake_imu import FakeImu
from parts.fake_gps import FakeGps
from parts.ekf_localizer import EKFLocalizer

R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


def latlon_to_local_m(lat, lon, lat0, lon0):
    cos_lat0 = np.cos(np.radians(lat0))
    cos_lat0 = np.clip(cos_lat0, 1e-6, None)
    y_north = (lat - lat0) / RAD2DEG * R_EARTH
    x_east = (lon - lon0) / RAD2DEG * (R_EARTH * cos_lat0)
    return x_east, y_north


def main():
    imu_rate_hz = 100
    gps_rate_hz = 10
    duration_s = 20.0

    origin_lat = 37.7749
    origin_lon = -122.4194

    fake_imu = FakeImu(imu_rate=imu_rate_hz)
    fake_gps = FakeGps(origin_lat=origin_lat, origin_lon=origin_lon, gps_rate=gps_rate_hz)
    ekf = EKFLocalizer(
        init_lat=origin_lat,
        init_lon=origin_lon,
        imu_rate=imu_rate_hz,
        gps_rate=gps_rate_hz,
    )

    imu_thread = threading.Thread(target=fake_imu.update, daemon=True)
    gps_thread = threading.Thread(target=fake_gps.update, daemon=True)
    imu_thread.start()
    gps_thread.start()

    gt_x = []
    gt_y = []
    ekf_x = []
    ekf_y = []

    gps_points_x = []
    gps_points_y = []
    gps_vec_x0 = []
    gps_vec_y0 = []
    gps_vec_u = []
    gps_vec_v = []

    imu_vec_x0 = []
    imu_vec_y0 = []
    imu_vec_u = []
    imu_vec_v = []

    prev_gps_x = None
    prev_gps_y = None

    loop_dt = 1.0 / imu_rate_hz
    t0 = time.monotonic()
    k = 0

    try:
        while (time.monotonic() - t0) < duration_s:
            ax, ay, heading_deg = fake_imu.run_threaded()
            gps_lat, gps_lon = fake_gps.run_threaded()

            lat, lon, vx, vy = ekf.run(ax, ay, heading_deg, gps_lat, gps_lon)

            with fake_imu.lock:
                x_true = float(fake_imu.x_east)
                y_true = float(fake_imu.y_north)

            x_est, y_est = latlon_to_local_m(lat, lon, origin_lat, origin_lon)

            gt_x.append(x_true)
            gt_y.append(y_true)
            ekf_x.append(x_est)
            ekf_y.append(y_est)

            if gps_lat is not None and gps_lon is not None:
                x_gps, y_gps = latlon_to_local_m(gps_lat, gps_lon, origin_lat, origin_lon)
                gps_points_x.append(x_gps)
                gps_points_y.append(y_gps)

                if prev_gps_x is not None and prev_gps_y is not None:
                    gps_vec_x0.append(prev_gps_x)
                    gps_vec_y0.append(prev_gps_y)
                    gps_vec_u.append(x_gps - prev_gps_x)
                    gps_vec_v.append(y_gps - prev_gps_y)

                prev_gps_x = x_gps
                prev_gps_y = y_gps

            if k % 10 == 0:
                heading_rad = np.radians(heading_deg)
                cos_h = np.cos(heading_rad)
                sin_h = np.sin(heading_rad)
                ax_world = ax * cos_h - ay * sin_h
                ay_world = ax * sin_h + ay * cos_h
                imu_vec_x0.append(x_est)
                imu_vec_y0.append(y_est)
                imu_vec_u.append(ax_world)
                imu_vec_v.append(ay_world)

            if k % imu_rate_hz == 0:
                print(
                    f"t={time.monotonic()-t0:5.1f}s "
                    f"lat={lat:.8f} lon={lon:.8f} vx={vx:.3f} vy={vy:.3f} yaw={heading_deg:.1f}"
                )

            k += 1
            time.sleep(loop_dt)

    finally:
        fake_imu.shutdown()
        fake_gps.shutdown()
        imu_thread.join(timeout=0.2)
        gps_thread.join(timeout=0.2)

    plt.figure(figsize=(10, 8))
    plt.plot(gt_x, gt_y, label="Ground Truth", linewidth=2)
    plt.plot(ekf_x, ekf_y, label="EKF Prediction", linewidth=2)

    if gps_points_x:
        plt.scatter(gps_points_x, gps_points_y, s=12, label="GPS Samples", alpha=0.8)

    if gps_vec_x0:
        plt.quiver(
            gps_vec_x0,
            gps_vec_y0,
            gps_vec_u,
            gps_vec_v,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.003,
            alpha=0.65,
            label="GPS Delta Vectors",
        )

    if imu_vec_x0:
        plt.quiver(
            imu_vec_x0,
            imu_vec_y0,
            imu_vec_u,
            imu_vec_v,
            angles="xy",
            scale_units="xy",
            scale=0.6,
            width=0.003,
            alpha=0.6,
            label="IMU Accel Vectors",
        )

    plt.axis("equal")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title("Ground Truth vs EKF with GPS and IMU Vectors")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig("fake_data_test_plot.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
