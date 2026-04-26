import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from gps_pid_pseudo_sim import SimplePIDSimConfig, run_simple_pid_sim
from gps_pure_pursuit_pseudo_sim import PurePursuitSimConfig, run_pure_pursuit_sim
from gps_mpc_pseudo_sim import SimConfig, load_raceline_track, run_pseudo_sim, write_xy_sidecar
from parts.controller import ClosedLoopController, MPC_Part
from parts.ekf_localizer import EKFLocalizer
from parts.gps_pure_pursuit_waypoint import GPSPurePursuitWaypointGenerator, R_EARTH, RAD2DEG
from parts.simple_gps_pid import R_EARTH_M, SimpleGPSPIDController
from parts.uart_backup import UART_backup_driver


class DummySerial:
    def __init__(self, *args, **kwargs):
        self.writes = []
        self.is_open = True

    def write(self, payload):
        self.writes.append(payload)

    def close(self):
        self.is_open = False


class TestMPCPathLoading(unittest.TestCase):
    def test_loads_named_xy_csv(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "track_xy.csv"
            path.write_text(
                "x_m,y_m, psi_rad\n"
                "0.0,0.0,0.0\n"
                "1.0,0.0,0.0\n"
                "2.0,0.0,0.0\n",
                encoding="utf-8",
            )

            mpc_part = MPC_Part(path_csv=path, horizon=2, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35))

        np.testing.assert_allclose(
            mpc_part.path,
            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
        )

    def test_loads_headerless_xy_csv(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "track_xy.csv"
            path.write_text(
                "0.0,0.0,0.1,123.0\n"
                "1.0,2.0,0.2,456.0\n"
                "2.0,4.0,0.3,789.0\n",
                encoding="utf-8",
            )

            mpc_part = MPC_Part(path_csv=path, horizon=2, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35))

        np.testing.assert_allclose(
            mpc_part.path,
            np.array([[0.0, 0.0, 0.1], [1.0, 2.0, 0.2], [2.0, 4.0, 0.3]]),
        )


class TestEKFLocalizer(unittest.TestCase):
    def test_heading_unit_rad_rotates_body_accel_correctly(self):
        ekf = EKFLocalizer(40.0, -86.0, imu_rate=10, gps_rate=4, heading_unit="rad")

        _, _, vx, vy = ekf.run(1.0, 0.0, 1.0, None, None)

        self.assertAlmostEqual(vx, math.cos(1.0) * 0.1, places=6)
        self.assertAlmostEqual(vy, math.sin(1.0) * 0.1, places=6)

    def test_heading_unit_deg_keeps_fake_sensor_compatibility(self):
        ekf = EKFLocalizer(40.0, -86.0, imu_rate=10, gps_rate=4, heading_unit="deg")

        _, _, vx, vy = ekf.run(1.0, 0.0, 90.0, None, None)

        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 0.1, places=6)

    def test_gps_velocity_update_tracks_straight_motion(self):
        from parts.gps_to_xy import GPS_to_xy

        origin_lat = 40.0
        origin_lon = -86.0
        converter = GPS_to_xy(origin_lat, origin_lon)
        ekf = EKFLocalizer(
            origin_lat,
            origin_lon,
            imu_rate=10,
            gps_rate=4,
            heading_unit="rad",
            fixed_dt=0.1,
        )

        true_x = 0.0
        true_speed = 5.0
        gps_period_steps = 3
        lat = origin_lat
        lon = origin_lon

        for step in range(30):
            true_x += true_speed * 0.1
            if step % gps_period_steps == 0:
                lat, lon = converter.to_latlon(true_x, 0.0)
            else:
                lat, lon = None, None

            est_lat, est_lon, est_vx, est_vy = ekf.run(0.0, 0.0, 0.0, lat, lon)

        est_x, est_y = converter.to_xy(est_lat, est_lon)
        self.assertAlmostEqual(est_x, true_x, delta=1.0)
        self.assertAlmostEqual(est_y, 0.0, delta=0.5)
        self.assertAlmostEqual(est_vx, true_speed, delta=1.5)
        self.assertAlmostEqual(est_vy, 0.0, delta=1.0)


class TestClosedLoopController(unittest.TestCase):
    def test_small_yaw_rate_error_does_not_immediately_saturate_steering(self):
        controller = ClosedLoopController(
            kp=0.5,
            ki=0.0,
            kd=0.0,
            wheelbase=1.000506,
            steering_scale=1 / 35.0,
            max_steering_deg=35,
            speed_to_throttle_scale=0.05,
            max_throttle=0.4,
        )

        steering, throttle = controller.run(0.001, 0.0, 7.5)

        self.assertLess(abs(steering), 0.1)
        self.assertAlmostEqual(throttle, 0.375, places=6)

    def test_throttle_output_is_scaled_and_clamped(self):
        controller = ClosedLoopController(
            kp=0.5,
            ki=0.0,
            kd=0.0,
            wheelbase=1.000506,
            steering_scale=1 / 35.0,
            max_steering_deg=35,
            speed_to_throttle_scale=0.05,
            max_throttle=0.4,
        )

        _, throttle = controller.run(0.0, 0.0, 50.0)

        self.assertEqual(throttle, 0.4)


class TestUARTBackupDriver(unittest.TestCase):
    @patch("parts.uart_backup.serial.Serial", return_value=DummySerial())
    def test_warmup_holds_zero_and_later_uses_requested_inputs(self, _mock_serial):
        driver = UART_backup_driver("/dev/null", warmup_iterations=2, throttle_scale=127.0, steering_scale=64.0)

        driver.run(0.5, 0.25, True)
        self.assertEqual(driver.ser.writes[-1], b"0,127\r")

        driver.run(0.5, 0.25, True)
        self.assertEqual(driver.ser.writes[-1], b"0,127\r")

        driver.run(0.5, 0.25, True)
        self.assertEqual(driver.ser.writes[-1], b"63,143\r")


class TestPseudoSimHelpers(unittest.TestCase):
    def test_load_raceline_and_generate_xy_sidecar(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            csv_path.write_text(
                "# s_m, x_m, y_m,longitude,latitude, psi_rad, kappa_radpm, vx_mps, ax_mps2\n"
                "0.0, 10.0, 20.0, -86.0, 40.0, 0.0, 0.0, 7.5, 0.0\n"
                "1.0, 11.0, 20.0, -85.9999882736, 40.0, 0.0, 0.0, 7.5, 0.0\n"
                "2.0, 12.0, 20.0, -85.9999765472, 40.0, 0.0, 0.0, 7.5, 0.0\n",
                encoding="utf-8",
            )

            track = load_raceline_track(csv_path)
            xy_path = write_xy_sidecar(track, overwrite=True)

            self.assertEqual(track.lat.shape[0], 3)
            self.assertTrue(xy_path.exists())
            np.testing.assert_allclose(track.x[0], 0.0, atol=1e-6)
            np.testing.assert_allclose(track.y[0], 0.0, atol=1e-6)

    def test_pseudo_sim_smoke(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            csv_path.write_text(
                "# s_m, x_m, y_m,longitude,latitude, psi_rad, kappa_radpm, vx_mps, ax_mps2\n"
                "0.0, 0.0, 0.0, -86.0, 40.0, 0.0, 0.0, 7.5, 0.0\n"
                "2.0, 2.0, 0.0, -85.9999765472, 40.0, 0.0, 0.0, 7.5, 0.0\n"
                "4.0, 4.0, 0.0, -85.9999530944, 40.0, 0.0, 0.0, 7.5, 0.0\n",
                encoding="utf-8",
            )

            track = load_raceline_track(csv_path)
            xy_path = write_xy_sidecar(track, overwrite=True)
            logs, summary = run_pseudo_sim(
                track,
                xy_path,
                SimConfig(
                    dt=0.1,
                    gps_rate_hz=4.0,
                    laps=1,
                    localization_mode="ekf",
                    wheelbase_m=1.000506,
                    max_steer_deg=35.0,
                    steering_kp=0.5,
                    steering_ki=0.0,
                    steering_kd=0.0,
                    speed_to_throttle_scale=0.05,
                    max_throttle=0.40,
                    throttle_speed_lag_s=0.75,
                    steering_lag_s=0.15,
                    gps_noise_m=0.0,
                    yaw_noise_deg=0.0,
                    yaw_rate_noise_degps=0.0,
                    accel_noise_mps2=0.0,
                    initial_speed_mps=7.5,
                    ekf_gps_pos_std_m=2.5,
                    ekf_gps_vel_std_mps=1.5,
                    ekf_use_gps_velocity=True,
                    seed=1,
                ),
            )

            self.assertGreater(len(logs["time_s"]), 0)
            self.assertTrue(np.isfinite(logs["steering_cmd"]).all())
            self.assertLess(summary["mean_cross_track_error_m"], 2.0)


class TestGPSPurePursuitStack(unittest.TestCase):
    def test_waypoint_generator_outputs_forward_target(self):
        origin_lat = 40.0
        origin_lon = -86.0
        cos_lat = math.cos(math.radians(origin_lat))

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            rows = ["latitude,longitude"]
            for x_m in [0.0, 2.0, 4.0, 6.0, 8.0]:
                lon = origin_lon + (x_m / (R_EARTH * cos_lat)) * RAD2DEG
                rows.append(f"{origin_lat},{lon}")
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            generator = GPSPurePursuitWaypointGenerator(csv_path, lookahead_m=4.0, verbose=False)
            waypoint = generator.run(origin_lat, origin_lon)

        self.assertIsNotNone(waypoint)
        self.assertGreater(waypoint[0], 0.0)
        self.assertAlmostEqual(waypoint[1], 0.0, delta=0.5)

    def test_pure_pursuit_pseudo_sim_smoke(self):
        origin_lat = 40.0
        origin_lon = -86.0
        cos_lat = math.cos(math.radians(origin_lat))

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            rows = ["latitude,longitude"]
            radius_m = 20.0
            for theta in np.linspace(0.0, 2.0 * math.pi, 81)[:-1]:
                x_m = radius_m * math.cos(theta)
                y_m = radius_m * math.sin(theta)
                lat = origin_lat + (y_m / R_EARTH) * RAD2DEG
                lon = origin_lon + (x_m / (R_EARTH * cos_lat)) * RAD2DEG
                rows.append(f"{lat},{lon}")
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            generator, logs, summary = run_pure_pursuit_sim(
                csv_path,
                PurePursuitSimConfig(
                    dt=0.1,
                    laps=1,
                    gps_noise_m=0.0,
                    lookahead_m=6.0,
                    throttle_cmd=0.35,
                    throttle_to_speed_mps=20.0,
                    speed_response_s=0.5,
                    seed=1,
                ),
            )

        self.assertGreater(len(logs["time_s"]), 0)
        self.assertLess(summary["mean_cross_track_error_m"], 3.0)
        self.assertLess(summary["max_cross_track_error_m"], 5.0)


class TestSimpleGPSPIDStack(unittest.TestCase):
    def test_simple_pid_controller_outputs_small_steering_on_straight_track(self):
        origin_lat = 40.0
        origin_lon = -86.0
        cos_lat = math.cos(math.radians(origin_lat))

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            rows = ["latitude,longitude"]
            for x_m in [0.0, 2.0, 4.0, 6.0, 8.0]:
                lon = origin_lon + (x_m / (R_EARTH_M * cos_lat)) * (180.0 / math.pi)
                rows.append(f"{origin_lat},{lon}")
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            controller = SimpleGPSPIDController(csv_path, fixed_dt=0.05, verbose=False)
            steering, throttle = controller.run(origin_lat, origin_lon, 0.0)

        self.assertAlmostEqual(steering, 0.0, delta=0.2)
        self.assertGreater(throttle, 0.0)

    def test_simple_pid_pseudo_sim_smoke(self):
        origin_lat = 40.0
        origin_lon = -86.0
        cos_lat = math.cos(math.radians(origin_lat))

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "track.csv"
            rows = ["latitude,longitude"]
            radius_m = 20.0
            for theta in np.linspace(0.0, 2.0 * math.pi, 81)[:-1]:
                x_m = radius_m * math.cos(theta)
                y_m = radius_m * math.sin(theta)
                lat = origin_lat + (y_m / R_EARTH_M) * (180.0 / math.pi)
                lon = origin_lon + (x_m / (R_EARTH_M * cos_lat)) * (180.0 / math.pi)
                rows.append(f"{lat},{lon}")
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            controller, logs, summary = run_simple_pid_sim(
                csv_path,
                SimplePIDSimConfig(
                    dt=0.05,
                    gps_rate_hz=5.0,
                    laps=1,
                    gps_noise_m=0.0,
                    yaw_noise_deg=0.0,
                    lookahead_m=6.0,
                    throttle_cmd=0.22,
                    throttle_to_speed_mps=20.0,
                    speed_response_s=0.5,
                    wheelbase_m=1.000506,
                    max_steer_deg=35.0,
                    kp=1.0,
                    ki=0.0,
                    kd=0.15,
                    seed=1,
                ),
            )

        self.assertGreater(len(logs["time_s"]), 0)
        self.assertEqual(summary["completed_laps"], 1.0)
        self.assertLess(summary["mean_cross_track_error_m"], 2.0)
        self.assertLess(summary["max_cross_track_error_m"], 2.0)


class TestOfflineControllerChain(unittest.TestCase):
    def test_chain_returns_finite_commands(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "track_xy.csv"
            path.write_text(
                "x_m,y_m, psi_rad\n"
                "0.0,0.0,0.0\n"
                "2.0,0.0,0.0\n"
                "4.0,0.0,0.0\n",
                encoding="utf-8",
            )

            mpc_part = MPC_Part(path_csv=path, horizon=2, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35))
            controller = ClosedLoopController(
                kp=0.5,
                ki=0.0,
                kd=0.0,
                wheelbase=1.000506,
                steering_scale=1 / 35.0,
                max_steering_deg=35,
                speed_to_throttle_scale=0.05,
                max_throttle=0.4,
            )

            desired_yaw_rate, desired_speed = mpc_part.run(0.0, 0.0, 0.0)
            steering, throttle = controller.run(desired_yaw_rate, 0.0, desired_speed)

        self.assertTrue(math.isfinite(desired_yaw_rate))
        self.assertTrue(math.isfinite(steering))
        self.assertTrue(math.isfinite(throttle))
        self.assertLessEqual(abs(steering), 1.0)
        self.assertLessEqual(abs(throttle), 0.4)


class TestDonkeyVehicleLoop(unittest.TestCase):
    def test_donkey_vehicle_runs_controller_chain(self):
        import donkeycar as dk

        class StaticSource:
            def __init__(self, lat, lon):
                self.lat = lat
                self.lon = lon

            def run(self):
                return 0.0, 0.0, 0.0, 0.0, self.lat, self.lon

        class Sink:
            def __init__(self):
                self.records = []

            def run(self, x, y, desired_yaw_rate, desired_speed, steering, throttle):
                self.records.append((x, y, desired_yaw_rate, desired_speed, steering, throttle))

        from parts.gps_to_xy import GPS_to_xy

        origin_lat = 40.42773956363986
        origin_lon = -86.91873017427434

        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "track_xy.csv"
            path.write_text(
                "x_m,y_m, psi_rad\n"
                "0.0,0.0,0.0\n"
                "2.0,0.0,0.0\n"
                "4.0,0.0,0.0\n",
                encoding="utf-8",
            )

            vehicle = dk.vehicle.Vehicle()
            vehicle.add(
                StaticSource(origin_lat, origin_lon),
                inputs=[],
                outputs=["yaw_rate", "yaw", "a_x", "a_y", "lat_raw", "lon_raw"],
            )
            vehicle.add(
                EKFLocalizer(origin_lat, origin_lon, imu_rate=10, gps_rate=4, heading_unit="rad"),
                inputs=["a_x", "a_y", "yaw", "lat_raw", "lon_raw"],
                outputs=["lat", "lon", "v_x", "v_y"],
            )
            vehicle.add(
                GPS_to_xy(ref_lat_deg=origin_lat, ref_lon_deg=origin_lon),
                inputs=["lat", "lon"],
                outputs=["x", "y"],
            )
            vehicle.add(
                MPC_Part(path_csv=path, horizon=2, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35)),
                inputs=["x", "y", "yaw"],
                outputs=["controls/desired_yaw_rate", "controls/desired_speed_mps"],
            )
            vehicle.add(
                ClosedLoopController(
                    kp=0.5,
                    ki=0.0,
                    kd=0.0,
                    wheelbase=1.000506,
                    steering_scale=1 / 35.0,
                    max_steering_deg=35,
                    speed_to_throttle_scale=0.05,
                    max_throttle=0.4,
                ),
                inputs=["controls/desired_yaw_rate", "yaw_rate", "controls/desired_speed_mps"],
                outputs=["controls/steering", "controls/throttle"],
            )
            sink = Sink()
            vehicle.add(
                sink,
                inputs=[
                    "x",
                    "y",
                    "controls/desired_yaw_rate",
                    "controls/desired_speed_mps",
                    "controls/steering",
                    "controls/throttle",
                ],
                outputs=[],
            )

            vehicle.start(rate_hz=20, max_loop_count=3)

        self.assertGreaterEqual(len(sink.records), 3)
        x, y, desired_yaw_rate, desired_speed, steering, throttle = sink.records[-1]
        self.assertAlmostEqual(float(x), 0.0, places=6)
        self.assertAlmostEqual(float(y), 0.0, places=6)
        self.assertAlmostEqual(float(desired_yaw_rate), 0.0, places=6)
        self.assertAlmostEqual(float(desired_speed), 7.5, places=6)
        self.assertAlmostEqual(float(steering), 0.0, places=6)
        self.assertAlmostEqual(float(throttle), 0.375, places=6)


if __name__ == "__main__":
    unittest.main()
