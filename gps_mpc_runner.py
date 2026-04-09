import donkeycar as dk
from pathlib import Path

from parts.uart_backup import UART_backup_driver
from parts.imu import IMU
from parts.gps import GPS
from parts.controller import MPC_Part, ClosedLoopController
from parts.ekf_localizer import EKFLocalizer
from parts.gps_to_xy import GPS_to_xy
from parts.logger2 import Logger2
from parts.health_check import HealthCheck

import numpy as np

'''
checklist if not working

1. verify IMU, GPS, Main PCB UART are plugged in.
2. verify ports for each. should be /dev/tty* or /dev/usb*. unplug to test
'''
import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help="gps waypoint csv file name")
    parser.add_argument("--max-loop-count", type=int, default=None, help="optional loop limit for bench testing")
    args = parser.parse_args()

    waypoint_path = Path(args.file_name).expanduser().resolve()
    if not waypoint_path.exists():
        raise FileNotFoundError(f"Waypoint CSV not found: {waypoint_path}")

    csv_xy_path = waypoint_path.with_name(f"{waypoint_path.stem}_xy{waypoint_path.suffix}")
    if not csv_xy_path.exists():
        raise FileNotFoundError(f"XY waypoint CSV not found: {csv_xy_path}")

    data = np.genfromtxt(waypoint_path, delimiter=',', skip_header=1, dtype=float, encoding='utf-8')
    ref_lat0 = data[0,0]
    ref_lon0 = data[0,1]

    V = dk.vehicle.Vehicle()

    # Heart beat
    heartbeat= HealthCheck("192.168.12.25", 6000) # aryamaan has ip 100
    V.add(heartbeat, inputs=[], outputs=["safety/heartbeat"])

    # UART driver
    uart = UART_backup_driver("/dev/ttyACM0")
    V.add(uart, inputs=["controls/throttle", "controls/steering", "safety/heartbeat"], outputs=[], threaded=False)

    # IMU
    imu = IMU()
    V.add(imu, inputs=[], outputs=["yaw_rate", "yaw", "a_x", "a_y"], threaded=False) # TODO: make this threaded

    # GPS
    gps = GPS()
    V.add(gps, inputs=[], outputs=['lat_raw', 'lon_raw', 'alt', 'fix', 'corr_age', 'hdop', 'sat_count'], threaded=True)

    # EKF Localizer
    ekf_localizer = EKFLocalizer(init_lat=ref_lat0, init_lon=ref_lon0, imu_rate=10, gps_rate=4, heading_unit="rad")
    V.add(ekf_localizer, inputs=["a_x", "a_y", "yaw", "lat_raw", "lon_raw"], outputs=["lat", "lon", "v_x", "v_y"], threaded=False)

    # GPS to XY
    gps_to_xy = GPS_to_xy(ref_lat_deg=ref_lat0, ref_lon_deg=ref_lon0) # BIDC as origin
    V.add(gps_to_xy, inputs=["lat", "lon"], outputs=["x", "y"], threaded=False)

    # MPC Controller
    mpc_controller = MPC_Part(path_csv=csv_xy_path, horizon=2, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35))
    V.add(mpc_controller, inputs=["x", "y", "yaw"], outputs=["controls/desired_yaw_rate", "controls/desired_speed_mps"], threaded=False)

    # Closed loop IMU controller
    closed_loop_controller = ClosedLoopController(
        kp=0.5, ki=0.0, kd=0.0, # TODO: Tune these parameters.
        wheelbase=1.000506,
        max_steering_deg=35,
        steering_scale=1/35.0,
        speed_to_throttle_scale=0.05,
        max_throttle=0.40,
    )
    V.add(closed_loop_controller, inputs=["controls/desired_yaw_rate", "yaw_rate", "controls/desired_speed_mps"], outputs=["controls/steering", "controls/throttle"], threaded=False)

    #logger = Logger2()
    #V.add(logger, inputs=["lat", "lon", "v_x", "v_y", "yaw", "a_x", "a_y", "x", "y", "controls/throttle", "controls/steering"], outputs=[], threaded=False)

    start_kwargs = {"rate_hz": 100}
    if args.max_loop_count is not None:
        start_kwargs["max_loop_count"] = args.max_loop_count
    V.start(**start_kwargs)
