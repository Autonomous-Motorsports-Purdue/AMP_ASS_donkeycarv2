import donkeycar as dk

from parts.uart_backup import UART_backup_driver
from parts.imu import IMU
from parts.gps import GPS
from parts.controller import MPC_Part, ClosedLoopController
from parts.ekf_localizer import EKFLocalizer
from parts.gps_to_xy import GPS_to_xy
from parts.logger2 import Logger2

import numpy as np

'''
checklist if not working

1. verify IMU, GPS, Main PCB UART are plugged in.
2. verify ports for each. should be /dev/tty* or /dev/usb*. unplug to test
'''

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()

    # UART driver
    uart = UART_backup_driver("/dev/ttyACM0")
    V.add(uart, inputs=["controls/throttle", "controls/steering"], outputs=[], threaded=False)

    # IMU
    imu = IMU()
    V.add(imu, inputs=[], outputs=["yaw_rate", "yaw", "a_x", "a_y"], threaded=False) # TODO: make this threaded

    # GPS
    gps = GPS()
    V.add(GPS(), inputs=[], outputs=['lat_raw', 'lon_raw', 'alt', 'fix', 'corr_age', 'hdop', 'sat_count'], threaded=True)

    # EKF Localizer
    ekf_localizer = EKFLocalizer(init_lat=40.427281, init_lon=-86.918915, imu_rate=10, gps_rate=4)
    V.add(ekf_localizer, inputs=["a_x", "a_y", "yaw", "lat_raw", "lon_raw"], outputs=["lat", "lon", "v_x", "v_y"], threaded=True)

    # GPS to XY
    gps_to_xy = GPS_to_xy(ref_lat_deg=40.427281, ref_lon_deg=-86.918915) # BIDC as origin
    V.add(gps_to_xy, inputs=["lat", "lon"], outputs=["x", "y"], threaded=False)

    # MPC Controller
    mpc_controller = MPC_Part(path_csv=None, horizon=2.0, dt_mpc=0.1, wheelbase=1.000506, max_steer=np.radians(35))
    V.add(mpc_controller, inputs=["x", "y", "yaw"], outputs=["controls/desired_yaw_rate", "controls/desired_throttle"], threaded=True)

    # Closed loop IMU controller
    closed_loop_controller = ClosedLoopController(
        kp=0.5, ki=0.0, kd=0.0, # TODO: Tune these parameters.
        wheelbase=1.000506, max_steering_deg=35, steering_scale=1/35.0
    )
    V.add(closed_loop_controller, inputs=["controls/desired_yaw_rate", "yaw_rate", "controls/desired_throttle"], outputs=["controls/steering", "controls/throttle"], threaded=True)

    logger = Logger2()
    V.add(logger, inputs=["lat", "lon", "v_x", "v_y", "yaw", "a_x", "a_y", "x", "y", "controls/throttle", "controls/steering"], outputs=[], threaded=False)

    V.start(rate_hz=100, max_loop_count=1000) # Remove this once we do more.