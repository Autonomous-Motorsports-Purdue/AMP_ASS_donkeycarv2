import donkeycar as dk

from parts.uart_backup import UART_backup_driver
#from parts.uart_backup2 import UART_VERSION2
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
    V = dk.vehicle.Vehicle()

    # IMU
    #TODO remove the delay(1000) in imu src firmware
    imu = IMU("/dev/ttyACM2")
    V.add(imu, inputs=[], outputs=["yaw_rate", "yaw", "a_x", "a_y"], threaded=False) # TODO: make this threaded



    V.start(rate_hz=1000) # Remove this once we do more.