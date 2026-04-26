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

    # Heart beat
    heartbeat= HealthCheck("192.168.12.25", 6000) # aryamaan has ip 100
    # just returns true rn
    V.add(heartbeat, inputs=[], outputs=["safety/heartbeat"])

    # UART driver
    uart = UART_backup_driver("/dev/ttyACM2")
    V.add(uart, inputs=["controls/throttle", "controls/steering", "safety/heartbeat"], outputs=[], threaded=False)
    #uart = UART_VERSION2()
    #V.add(uart, inputs=[], outputs=[])


    V.start(rate_hz=1000) # Remove this once we do more.
