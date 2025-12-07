import donkeycar as dk

from parts.controller import MPC_Part, AngVel_To_Steering_PID
from parts.imu import IMU
from parts.uart_backup import UART_backup_driver

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    
    V.add(IMU(), inputs=[], outputs=['yaw_rate', 'yaw'])
    V.add(AngVel_To_Steering_PID(3,.1, 1, 1, 1, 35), inputs=['yaw_rate'], outputs=['steering','throttle'])
    V.add(UART_backup_driver("/dev/ttyACM0"), inputs=['throttle', 'steering', 'yaw_rate'], outputs=[])
    V.start(rate_hz=30)
