import serial
import io
import math
import os

class IMU:
    """
    DonkeyCar IMU part for 6-value serial output:
    gx, gy, gz, ax, ay, az
    gz = yaw rate (deg/s)
    """
    def __init__(self, port="/dev/ttyUSB0", baud=115200, debug=False):
        self.port = port
        self.baud = baud
        self.debug = debug
        self.yaw_rate = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.ax = self.ay = self.az = 0.0

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
            print(f"[IMUPart] Connected to {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[IMUPart] Could not open serial port: {e}")
            self.ser = None
            self.ser_io = None

    def run(self):
        """Read a line of 6 comma-separated IMU values and return yaw_rate."""
        if not self.ser_io:
            return self.yaw_rate

        line = self.ser_io.readline().strip()
        if not line:
            return self.yaw_rate

        try:
            parts = [p.strip() for p in line.split(",")]
            parts = [p[3:] if "=" in p else p for p in parts]  # remove 'gx=', etc.
            ax, ay, az, gx, gy, gz = [float(v) for v in parts]

            self.yaw_rate = math.radians(gz)

            if self.debug:
                print(f"Yaw rate: {self.yaw_rate:.4f} rad/s")

        except Exception as e:
            if self.debug:
                print(f"[IMUPart] Parse error: {e} | Line: {line}")

        return self.yaw_rate