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
    def __init__(self, port="/dev/ttyACM0", baud=115200, debug=False):
        self.port = port
        self.baud = baud
        self.debug = debug
        self.yaw_rate = 0.0
        self.yaw = 0.0
        self.iter = 0

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
            return self.yaw_rate, self.yaw

        line = self.ser_io.readline().strip()
        self.iter += 1
        #if self.iter % 10 == 0:
           #print(f"DEBUG: line={line}")
        if not line:
            return self.yaw_rate, self.yaw
        ax = 0
        ay = 0
        ox = 0
        gx = 0
        print("here")
        try:
            parts = [p.strip() for p in line.split(",")]
            print(line)
            print(parts)
            parts = [p[3:] if ":" in p else p for p in parts]  # remove 'gx=', etc.

            print(parts)
            ox, oy, oz, gx, gy, gz, ax, ay, az = [float(v) for v in parts]
            print(f"yaw: {ox} x accel: {ax} y accel: {ay}")

            self.yaw_rate = gx
            self.yaw = ox

            #if self.debug:
            #    print(f"Yaw rate: {self.yaw_rate:.4f} rad/s")

        except Exception as e:
            print(f"[IMUPart] Parse error: {e} | Line: {line}")

        #if (abs(self.yaw_rate) > 0.01):
       # if self.iter % 30 == 0:
           #print(f"yaw_rate={self.yaw_rate}, yaw={self.yaw}")
        #return self.yaw_rate, self.yaw

        return ax, ay, ox, gx
