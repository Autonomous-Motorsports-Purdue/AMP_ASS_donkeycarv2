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
    def __init__(self, port="COM5", baud=115200, debug=False):
        self.port = port
        self.baud = baud
        self.debug = debug
        self.yaw_rate = 0.0
        self.yaw = 0.0
        self.ax = 0.0
        self.ay = 0.0

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
            return self.yaw_rate, self.yaw, 0, 0

        line = self.ser_io.readline().strip()
        cal = self.ser_io.readline().strip()
        newline = self.ser_io.readline().strip()    
        # print(f"Raw IMU line: '{line}'")  # Debug print of raw line
        # print(f"Raw IMU cal line: '{cal}'")  # Debug print of calibration line
        # print(f"Raw IMU newline: '{newline}'")  # Debug print of newline
        if "Cal" in line:
            print("Recieved Calibration Packet, ret 0's")
            # print(f"Received calibration packet: {line}")
            return 0, 0, 0, 0
        if not line:
            print("Error reading from IMU")
            return self.yaw_rate, self.yaw, 0, 0
        try:
            parts = [p.strip() for p in line.split(",")]
            # print(f"Raw IMU parts: {parts}")
            parts = [p[3:] if ":" in p else p for p in parts]  # remove 'gx=', etc.
            # print(f"Parsed IMU parts: {parts}")
            ox, oy, oz, gx, gy, gz, ax, ay, az = [float(v) for v in parts]

            self.yaw_rate = math.radians(gz)
            self.yaw = math.radians(ox)

            self.ax, self.ay = ax, ay

            if self.debug:
                print(f"Yaw rate: {self.yaw_rate:.4f} rad/s")

        except Exception as e:
            if self.debug:
                print(f"[IMUPart] Parse error: {e} | Line: {line}")

        # print(f"Returning IMU values: yaw_rate={self.yaw_rate:.4f}, yaw={self.yaw:.4f}, ax={self.ax:.4f}, ay={self.ay:.4f}")
        return self.yaw_rate, self.yaw, self.ax, self.ay
