import serial
import time

# ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
# time.sleep(2)
# # ser.write(f"0,0\r".encode("ascii"))
# # ser.flush()

# while 1: 
#     for i in range(0,255, 5):
#         time.sleep(0.1)
#         ser.write(f"0,{i}\r".encode("ascii"))
#         ser.flush()
#     for i in range(255,0, -5):
#         time.sleep(0.1)
#         ser.write(f"0,{i}\r".encode("ascii"))
#         ser.flush()

class UART_VERSION2:
    def __init__(self):
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        time.sleep(2)
     

    def run(self, n: int):
        print("Sleeping for 0.1s")
        time.sleep(0.1)
        self.ser.write(f"0,{n}\r".encode("ascii"))
        self.ser.flush()

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()

if __name__ == "__main__":
    u = UART_VERSION2()
    while True:
        for i in range(0, 255, 5):
            u.run(i)
        for i in range(255, 0, -5):
            u.run(i)
