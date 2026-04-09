import serial
import time


class UART_backup_driver:
    def __init__(
        self,
        port_name: str = "/dev/tty.usbserial-D30IDHO4",
        warmup_iterations: int = 10,
        throttle_scale: float = 127.0,
        steering_scale: float = 64.0,
    ):
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.ser = serial.Serial(port=port_name, baudrate=115200)

        self.curr_v = 0
        self.curr_s = 0
        self._iter = 0 # iteration counter. used to delay start.
        self.warmup_iterations = warmup_iterations
        self.throttle_scale = throttle_scale
        self.steering_scale = steering_scale

    def update_velocity(
        self, new_v: int
    ):  # shifting values into UART accepted range (128-255) (zero at 191)
        if new_v < -128:
            new_v = -128
        elif new_v > 255:
            new_v = 255

        self.curr_v = new_v

    def update_steering(
        self, new_s: int
    ):  # shifting values into UART accepted range (128-255) (zero at 191)
        if new_s <= -63:
            new_s = 0
        elif new_s >= 64:
            new_s = 255
        else:
            new_s = new_s + 127

        self.curr_s = new_s

    def reset_kart(
        self,
    ):
        self.update_velocity(0)
        self.update_steering(0)
        self.write_serial()

    def write_serial(
        self,
    ):  # the exposed keyword at the front allows the object to be accesible.

        # send start byte
        self.ser.write(f"{self.curr_v},{self.curr_s}\r".encode("ascii"))

    def run(self, v, s, alive):
        """
        Donkeycar compatible run function
        DOnkeycar gives (-1, 1) for steering and (-1, 1) for throttle
        """
        print(f"T:{v}, S:{s}")
        if not alive:
            self.reset_kart()
            return
        if self._iter < self.warmup_iterations:
            print("warming up kart -- not moving")
            self.reset_kart()
            self._iter += 1
            return
        self._iter += 1 # increment iteration.

        if s is None:
            s = 0
            print("s is None")
        if v is None:
            v = 0
            print("v is None")
        v = int(v * self.throttle_scale)

        # steering is centered at 128
        s = int(s * self.steering_scale)

        print(f"Throttle: {v}, Steering: {s}")

        self.update_velocity(v)
        self.update_steering(s)
        print(f"New steering: {self.curr_s}")

        self.write_serial()

    def shutdown(self):
        self.reset_kart()
        time.sleep(0.1)
        self.ser.close()
