import argparse
from pathlib import Path

import donkeycar as dk

from parts.gps import GPS
from parts.imu import IMU
from parts.simple_gps_pid import SimpleGPSPIDController
from parts.uart_backup import UART_backup_driver


class AlwaysOn:
    def run(self):
        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Minimal GPS + IMU + single PID path follower.")
    parser.add_argument("file_name", help="GPS waypoint CSV with latitude/longitude columns")
    parser.add_argument("--lookahead-m", type=float, default=6.0, help="distance ahead of the closest path point")
    parser.add_argument("--throttle", type=float, default=0.22, help="constant throttle command")
    parser.add_argument("--kp", type=float, default=1.0, help="heading error proportional gain")
    parser.add_argument("--ki", type=float, default=0.0, help="heading error integral gain")
    parser.add_argument("--kd", type=float, default=0.15, help="heading error derivative gain")
    parser.add_argument("--uart-port", default="/dev/ttyACM0", help="UART device for drive-by-wire")
    parser.add_argument("--max-loop-count", type=int, default=None, help="optional loop limit for bench testing")
    args = parser.parse_args()

    waypoint_path = Path(args.file_name).expanduser().resolve()
    if not waypoint_path.exists():
        raise FileNotFoundError(f"Waypoint CSV not found: {waypoint_path}")

    vehicle = dk.vehicle.Vehicle()

    gps = GPS()
    vehicle.add(
        gps,
        inputs=[],
        outputs=["lat_raw", "lon_raw", "alt", "fix", "corr_age", "hdop", "sat_count"],
        threaded=True,
    )

    imu = IMU(debug=False)
    vehicle.add(
        imu,
        inputs=[],
        outputs=["yaw_rate", "yaw", "a_x", "a_y"],
        threaded=False,
    )

    controller = SimpleGPSPIDController(
        path_csv=waypoint_path,
        lookahead_m=args.lookahead_m,
        throttle=args.throttle,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
    )
    vehicle.add(
        controller,
        inputs=["lat_raw", "lon_raw", "yaw"],
        outputs=["controls/steering", "controls/throttle"],
        threaded=False,
    )

    vehicle.add(AlwaysOn(), inputs=[], outputs=["safety/heartbeat"], threaded=False)

    uart = UART_backup_driver(args.uart_port)
    vehicle.add(
        uart,
        inputs=["controls/throttle", "controls/steering", "safety/heartbeat"],
        outputs=[],
        threaded=False,
    )

    start_kwargs = {"rate_hz": 20}
    if args.max_loop_count is not None:
        start_kwargs["max_loop_count"] = args.max_loop_count
    vehicle.start(**start_kwargs)
