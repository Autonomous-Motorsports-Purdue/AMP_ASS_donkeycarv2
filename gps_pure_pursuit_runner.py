import argparse
from pathlib import Path

import donkeycar as dk

from parts.gps import GPS
from parts.gps_pure_pursuit_waypoint import GPSPurePursuitWaypointGenerator
from parts.pure_pursuit import Pure_Pursuit
from parts.uart_backup import UART_backup_driver


class AlwaysOn:
    def run(self):
        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help="GPS waypoint CSV with latitude/longitude columns")
    parser.add_argument("--lookahead-m", type=float, default=6.0, help="lookahead distance along path")
    parser.add_argument("--speed", type=float, default=0.35, help="pure pursuit throttle output")
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

    waypoint_generator = GPSPurePursuitWaypointGenerator(
        path_csv=waypoint_path,
        lookahead_m=args.lookahead_m,
    )
    vehicle.add(
        waypoint_generator,
        inputs=["lat_raw", "lon_raw"],
        outputs=["waypoint"],
        threaded=False,
    )

    pure_pursuit = Pure_Pursuit(args.speed)
    vehicle.add(
        pure_pursuit,
        inputs=["waypoint"],
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
