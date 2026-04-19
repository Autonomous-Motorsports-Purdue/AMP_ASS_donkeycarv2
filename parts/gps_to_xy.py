import numpy as np


class GPS_to_xy:
    """
    Convert geodetic coordinates (lat/lon in degrees) to a local Cartesian frame.

    Frame definition:
    - x: East (+x is to the right on a north-up plot)
    - y: North (+y is upward on a north-up plot)

    This uses a local tangent-plane approximation around a reference latitude/longitude.
    It is accurate for typical small- to medium-sized driving areas.
    """

    EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius

    def __init__(self, ref_lat_deg: float, ref_lon_deg: float):
        self.ref_lat_deg = float(ref_lat_deg)
        self.ref_lon_deg = float(ref_lon_deg)
        self.ref_lat_rad = np.radians(self.ref_lat_deg)
        self.ref_lon_rad = np.radians(self.ref_lon_deg)
        self.cos_ref_lat = np.cos(self.ref_lat_rad)

    def to_xy(self, lat_deg: float, lon_deg: float):
        """Return (x, y) in meters where x=east and y=north."""
        lat_rad = np.radians(lat_deg)
        lon_rad = np.radians(lon_deg)

        dlat = lat_rad - self.ref_lat_rad
        dlon = lon_rad - self.ref_lon_rad

        x_east_m = dlon * self.cos_ref_lat * self.EARTH_RADIUS_M
        y_north_m = dlat * self.EARTH_RADIUS_M
        print(f"GPS_to_xy: lat {lat_deg:.6f}, lon {lon_deg:.6f} -> x {x_east_m:.2f} m, y {y_north_m:.2f} m")
        return x_east_m, y_north_m

    def to_latlon(self, x_east_m: float, y_north_m: float):
        """Inverse of to_xy: return (lat_deg, lon_deg)."""
        lat_rad = self.ref_lat_rad + (y_north_m / self.EARTH_RADIUS_M)
        lon_rad = self.ref_lon_rad + (x_east_m / (self.EARTH_RADIUS_M * self.cos_ref_lat))
        return float(np.degrees(lat_rad)), float(np.degrees(lon_rad))

    def run(self, lat_deg: float, lon_deg: float):
        """Run method for DonkeyCar part interface."""
        return self.to_xy(lat_deg, lon_deg)