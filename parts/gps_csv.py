import csv
import threading
import time


class GPS_CSV:
    """
    Donkeycar part that replays GPS fixes from a CSV file in place of the live
    GPS receiver. Matches the output signature of parts.gps.GPS:

        lat_raw, lon_raw, alt, fix, corr_age, hdop, sat_count

    The CSV is expected to have a header row. Columns are matched by name with
    sensible fallbacks; only lat/lon are required. Missing optional columns are
    emitted as default values ("NO FIX", 0, 0.0, 0).
    """

    _LAT_KEYS = ("lat_raw", "lat", "latitude")
    _LON_KEYS = ("lon_raw", "lon", "longitude")
    _ALT_KEYS = ("alt", "altitude")
    _FIX_KEYS = ("fix",)
    _AGE_KEYS = ("corr_age", "diffage", "diff_age")
    _HDOP_KEYS = ("hdop",)
    _SAT_KEYS = ("sat_count", "numSV", "sip")

    def __init__(self, path, playback_rate_hz=4.0, loop=True):
        self.path = path
        self.dt = 1.0 / float(playback_rate_hz) if playback_rate_hz > 0 else 0.0
        self.loop = loop

        self.rows = self._load(path)
        if not self.rows:
            raise ValueError(f"GPS_CSV: no rows parsed from {path}")

        self.idx = 0
        self.lock = threading.Lock()
        self.running = True
        self.latest = self.rows[0]

    @classmethod
    def _pick(cls, row, keys, default, cast):
        for k in keys:
            if k in row and row[k] not in (None, ""):
                try:
                    return cast(row[k])
                except (ValueError, TypeError):
                    return default
        return default

    @classmethod
    def _parse_row(cls, row):
        lat = cls._pick(row, cls._LAT_KEYS, None, float)
        lon = cls._pick(row, cls._LON_KEYS, None, float)
        if lat is None or lon is None:
            return None
        alt = cls._pick(row, cls._ALT_KEYS, 0.0, float)
        fix = cls._pick(row, cls._FIX_KEYS, "NO FIX", str)
        corr_age = cls._pick(row, cls._AGE_KEYS, 0, int)
        hdop = cls._pick(row, cls._HDOP_KEYS, 0.0, float)
        sat_count = cls._pick(row, cls._SAT_KEYS, 0, int)
        return (lat, lon, alt, fix, corr_age, hdop, sat_count)

    @classmethod
    def _load(cls, path):
        rows = []
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for raw in reader:
                parsed = cls._parse_row(raw)
                if parsed is not None:
                    rows.append(parsed)
        return rows

    def _advance(self):
        with self.lock:
            self.latest = self.rows[self.idx]
            self.idx += 1
            if self.idx >= len(self.rows):
                if self.loop:
                    self.idx = 0
                else:
                    self.idx = len(self.rows) - 1
                    self.running = False

    def update(self):
        next_tick = time.monotonic()
        while self.running:
            self._advance()
            next_tick += self.dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def run_threaded(self):
        with self.lock:
            return self.latest

    def run(self):
        self._advance()
        with self.lock:
            return self.latest

    def shutdown(self):
        self.running = False
