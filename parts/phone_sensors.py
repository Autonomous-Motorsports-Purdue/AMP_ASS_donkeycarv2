import threading
import requests
from collections import deque

BUFFER_SIZE = 200
FLUSH_INTERVAL = 90

KNOWN_GROUPS = [
    ("acc_time",      ["accX", "accY", "accZ"],           "Accelerometer",       "m/s²"),
    ("gyr_time",      ["gyrX", "gyrY", "gyrZ"],           "Gyroscope",           "rad/s"),
    ("gyro_time",     [("gyroX", "gyrX"), ("gyroY", "gyrY"), ("gyroZ", "gyrZ")],
                                                          "Gyroscope",           "rad/s"),
    ("mag_time",      ["magX", "magY", "magZ"],            "Magnetometer",        "µT"),
    ("lin_time",      ["linX", "linY", "linZ"],            "Linear Acceleration", "m/s²"),
    ("lin_acc_time",  [("lin_accX", "linX"), ("lin_accY", "linY"), ("lin_accZ", "linZ")],
                                                           "Linear Acceleration", "m/s²"),
    ("pressure_time", ["pressure"],                        "Pressure",            "hPa"),
    ("prox_time",     ["prox"],                            "Proximity",           "cm"),
    ("light_time",    ["light"],                           "Light",               "lx"),
    ("loc_time",      ["locLat", "locLon", "locZ", "locV", "locDir",
                       "locAccuracy", "locZAccuracy", "locSatellites", "locStatus"],
                                                           "Location",            "mixed"),
    ("attT",          ["yaw", "pitch", "roll", "direct"],  "Attitude (Euler)",    "degrees"),
]

# android tool required: adb, command to run before donkeycar: adb forward tcp:8080 tcp:8080
# ios tool required: libusbmuxd, command to run before donkeycar: iproxy 8080:80

# REMOTE ACCESS MUST BE ENABLED IN PHYPHOX APP TO RUN

class PhoneSensors():
    def __init__(self, host="localhost", port=8080, poll_hz=60):
        self.base_url = f"http://{host}:{port}"

        self.session = requests.Session()
        print(f"Connecting to phyphox at {self.base_url} ...")
        self.groups = self._discover_buffers()

        self.last_time = {g[0]: 0.0 for g in self.groups}
        self.data = {}
        for time_key, data_keys, _, _ in self.groups:
            self.data[time_key] = deque(maxlen=BUFFER_SIZE)
            for _, canonical in data_keys:
                self.data[canonical] = deque(maxlen=BUFFER_SIZE)

        self.latest = (None,) * 9
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._poll_loop, args=(poll_hz,), daemon=True)
        self.thread.start()

    def _discover_buffers(self):
        resp = self.session.get(f"{self.base_url}/config", timeout=3)
        resp.raise_for_status()
        config = resp.json()

        available = {b["name"] for b in config.get("buffers", [])}
        print(f"Experiment: {config.get('title', '(unknown)')}")
        print(f"Available buffers: {sorted(available)}")

        groups = []
        claimed = set()

        for time_key, data_keys, title, ylabel in KNOWN_GROUPS:
            if time_key not in available:
                continue
            # data_keys entries may be "name" or ("remote_name", "canonical_name")
            present = []
            for entry in data_keys:
                remote, canonical = entry if isinstance(entry, tuple) else (entry, entry)
                if remote in available:
                    present.append((remote, canonical))
            if not present:
                continue
            groups.append((time_key, present, title, ylabel))
            claimed.add(time_key)
            claimed.update(remote for remote, _ in present)

        remaining_time = [b for b in available - claimed if b.endswith("_time")]
        for tb in sorted(remaining_time):
            prefix = tb.replace("_time", "")
            data = [b for b in available - claimed - {tb}
                    if b.startswith(prefix) or b == prefix]
            if data:
                groups.append((tb, [(d, d) for d in sorted(data)], prefix.title(), ""))
                claimed.add(tb)
                claimed.update(data)

        if not groups:
            raise RuntimeError("No recognized sensor groups found in this experiment.")

        for time_key, data_keys, title, _ in groups:
            shown = [c if r == c else f"{r}->{c}" for r, c in data_keys]
            print(f"  {title}: {shown} (time: {time_key})")

        return groups

    def _build_url(self):
        parts = []
        for time_key, data_keys, _, _ in self.groups:
            t = self.last_time[time_key]
            parts.append(f"{time_key}={t}")
            for remote, _ in data_keys:
                parts.append(f"{remote}={t}|{time_key}")
        return f"{self.base_url}/get?{'&'.join(parts)}"

    def _fetch_all(self):
        url = self._build_url()
        try:
            resp = self.session.get(url, timeout=1)
            resp.raise_for_status()
            payload = resp.json()
        except requests.RequestException:
            return
        except ValueError:
            return

        buffers = payload.get("buffer", {})
        for time_key, data_keys, _, _ in self.groups:
            time_vals = buffers.get(time_key, {}).get("buffer", [])
            if not time_vals:
                continue
            self.data[time_key].extend(time_vals)
            self.last_time[time_key] = time_vals[-1]
            for remote, canonical in data_keys:
                vals = buffers.get(remote, {}).get("buffer", [])
                self.data[canonical].extend(vals)

    def _get_latest(self, key):
        buf = self.data.get(key)
        if buf and len(buf) > 0:
            return buf[-1]
        return None

    def _poll_loop(self, hz):
        import time
        interval = 1.0 / hz
        last_flush = time.time()
        flush_interval = FLUSH_INTERVAL  # seconds
        while self.running:
            self._fetch_all()
            result = (
                self._get_latest("accX"), self._get_latest("accY"), self._get_latest("accZ"),
                self._get_latest("gyrX"), self._get_latest("gyrY"), self._get_latest("gyrZ"),
                self._get_latest("yaw"),  self._get_latest("pitch"), self._get_latest("roll"),
            )
            with self.lock:
                self.latest = result
            if time.time() - last_flush >= flush_interval:
                threading.Thread(target=self._flush_and_reset, daemon=True).start()
                last_flush = time.time()
            time.sleep(interval)
    
    def flush_phyphox_ram(self):
        self.session.get(f"{self.base_url}/control?cmd=clear")
        self.session.get(f"{self.base_url}/control?cmd=start")

    def _flush_and_reset(self):
        try:
            self.flush_phyphox_ram()
            for key in self.last_time:
                self.last_time[key] = 0.0
        except requests.RequestException:
            pass

    def run(self):
        with self.lock:
            return self.latest

    def shutdown(self):
        self.running = False
        self.thread.join()