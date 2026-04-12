import threading
import requests
from collections import deque

BUFFER_SIZE = 200

KNOWN_GROUPS = [
    ("acc_time",      ["accX", "accY", "accZ"],           "Accelerometer",       "m/s²"),
    ("gyr_time",      ["gyrX", "gyrY", "gyrZ"],           "Gyroscope",           "rad/s"),
    ("mag_time",      ["magX", "magY", "magZ"],            "Magnetometer",        "µT"),
    ("lin_time",      ["linX", "linY", "linZ"],            "Linear Acceleration", "m/s²"),
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

        print(f"Connecting to phyphox at {self.base_url} ...")
        self.groups = self._discover_buffers()

        self.last_time = {g[0]: 0.0 for g in self.groups}
        self.data = {}
        for time_key, data_keys, _, _ in self.groups:
            self.data[time_key] = deque(maxlen=BUFFER_SIZE)
            for dk in data_keys:
                self.data[dk] = deque(maxlen=BUFFER_SIZE)

        self.latest = (None,) * 9
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(target=self._poll_loop, args=(poll_hz,), daemon=True)
        self.thread.start()

    def _discover_buffers(self):
        resp = requests.get(f"{self.base_url}/config", timeout=3)
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
            present = [k for k in data_keys if k in available]
            if not present:
                continue
            groups.append((time_key, present, title, ylabel))
            claimed.add(time_key)
            claimed.update(present)

        remaining_time = [b for b in available - claimed if b.endswith("_time")]
        for tb in sorted(remaining_time):
            prefix = tb.replace("_time", "")
            data = [b for b in available - claimed - {tb}
                    if b.startswith(prefix) or b == prefix]
            if data:
                groups.append((tb, sorted(data), prefix.title(), ""))
                claimed.add(tb)
                claimed.update(data)

        if not groups:
            raise RuntimeError("No recognized sensor groups found in this experiment.")

        for time_key, data_keys, title, _ in groups:
            print(f"  {title}: {data_keys} (time: {time_key})")

        return groups

    def _build_url(self):
        parts = []
        for time_key, data_keys, _, _ in self.groups:
            t = self.last_time[time_key]
            parts.append(f"{time_key}={t}")
            for dk in data_keys:
                parts.append(f"{dk}={t}|{time_key}")
        return f"{self.base_url}/get?{'&'.join(parts)}"

    def _fetch_all(self):
        url = self._build_url()
        try:
            resp = requests.get(url, timeout=1)
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
            for dk in data_keys:
                vals = buffers.get(dk, {}).get("buffer", [])
                self.data[dk].extend(vals)

    def _get_latest(self, key):
        buf = self.data.get(key)
        if buf and len(buf) > 0:
            return buf[-1]
        return None

    def _poll_loop(self, hz):
        import time
        interval = 1.0 / hz
        while self.running:
            self._fetch_all()
            result = (
                self._get_latest("accX"), self._get_latest("accY"), self._get_latest("accZ"),
                self._get_latest("gyrX"), self._get_latest("gyrY"), self._get_latest("gyrZ"),
                self._get_latest("yaw"),  self._get_latest("pitch"), self._get_latest("roll"),
            )
            with self.lock:
                self.latest = result
            time.sleep(interval)

    def run(self):
        with self.lock:
            return self.latest

    def shutdown(self):
        self.running = False
        self.thread.join()
