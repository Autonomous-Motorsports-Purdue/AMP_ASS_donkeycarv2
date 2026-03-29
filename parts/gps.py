from threading import Event
from queue import Queue
from time import sleep
from serial import Serial

from pygnssutils.gnssstreamer import GNSSStreamer
from pygnssutils.gnssntripclient import GNSSNTRIPClient
from pygnssutils.helpers import parse_url
from pygnssutils.globals import CLIAPP, FORMAT_PARSED, ENCODE_NONE

import subprocess
import re
import time

class GNSSCLIStreamer:
    def __init__(self, gnss_path="gnssstreamer"):
        #Command to run gnssstreamer
        self.cmd = [
            gnss_path,  # <- You can replace this with full path if needed
            "--port", "/dev/ttyACM3", # COM4 for windows
            "--msgfilter", "GNGGA",
            "--cliinput", "1",
            "--input", "108.59.49.226:9000/RTCM3_MAX",
            "--rtkuser", "automp1",
            "--rtkpassword", "automp1",
            "--rtkggaint", "10",
            "--clioutput", "4",
            "--output",
            "lambda msg: print('lat: {}, lon: {}, alt: {}, fix: {}, age: {}'.format(msg.lat, msg.lon, msg.alt, ['NO FIX','2D','3D','N/A','RTK FIXED','RTK FLOAT'][msg.quality], msg.diffAge))"
        ]

        # Start subprocess
        try:
            self.process = subprocess.Popen(
                self.cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1  # line-buffered
            )
            # print("GNSS process started")

            print(" ".join(self.cmd))

        except Exception as e:
            print(f"Failed to start gnssstreamer: {e}")
            self.process = None

        # Regex to extract GNSS values
        self.regex = re.compile(
        r"lat:\s([-\d.]+)\s,\slon:\s([-\d.]+)\s,\salt:\s([-\d.]+)\s,\sfix:\s([A-Za-z\s]+?)\s,\sage:\s*([-\d.]+)",
            re.IGNORECASE,
        )

        # Latest GNSS values
        self.lat = None
        self.lon = None
        self.alt = None
        self.fix = "NO DATA"
        self.age = None

        self.running = True

    def update(self):
        """Background thread that reads GNSS data."""
        if not self.process:
            print("GNSS process not running. Exiting update loop.")
            return

        idle_time = 0
        line = self.process.stdout.readline()
        print("line: ", line)
        print(" -------- ")
            # print("stderr: ", self.process.stderr.readline())
        if not line:
            time.sleep(0.1)
            print("not blocked")
            idle_time += 0.1
            if idle_time > 100:
                print("No GNSS data for 10 seconds — exiting update loop.")
                self.running = False
        else:
            print("HERE 2")
        idle_time = 0
        line = line.strip()
        if "lat:" not in line:
            return

        # print("GNSS >", line)
        self.process.stdout.flush()
        parts = [p.strip() for p in line.split(",")]
        data = {}
            
            # print(len(parts))
        if len(parts) !=5:
            print(parts)
            # continue
        print(f"line={line}")
        for part in parts:
            # print("part:",part)
            values = part.split(": ", 1)
            value = None
            if(len(values) == 2):
                value = values[1]
            key = values[0]
            if ":" in key:
                key = key[:-1]
            # print(f"Key: {key} Value: {value}")
            data[key] = value

        self.lat = float(data["lat"])
        self.lon = float(data["lon"])
        self.alt = float(data["alt"])
        self.fix = data["fix"]
        if(data["age"] is not None):
            self.age = float(data["age"])
        else:
            self.age = None


        # print(self.lat, self.lon, self.alt, self.fix, self.age)
            # match = self.regex.search(line)
            # if match:
            #     self.lat = float(match.group(1))
            #     self.lon = float(match.group(2))
            #     self.alt = float(match.group(3))
            #     self.fix = match.group(4)
            #     self.age = float(match.group(5))
            # else:
            #     print("No match found: ", list(line))

        time.sleep(0.01)
        print("about to break", iters)

    def run(self):
        """Return latest GNSS values."""
        self.update()
        #print("hello")
        #print(self.lat, self.lon)
        return self.lat, self.lon, self.alt, self.fix, self.age
        

    def shutdown(self):
        """Stop subprocess and cleanup."""
        print("Shutting down GNSS...")
        self.running = False
        if self.process:
            self.process.terminate()
            print("GNSS subprocess terminated")

if __name__ == "__main__":
    part = GNSSCLIStreamer()
    for i in range(10):
        part.run()
