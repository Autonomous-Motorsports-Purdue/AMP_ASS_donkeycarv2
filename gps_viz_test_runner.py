import static_donkeycar.donkeycar.donkeycar as dk

from parts.socket_pub_part import TelemetryStreamer
from parts.threaded_socket_pub_part import ThreadedTelemetryStreamer
from parts.image_publisher import Image_Publisher
from parts.image_cv import Object_Detection
from parts.log import Logger
import math
import random
import time
import numpy as np
LAT_START = 40.43785675963577
LON_START = -86.94426683849075

x = time.time()
class Fake_GPS():
    def run(self):
        t = time.time() - x
        current_lat = LAT_START + (math.sin(t) * 0.00005)
        current_lon = LON_START + (math.cos(t) * 0.00005)
        current_heading = (t * 50) % 360
        current_heading = np.deg2rad(current_heading)
        steer = 1 
        print(f"lat:{current_lat},lon:{current_lon},head:{current_heading}, steer:{steer}")
        return current_lat, current_lon, current_heading, steer

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    V.add(Fake_GPS(), inputs=[], outputs=['lat','lon','heading','steer'])
    V.add(Image_Publisher(), inputs=[], outputs=['image'])
    V.add(Object_Detection(), inputs=['image'], outputs=['image_cv', 'object_x', 'object_y', 'contour_area'])
    V.add(Logger(), inputs=['object_x', 'object_y', 'contour_area'], outputs=[])
    # V.add(TelemetryStreamer(), inputs=['lat','lon','heading'])
    V.add(ThreadedTelemetryStreamer(), inputs=['lat','lon','heading', 'steer'])

    V.start(rate_hz=30)