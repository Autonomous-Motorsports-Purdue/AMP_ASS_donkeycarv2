import numpy as np
import argparse

def get_bbox(lat, lon, size_meters):
    delta_lat = size_meters / 111111.0
    delta_lon = size_meters / (111111.0 * np.cos(np.radians(lat)))
    
    # [min_lon, max_lon, min_lat, max_lat]
    return [lon - delta_lon/2, lon + delta_lon/2, lat - delta_lat/2, lat + delta_lat/2]

# Behind bidc 
# LAT = 40.42773956363986
# LON = -86.91873017427434
# SIZE = 50

# GP track
# LAT = -86.94443853
# LON = 40.43769648
# SIZE = 75 
LAT = 40.43785675963577
LON= -86.94426683849075
SIZE = 145 
BBOX = get_bbox(LAT,LON,SIZE)
print(f"Use this for Matplotlib extent: {BBOX}")

# GP Map
# [np.float64(-86.94512413996848), np.float64(-86.94340953701301), 40.43720425898327, 40.43850926028827]