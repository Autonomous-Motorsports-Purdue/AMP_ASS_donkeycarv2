# import pandas as pd
import numpy as np
import argparse

R_EARTH = 6378137.0
RAD2DEG = 180.0 / np.pi


def latlon_to_local_m(lat, lon, lat0, lon0):
    cos_lat0 = np.cos(np.radians(lat0))
    cos_lat0 = np.clip(cos_lat0, 1e-6, None)
    y_north = (lat - lat0) / RAD2DEG * R_EARTH
    x_east = (lon - lon0) / RAD2DEG * (R_EARTH * cos_lat0)
    return x_east, y_north

def convert_gps_to_xy(gps_file_name):
    data = np.genfromtxt(gps_file_name, delimiter=',', skip_header=1, dtype=float, encoding='utf-8')
    lat0 = data[0,0]
    lon0 = data[0,1]
    x_coords, y_coords = latlon_to_local_m(data[:,0], data[:,1], lat0, lon0)
    coords = np.stack((x_coords, y_coords), axis=1)
    new_file = gps_file_name.split('.')[0] + "_xy" + ".csv"
    np.savetxt(new_file, coords, delimiter=',', fmt="%f")
    print(coords)


if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument("file_name", help="name of gps csv")

    args = argparse.parse_args()
    convert_gps_to_xy(args.file_name)