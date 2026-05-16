# host_visualizer.py (Run on Host Computer)
import socket
import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import pandas as pd

# --- MAP CONFIGURATION ---
# Replace these with the actual bounding box of your map_image.png
# [min_lon, max_lon, min_lat, max_lat]
# BBOX = [-86.9090, -86.9070, 40.4250, 40.4265] 
# behind bidc
# BBOX = [np.float64(-86.91902575099941), np.float64(-86.91843459754928), 40.42751456341486, 40.42796456386486]
# gp track
BBOX = [np.float64(-86.94512413996848), np.float64(-86.94340953701301), 40.43720425898327, 40.43850926028827]
MAP_FILE = 'gp_map.png'

# Load the offline satellite image
try:
    map_img = plt.imread(MAP_FILE)
except FileNotFoundError:
    print(f"Error: Could not find {MAP_FILE}. Please provide an image.")
    exit()

# --- PLOT SETUP ---
plt.ion() # Turn on interactive mode for live plotting
fig, ax = plt.subplots(figsize=(8, 8))
ax.imshow(map_img, extent=BBOX, aspect='equal')

# Initialize plot elements
robot_marker, = ax.plot([], [], 'ro', markersize=6, label='Current Pos') # Red dot
# Quiver for heading (arrow). Initialized at 0,0
heading_arrow = ax.quiver(0, 0, 0, 0, color='yellow', scale=15, pivot='tail', zorder=5)


steer_arrow = ax.quiver(0, 0, 0, 0, color='red', scale=20, pivot='tail', zorder=5)

# file_path = "../gps_paths/gp_track/gp_raceline_gps.csv"
# file_path = "../gps_paths/gps_raceline_edited_v2.csv"
# file_path = "../gps_paths/gp_midline_dense2.csv"
file_path_1 = "../gps_paths/planned_aligned.csv"


ref_gp_path = "../gps_paths/2026-05-01-14-59-55_gps_full.csv"
reference_path_df = pd.read_csv(ref_gp_path)
ref_path = ax.plot(reference_path_df["longitude"],reference_path_df["latitude"], label='Reference')

# align_path_file = "../gps_paths/planned_aligned.csv"
# align_path_df = pd.read_csv(align_path_file)
# align_path = ax.plot(align_path_df["longitude"],align_path_df["latitude"], label='Aligned')

follow_path_file = "../gps_paths/gp_raceline_gps.csv" 
follow_path_df = pd.read_csv(follow_path_file)
follow_path = ax.plot(follow_path_df["longitude"], follow_path_df["latitude"],label="Follow")

trajectory_line, = ax.plot([], [], 'b-', linewidth=2, label='Trajectory') # Blue line

ax.set_xlim(BBOX[0], BBOX[1])
ax.set_ylim(BBOX[2], BBOX[3])
ax.set_title("Live Robot Telemetry (Offline Map)")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.legend()
# ax.show()
# while True:
#     pass

# Data history arrays
history_lat = []
history_lon = []

# ssh -L 5005:localhost:5005 amp@192.168.1.50
# --- NETWORK SETUP ---
PORT = 5005
buffer = ""

max_steer = 22

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('localhost', PORT)) # Connects via the SSH tunnel
    print("Connected to Jetson data stream.")
    
    while True:
        try:
            # Receive data chunk
            chunk = s.recv(1024).decode('utf-8')
            if not chunk:
                break
            buffer += chunk
            
            # Process complete JSON messages separated by newline
            lines = buffer.split('\n')
            while '\n' in buffer:
                # line, buffer = buffer.split('\n', 1)
                latest_line = lines[-2] 
                buffer = lines[-1] # Keep the remainder for next time
                # data = json.loads(line)
                data = json.loads(latest_line)
                print(data)
                
                has_null = any(v is None for v in data.values())
                if data['lat'] != None and data['heading'] == None:
                    print(f"override heading null")
                    has_null = False
                if has_null:
                    print("null found, continuing")
                    continue
                lat = data['lat']
                lon = data['lon']
                # heading is in radians?
                if data['heading'] == None:
                    heading_deg = 0
                else:
                    heading_deg = np.rad2deg(data['heading'])

                if 'steer' in data:
                    steer_val = data['steer'] * max_steer
                    steer_rad = np.radians(90 - heading_deg - steer_val)
                    dx = np.cos(steer_rad)
                    dy = np.sin(steer_rad)
                    steer_arrow.set_offsets([lon,lat])
                    steer_arrow.set_UVC(dx,dy)

                    
                
                # Update history
                history_lat.append(lat)
                history_lon.append(lon)
                
                # 1. Update Trajectory Line
                trajectory_line.set_data(history_lon, history_lat)
                
                # 2. Update Robot Marker
                robot_marker.set_data([lon], [lat])
                
                # 3. Update Heading Arrow
                # Convert heading degrees to dx/dy (assuming 0 deg is North, clockwise)
                heading_rad = np.radians(90 - heading_deg) 
                dx = np.cos(heading_rad)
                dy = np.sin(heading_rad)
                heading_arrow.set_offsets([lon, lat])
                heading_arrow.set_UVC(dx, dy)


                
                # Refresh plot
                fig.canvas.flush_events()
                # plt.pause(0.01)
                
        except KeyboardInterrupt:
            print("Visualization stopped by user.")
            break