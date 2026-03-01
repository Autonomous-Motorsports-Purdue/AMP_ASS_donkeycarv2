import velodyne_decoder as vd
import dpkt
import socket
import time
import open3d as o3d
import numpy as np
import cv2

# --- Configuration ---
PORT = 2368
IS_LIVE = 1
GRID_RES = 0.1      # Meters per cell
GRID_RANGE = 20     # +/- 20 meters (Total 40m x 40m grid)

HEIGHT_RANGE = (-.25, 2) # Only include points between values in meters relative to the puck 

def create_occupancy_grid(points, resolution, grid_range):
    """
    Translates world coordinates to a 2D occupancy grid.
    Formula: $index = \lfloor \frac{point + range}{resolution} \rfloor$
    """
    # 1. Height filtering (Ignore ground and ceiling)
    mask_z = (points[:, 2] > HEIGHT_RANGE[0]) & (points[:, 2] < HEIGHT_RANGE[1])
    points = points[mask_z]

    # 2. Spatial filtering (X and Y bounds)
    mask_xy = (np.abs(points[:, 0]) < grid_range) & (np.abs(points[:, 1]) < grid_range)
    points = points[mask_xy]

    # 3. Calculate Grid Size
    grid_size = int((grid_range * 2) / resolution)
    
    # 4. Map to indices (Shift by grid_range to keep values positive)
    grid_x = ((points[:, 1] + grid_range) / resolution).astype(np.int32)
    grid_y = ((points[:, 0] + grid_range) / resolution).astype(np.int32)

    # 5. Populate Grid
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    # Clip to avoid index errors at the very edge of the range
    grid_x = np.clip(grid_x, 0, grid_size - 1)
    grid_y = np.clip(grid_y, 0, grid_size - 1)
    
    grid[grid_y, grid_x] = 255  # 255 for high visibility (Occupied)
    
    return grid

def create_point_cloud(points):
    """Standard O3D PointCloud creation with intensity coloring."""
    xyz = points[:, :3].copy()
    xyz[:, 2] = 0 # Flatten for BEV visualization
    intensity = points[:, 3]

    norm = intensity / intensity.max() if intensity.max() > 0 else intensity
    colors = np.zeros((len(norm), 3))
    colors[:, 0] = np.clip(1.5 - np.abs(4 * norm - 3), 0, 1) # R
    colors[:, 1] = np.clip(1.5 - np.abs(4 * norm - 2), 0, 1) # G
    colors[:, 2] = np.clip(1.5 - np.abs(4 * norm - 1), 0, 1) # B

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def read_live_data():
    # 1. Setup the decoder
    config = vd.Config(min_range=0, max_range=20, min_angle=270, max_angle=270)
    decoder = vd.StreamDecoder(config)

    # 2. Setup the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))

    print(f"Listening for Velodyne data on port {PORT}...")

    # 3. Setup Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window('Lidar 3D View')

    pcd = o3d.geometry.PointCloud()
    first_scan = True

    while True:
        data, address = sock.recvfrom(2048)
        stamp = time.time()

        scan = decoder.decode(stamp, data)

        if scan is not None:
            points = scan[1]
            if len(points) == 0: continue

            # Occupancy Grid logic
            occ_grid = create_occupancy_grid(points, GRID_RES, GRID_RANGE)
            # --- VISUALIZE THE GRID ---
            visualize_pro_occupancy_grid(occ_grid)
            # Point Cloud Visualization
            new_pcd = create_point_cloud(points)
            pcd.points = new_pcd.points
            pcd.colors = new_pcd.colors

            if first_scan:
                vis.add_geometry(pcd)
                # Set a top-down viewpoint for BEV-like perspective
                ctr = vis.get_view_control()
                ctr.set_front([0, 0, 1])
                ctr.set_up([0, 1, 0])
                ctr.set_lookat([0, 0, 0])
                ctr.set_zoom(0.3)
                first_scan = False
                print("First scan received, visualizer initialized.")
                print(np.asarray(pcd.points)[:5])
            else:
                vis.update_geometry(pcd)

            vis.poll_events()
            vis.update_renderer()

            if not vis.poll_events():
                break

    vis.destroy_window()


def read_from_file(file_path):
    config = vd.Config(model=vd.Model.VLP16, min_range=0.5)
    decoder = vd.StreamDecoder(config)

    vis = o3d.visualization.Visualizer()
    vis.create_window('Lidar BEV + Occupancy Grid', width=800, height=800)
    
    pcd = o3d.geometry.PointCloud()
    first_scan = True

    with open(file_path, 'rb') as f:
        try:
            reader = dpkt.pcapng.Reader(f)
        except:
            f.seek(0)
            reader = dpkt.pcap.Reader(f)

        for stamp, buf in reader:
            try:
                eth = dpkt.ethernet.Ethernet(buf)
                # Walk down the layers: Ethernet -> IP -> UDP
                if not isinstance(eth.data, dpkt.ip.IP): continue
                ip = eth.data
                if not isinstance(ip.data, dpkt.udp.UDP): continue
                udp = ip.data
                
                # The FIX: Convert bytes to bytearray for the decoder
                payload = bytearray(udp.data)
                
                # Optional: Velodyne data packets are always 1206 bytes
                if len(payload) != 1206:
                    continue
                    
            except Exception:
                continue

            # The decoder now receives the correct bytearray type
            scan = decoder.decode(float(stamp), payload)

            if scan is not None:
                points = scan[1]
                if len(points) == 0: continue

                # Occupancy Grid logic
                occ_grid = create_occupancy_grid(points, GRID_RES, GRID_RANGE)
                # --- VISUALIZE THE GRID ---
                visualize_pro_occupancy_grid(occ_grid)
                # Point Cloud Visualization
                new_pcd = create_point_cloud(points)
                pcd.points = new_pcd.points
                pcd.colors = new_pcd.colors

                if first_scan:
                    vis.add_geometry(pcd)
                    ctr = vis.get_view_control()
                    ctr.set_front([0, 0, 1])
                    ctr.set_up([0, 1, 0])
                    ctr.set_lookat([0, 0, 0])
                    ctr.set_zoom(0.3)
                    first_scan = False
                else:
                    vis.update_geometry(pcd)

                if not vis.poll_events(): break
                vis.update_renderer()
                
    vis.destroy_window()

def visualize_pro_occupancy_grid(grid):
    # Create a 3-channel BGR image
    display_size = (800, 800)
    color_grid = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    
    # Color all 'occupied' pixels (255) as Red
    color_grid[grid == 255] = [0, 0, 255] 
    
    # Draw a small circle in the middle to represent your sensor (0,0)
    center = (display_size[0] // 2, display_size[1] // 2)
    cv2.circle(color_grid, center, 5, (0, 255, 0), -1) # Green dot for sensor
    
    resized = cv2.resize(color_grid, display_size, interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Occupancy Grid", cv2.flip(resized, 0))
    cv2.waitKey(1)

if __name__ == "__main__":
    if IS_LIVE:
        read_live_data()
    else:
        read_from_file('test_pcap.pcap')