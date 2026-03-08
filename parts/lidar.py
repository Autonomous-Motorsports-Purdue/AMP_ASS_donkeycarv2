import velodyne_decoder as vd
import dpkt
import socket
import time
import open3d as o3d
import numpy as np
import cv2

# --- Config ---
GRID_RES = 0.1                      # Meters per cell
GRID_RANGE = 20                     # Range of grid in meters (Total ~140m square grid for our VLP 16)
HEIGHT_RANGE = (-.25, 2)            # Only include points between values in meters relative to the LiDAR

# --- Constants that should not change ---
PORT = 2368                         # Default port for Velodyne LiDAR sensors


class Lidar():
    def __init__(self):
        # Setup the decoder
        self.config = vd.Config(min_range=0, max_range=GRID_RANGE)
        self.decoder = vd.StreamDecoder(self.config)

        # Setup the UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', PORT))

        print(f"Listening for Velodyne data on port {PORT}...")

        # VISUAL - Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window('Lidar 3D View')

        self.pcd = o3d.geometry.PointCloud()
        self.first_scan = True

    def run(self):
        print("Starting Lidar module...")
        data, address = self.sock.recvfrom(2048)
        stamp = time.time()
        scan = self.decoder.decode(stamp, data)

        if scan is None:
            return

        points = scan[1]
        if len(points) == 0: return

        occ_grid = self.create_occupancy_grid(points, GRID_RES, GRID_RANGE)
        
        # VISUAL - updates the occupancy grid visualizer
        self.visualize_pro_occupancy_grid(occ_grid)
        
        new_pcd = self.create_point_cloud(points)
        self.pcd.points = new_pcd.points

        # VISUAL - updates the OPEN3D visualizer
        self.pcd.colors = new_pcd.colors
        if self.first_scan:
            self.vis.add_geometry(self.pcd)
            self.first_scan = False
            
            # Set a fixed view (looking down from above)
            ctr = self.vis.get_view_control()
            ctr.set_front([0, 0, 1])
            ctr.set_up([0, 1, 0])
            ctr.set_lookat([0, 0, 0])
            ctr.set_zoom(0.3)
            
            print("First scan received, visualizer initialized.")
            print(np.asarray(self.pcd.points)[:5])
        else:
            self.vis.update_geometry(self.pcd)

        # VISUAL - Show a new frame in the OPEN3D visualizer
        self.vis.poll_events()
        self.vis.update_renderer()

        if not self.vis.poll_events():
            return


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
        grid_x = ((points[:, 0] + grid_range) / resolution).astype(np.int32)
        grid_y = ((points[:, 1] + grid_range) / resolution).astype(np.int32)

        # 5. Populate Grid
        grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
        # Clip to avoid index errors at the very edge of the range
        grid_x = np.clip(grid_x, 0, grid_size - 1)
        grid_y = np.clip(grid_y, 0, grid_size - 1)
        
        grid[grid_y, grid_x] = 1  # 1 for binary data (Occupied)

        return grid

    def create_point_cloud(points):
        """Standard O3D PointCloud creation with intensity coloring."""
        xyz = points[:, :3].copy()
        xyz[:, 2] = 0 # Flatten points to 2D plane for visualization
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

    

'''
    def visualize_pro_occupancy_grid(grid):
        # 1. Setup Canvas
        display_size = (800, 800)
        # Convert grayscale to BGR for colored markers
        color_grid = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
        
        # 2. Draw Distance Markers (Meters)
        # We iterate through the meter range to find pixel indices
        for m in range(-GRID_RANGE, GRID_RANGE + 1):
            
            # Convert meter coordinate to pixel coordinate
            # Formula: (meter + range) / resolution
            pix_pos = int((m + GRID_RANGE) / GRID_RES)
            
            # Draw Vertical Meter Lines (Constant X)
            cv2.line(color_grid, (pix_pos, 0), (pix_pos, grid.shape[0]), (40, 40, 40), 1)
            # Draw Horizontal Meter Lines (Constant Y)
            cv2.line(color_grid, (0, pix_pos), (grid.shape[1], pix_pos), (40, 40, 40), 1)

        # 3. Draw Main Axes (0,0)
        origin_pix = int(GRID_RANGE / GRID_RES)
        cv2.line(color_grid, (origin_pix, 0), (origin_pix, grid.shape[0]), (100, 100, 100), 1) # Y-axis
        cv2.line(color_grid, (0, origin_pix), (grid.shape[1], origin_pix), (100, 100, 100), 1) # X-axis

        # 4. Color 'occupied' pixels Red
        color_grid[grid == 1] = [0, 0, 255] 
        
        # 5. Final Display
        # Note: We flip it because image Y increases downwards, but spatial Y increases upwards
        resized = cv2.resize(color_grid, display_size, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Occupancy Grid (with Scale)", cv2.flip(resized, 0))
        cv2.waitKey(1)
'''