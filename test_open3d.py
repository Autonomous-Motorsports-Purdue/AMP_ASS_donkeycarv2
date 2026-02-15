import velodyne_decoder as vd
import dpkt
import socket
import time
import open3d as o3d
import numpy as np

# Standard Velodyne port
PORT = 2368
IS_LIVE = 1


def create_point_cloud(points):
    """
    Creates an Open3D point cloud from LiDAR points.
    Colors points by intensity using a jet-like colormap.
    """
    xyz = points[:, :3]
    xyz[:, 2] = 0  # Flatten Z-axis
    intensity = points[:, 3]

    # Normalize intensity to [0, 1]
    i_max = intensity.max()
    if i_max > 0:
        norm = intensity / i_max
    else:
        norm = intensity

    # Jet-like colormap: blue -> cyan -> green -> yellow -> red
    colors = np.zeros((len(norm), 3))
    colors[:, 0] = np.clip(1.5 - np.abs(4 * norm - 3), 0, 1)  # R
    colors[:, 1] = np.clip(1.5 - np.abs(4 * norm - 2), 0, 1)  # G
    colors[:, 2] = np.clip(1.5 - np.abs(4 * norm - 1), 0, 1)  # B

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def read_live_data():
    # 1. Setup the decoder
    config = vd.Config(min_range=0, max_range=5, min_angle=270, max_angle=270)
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
            if len(points) == 0:
                continue

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
    # 1. Setup decoder config
    config = vd.Config(model=vd.Model.VLP16, min_range=0)
    decoder = vd.StreamDecoder(config)

    # 2. Setup Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window('Lidar 3D View')

    # Rendering options
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0, 0, 0])

    pcd = o3d.geometry.PointCloud()
    first_scan = True

    # 3. Open the pcapng file
    with open(file_path, 'rb') as f:
        try:
            reader = dpkt.pcapng.Reader(f)
        except ValueError:
            f.seek(0)
            reader = dpkt.pcap.Reader(f)

        for stamp, buf in reader:
            try:
                eth = dpkt.ethernet.Ethernet(buf)
                if not isinstance(eth.data, dpkt.ip.IP):
                    continue
                ip = eth.data
                if not isinstance(ip.data, dpkt.udp.UDP):
                    continue
                udp = ip.data
                if udp.dport != 2368:
                    continue
                payload = udp.data
            except (dpkt.NeedData, dpkt.UnpackError):
                continue

            scan = decoder.decode(stamp, payload)

            if scan is not None:
                points = scan[1]
                if len(points) == 0:
                    continue

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
                    print(np.asarray(pcd.points)[:5])
                else:
                    vis.update_geometry(pcd)

                vis.poll_events()
                vis.update_renderer()

                if not vis.poll_events():
                    break

    vis.destroy_window()

if __name__ == "__main__":
    if IS_LIVE:
        read_live_data()
    else:
        read_from_file('other_pcap_long.pcap')
