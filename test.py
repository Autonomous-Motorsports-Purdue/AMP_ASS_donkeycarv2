import velodyne_decoder as vd
import dpkt
import socket
import time
import cv2
import numpy as np

# Standard Velodyne port
PORT = 2368


def create_bev_image(points, resolution=0.1, side_range=(-30, 30), fwd_range=(-30, 30)):
    """
    Creates a Bird's Eye View image from LiDAR points.
    resolution: meters per pixel
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    intensity = points[:, 3]

    # 1. Filter points within the visual range
    f_filt = np.logical_and((x > fwd_range[0]), (x < fwd_range[1]))
    s_filt = np.logical_and((y > side_range[0]), (y < side_range[1]))
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()

    x = x[indices]
    y = y[indices]
    intensity = intensity[indices]

    # 2. Convert to pixel coordinates
    # -y corresponds to x_img (width), -x corresponds to y_img (height) due to rotation
    x_img = (-y * 10 / resolution).astype(np.int32)
    y_img = (-x * 10 / resolution).astype(np.int32)

    # Shift origins to center of image
    x_img -= int(np.floor(side_range[0] / resolution))
    y_img += int(np.floor(fwd_range[1] / resolution))

    # 3. Create Image Buffer
    width = int((side_range[1] - side_range[0]) / resolution)
    height = int((fwd_range[1] - fwd_range[0]) / resolution)
    im = np.zeros([height, width], dtype=np.uint8)

    # 4. Fill Pixel Values (Clip to image boundaries just in case)
    x_img = np.clip(x_img, 0, width - 1)
    y_img = np.clip(y_img, 0, height - 1)
    
    # Scale intensity to 0-255
    im[y_img, x_img] = (intensity / intensity.max() * 255).astype(np.uint8)
    
    return im

def read_live_data():
    # 1. Setup the decoder
    config = vd.Config(min_range=0, max_range=5)
    # You can set specific options like model or rpm if needed, 
    # but the library often auto-detects them.
    decoder = vd.StreamDecoder(config)

    # 2. Setup the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))
    
    print(f"Listening for Velodyne data on port {PORT}...")

    while True:
        # 3. Receive raw packet (Velodyne packets are usually ~1206 bytes)
        data, address = sock.recvfrom(2048) 
        stamp = time.time() # Use current host time
        
        # 4. Decode the packet
        # The decoder buffers packets until a full scan (360 degrees) is complete
        # or a specific cut_angle is reached.
        scan = decoder.decode(stamp, data)
        
        if scan is not None:
            # 'scan' contains the point cloud data for a full revolution
            # print(f"Decoded scan with {len(scan[1])} points")
            # Generate BEV
            bev_img = create_bev_image(scan[1], resolution=0.1)
            
            # Apply colormap for better visualization
            color_img = cv2.applyColorMap(bev_img, cv2.COLORMAP_JET)

            cv2.imshow('Lidar BEV', color_img)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
 
def read_from_file(file_path):
    # 1. Setup decoder config (adjust model/range to match your sensor)
    config = vd.Config(model=vd.Model.VLP16, min_range=0, max_range=100)
    decoder = vd.StreamDecoder(config)

    # 2. Open the pcapng file with dpkt's pcapng reader
    with open(file_path, 'rb') as f:
        try:
            reader = dpkt.pcapng.Reader(f)
        except ValueError:
            # Fall back to pcap if it's actually a pcap file
            f.seek(0)
            reader = dpkt.pcap.Reader(f)

        for stamp, buf in reader:
            # 3. Parse Ethernet → IP → UDP to extract the raw payload
            try:
                eth = dpkt.ethernet.Ethernet(buf)
                if not isinstance(eth.data, dpkt.ip.IP):
                    continue
                ip = eth.data
                if not isinstance(ip.data, dpkt.udp.UDP):
                    continue
                udp = ip.data
                # Velodyne data packets come on port 2368
                if udp.dport != 2368:
                    continue
                payload = udp.data
            except (dpkt.NeedData, dpkt.UnpackError):
                continue

            # 4. Feed raw packet into the streaming decoder
            scan = decoder.decode(stamp, payload)

            if scan is not None:
                points = scan[1]
                if len(points) == 0:
                    continue
                # Generate BEV from the decoded points
                bev_img = create_bev_image(points, resolution=0.1)
                color_img = cv2.applyColorMap(bev_img, cv2.COLORMAP_JET)

                cv2.imshow('Lidar BEV', color_img)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()
    
def flattenZAxis(data):
    # 1. Sort by keys (X, then Y)
    # lexsort sorts by the last key passed first, so we pass (y, x) to sort by x then y
    sort_idx = np.lexsort((data[:, 1], data[:, 0]))
    sorted_data = data[sort_idx]

    # 2. Identify where the group (X, Y) changes
    # Create a boolean mask where the current row is different from the previous row
    # We check columns 0 (x) and 1 (y)
    group_change = np.any(sorted_data[1:, :2] != sorted_data[:-1, :2], axis=1)

    # Get the indices where groups start
    # We force 0 to be the first index
    split_indices = np.concatenate(([0], np.flatnonzero(group_change) + 1))

    # 3. Calculate Average Intensity using reduceat
    # reduceat sums slices of the array defined by split_indices
    intensity_sums = np.add.reduceat(sorted_data[:, 3], split_indices)

    # Calculate the count of points in each group
    # The count is the difference between split indices
    counts = np.diff(np.append(split_indices, len(sorted_data)))

    # 4. Compute Final Averages
    avg_intensities = intensity_sums / counts

    # 5. Construct Result: [Unique X, Unique Y, Avg Intensity]
    # We pick the X, Y coordinates from the start of each group
    unique_coords = sorted_data[split_indices, :2]
    flattened_data = np.hstack((unique_coords, avg_intensities[:, None]))
    return flattened_data

if __name__ == "__main__":
    # read_live_data()
    read_from_file('other_pcap_long.pcap')

    