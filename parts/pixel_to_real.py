import numpy as np
import pickle

class PixelToReal:
    def __init__(self):
        camera_height = 40.625
        camera_pitch = np.deg2rad(0) # level is 0
        yaw = np.deg2rad(0)
        roll = np.deg2rad(0)

        zed_calib = pickle.load(open("zed_calibration_params.bin", "rb"))
        camera_matrix = np.array([[zed_calib["fx"], 0, zed_calib["cx"]],
                                [0, zed_calib["fy"], zed_calib["cy"]],
                                [0, 0, 1]])
        self.camera_matrix_inv = np.linalg.inv(camera_matrix)
        r_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])

        r_roll = np.array([[np.cos(roll), 0, np.sin(roll)],
                           [0, 1, 0],
                           [-np.sin(roll), 0, np.cos(roll)]])

        r_pitch = np.array([[1, 0, 0],
                            [0, np.cos(camera_pitch), -np.sin(camera_pitch)],
                            [0, np.sin(camera_pitch), np.cos(camera_pitch)]])
        self.rotation_matrix = r_yaw @ r_roll @ r_pitch @ np.diag([1, -1, 1])
        self.translation = np.array([0, camera_height, 0])

    def run (self, depth, point_cloud, u, v):
        # z of 76 but actual was 81 in
        # z of 91 but actual was 100 in
        # z of 146 but actual was 166 in
        # z of 166 but actual was 192 in
        # z of 192 but actual was 223 in

        # x of -31 but actual was 48 in with z of 250 in
        # y of 26 but actual was 12 in with z of 250 in
        """
        Object with positive x is right of the camera.
        Y is the height of the object from the ground.
        Z is the depth the object is from the camera.
        """

        #u = round(self.zed_calib["cx"]) # center of image
        #v = round(self.zed_calib["cy"]) # center of image
        if u is None or v is None:
            return None, None, None
        # point cloud start
        err, point_3d = point_cloud.get_value(u, v)
        pc_x, pc_y, pc_z = point_3d[:3]
        world_coords = self.translation + self.rotation_matrix @ np.array([pc_x, pc_y, pc_z])
        # point cloud end

        depth = depth.get_value(u, v)[1]
        homogeneous_image = np.array([u, v, 1])
        x_c = self.camera_matrix_inv @ homogeneous_image * depth
        result = self.translation + self.rotation_matrix @ x_c
        # scaled_result = result.copy()
        #scaled_result[2] *= 1.12 # attempted depth correction
        #print("u:", u, "v:", v)

        # start force y=0

        # 1. Original world point
        camera_point = np.array([pc_x, pc_y, pc_z])
        world_original = self.translation + self.rotation_matrix @ camera_point

        # 2. Ray direction in camera frame
        ray_cam = camera_point / np.linalg.norm(camera_point)

        # 3. Ray in world frame
        ray_world = self.rotation_matrix @ ray_cam
        camera_origin_world = self.translation

        # 4. Compute ground intersection (y = 0)
        t = -camera_origin_world[1] / ray_world[1]
        ground_point = camera_origin_world + t * ray_world

        # 5. Blend between original and ground
        alpha = 0.5  # adjust this 0–1
        partial_ground_point = (1 - alpha) * world_original + alpha * ground_point

        # end force y=0

        np.set_printoptions(suppress=True)
        #print("depth:", depth)
        #print("pc_x, pc_y, pc_z:", pc_x, pc_y, pc_z)
        #print("distance match:", np.sqrt(pc_x ** 2 + pc_y ** 2 + pc_z ** 2))
        #print("ground_point:", ground_point)
        print("partial_ground_point:", partial_ground_point)
        #print("point cloud results: ", world_coords)
        #print("results:", result)
        #print("scaled_result:", scaled_result)
        return partial_ground_point