import numpy as np
import pickle

class PixelToReal:
    def __init__(self):
        camera_height = 31.875
        camera_pitch = np.deg2rad(34.5)
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
        self.rotation_matrix = r_yaw @ r_roll @ r_pitch
        self.translation = np.array([0, camera_height, 0])

    def run (self, depth, point_cloud, u, v):

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
        cam_to_world = np.diag([1, -1, 1]) @ x_c # making y=0 be the floor
        result = self.translation + self.rotation_matrix @ cam_to_world
        print("u:", u, "v:", v)
        print("depth:", depth)
        print("point cloud results: ", world_coords)
        print("results:", result)
        return result







