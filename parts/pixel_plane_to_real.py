import numpy as np
import pickle

class PixelPlaneToReal:
    def __init__(self):
        camera_height = 40.625

        # --- load intrinsics ---
        zed_calib = pickle.load(open("zed_calibration_params.bin", "rb"))
        K = np.array([[zed_calib["fx"], 0, zed_calib["cx"]],
                      [0, zed_calib["fy"], zed_calib["cy"]],
                      [0, 0, 1]])
        self.K_inv = np.linalg.inv(K)

        # --- your fixed world transform ---
        self.rotation_matrix = np.diag([1, -1, 1])
        self.translation = np.array([0, camera_height, 0])

    def run(self, u, v, plane_eq):
        if u is None or v is None or plane_eq is None:
            return None

        a, b, c, d = plane_eq
        norm = np.array([a, b, c], dtype=float)

        # Ray from pixel (camera frame)
        ray = self.K_inv @ np.array([u, v, 1.0])
        ray = ray / np.linalg.norm(ray)

        denom = np.dot(norm, ray)
        if abs(denom) < 1e-9:
            return None

        # Ray-plane intersection (camera frame)
        t = -d / denom
        if t <= 0:
            return None

        hit_cam = t * ray

        # Transform to your world frame
        world_point = self.translation + self.rotation_matrix @ hit_cam

        print("World point from plane:", world_point)


        return world_point
