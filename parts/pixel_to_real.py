import math
import pyzed.sl as sl
import numpy as np

class PixelToReal:
    def __init__(self, height = 10, pitch = 10):
        zed_calib = {'fx': 1415.85888671875, 'fy': 1415.85888671875, 'cx': 891.181884765625, 'cy': 570.443115234375}
        self.camera_height = height
        self.camera_pitch = np.deg2rad(pitch)
        yaw = 0
        roll = 0
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
                            [0, np.cos(self.camera_pitch), -np.sin(self.camera_pitch)],
                            [0, np.sin(self.camera_pitch), np.cos(self.camera_pitch)]])
        self.rotation_matrix = (r_yaw @ r_roll @ r_pitch).T
        self.C = np.array([0, 0, height])
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()

        self.runtime_parameters = sl.RuntimeParameters()
        self.left_image = sl.Mat()
        self.right_image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
    def run (self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            #v, u = point # x, y of image
            #v = np.arange(0, self.left_image.get_width(), 20)
            v = round(891.181884765625)
            #u = np.arange(0, 570.443115234375, 20)
            u = round(570.443115234375)
            print("start")
            results = []
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            #for y in u:
            depth = self.depth.get_value(v,int(u))[1]
            print(depth)
            homogeneous_image = np.array([v, int(u), 1])
            ray_cam = self.camera_matrix_inv @ homogeneous_image
            if math.isfinite(depth): # find better check
                x_c = ray_cam * depth # X_c = Z * K^{−1} * x (homogeneous image)
                result = self.rotation_matrix @ x_c + self.C
                results.append((u, result))
                #print(result)
                #return result
            else:
                # old code

                ray_world = self.rotation_matrix @ ray_cam
                # Intersect with ground plane (Z = 0)
                if abs(ray_world[2]) < 1e-9:
                    raise ValueError("Ray is parallel to the ground plane.")
                lam = -self.C[2] / ray_world[2]
                x_w = self.C + lam * ray_world
                x_w[2] = 0.0  # enforce exact planarity
                results.append((u, x_w))
                #print(x_w)
                print("failed depth")
                #return x_w
            print(results)
        return "fail"


# using point cloud instead of matrix multiplication
    def runs(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            #v, u = point
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            v = round(self.left_image.get_width() / 2)
            u = round(self.left_image.get_height() / 2)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            err, point_cloud_value = self.point_cloud.get_value(v, u)
            print(point_cloud_value)
            if math.isfinite(point_cloud_value[2]):
                result = self.rotation_matrix @ np.array(point_cloud_value[:3]) + self.C
                print(result)
                return result
        return "fail"





