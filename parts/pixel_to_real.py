import math
import pyzed.sl as sl
import numpy as np
import cv2

class PixelToReal:
    def __init__(self, height = 103.5, pitch = 0):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.CENTIMETER
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()

        self.runtime_parameters = sl.RuntimeParameters()
        self.left_image = sl.Mat()
        self.right_image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters

        # Access intrinsic parameters
        fx = calibration_params.left_cam.fx  # Focal length in x
        fy = calibration_params.left_cam.fy  # Focal length in y
        cx = calibration_params.left_cam.cx  # Principal point x
        cy = calibration_params.left_cam.cy  # Principal point y
        self.zed_calib = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}
        self.camera_height = height
        self.camera_pitch = np.deg2rad(pitch)
        yaw = 0
        roll = 0
        camera_matrix = np.array([[self.zed_calib["fx"], 0, self.zed_calib["cx"]],
                                [0, self.zed_calib["fy"], self.zed_calib["cy"]],
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
        self.C = np.array([0, height, 0])
    def run (self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            #u, v = point # x, y of image
            #u = round(self.zed_calib["cx"])
            u = 560
            v = 571
            #v = round(self.zed_calib["cy"])
            print("start")
            results = []
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            #for y in u:
            depth = self.depth.get_value(u,v)[1]
            homogeneous_image = np.array([u, v, 1])
            ray_cam = self.camera_matrix_inv @ homogeneous_image
            if math.isfinite(depth): # find better check
                #x = ((v - self.zed_calib["cx"] * depth) / self.zed_calib["fx"])
                #y = ((u - self.zed_calib["cy"] * depth) / self.zed_calib["fy"])
                #print("shortcut: ", x, y, depth)
                x_c = ray_cam * depth # X_c = Z * K^{−1} * x (homogeneous image)
                result = self.rotation_matrix @ x_c + self.C
                results.append((v, result))
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
                results.append((v, x_w))
                #print(x_w)
                print("failed depth")
                #return x_w

            print("depth:", depth)
            print("results:", results)
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







