import math
import pyzed.sl as sl
import numpy as np

class DepthPrinter:
    def __init__(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()

        self.runtime_parameters = sl.RuntimeParameters()

        self.left_image = sl.Mat()
        self.right_image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()

    def run(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.right_image, sl.VIEW.RIGHT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            x = round(self.left_image.get_width() / 2)
            y = round(self.left_image.get_height() / 2)
            err, point_cloud_value = self.point_cloud.get_value(x, y)

            if math.isfinite(point_cloud_value[2]):
                distance = math.sqrt(sum(v*v for v in point_cloud_value[:3]))
                print(f"Distance to Camera at {{{x};{y}}}: {distance} mm")
            else:
                print(f"The distance cannot be computed at {{{x};{y}}}")

            # return np.array([self.left_image, self.right_image, self.depth])
            #return np.array([self.left_image]), np.array([self.right_image]), np.array([self.depth])
            return self.left_image.get_data(), self.right_image.get_data(), self.depth.get_data()
        else:
            print("Failed to grab frame")
            return np.array(None)

    def close(self):
        self.zed.close()
