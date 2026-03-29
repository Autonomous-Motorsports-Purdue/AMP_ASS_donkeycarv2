import donkeycar.donkeycar as dk

from parts.false_camera import False_Camera
from parts.viewer import Viewer
from parts.curve_fit import Curve_fit
from parts.onnx import Onnx
from parts.false_lidar import False_Lidar
from parts.lidar import Lidar

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    V.add(False_Lidar(), inputs=[], outputs=[])
    V.add(Lidar(), inputs=[], outputs=[])
    
    # BELOW CODE IS USED TO TEST PERFORMANCE OF LIDAR
    
    V.add(False_Camera(), inputs=[], outputs=['image'])

    # V.add(Preprocessor(), inputs=['image'], outputs=['test'])
    V.add(Onnx(), inputs=['image'], outputs=['lane', 'drive'])
    V.add(Curve_fit(), inputs=['lane', 'drive'], outputs=['waypoint', 'lines'])
    V.add(Viewer("Image"), inputs=['image'], outputs=[])
    V.add(Viewer("Segmented"), inputs=['drive'], outputs=[])
    V.add(Viewer("Lines"), inputs=['lines'], outputs=[])

    # V.start(rate_hz=0.00001)
    V.start(rate_hz=60)