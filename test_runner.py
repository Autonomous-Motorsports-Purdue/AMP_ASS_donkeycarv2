import donkeycar as dk

from parts.false_camera import False_Camera
from parts.viewer import Viewer
from parts.curve_fit import Curve_fit
from parts.onnx import Onnx

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    V.add(False_Camera(), inputs=[], outputs=['image'])

    # V.add(Preprocessor(), inputs=['image'], outputs=['test'])
    V.add(Onnx(), inputs=['image'], outputs=['lane', 'drive'])
    V.add(Curve_fit(), inputs=['lane', 'drive'], outputs=['waypoint', 'lines'])
    V.add(Viewer("Image"), inputs=['image'], outputs=[])
    V.add(Viewer("Segmented"), inputs=['drive'], outputs=[])
    V.add(Viewer("Lines"), inputs=['lines'], outputs=[])

    V.start(rate_hz=30)