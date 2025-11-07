import static_donkeycar.donkeycar.donkeycar as dk
from parts.pixel_to_real import PixelToReal

# from parts.false_camera import False_Camera
from parts.viewer import Viewer
# from parts.curve_fit import Curve_fit
# from parts.onnx import Onnx
from parts.depth_printer import DepthPrinter
from parts.zed_frame_publisher import Zed_Frame_Publisher
from parts.pixel_to_real import PixelToReal
from parts.pixel_picker import PixelPicker
if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    #V.add(DepthPrinter(), inputs=[], outputs=['left', 'right', 'depth'])

    # V.add(Preprocessor(), inputs=['image'], outputs=['test'])
    # V.add(Onnx(), inputs=['image'], outputs=['lane', 'drive'])
    # V.add(Curve_fit(), inputs=['lane', 'drive'], outputs=['waypoint', 'lines'])
    V.add(Zed_Frame_Publisher(), outputs=['left', 'right', 'depth', 'point_cloud'])
    V.add(PixelPicker(), inputs=['left'], outputs=['u', 'v'])
    V.add(PixelToReal(), inputs=['depth', 'point_cloud', 'u','v'], outputs=['x', 'y', 'z'])
    #V.add(Viewer("Image"), inputs=['left'], outputs=[])
    # V.add(Viewer("Segmented"), inputs=['right'], outputs=[])
    # V.add(Viewer("Lines"), inputs=['depth'], outputs=[])

    V.start(rate_hz=30)