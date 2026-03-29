import donkeycar as dk

from parts.lidar import Lidar
from parts.lidar_visualizer import LidarVisualizer

if __name__ == "__main__":
    V = dk.vehicle.Vehicle()
    print("starting")

    #V.add(False_Lidar(), inputs=[], outputs=[])
    V.add(Lidar(), inputs=[], outputs=['occupancy_grid', 'points'])
    V.add(LidarVisualizer(), inputs=['occupancy_grid', 'points'], outputs=[])
    # BELOW CODE IS USED TO TEST PERFORMANCE OF LIDAR
    

    # V.add(Preprocessor(), inputs=['image'], outputs=['test'])
    '''
    V.add(Onnx(), inputs=['image'], outputs=['lane', 'drive'])
    V.add(Viewer("Image"), inputs=['image'], outputs=[])
    V.add(Viewer("Segmented"), inputs=['drive'], outputs=[])
    V.add(Viewer("Lines"), inputs=['lines'], outputs=[])
'''
    # V.start(rate_hz=0.00001)
    V.start(rate_hz=60)