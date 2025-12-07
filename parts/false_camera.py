import cv2
import numpy as np

class False_Camera():
    def __init__(self):
        self.count = 0
    def run(self):
        self.count += 1
        if (self.count > 734):
            self.count = 1
        return np.array(cv2.imread(f'images/img ({self.count}).jpg'))

        
