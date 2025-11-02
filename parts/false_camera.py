import cv2

MAX_IMG = 1543

class False_Camera():
    def resize(self, image):
        height, width = image.shape[:2]
        return cv2.resize(image, (640, 360))
    def __init__(self):
        self.count = 0
    def run(self):
        self.count += 1
        if (self.count > 1543):
            self.count = 1
        return self.resize(cv2.imread(f'images2/img ({self.count}).jpg'))

        
