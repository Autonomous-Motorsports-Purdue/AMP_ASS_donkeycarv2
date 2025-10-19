import cv2

class Viewer:
    def __init__(self, name):
        self.name = name
    def run(self, image):
        print(type(image))
        cv2.imshow(self.name, image)
        cv2.waitKey(1)