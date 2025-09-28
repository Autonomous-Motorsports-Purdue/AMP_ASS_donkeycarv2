import cv2

MAX_IMG = 1543

class False_Camera():
    def resize(self, image):
        height, width = image.shape[:2]
        return cv2.resize(image, (640, 360))
    
    def grab_middle_section(self, image, width_ratio, height_ratio):
        """
        Grabs the middle section of an image based on given ratios.

        Args:
            image (numpy.ndarray): The input image.
            width_ratio (float): Ratio for the width of the middle section (0 < ratio < 1).
            height_ratio (float): Ratio for the height of the middle section (0 < ratio < 1).

        Returns:
            numpy.ndarray: The cropped middle section of the image.
        """

        height, width = image.shape[:2]

        start_x = int((1 - width_ratio) / 2 * width)
        end_x = int(start_x + width_ratio * width)
        start_y = int((1 - height_ratio) / 2 * height)
        end_y = int(start_y + height_ratio * height)

        return image[start_y:end_y, start_x:end_x]
    def __init__(self):
        self.count = 0
    def run(self):
        self.count += 1
        if (self.count > 1543):
            self.count = 1
        return self.resize(self.grab_middle_section(cv2.imread(f'images/img ({self.count}).jpg'), 0.6, 0.6))

        
