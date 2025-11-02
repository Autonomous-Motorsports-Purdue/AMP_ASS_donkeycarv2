import pyzed.sl as sl
import cv2

class PixelPicker:
    """
    DonkeyCar part for repeatedly selecting pixels (u, v) from the ZED feed.
    Each call to run() will wait until the user clicks on a pixel.
    After a click, it returns (u, v), then waits again the next time run() is called.
    """

    def __init__(self, zed):
        # Initialize ZED

        self.zed = zed
        self.runtime_params = zed.runtime_params
        self.image = None
        self.clicked_point = None

        print("ZED Pixel Picker initialized — will open window each time run() is called.")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            print(f"Clicked pixel: u={x}, v={y}")

    def run(self):
        """Wait until user clicks or presses ESC."""
        cv2.namedWindow("ZED View", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("ZED View", self.mouse_callback)
        self.clicked_point = None

        print("Click a pixel to select (u, v). Press ESC to cancel.")

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()
            cv2.imshow("ZED View", frame)
            key = cv2.waitKey(10) & 0xFF

            if key == 27:  # ESC
                print("Selection cancelled.")
                cv2.destroyWindow("ZED View")
                return None, None

            if self.clicked_point is not None:
                u, v = self.clicked_point
                cv2.destroyWindow("ZED View")
                return u, v
        return 0,0
    # def shutdown(self):
    #     """Close resources."""
    #     print("Shutting down ZED Pixel Picker...")
    #     self.zed.close()
    #     cv2.destroyAllWindows()
