import cv2
import sys  # <— needed to exit the whole program

class PixelPicker:
    """
    DonkeyCar part for interactively selecting pixels (u, v) from a camera feed.
    - Keeps window open continuously.
    - Returns (u, v) only once per click.
    - Waits again after each click.
    - Press ESC to shut down the entire program.
    """

    def __init__(self):
        self.clicked_point = None
        self.new_click_ready = False
        self.window_open = False
        print("PixelPicker initialized — click on the image to pick pixels.")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            self.new_click_ready = True
            print(f"Clicked pixel: u={x}, v={y}")

    def run(self, left):
        """
        left: image frame (numpy array) from ZED camera.
        Returns (u, v) once per click, otherwise (None, None).
        """
        if not self.window_open:
            cv2.namedWindow("ZED View", cv2.WINDOW_NORMAL)
            cv2.setMouseCallback("ZED View", self.mouse_callback)
            self.window_open = True
            print("ZED View window opened. Click to select (u, v). Press ESC to quit.")

        if left is not None:
            cv2.imshow("ZED View", left)
        key = cv2.waitKey(1) & 0xFF

        # ESC key → close everything and terminate the program
        if key == 27:
            print("ESC pressed — shutting down PixelPicker and exiting program.")
            self.shutdown()
            sys.exit(0)

        # Return (u,v) only once per new click
        if self.new_click_ready:
            self.new_click_ready = False
            return self.clicked_point

        return None, None

    def shutdown(self):
        """Close the window gracefully."""
        if self.window_open:
            cv2.destroyWindow("ZED View")
            self.window_open = False
        print("PixelPicker shut down cleanly.")
