import cv2
import numpy as np

class RingDetector:
    def __init__(self):
        self.hough_param1 = 90
        self.hough_param2 = 40
        self.min_radius = 75
        self.max_radius = 400
        self.minDist = 30

    def detect_ring(self, image):
        """
        Detect the ring in the image using edge detection and Hough Circle Transform.
        Returns the ring's center (x, y) and radius (r).
        """
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise and improve circle detection
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)

        circles = cv2.HoughCircles(blurred, 
                                   cv2.HOUGH_GRADIENT, dp=1, minDist=self.minDist, 
                                   param1=self.hough_param1, param2=self.hough_param2, 
                                   minRadius=self.min_radius, maxRadius=self.max_radius)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            x, y, r = circles[0]
            return (x, y, r)
        else:

            return None

    def is_circular(self, ellipse):
        """
        Check if the detected ellipse is approximately circular by comparing the major and minor axes.
        Returns True if the ellipse is approximately circular, otherwise False.
        """

        # # Unpack the ellipse (major axis, minor axis, and angle of orientation)
        # (x, y), (MA, ma), angle = ellipse

        # # If the difference between the major and minor axis is small, it's circular
        # if abs(MA - ma) < 0.2 * max(MA, ma):
        #     return True
        # return False
