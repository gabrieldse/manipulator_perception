import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # To convert ROS Image messages to OpenCV images

class DisplayCalibratedImage(Node):

    def __init__(self):
        super().__init__('display_calibrated_image')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the raw image topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Replace with your raw image topic
            self.image_callback,
            10
        )

        # Calibration parameters (replace with your actual calibration data)
        self.camera_matrix = np.array([[438000.783367, 0, 305.593336], [0, 437.302876, 243.738352], [0, 0, 1]])  # Replace fx, fy, cx, cy with your values
        self.dist_coeffs = np.array([-0.361976, 0.110510, 0.001014, 0.000505, 0.000000])  # Replace with your distortion coefficients

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort the raw image using calibration parameters
            corrected_image = cv2.undistort(raw_image, self.camera_matrix, self.dist_coeffs)

            # Resize images to ensure they have the same dimensions (if needed)
            raw_image_resized = cv2.resize(raw_image, (640, 480))  # Adjust size as needed
            corrected_image_resized = cv2.resize(corrected_image, (640, 480))  # Adjust size as needed

            # Combine the raw and corrected images side by side
            combined_image = np.hstack((raw_image_resized, corrected_image_resized))

            # Display the combined image
            cv2.imshow('Raw (Left) vs Corrected (Right)', combined_image)
            cv2.waitKey(1)  # Refresh the display

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = DisplayCalibratedImage()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()