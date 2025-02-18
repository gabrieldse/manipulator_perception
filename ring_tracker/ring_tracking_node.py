#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ring_tracker_interface.srv import GrabRing, MoveCamera
from geometry_msgs.msg import Pose

from ring_tracker.ring_detector import RingDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RingTrackingNode(Node):
    def __init__(self):
        super().__init__('ring_tracking_node')
        self.get_logger().info("Ring Tracking Node has been started.")

        # Service & Subscriber definitions
        self.grab_ring_srv = self.create_service(
            GrabRing, '/grab_ring', self.grab_ring_callback)
        # self.move_camera_srv = self.create_service(
        #     MoveCamera, '/move_camera', self.move_camera_callback)
        self.ring_position_pub = self.create_publisher(
            Pose, '/ring_position', 10)
        
        ## Image initializations
        # Suscribers definitions
        self.sub_img = self.create_subscription(Image,'/image_raw',self.cb_image,1)
        self.bridge = CvBridge()
        # frequency = 10 # Hz
        # self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
        self.ring_detector = RingDetector()
        self.latest_image = None

        # Simulate ring position updates
        self.timer = self.create_timer(1.0, self.publish_ring_position)

    def grab_ring_callback(self, request, response):
        self.get_logger().info("Grabbing ring...")
        # Simulate grab action (replace with actual logic)
        response.success = True
        return response

    # def move_camera_callback(self, request, response):
    #     self.get_logger().info(f"Moving camera by: {request.delta_x}, {request.delta_y}, {request.delta_z}")
    #     # Simulate movement (replace with actual logic)
    #     response.success = True
    #     return response

    def publish_ring_position(self):
        
        ring_position = self.ring_detector.detect_ring(self.latest_image)

        if ring_position:
            x, y, r = ring_position  # Ring's center (x, y) and radius (r)
            msg = Pose()
            msg.position.x = float(x/100)
            msg.position.y = float(y/100)
            msg.position.z = float(0)
            self.ring_position_pub.publish(msg)
            self.get_logger().info(f"Publishing ring position: ({x}, {y}, {0})")
            
            # OPTIONAL
            self.draw_ring(self.latest_image, x, y, r)

            # Optionally, display the image with the ring drawn on it
            cv2.imshow("Ring Detection", self.latest_image)
            cv2.waitKey(1)  # Update the window
        else:
            self.get_logger().info("No ring detected.")
            
    # def timer_callback(self):
        """
        Timer callback for frequency of circle detection.
        """
        if self.latest_image is not None:
            self.get_logger().info(f"Image dimensions: {self.latest_image.shape}")
            self.ring_detector.detect_ring(self.latest_image)
        else:
            self.get_logger().info("No image received yet.")

    def cb_image(self, msg):
        """
        Callback for receiving image from camera feed.
        Converts the ROS image to OpenCV format and processes it.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image 
        except Exception as e:
            self.get_logger().error(f"Error in image conversion: {e}")
            
    def draw_ring(self, image, x, y, r):
        if image is not None:
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)  # Green circle
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)  # Center dot
        
        # Display distances on the screen
        text_x = f"X Distance: {x:.2f} cm"
        text_y = f"Y Distance: {y:.2f} cm"
        text_z = f"Z Distance: {r:.2f} cm"
        
        # Position of the text
        text_position_z = (10, 30)  # (x, y) coordinates
        text_position_x = (10, 50)
        text_position_y = (10, 70)
        
        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_color = (255, 255, 255)  # White color
        font_thickness = 1
        
        # Put text on the image
        cv2.putText(self.latest_image, text_z, text_position_z, font, font_scale, font_color, font_thickness)
        cv2.putText(self.latest_image, text_x, text_position_x, font, font_scale, font_color, font_thickness)
        cv2.putText(self.latest_image, text_y, text_position_y, font, font_scale, font_color, font_thickness)
            
        
    
def main(args=None):
    rclpy.init(args=args)
    node = RingTrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()