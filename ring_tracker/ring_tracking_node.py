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
        
        self.sub_img = self.create_subscription(Image,'/image_raw',self.cb_image,1)
        self.bridge = CvBridge()
        self.ring_detector = RingDetector()
        self.latest_image = None
        self.timer = self.create_timer(1.0, self.publish_ring_position)

    def grab_ring_callback(self, request, response):
        self.get_logger().info("Grabbing ring...")
        # Simulate grab action (replace with actual logic)
        response.success = True
        return response

    # def move_camera_callback(self, request, response):
    #     self.get_logger().info(f"Moving camera by: {request.delta_x}, {request.delta_y}, {request.delta_z}")
    #     # Simulate movement TODO
    #     response.success = True
    #     return response

    def publish_ring_position(self):
        if self.latest_image is not None:
            ring_position = self.ring_detector.detect_ring(self.latest_image)
            
            if ring_position is not None:
                x, y, r = ring_position

                ring_position_camera2real = self.ring_position_camera2real(ring_position)
            
                if ring_position_camera2real:
                    x_m, y_m, z_m = ring_position_camera2real  # Ring's center (x, y) and radius (r)
                    msg = Pose()
                    msg.position.x = float(x_m)
                    msg.position.y = float(y_m)
                    msg.position.z = float(z_m)
                    self.ring_position_pub.publish(msg)
                    # self.get_logger().info(f"Publishing ring position [m]: ({x_m}, {y_m}, {z_m})")
                    
                    # OPTIONAL
                    self.draw_ring(self.latest_image, x, y, r, x_m, y_m, z_m)
                    cv2.imshow("Ring Detection", self.latest_image)
                    cv2.waitKey(1)  # Update the window
                else:
                    self.get_logger().info("No ring detected.")
                    
            else:
                self.get_logger().info("No ring detected in the image.")
        else:
            self.get_logger().warn("No image received yet. Skipping ring detection.")
            
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
          
# MOVE TO ring_detector.py  
    def draw_ring(self, image, x, y, r, x_m, y_m, z_m):
        if image is not None:
            cv2.circle(image, (x, y), r, (0, 255, 0), 4)  # Green circle
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)  # Center dot
        
        # Display distances on the screen
        text_x = f"X Distance: {x_m:.2f} m"
        text_y = f"Y Distance: {y_m:.2f} m"
        text_z = f"Z Distance: {z_m:.2f} m"
        
        # Position of the text
        text_position_z = (10, 30)  # (x, y) coordinates
        text_position_x = (10, 50)
        text_position_y = (10, 70)
        
        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)  # White
        font_thickness = 1
        
        # Put text on the image
        cv2.putText(self.latest_image, text_z, text_position_z, font, font_scale, font_color, font_thickness)
        cv2.putText(self.latest_image, text_x, text_position_x, font, font_scale, font_color, font_thickness)
        cv2.putText(self.latest_image, text_y, text_position_y, font, font_scale, font_color, font_thickness)

    def ring_position_camera2real(self, ring_position):
        
            x, y, r =  ring_position
            
            f_x = 350.15
            real_radius = 0.208  # m
            r_px = r
            
            Z = (f_x * real_radius) / r_px
            
            # Image center px
            c_x = self.latest_image.shape[1] / 2
            c_y = self.latest_image.shape[0] / 2
            
            # Intrinsic matrix for the conversion
            f_y = f_x # square pixels
            u_prime = (x - c_x) / f_x
            v_prime = (y - c_y) / f_y

            # Calculate real-world coordinates in meters
            X = u_prime * Z 
            Y = v_prime * Z
            
            return (X, Y, Z)

def main(args=None):
    rclpy.init(args=args)
    node = RingTrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()