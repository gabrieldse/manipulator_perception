#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


"""
Ref 2 : https://www.traimaocv.fr/CoursStereoVision/co/ModCamera_1.html

La taille du pixel est :

"« CMOS Sensor Data »"

"« The CMOS sensor in the Logitech C270 webcam features the following data : »"

"« Sensor Resolution = 1280 x 960 »"

"« Pixel Dimension = 2,8 μm x 2,8 μm »"

"« Sensor Dimension = 3,5 mm x 2,7 mm »"

"« Sensor area = 9,45 mm2 »"
"""

"""
Camera parameters:
focal distance = 4.0 mm
focal lenght = 1350px
FOV - 60°
Optical Resolution (True)	1280 x 960 1.2MP
Image Capture (4:3 SD)	320x240, 640x480 1.2 MP, 3.0 MP
Image Capture (16:9 W)	360p, 480p, 720p
Video Capture (4:3 SD)	320x240, 640x480, 800x600
Video Capture (16:9 W)	360p, 480p, 720p,
Frame Rate (max)	30fps @ 640x480

1430 px

"""

class DetectCircle(Node):
    
    def __init__(self):
        super().__init__('CircleDetect')
        self.get_logger().info("CircleDetect start")
        
        self.sub_img = self.create_subscription(Image,'/image_raw',self.cb_image,1)
        frequency = 10 # Hz
        # self.ts = ApproximateTimeSynchronizer([self.sub_img], queue_size=1, slop=1.0 / frequency)
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)  # 5 Hz
        # self.ts.registerCallback(self.cb_image)
        
        self.bridge = CvBridge()
        self.tracker = KalmanTracker()
        self.latest_image = None
        
        self.distance_pub = self.create_publisher(Float32, '/circle_distance_m', 10)
        
    def timer_callback(self):
        """
        Timer callback for 5 Hz processing.
        """
        if self.latest_image is not None:
            # Process the latest image
            self.get_logger().info(f"Image dimensions: {self.latest_image.shape}")
            self.detect_circle(self.latest_image)
        else:
            self.get_logger().info("No image received yet.")
        
    def cb_image(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            
    def detect_circle(self, latest_image):
        gray = cv2.cvtColor(latest_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=90,
            param2=40,
            minRadius=75,
            maxRadius=400
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            best_circle = circles[0, 0]
            
            # Kalman tracker
            measurement = [best_circle[0], best_circle[1]]
            self.tracker.predict()
            updated_state = self.tracker.update(measurement)
            self.get_logger().info(f"[KALMAN] center of circle: {updated_state[0], updated_state[1]} ")
            
            self.tracker.draw_trajectory(latest_image)
            cv2.circle(latest_image, (best_circle[0], best_circle[1]), best_circle[2], (0, 255, 0), 2)  # Draw the circle
            cv2.circle(latest_image, (best_circle[0], best_circle[1]), 2, (0, 0, 255), 3)  # Red dot at the center
            
            # Calculate the distance to the circle
            focal_length_px = 350.15  # Focal length in pixels
            real_radius = 20.8  # Radius of the circle in real life (in cm)
            detected_radius = best_circle[2]  # Radius of the circle in the image (in pixels)
            
            distance_z = (focal_length_px * real_radius) / detected_radius
            distance2 = (4 * 20.8 * 480) / (detected_radius * 2 * 2.7)  # 2.7 is the sensor height in mm
            self.get_logger().info(f"Distance to the circle (Z-axis): {distance_z:.2f} cm")
            self.get_logger().info(f"Distance to the circle, method 2 (Z-axis): {distance2:.2f} cm") # ref: 2
            
            # Calculate X and Y distances
            image_center_x = latest_image.shape[1] / 2
            image_center_y = latest_image.shape[0] / 2
            
            delta_x = image_center_x - best_circle[0]
            delta_y = image_center_y - best_circle[1]
            
            distance_x = (delta_x * real_radius) / detected_radius
            distance_y = (delta_y * real_radius) / detected_radius
            
            self.get_logger().info(f"Distance to the circle (X-axis): {distance_x:.2f} cm")
            self.get_logger().info(f"Distance to the circle (Y-axis): {distance_y:.2f} cm")
            
            # Display distances on the screen
            text_z = f"Z Distance: {distance_z:.2f} cm"
            text_x = f"X Distance: {distance_x:.2f} cm"
            text_y = f"Y Distance: {distance_y:.2f} cm"
            
            # Position of the text
            text_position_z = (10, 30)  # (x, y) coordinates
            text_position_x = (10, 60)
            text_position_y = (10, 90)
            
            # Font settings
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            font_color = (255, 255, 255)  # White color
            font_thickness = 2
            
            # Put text on the image
            cv2.putText(latest_image, text_z, text_position_z, font, font_scale, font_color, font_thickness)
            cv2.putText(latest_image, text_x, text_position_x, font, font_scale, font_color, font_thickness)
            cv2.putText(latest_image, text_y, text_position_y, font, font_scale, font_color, font_thickness)
            
            # Publish the Z distance
            distance_msg = Float32()
            distance_msg.data = distance2 / 100
            self.distance_pub.publish(distance_msg)
            
            self.get_logger().info(f"Detected {len(circles[0])} circle(s). Best one of radius {best_circle[2]}. Center: {best_circle[1], best_circle[2]} ")
        else:
            self.get_logger().info("No circles detected.")

        cv2.imshow("Detected Circles", latest_image)
        cv2.waitKey(1)
    
               
class KalmanTracker:
    def __init__(self, delta_t=1.0, max_speed=5, trajectory_length=100):
        # Initialize state vector [x, y, vx, vy]
        self.state = np.zeros((4, 1), dtype=np.float32)
        
        # State transition matrix
        self.F = np.array([
            [1, 0, delta_t, 0],
            [0, 1, 0, delta_t],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Process noise covariance
        self.Q = np.eye(4, dtype=np.float32) * 0.1
        
        # Measurement noise covariance 
        self.R = np.eye(2, dtype=np.float32) * 0.1
        
        # Initial estimation error covariance
        self.P = np.eye(4, dtype=np.float32) * 1.0
        
        # Kalman Gain placeholder
        self.K = np.zeros((4, 2), dtype=np.float32)
        
        # Maximum expected speed
        self.max_speed = max_speed

        # Store the trajectory of the tracked point
        self.trajectory = []  # List to store [(x, y), ...]
        self.trajectory_length = trajectory_length  # Max number of points to display

    def predict(self):
        # Prediction step
        self.state = np.dot(self.F, self.state)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        # Convert measurement to a column vector
        z = np.array([[measurement[0]], [measurement[1]]], dtype=np.float32)
        
        # Update step
        y = z - np.dot(self.H, self.state)  # Measurement residual
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R  # Innovation covariance
        self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman Gain
        self.state = self.state + np.dot(self.K, y)  # Update state estimate
        self.P = self.P - np.dot(np.dot(self.K, self.H), self.P)  # Update error covariance

        # Limit speed to max expected speed (to prevent unrealistic values)
        speed = np.linalg.norm([self.state[2], self.state[3]])
        if speed > self.max_speed:
            scaling_factor = self.max_speed / speed
            # Option A - Scale the speed
            self.state[2] *= scaling_factor
            self.state[3] *= scaling_factor
            # Option B - ignore the movement TODO

        # Add the updated position to the trajectory
        x, y = int(self.state[0]), int(self.state[1])
        self.trajectory.append((x, y))

        # Keep the trajectory within the maximum length
        if len(self.trajectory) > self.trajectory_length:
            self.trajectory.pop(0)

        return self.state

    def draw_trajectory(self, image):
        # Draw the trajectory on the image
        if len(self.trajectory) > 1:
            for i in range(1, len(self.trajectory)):
                cv2.line(image, self.trajectory[i - 1], self.trajectory[i], (0, 255, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = DetectCircle()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
