#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
        self.bridge = CvBridge()
        self.tracker = KalmanTracker()
        
    def cb_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"Image dimensions: {cv_image.shape}")
            self.detect_circle(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            
    def detect_circle(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
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
            
            # Kakman tracker
            measurement = [best_circle[0],best_circle[1]]
            self.tracker.predict()
            updated_state = self.tracker.update(measurement)
            self.get_logger().info(f"[KALMAN] center of circle: {updated_state[0],updated_state[1]} ")
            
            self.tracker.draw_trajectory(cv_image)
            cv2.circle(cv_image, (best_circle[0], best_circle[1]), best_circle[2], (0, 255, 0), 2) # (image, (x,y), radius, (BGR of circle draw), thickness)
            cv2.circle(cv_image, (best_circle[0], best_circle[1]), 2, (0, 0, 255), 3) # Red dot at the center
            self.get_logger().info(f"Detected {len(circles[0])} circle(s). Best one of radius {best_circle[2]}. Center:{best_circle[1],best_circle[2]} ")
        else:
            self.get_logger().info("No circles detected.")

        cv2.imshow("Detected Circles", cv_image)
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
