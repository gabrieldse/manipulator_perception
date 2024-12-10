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
            cv2.circle(cv_image, (best_circle[0], best_circle[1]), best_circle[2], (0, 255, 0), 2) # (image, (x,y), radius, (BGR of circle draw), thickness)
            cv2.circle(cv_image, (best_circle[0], best_circle[1]), 2, (0, 0, 255), 3) # Red dot at the center
            self.get_logger().info(f"Detected {len(circles[0])} circle(s). Best one of radius {best_circle[2]}")
        else:
            self.get_logger().info("No circles detected.")

        cv2.imshow("Detected Circles", cv_image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = DetectCircle()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
