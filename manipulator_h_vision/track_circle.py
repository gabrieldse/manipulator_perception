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

class CircleDetect(Node):
    
    def __init__(self):
        super().__init__('CircleDetect')
        self.get_logger().info("CircleDetect start")
        
        self.sub_img = self.create_subscription(Image,'/image_raw',self.cb_image,1)

    def cb_image(self, msg):
            self.image=msg
            self.get_logger().info(f"Pixel 0,0 is of color: {msg.data[0]}")
            self.get_logger().error(f"Is there a camera publishing data ?")

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetect()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
