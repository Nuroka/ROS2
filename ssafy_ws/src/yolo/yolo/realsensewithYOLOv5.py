import rclpy
from rclpy.node import Node

import cv2
import numpy as np

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Iamge
from cv_bridge import CvBridge

import torch

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        self.yolo_model = torch.hub.load('ultralytics/yolov5','custom',path='/home/')

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Iamge, 'detection_image', 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)


    def get_color_name(Self, hsv_color):
        h, s, v = hsv_color
        if 70 < h < 120 and s < 40 and 120 < v < 180:
            return 'white'
        elif h > 150 and s > 200 and v > 150:
            return 'red'
        elif 80 < h < 150 and s > 200 and 70 < v < 120:
            return 'blue'
        return 'unknown'
    
    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)
    
    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height //2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size //2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))

        return average_color
