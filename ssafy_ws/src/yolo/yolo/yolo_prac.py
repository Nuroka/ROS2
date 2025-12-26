#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import torch
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class D435iYoloNode(Node):
    def __init__(self):
        super().__init__('d435i_yolo_node')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'detected_image', 10)

        # YOLOv5 모델 로딩
        self.get_logger().info("Loading YOLOv5 model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # RealSense pipeline 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 타이머 생성 (30Hz)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # numpy array 변환
        color_image = np.asanyarray(color_frame.get_data())

        # YOLO inference
        results = self.model(color_image)

        # Bounding box 결과 시각화
        annotated = np.squeeze(results.render())

        # ROS 메시지 변환 후 publish
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = D435iYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
