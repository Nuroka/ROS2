#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov5 import YOLOv5   # YOLO 추론 코드 (아래 제공)

class ObjectTFNode(Node):
    def __init__(self):
        super().__init__('object_tf_node')

        # YOLO 모델 로드
        self.yolo = YOLOv5("yolov5s.pt", device="cpu")

        # RealSense 파이프라인 구성
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # Camera intrinsic
        profile = self.pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        # TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # pub
        self.point_pub = self.create_publisher(Point, "/detected_object_coordinates", 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.timer_callback)  # 30 FPS

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # YOLO detect
        results = self.yolo.detect(color)
        if len(results.xyxy[0]) == 0:
            return
        
        # 가장 자신있는 객체 1개만 추출 (필요시 multiple loop 가능)
        x1, y1, x2, y2, score, class_id = results.xyxy[0][0].tolist()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 깊이 값 얻기
        depth_value = depth[cy, cx] * 0.001  # meters
        
        # RealSense 픽셀 → 3D 좌표 변환
        X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [cx, cy], depth_value)

        # TF 프레임 생성 및 발행
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = "detected_object"

        t.transform.translation.x = float(X)
        t.transform.translation.y = float(Y)
        t.transform.translation.z = float(Z)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

        # Point 메시지 발행
        msg = Point()
        msg.x, msg.y, msg.z = X, Y, Z
        self.point_pub.publish(msg)

        # 디버깅용 출력
        self.get_logger().info(f"Object @ (X:{X:.3f}, Y:{Y:.3f}, Z:{Z:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()