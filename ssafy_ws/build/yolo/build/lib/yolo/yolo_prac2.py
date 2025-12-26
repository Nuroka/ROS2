#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

import pyrealsense2 as rs
import torch
import cv2
import numpy as np


class FacePickAndPlace(Node):
    def __init__(self):
        super().__init__('face_pick_and_place_demo')

        # ==============================
        # 1) Dobot 액션 / 서비스 클라이언트
        # ==============================
        self._action_client = ActionClient(
            self,
            PointToPoint,
            'PTP_action',
            callback_group=ReentrantCallbackGroup()
        )

        self.cli = self.create_client(
            srv_type=GripperControl,
            srv_name='dobot_gripper_service',
            callback_group=ReentrantCallbackGroup()
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gripper service not available, waiting again...')

        self.req = GripperControl.Request()

        # ==============================
        # 2) RealSense + YOLO 초기화
        # ==============================
        self.get_logger().info("Initializing RealSense D435i...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)

        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

        self.get_logger().info("Loading YOLOv5 model (face/person)...")
        # 얼굴 전용 가중치가 있으면 그걸 쓰고,
        # 없다면 기본 yolov5s 사용 후 cls=0(person)만 사용
        self.model = torch.hub.load(
            'ultralytics/yolov5', 'yolov5s', pretrained=True
        )
        self.model.eval()

        # ==============================
        # 3) 카메라에서 얼굴 위치 한 번 인식
        # ==============================
        self.get_logger().info("Searching face on the table...")
        target_x_mm, target_y_mm, target_z_mm = self.detect_face_position()
        self.get_logger().info(
            f"Detected face target (Dobot frame approx): "
            f"x={target_x_mm:.1f} mm, y={target_y_mm:.1f} mm, z={target_z_mm:.1f} mm"
        )

        # ==============================
        # 4) Pick & Place 시퀀스 구성
        #    (네가 준 tasks_list 형식 그대로)
        # ==============================

        # 안전 높이 / 작업 높이 (mm) - 실제 환경에 맞게 조정
        safe_z = target_z_mm + 50.0     # 얼굴 위에서 살짝 띄운 높이
        work_z = target_z_mm           # 얼굴 높이 근처

        self.tasks_list = [
            # 얼굴 위 안전 높이로 이동
            ["move", [target_x_mm, target_y_mm, safe_z, 0.0], 1],
            # 그리퍼 열어두기 (또는 suction off)
            ["gripper", "open", True],
            # 얼굴 위치까지 내려가기
            ["move", [target_x_mm, target_y_mm, work_z, 0.0], 1],
            # 여기에서 필요하면 suction on/off 동작 넣기
            # 다시 안전 높이로 복귀
            ["move", [target_x_mm, target_y_mm, safe_z, 0.0], 1],
            # 홈으로 복귀 예시
            ["move", [200.0, 0.0, 100.0, 0.0], 1],
        ]

        self.goal_num = 0

    # ------------------------------------------------------------------
    # RealSense + YOLO로 얼굴 위치 찾기
    # ------------------------------------------------------------------
    def detect_face_position(self):
        """
        RealSense 컬러 + depth 프레임에서 YOLO로 얼굴(또는 person) bbox 검출 후
        가장 큰 bbox 중심 픽셀을 사용해 3D 좌표(X,Y,Z[m])를 계산하고
        Dobot 작업 좌표(mm)로 근사 변환하여 리턴.
        """

        # 카메라→Dobot 좌표 변환용 대략적인 파라미터 (실제 환경에서 튜닝 필수)
        # RealSense 카메라가 Dobot base에 대해 어떻게 놓였는지에 따라 달라짐
        X_OFFSET_MM = 200.0   # Dobot 기준에서 카메라 원점의 x 오프셋
        Y_OFFSET_MM = 0.0     # y 오프셋
        Z_OFFSET_MM = 0.0     # z 오프셋
        Y_SIGN = -1.0         # 좌우 반전 필요하면 -1, 아니면 1

        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = self.model(color_image)

            # YOLO 결과에서 가장 큰 bbox 찾기 (cls=0: person)
            boxes = results.xyxy[0].cpu().numpy()
            if len(boxes) == 0:
                # 얼굴 없으면 계속 탐색
                cv2.imshow("Face Search (no detection)", color_image)
            else:
                # 가장 넓은 bbox 선택
                max_area = 0
                best_box = None
                for x1, y1, x2, y2, conf, cls in boxes:
                    # 필요한 경우 cls 체크 (예: if int(cls)==0: ...)
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        best_box = (x1, y1, x2, y2)

                if best_box is not None:
                    x1, y1, x2, y2 = best_box
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    depth_value = depth_frame.get_distance(cx, cy)  # 단위: m
                    if depth_value > 0:
                        # 픽셀 → 카메라 3D 좌표(m)
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(
                            self.intrinsics, [cx, cy], depth_value
                        )

                        # 시각화용으로 bbox 그리기
                        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.rectangle(
                            color_image,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            (0, 255, 0),
                            2,
                        )
                        text = f"X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m"
                        cv2.putText(
                            color_image, text,
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2
                        )

                        # -------------------------------
                        # 카메라 좌표 → Dobot 좌표(mm)
                        # -------------------------------
                        x_mm = X * 1000.0 + X_OFFSET_MM
                        y_mm = Y_SIGN * Y * 1000.0 + Y_OFFSET_MM
                        z_mm = Z * 1000.0 + Z_OFFSET_MM

                        cv2.putText(
                            color_image,
                            f"Dobot approx: x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f}",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 0), 2
                        )

                        cv2.imshow("Face Search (detected)", color_image)
                        cv2.waitKey(1)

                        # 한 번 검출되면 그 값을 사용해서 종료
                        return x_mm, y_mm, z_mm

            cv2.imshow("Face Search", color_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                # 사용자가 q를 누르면 적당한 기본 위치 리턴
                self.get_logger().warn("User aborted face detection, use default pose.")
                return 125.0, 0.0, 60.0


    # ------------------------------------------------------------------
    # 여기서부터는 네가 준 PickAndPlace 코드와 동일한 구조
    # ------------------------------------------------------------------
    def execute(self):
        if self.goal_num > len(self.tasks_list) - 1:
            self.get_logger().info("All tasks done. Shutting down.")
            rclpy.shutdown()
            sys.exit()
        else:
            self.get_logger().info('*** TASK NUM ***: {0}'.format(self.goal_num))

            if self.tasks_list[self.goal_num][0] == "gripper":
                self.send_request(*self.tasks_list[self.goal_num][1:])
                self.timer = self.create_timer(
                    0.1,
                    self.timer_callback,
                    callback_group=ReentrantCallbackGroup()
                )
                self.goal_num = self.goal_num + 1

            elif self.tasks_list[self.goal_num][0] == "move":
                self.send_goal(*self.tasks_list[self.goal_num][1:])
                self.goal_num = self.goal_num + 1

    def timer_callback(self):
        if self.srv_future.done():
            result = self.srv_future.result()
            self.get_logger().info('Result of service call: {0}'.format(result))
            self.timer.cancel()
            self.execute()

    def send_request(self, gripper_state, keep_compressor_running):
        self.req.gripper_state = gripper_state
        self.req.keep_compressor_running = keep_compressor_running
        self.srv_future = self.cli.call_async(self.req)

    def send_goal(self, _target, _type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result of action call: {0}'.format(result))
            self.execute()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def main(args=None):
    rclpy.init(args=args)
    node = FacePickAndPlace()
    node.execute()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)


if __name__ == '__main__':
    main()
