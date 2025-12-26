import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import threading
import time


# 컨베이어 벨트 서버 시작 함수
def start_conveyor_server(host='127.0.0.1', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server is listening on {host}:{port}...")

        # client 접속
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        return conn


# 컨베이어 벨트를 client로 받아서 처리하는 함수
def handle_conveyor_client(conn: socket.socket, machine, status):
    if machine == 'conv':
        if status == 'conv_run':
            command = '1'
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client")

        elif status == 'conv_stop':
            command = '2'
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client")

    elif machine == 'seperator':
        if status == 3:
            command = '3'
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client")

        elif status == 4:
            command = '4'
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client")

    else:
        print("Please check the machine name")


# roboDK 서버 시작 함수
def start_roboDK_server(host='127.0.0.1', port=20000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server is listening on {host}:{port}...")

        # client 접속
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        return conn


# roboDK를 client로 받아서 처리하는 함수
def handle_roboDK_client(conn: socket.socket, status):
    if status == 1:
        command = '1'
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client")

    elif status == 2:
        command = '2'
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client")

    elif status == 3:
        command = '3'
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client")

    else:
        print("Wrong status")


class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_subscriber')

        # 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/detection_results',
            self.listener_callback,
            10
        )

        self.conv_server_conn = False
        self.roboDK_server_conn = False

        self.detection_buffer = []
        self.buffer_size = 20
        self.detection_threshold = 12

        self.get_logger().info("Object Detection Subscriber has started")

        # Conveyor server on
        server_thread = threading.Thread(target=self.start_conveyor_server_in_thread)
        server_thread.daemon = True
        server_thread.start()

        # RoboDK server on
        server_thread = threading.Thread(target=self.start_roboDK_server_in_thread)
        server_thread.daemon = True
        server_thread.start()

    def listener_callback(self, msg):
        detection_results = msg.data
        self.get_logger().info(f"Received detection results: {detection_results}")

        detected_object = self.parse_detection_results(detection_results)

        if 'back panel' in detected_object:
            self.detection_buffer.append('back panel')

        elif 'board panel' in detected_object:
            self.detection_buffer.append('board panel')

        else:
            self.detection_buffer.append('none')

        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        self.check_object_detection()

    def parse_detection_results(self, detected_results: str):
        try:
            if not detected_results:
                self.get_logger().warn("Empty detection results received")
                return []

            detected_objects = [obj.strip() for obj in detected_results.split(',')]
            return detected_objects

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            return []

    def check_object_detection(self):
        if self.detection_buffer.count('back panel') >= self.detection_threshold:
            self.perform_task_for_ojbect('back panel')

        elif self.detection_buffer.count('board panel') >= self.detection_threshold:
            self.perform_task_for_ojbect('board panel')

    def perform_task_for_ojbect(self, object_name: str):
        self.get_logger().info(f"{object_name} detected consistently! Performing task...")

        if object_name == 'back panel':
            self.perform_task_back_panel()

        elif object_name == 'board panel':
            self.perform_task_board_panel()

        self.detection_buffer = []

    def perform_task_back_panel(self):
        self.get_logger().info("Executing task for back panel")
        self.wait_for_conveyor_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, 'seperator', 3)

    def perform_task_board_panel(self):
        self.get_logger().info("Executing task for back panel")

        self.wait_for_conveyor_server_connection()
        self.wait_for_roboDK_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, 'conv', 'conv_stop')

        self.roboDKStatus = 1

        if self.roboDK_server_conn:
            handle_roboDK_client(self.roboDK_server_conn, self.roboDKStatus)

        time.sleep(5)

    def start_conveyor_server_in_thread(self):
        self.conv_server_conn = start_conveyor_server()

    def wait_for_conveyor_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")
            time.sleep(1)

        if self.conv_server_conn is None:
            self.get_logger().error("Failed to establish server connection within timeout")
        else:
            self.get_logger().info("Conveyor server connection established")

    def start_roboDK_server_in_thread(self):
        self.roboDK_server_conn = start_roboDK_server()

    def wait_for_roboDK_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")
            time.sleep(1)

        if self.conv_server_conn is None:
            self.get_logger().error("Failed to establish server connection within timeout")
        else:
            self.get_logger().info("RoboDK server connection established")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()