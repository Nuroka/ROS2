import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import websocket
import json
import threading

import math

SERVER_URL = 'robots.mincodinglab.com'

class WebSocketClientNode(Node):
    def __init__(self):
        super().__init__('websocket_client_node')

        self.ws = self.connect()
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.joint_state_subscription

    def connect(self):
        url = f"ws://{SERVER_URL}/socket/ws/common/"
        ws = websocket.WebSocket()
        try:
            ws.connect(url)
            self.get_logger().info("Websocket connection established!")
            return ws
        except Exception as e:
            self.get_logger().info(f"WebSocket Error: {e}")
            return None

    def send(self,message):
        try:
            if self.ws:
                request_obj = {"message: ": message}
                json_parsed_obj = json.dumps(request_obj)
                self.ws.send(json_parsed_obj)
                self.get_logger().info(f"Sent message: {message}")
        except Exception as e:
            self.get_logger().info(f"WebSocket Send error: {e}")

    def joint_state_callback(self, msg):
        if len(msg.position) >= 4:
            motor_1 = round(math.degrees(msg.position[0]), 2)
            motor_2 = round(math.degrees(msg.position[1]), 2)
            motor_3 = round(math.degrees(msg.position[2]), 2)
            motor_4 = round(math.degrees(msg.position[3]), 2)
            
            motor_values = {
                "joint_angle":{
                    "motor_1": motor_1,
                    "motor_2": motor_2,
                    "motor_3": motor_3,
                    "motor_4": motor_4
                }
            }

            motor_values_json = json.dumps(motor_values)

            self.get_logger().info(f"Sending motor values: {motor_values_json}")
            threading.Thread(target=self.send, args=(motor_values_json,)).start()
        else:
            self.get_logger().warn("Recieved something wrong...")

    def receive(self):
        try:
            while True:
                if self.ws:
                    response = self.ws.recv()
                    if response:
                        data = json.loads(response)
                        self.get_logger().info(f"Received response: {data['message']}")
                        self.process_received_data()
        except Exception as e:
            self.get_logger().info(f"WebSocket receive error: {e}")
        
        finally:
            if self.ws:
                self.ws.close()


def main(args=None):
    rclpy.init(args=args)

    websocket_client_node = WebSocketClientNode()

    try:
        rclpy.spin(websocket_client_node)
    except KeyboardInterrupt:
        websocket_client_node.get_logger().info('Shutting down WebSocket Client node')
    finally:
        if websocket_client_node.ws:
            websocket_client_node.ws.close()
        websocket_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()