import os
import sys
import urllib.request   
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# sudo apt-get install mpg123

# ros2 topic pub --once /text_to_speech std_msgs/msg/String "{data: '반갑습니다. 싸피에 오신 것을 환영합니다.'}"

MY_CLIENT_ID = "41xm04n3yf"
MY_CLIENT_SECRET = "4WDoD2GCabWPZoEnwIh7im00HcnAVCIEIS6ciXfG"

class ClovaTTSNode(Node):
    def __init__(self):
        super().__init__('clova_tts_node')
        
        # Subscribe to 'text_to_speech' topic
        self.subscription = self.create_subscription(
            String,
            'text_to_speech',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.client_id = MY_CLIENT_ID
        self.client_secret = MY_CLIENT_SECRET
        self.api_url = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts"
        
        # File path to save the temporary MP3
        self.output_file = "/home/ssafy/Downloads/naver_clova/tts_output.mp3"

        print("-" * 50)
        print(" Clova TTS Node Ready")
        print(" Subscribing to topic: /text_to_speech")
        print("-" * 50)

    def listener_callback(self, msg):
        target_text = msg.data
        self.get_logger().info(f"Received text: '{target_text}'")
        
        try:
            # 1. Encode text
            enc_text = urllib.parse.quote(target_text)
            
            # 2. Prepare Data (speaker=nara, mp3)
            data = f"speaker=nara&volume=0&speed=0&pitch=0&format=mp3&text={enc_text}"
            
            # 3. Create Request
            request = urllib.request.Request(self.api_url)
            request.add_header("X-NCP-APIGW-API-KEY-ID", self.client_id)
            request.add_header("X-NCP-APIGW-API-KEY", self.client_secret)
            
            # 4. Send Request
            response = urllib.request.urlopen(request, data=data.encode('utf-8'))
            rescode = response.getcode()

            if rescode == 200:
                self.get_logger().info("TTS conversion successful. Saving file...")
                
                # 5. Save MP3 file
                response_body = response.read()
                with open(self.output_file, 'wb') as f:
                    f.write(response_body)
                
                # 6. Play Audio immediately using mpg123
                self.get_logger().info("Playing audio...")
                os.system(f"mpg123 -q {self.output_file}")
                self.get_logger().info("Playback finished.")
                
            else:
                self.get_logger().error(f"Error Code: {rescode}")

        except Exception as e:
            self.get_logger().error(f"System Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClovaTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()