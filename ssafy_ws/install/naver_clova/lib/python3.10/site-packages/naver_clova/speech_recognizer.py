import os
import sys
import json
import select
import termios
import tty
import requests
import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ctypes import *

# sudo apt-get install portaudio19-dev python3-pyaudio
# pip3 install SpeechRecognition requests

MY_CLIENT_ID = "41xm04n3yf"
MY_CLIENT_SECRET = "4WDoD2GCabWPZoEnwIh7im00HcnAVCIEIS6ciXfG"


# [Utility] Suppress ALSA Error Logs
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)# ------------------------------------------

# [Utility] Read a single key press (Linux only)
def get_key():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ClovaSpeechNode(Node):
    def __init__(self):
        super().__init__('clova_speech_node')
        self.publisher_ = self.create_publisher(String, 'recognized_speech', 10)
        
        self.client_id = MY_CLIENT_ID
        self.client_secret = MY_CLIENT_SECRET
        self.api_url = "https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=Kor"

        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        
        # Pause threshold (1.0 sec)
        self.recognizer.pause_threshold = 1.0

        # Initial noise adjustment
        with self.mic as source:
            print("\n" + "="*50)
            print("Initializing microphone... (Please remain silent)")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            print("Ready!")
            print("="*50 + "\n")

    def recognize_once(self):
        """Triggered when 's' key is pressed"""
        try:
            with self.mic as source:
                print("\nListening... Speak now (Max 5s)")
                
                try:
                    # Listen for max 5 seconds
                    audio = self.recognizer.listen(source, timeout=3.0, phrase_time_limit=5.0)
                except sr.WaitTimeoutError:
                    print("Timeout: No speech detected.")
                    return

                print("Sending data to Naver Cloud...")
                
                # Convert to WAV and send
                audio_data = audio.get_wav_data(convert_rate=16000, convert_width=2)
                
                headers = {
                    "X-NCP-APIGW-API-KEY-ID": self.client_id,
                    "X-NCP-APIGW-API-KEY": self.client_secret,
                    "Content-Type": "application/octet-stream"
                }
                
                response = requests.post(self.api_url, data=audio_data, headers=headers)
                
                if response.status_code == 200:
                    result_json = json.loads(response.text)
                    text = result_json.get("text", "")
                    
                    if text:
                        print(f"Result: \"{text}\"")
                        
                        # Publish to ROS topic
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)
                    else:
                        print("No text recognized.")
                else:
                    print(f"API Error: {response.text}")

        except Exception as e:
            self.get_logger().error(f"System Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClovaSpeechNode()
    
    print("-" * 50)
    print("   Keyboard Instructions")
    print("   [s] : Start Speech Recognition (Press and speak)")
    print("   [q] : Quit")
    print("-" * 50)

    try:
        while rclpy.ok():
            key = get_key()
            
            if key == 's':
                node.recognize_once()
                print("\nWaiting for command... ('s' to speak, 'q' to quit)")
                
            elif key == 'q' or key == '\x03': # q or Ctrl+C
                print("\nQuitting.")
                break
            
            else:
                print(f"\r Invalid key: {key} (s: start, q: quit)", end='')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()