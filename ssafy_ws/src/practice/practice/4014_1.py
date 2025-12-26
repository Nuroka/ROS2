# -*- coding: utf-8 -*-
import socket
import time
import threading

# Define the server host and port
HOST = '127.0.0.1'  # server IP address
PORT = 65432        # server port

# 서버 접속
def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))  
            print(f"Connected to {HOST}:{PORT}")
            return s

        except socket.error as e:
            print(f"Connect failed: {e}, retrying in 5 seconds...")
            s.close()
            time.sleep(5)


try:
    while True:
        s = connect_to_server()

        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break

                command = data.decode('utf-8')

                # 컨베이어 벨트 제어 구문 
                if command == '1':
                    print("Command 1 received")
                    # for example, conveyor run

                elif command == '2':
                    print("Command 2 received")
                    # for example, conveyor stop

                # 분류기 제어 구문 
                elif command == '3':
                    print("Command 3 received")
                    # separator right

                elif command == '4':
                    print("Command 4 received")
                    # separator left

                else:
                    print("Unknown command received")

                time.sleep(0.1)

            except socket.error as e:
                print(f"Socket error: {e}. Reconnecting...")
                break

        s.close()

except KeyboardInterrupt:
    print("Program terminated")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    print("Resources released and motor stopped.")
