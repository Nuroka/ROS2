import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssafy/ros_ws/ros_prac3/install/turtle_spawner'
