import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dkflippo/aug2024/src/install/ros2_controllers_test_nodes'
