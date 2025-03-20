import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zainstark/MATE-ROV-2025/software/gui/ros_ws/install/gui'
