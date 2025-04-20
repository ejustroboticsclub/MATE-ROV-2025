import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abdelrhman/MATE-ROV-2025/software/rov_ws/install/rov25'
