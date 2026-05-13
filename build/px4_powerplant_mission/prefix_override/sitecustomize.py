import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pawn/px4_ros_ws/install/px4_powerplant_mission'
