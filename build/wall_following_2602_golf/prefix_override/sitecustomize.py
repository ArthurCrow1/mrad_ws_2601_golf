import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arthur/mrad_ws_2602_golf/install/wall_following_2602_golf'
