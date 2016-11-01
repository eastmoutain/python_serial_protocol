

import sys

sys.path.append(".")

from  xocket import *
port = "/dev/ttyUSB0"
my_board_id = 0
request_packet(port = port, target_board_id = 2, pid = 12, board_id = my_board_id)

