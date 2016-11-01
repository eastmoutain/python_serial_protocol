

import sys

sys.path.append(".")

from  xocket import *

port = "/dev/ttyUSB1"

my_board_id = 2
reply_packet(port, board_id = my_board_id, target_board_id = 0, send_file_name = "sendfile.txt")

